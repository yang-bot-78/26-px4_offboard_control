#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <map>
#include <set>
#include <string>
#include <tuple>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2/utils.h>

#include "race_ego_bridge/frame_utils.hpp"

using namespace std::chrono_literals;

class EgoCloudBridge : public rclcpp::Node
{
public:
  EgoCloudBridge()
  : Node("race_ego_cloud_bridge")
  {
    map_source_ = declare_parameter<std::string>("map_source", "static");
    static_topic_ = declare_parameter<std::string>("static_topic", "/saved_map");
    live_topic_ = declare_parameter<std::string>("live_topic", "/cloud_registered");
    output_topic_ = declare_parameter<std::string>("output_topic", "/race/ego/cloud");
    required_frame_ = declare_parameter<std::string>("required_frame", "map");
    min_live_period_sec_ = declare_parameter<double>("min_live_period_sec", 0.10);
    static_republish_period_sec_ = declare_parameter<double>(
      "static_republish_period_sec", 0.50);
    // FAST-LIO and saved PCD clouds are in camera_init. Resolve both through TF
    // so world_yaw_alignment_rad is applied exactly once to their coordinates.
    transform_live_to_required_frame_ = declare_parameter<bool>(
      "transform_live_to_required_frame", true);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    // live_px4 inputs: body-frame cloud plus the PX4 pose used to project it.
    body_topic_ = declare_parameter<std::string>(
      "body_topic", "/cloud_registered_body");
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/race/pose");
    // Reject a pose older than this so a stalled PX4 stream cannot smear the
    // cloud across the map. Fail closed instead: publish nothing and let the
    // trajectory bridge brake on occupancy_timeout_sec.
    max_pose_age_sec_ = declare_parameter<double>("max_pose_age_sec", 0.30);
    pose_buffer_duration_sec_ = declare_parameter<double>("pose_buffer_duration_sec", 1.0);
    pose_buffer_duration_sec_ = std::max(
      pose_buffer_duration_sec_, max_pose_age_sec_ * 2.0);
    // Drop returns closer than this to the body origin. The mid360 blind zone
    // is 0.5 m and the measured nearest return is 0.725 m, so this only guards
    // against self-hits appearing after a model change; it is not load-bearing
    // today.
    min_point_range_m_ = declare_parameter<double>("min_point_range_m", 0.20);
    live_obstacle_memory_sec_ = declare_parameter<double>(
      "live_obstacle_memory_sec", 0.0);
    live_obstacle_voxel_size_m_ = declare_parameter<double>(
      "live_obstacle_voxel_size_m", 0.10);
    live_obstacle_min_observations_ = declare_parameter<int>(
      "live_obstacle_min_observations", 2);
    if (live_obstacle_memory_sec_ < 0.0 || live_obstacle_memory_sec_ > 1.5 ||
      live_obstacle_voxel_size_m_ < 0.05 || live_obstacle_voxel_size_m_ > 0.25 ||
      live_obstacle_min_observations_ < 1 || live_obstacle_min_observations_ > 5)
    {
      throw std::runtime_error(
              "live obstacle memory must be 0..1.5s with a 0.05..0.25m voxel");
    }

    if (map_source_ != "static" && map_source_ != "live" && map_source_ != "live_px4" &&
      map_source_ != "static_live_px4")
    {
      throw std::runtime_error(
              "map_source must be static, live, live_px4 or static_live_px4");
    }

    const bool has_camera_init_cloud =
      map_source_ == "static" || map_source_ == "live" || map_source_ == "static_live_px4";
    if (has_camera_init_cloud && transform_live_to_required_frame_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    }

    auto output_qos = rclcpp::QoS(1).reliable();
    if (map_source_ == "static") {
      output_qos.transient_local();
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, output_qos);

    if (map_source_ == "static") {
      subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        static_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&EgoCloudBridge::staticCallback, this, std::placeholders::_1));
      static_republish_timer_ = create_wall_timer(
        std::chrono::duration<double>(static_republish_period_sec_),
        std::bind(&EgoCloudBridge::republishStaticMap, this));
    } else if (map_source_ == "live_px4" || map_source_ == "static_live_px4") {
      if (map_source_ == "static_live_px4") {
        static_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          static_topic_, rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&EgoCloudBridge::staticCallback, this, std::placeholders::_1));
      }
      pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&EgoCloudBridge::poseCallback, this, std::placeholders::_1));
      subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        body_topic_, rclcpp::SensorDataQoS(),
        std::bind(&EgoCloudBridge::bodyCallback, this, std::placeholders::_1));
    } else {
      subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        live_topic_, rclcpp::SensorDataQoS(),
        std::bind(&EgoCloudBridge::callback, this, std::placeholders::_1));
    }

    std::string input = static_topic_;
    if (map_source_ == "live") {
      input = live_topic_;
    } else if (map_source_ == "live_px4") {
      input = body_topic_;
    } else if (map_source_ == "static_live_px4") {
      input = static_topic_ + "+" + body_topic_;
    }
    RCLCPP_INFO(
      get_logger(),
      "EGO cloud bridge: source=%s input=%s output=%s required_frame=%s frame_transform=%s",
      map_source_.c_str(), input.c_str(), output_topic_.c_str(), required_frame_.c_str(),
      ((map_source_ == "static" || map_source_ == "live") &&
      transform_live_to_required_frame_) ? "tf_to_map" :
      ((map_source_ == "live_px4" || map_source_ == "static_live_px4") ?
      "map_pose_projection" : "disabled"));
  }

private:
  void staticCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->width * msg->height == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reject empty static EGO map");
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud;
    if (!transformToRequiredFrame(*msg, cloud, "static_map")) {
      return;
    }
    static_map_ = cloud;
    have_static_map_ = true;
    RCLCPP_INFO(
      get_logger(), "[EGO_STATIC_MAP_READY] topic=%s frame=%s points=%u fused=%s",
      static_topic_.c_str(), static_map_.header.frame_id.c_str(),
      static_map_.width * static_map_.height,
      map_source_ == "static_live_px4" ? "true" : "false");
    if (map_source_ == "static") {
      publishMap(static_map_);
    }
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->width * msg->height == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reject empty EGO cloud");
      return;
    }
    // Rate-limit before the TF lookup so a fast live stream does not pay for a
    // transform on frames it is about to drop anyway.
    if (map_source_ == "live" && last_publish_.nanoseconds() != 0 &&
      (now() - last_publish_).seconds() < min_live_period_sec_)
    {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    if (!transformToRequiredFrame(*msg, cloud, "live_cloud")) {
      return;
    }

    publishMap(cloud);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != required_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "POSE_FRAME_MISMATCH pose=%s expected=%s",
        msg->header.frame_id.c_str(), required_frame_.c_str());
      return;
    }
    const race_ego_bridge::Vec3 map{
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    const double yaw_map = tf2::getYaw(msg->pose.orientation);
    if (!race_ego_bridge::finite(map) || !std::isfinite(yaw_map)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "POSE_REJECT non-finite");
      return;
    }
    rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }
    if (!pose_history_.empty() && stamp < pose_history_.back().stamp) {
      pose_history_.clear();
    }
    pose_history_.push_back({
      stamp,
      race_ego_bridge::TimedYawPose{
        stamp.seconds(), map, yaw_map}});
    const auto retention = rclcpp::Duration::from_seconds(pose_buffer_duration_sec_);
    while (!pose_history_.empty() && stamp - pose_history_.front().stamp > retention) {
      pose_history_.pop_front();
    }
    have_pose_ = true;
  }

  bool transformToRequiredFrame(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output,
    const char * source_name)
  {
    output = input;
    if (input.header.frame_id == required_frame_) {
      return true;
    }
    if (!transform_live_to_required_frame_ || !tf_buffer_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "FRAME_MISMATCH %s=%s expected=%s",
        source_name, input.header.frame_id.c_str(), required_frame_.c_str());
      return false;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        required_frame_, input.header.frame_id, input.header.stamp,
        tf2::durationFromSec(tf_timeout_sec_));
      tf2::doTransform(input, output, transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FRAME_TRANSFORM_FAILED %s=%s expected=%s reason=%s", source_name,
        input.header.frame_id.c_str(), required_frame_.c_str(), error.what());
      return false;
    }
    if (output.header.frame_id != required_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FRAME_MISMATCH_AFTER_TRANSFORM %s=%s expected=%s", source_name,
        output.header.frame_id.c_str(), required_frame_.c_str());
      return false;
    }
    if (!logged_transform_once_) {
      logged_transform_once_ = true;
      RCLCPP_INFO(
        get_logger(), "[EGO_MAP_FRAME_TRANSFORM] input_frame=%s output_frame=%s timeout=%.3fs",
        input.header.frame_id.c_str(), required_frame_.c_str(), tf_timeout_sec_);
    }
    return true;
  }

  bool poseAt(
    const rclcpp::Time & target_stamp, race_ego_bridge::TimedYawPose & pose,
    double & nearest_skew_sec) const
  {
    nearest_skew_sec = std::numeric_limits<double>::infinity();
    if (pose_history_.empty()) {
      return false;
    }
    const double target_sec = target_stamp.seconds();
    const auto after = std::lower_bound(
      pose_history_.begin(), pose_history_.end(), target_sec,
      [](const TimedPoseSample & sample, double stamp_sec) {
        return sample.pose.stamp_sec < stamp_sec;
      });
    if (after == pose_history_.begin()) {
      pose = after->pose;
      nearest_skew_sec = std::abs(after->pose.stamp_sec - target_sec);
      return nearest_skew_sec <= max_pose_age_sec_;
    }
    if (after == pose_history_.end()) {
      pose = pose_history_.back().pose;
      nearest_skew_sec = std::abs(pose.stamp_sec - target_sec);
      return nearest_skew_sec <= max_pose_age_sec_;
    }
    const auto before = std::prev(after);
    nearest_skew_sec = std::min(
      std::abs(before->pose.stamp_sec - target_sec),
      std::abs(after->pose.stamp_sec - target_sec));
    if (nearest_skew_sec > max_pose_age_sec_) {
      return false;
    }
    pose = race_ego_bridge::interpolateYawPose(before->pose, after->pose, target_sec);
    return true;
  }

  // Project the body-frame cloud with the PX4 pose. Keeps the occupancy map and
  // the EGO odometry (also PX4-sourced) on one pose, so a wrong SLAM attitude
  // cannot rotate the map out from under the planner.
  void bodyCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->width * msg->height == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reject empty EGO cloud");
      return;
    }
    if (last_publish_.nanoseconds() != 0 &&
      (now() - last_publish_).seconds() < min_live_period_sec_)
    {
      return;
    }
    if (!have_pose_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "PX4_POSE_MISSING topic=%s",
        pose_topic_.c_str());
      return;
    }
    rclcpp::Time cloud_stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (cloud_stamp.nanoseconds() == 0) {
      cloud_stamp = now();
    }
    race_ego_bridge::TimedYawPose synchronized_pose;
    double pose_skew_sec = std::numeric_limits<double>::infinity();
    if (!poseAt(cloud_stamp, synchronized_pose, pose_skew_sec)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PX4_POSE_SYNC_FAILED cloud_stamp=%.9f nearest_skew=%.3fs limit=%.3fs",
        cloud_stamp.seconds(), pose_skew_sec, max_pose_age_sec_);
      return;
    }
    if (map_source_ == "static_live_px4" && !have_static_map_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STATIC_MAP_MISSING_FOR_FUSION topic=%s", static_topic_.c_str());
      return;
    }
    publishProjected(*msg, synchronized_pose.position, synchronized_pose.yaw);
  }

  void publishProjected(
    const sensor_msgs::msg::PointCloud2 & input, const race_ego_bridge::Vec3 & position,
    double yaw_map)
  {
    sensor_msgs::msg::PointCloud2 output;
    output.header.frame_id = required_frame_;
    output.header.stamp = input.header.stamp;
    output.height = 1;
    output.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(input.width * input.height);

    sensor_msgs::PointCloud2ConstIterator<float> in_x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(input, "z");
    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");

    const double min_range_sq = min_point_range_m_ * min_point_range_m_;
    size_t kept = 0;
    for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z) {
      const race_ego_bridge::Vec3 body{*in_x, *in_y, *in_z};
      if (!race_ego_bridge::finite(body)) {
        continue;
      }
      const double range_sq = body.x * body.x + body.y * body.y + body.z * body.z;
      if (range_sq < min_range_sq) {
        continue;
      }
      const auto point = race_ego_bridge::bodyPointToMap(body, position, yaw_map);
      *out_x = static_cast<float>(point.x);
      *out_y = static_cast<float>(point.y);
      *out_z = static_cast<float>(point.z);
      ++out_x; ++out_y; ++out_z;
      ++kept;
    }
    if (kept == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "PROJECTED_CLOUD_EMPTY input_points=%u",
        input.width * input.height);
      return;
    }
    modifier.resize(kept);

    if (!logged_projection_once_) {
      logged_projection_once_ = true;
      RCLCPP_INFO(
        get_logger(),
        "[EGO_MAP_PX4_PROJECTION] input_frame=%s output_frame=%s pose_topic=%s "
        "max_pose_age=%.3fs min_range=%.2fm",
        input.header.frame_id.c_str(), required_frame_.c_str(), pose_topic_.c_str(),
        max_pose_age_sec_, min_point_range_m_);
    }
    if (map_source_ == "static_live_px4") {
      rememberLiveMap(output);
      publishFusedMap();
    } else {
      publishMap(output);
    }
  }

  void rememberLiveMap(const sensor_msgs::msg::PointCloud2 & live_map)
  {
    const auto timestamp = now();
    if (!live_map_history_.empty() && timestamp < live_map_history_.back().stamp) {
      live_map_history_.clear();
    }
    live_map_history_.push_back({timestamp, live_map});
    const auto max_age = rclcpp::Duration::from_seconds(live_obstacle_memory_sec_);
    while (!live_map_history_.empty() &&
      timestamp - live_map_history_.front().stamp > max_age)
    {
      live_map_history_.pop_front();
    }

    // Keep confirmation state independently of the short raw-cloud history.
    // A pillar can be missed by one scan while the vehicle is moving; rebuilding
    // counts from only the recent clouds made an already confirmed pillar vanish.
    std::set<LiveVoxelKey> frame_voxels;
    sensor_msgs::PointCloud2ConstIterator<float> in_x(live_map, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(live_map, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(live_map, "z");
    for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z) {
      if (!std::isfinite(*in_x) || !std::isfinite(*in_y) || !std::isfinite(*in_z)) {
        continue;
      }
      const LiveVoxelKey voxel(
        static_cast<int>(std::floor(*in_x / live_obstacle_voxel_size_m_)),
        static_cast<int>(std::floor(*in_y / live_obstacle_voxel_size_m_)),
        static_cast<int>(std::floor(*in_z / live_obstacle_voxel_size_m_)));
      if (!frame_voxels.insert(voxel).second) {
        continue;
      }
      auto & evidence = confirmed_live_voxels_[voxel];
      ++evidence.observations;
      evidence.last_seen = timestamp;
      evidence.point = {*in_x, *in_y, *in_z};
    }
    for (auto it = confirmed_live_voxels_.begin(); it != confirmed_live_voxels_.end();) {
      if ((live_obstacle_memory_sec_ <= 0.0 && it->second.last_seen != timestamp) ||
        (timestamp - it->second.last_seen).seconds() > live_obstacle_memory_sec_)
      {
        it = confirmed_live_voxels_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void publishFusedMap()
  {
    sensor_msgs::msg::PointCloud2 fused;
    fused.header.frame_id = required_frame_;
    fused.height = 1;
    fused.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(fused);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    size_t maximum_points = static_cast<size_t>(static_map_.width) * static_map_.height;
    for (const auto & sample : live_map_history_) {
      maximum_points += static_cast<size_t>(sample.cloud.width) * sample.cloud.height;
    }
    modifier.resize(maximum_points);

    sensor_msgs::PointCloud2Iterator<float> out_x(fused, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(fused, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(fused, "z");
    size_t kept = 0;
    size_t live_raw_points = 0;
    const auto append_static_cloud = [&](const sensor_msgs::msg::PointCloud2 & cloud) {
        sensor_msgs::PointCloud2ConstIterator<float> in_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(cloud, "z");
        for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z) {
          if (!std::isfinite(*in_x) || !std::isfinite(*in_y) || !std::isfinite(*in_z)) {
            continue;
          }
          *out_x = *in_x;
          *out_y = *in_y;
          *out_z = *in_z;
          ++out_x;
          ++out_y;
          ++out_z;
          ++kept;
        }
      };
    append_static_cloud(static_map_);

    for (const auto & sample : live_map_history_) {
      live_raw_points += static_cast<size_t>(sample.cloud.width) * sample.cloud.height;
    }
    size_t confirmed_live_voxels = 0;
    for (const auto & item : confirmed_live_voxels_) {
      if (item.second.observations < live_obstacle_min_observations_) {
        continue;
      }
      *out_x = item.second.point[0];
      *out_y = item.second.point[1];
      *out_z = item.second.point[2];
      ++out_x;
      ++out_y;
      ++out_z;
      ++kept;
      ++confirmed_live_voxels;
    }
    modifier.resize(kept);
    if (kept == 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "FUSED_EGO_MAP_EMPTY");
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[EGO_MAP_FUSION] static_points=%u live_raw_points=%zu confirmed_live_voxels=%zu "
      "min_observations=%d memory_sec=%.2f fused_points=%zu",
      static_map_.width * static_map_.height, live_raw_points, confirmed_live_voxels,
      live_obstacle_min_observations_, live_obstacle_memory_sec_, kept);
    publishMap(fused);
  }

  void republishStaticMap()
  {
    if (have_static_map_) {
      publishMap(static_map_);
    }
  }

  void publishMap(const sensor_msgs::msg::PointCloud2 & msg)
  {
    auto output = msg;
    output.header.stamp = now();
    publisher_->publish(output);
    last_publish_ = now();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[EGO_MAP_INPUT] topic=%s frame=%s points=%u source=%s refresh_period=%.2fs",
      output_topic_.c_str(), output.header.frame_id.c_str(), output.width * output.height,
      map_source_.c_str(), map_source_ == "static" ? static_republish_period_sec_ : 0.0);
  }

  std::string map_source_;
  std::string static_topic_;
  std::string live_topic_;
  std::string output_topic_;
  std::string required_frame_;
  double min_live_period_sec_{0.10};
  double static_republish_period_sec_{0.50};
  bool transform_live_to_required_frame_{true};
  double tf_timeout_sec_{0.05};
  bool logged_transform_once_{false};
  std::string body_topic_;
  std::string pose_topic_;
  double max_pose_age_sec_{0.30};
  double pose_buffer_duration_sec_{1.0};
  double min_point_range_m_{0.20};
  double live_obstacle_memory_sec_{0.0};
  double live_obstacle_voxel_size_m_{0.10};
  int live_obstacle_min_observations_{2};
  bool logged_projection_once_{false};
  bool have_pose_{false};
  struct TimedPoseSample
  {
    rclcpp::Time stamp;
    race_ego_bridge::TimedYawPose pose;
  };
  std::deque<TimedPoseSample> pose_history_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool have_static_map_{false};
  sensor_msgs::msg::PointCloud2 static_map_;
  struct TimedLiveMap
  {
    rclcpp::Time stamp;
    sensor_msgs::msg::PointCloud2 cloud;
  };
  using LiveVoxelKey = std::tuple<int, int, int>;
  struct LiveVoxelEvidence
  {
    int observations{0};
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
    std::array<float, 3> point{0.0F, 0.0F, 0.0F};
  };
  std::deque<TimedLiveMap> live_map_history_;
  std::map<LiveVoxelKey, LiveVoxelEvidence> confirmed_live_voxels_;
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_subscription_;
  rclcpp::TimerBase::SharedPtr static_republish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoCloudBridge>());
  rclcpp::shutdown();
  return 0;
}
