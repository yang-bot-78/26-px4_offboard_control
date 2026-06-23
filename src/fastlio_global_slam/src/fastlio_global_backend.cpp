#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "fastlio_global_slam/srv/load_map.hpp"
#include "fastlio_global_slam/srv/relocalize.hpp"
#include "fastlio_global_slam/srv/save_map.hpp"

#ifdef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#endif

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using OdomMsg = nav_msgs::msg::Odometry;
using CloudMsg = sensor_msgs::msg::PointCloud2;
using SaveMapSrv = fastlio_global_slam::srv::SaveMap;
using LoadMapSrv = fastlio_global_slam::srv::LoadMap;
using RelocalizeSrv = fastlio_global_slam::srv::Relocalize;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<OdomMsg, CloudMsg>;

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

std::string expandUserPath(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }

  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return path;
  }

  if (path.size() == 1) {
    return std::string(home);
  }

  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }

  return path;
}

Eigen::Isometry3d poseFromOdometry(const OdomMsg & msg)
{
  Eigen::Quaterniond q(
    msg.pose.pose.orientation.w,
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z);
  q.normalize();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    msg.pose.pose.position.z);
  return pose;
}

geometry_msgs::msg::Pose poseMsgFromEigen(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();

  const Eigen::Quaterniond q(pose.rotation());
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  return msg;
}

Eigen::Isometry3d poseFromComponents(
  double x,
  double y,
  double z,
  double qx,
  double qy,
  double qz,
  double qw)
{
  Eigen::Quaterniond q(qw, qx, qy, qz);
  q.normalize();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

CloudT::Ptr transformCloud(const CloudT::Ptr & cloud, const Eigen::Isometry3d & tf)
{
  CloudT::Ptr out(new CloudT());
  out->reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    const Eigen::Vector3d p = tf * Eigen::Vector3d(pt.x, pt.y, pt.z);
    PointT q;
    q.x = static_cast<float>(p.x());
    q.y = static_cast<float>(p.y());
    q.z = static_cast<float>(p.z());
    q.intensity = pt.intensity;
    out->push_back(q);
  }
  out->width = static_cast<std::uint32_t>(out->size());
  out->height = 1;
  out->is_dense = false;
  return out;
}

CloudT::Ptr downsampleCloud(const CloudT::Ptr & input, double leaf_size)
{
  if (input == nullptr || input->empty()) {
    return std::make_shared<CloudT>();
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(input);
  CloudT::Ptr output(new CloudT());
  voxel.filter(*output);
  return output;
}

#ifdef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
gtsam::Pose3 pose3FromEigen(const Eigen::Isometry3d & pose)
{
  return gtsam::Pose3(
    gtsam::Rot3(pose.rotation()),
    gtsam::Point3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
}

Eigen::Isometry3d eigenFromPose3(const gtsam::Pose3 & pose)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = pose.rotation().matrix();
  tf.translation() = Eigen::Vector3d(pose.x(), pose.y(), pose.z());
  return tf;
}
#endif

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    fields.push_back(item);
  }
  return fields;
}

}  // namespace

class FastlioGlobalBackend : public rclcpp::Node
{
public:
  FastlioGlobalBackend()
  : Node("fastlio_global_backend")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    global_map_topic_ = declare_parameter<std::string>("global_map_topic", "/fastlio_global/map");
    optimized_path_topic_ = declare_parameter<std::string>("optimized_path_topic", "/fastlio_global/path");
    loop_marker_topic_ = declare_parameter<std::string>("loop_marker_topic", "/fastlio_global/loop_markers");
    relocalized_pose_topic_ = declare_parameter<std::string>("relocalized_pose_topic", "/fastlio_global/relocalized_pose");
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "camera_init");
    map_save_directory_ = expandUserPath(declare_parameter<std::string>("map_save_directory", "~/fastlio_global_map"));
    map_load_directory_ = expandUserPath(declare_parameter<std::string>("map_load_directory", "~/fastlio_global_map"));
    cloud_is_world_frame_ = declare_parameter<bool>("cloud_is_world_frame", true);
    keyframe_translation_thresh_m_ = declare_parameter<double>("keyframe_translation_thresh_m", 0.5);
    keyframe_rotation_thresh_deg_ = declare_parameter<double>("keyframe_rotation_thresh_deg", 10.0);
    keyframe_voxel_leaf_m_ = declare_parameter<double>("keyframe_voxel_leaf_m", 0.2);
    submap_voxel_leaf_m_ = declare_parameter<double>("submap_voxel_leaf_m", 0.3);
    save_keyframe_leaf_m_ = declare_parameter<double>("save_keyframe_leaf_m", 0.15);
    scan_context_rings_ = declare_parameter<int>("scan_context_rings", 20);
    scan_context_sectors_ = declare_parameter<int>("scan_context_sectors", 60);
    scan_context_max_radius_m_ = declare_parameter<double>("scan_context_max_radius_m", 50.0);
    scan_context_similarity_threshold_ = declare_parameter<double>("scan_context_similarity_threshold", 0.82);
    loop_recent_exclusion_count_ = declare_parameter<int>("loop_recent_exclusion_count", 30);
    loop_submap_half_width_ = declare_parameter<int>("loop_submap_half_width", 5);
    loop_detection_stride_ = declare_parameter<int>("loop_detection_stride", 5);
    auto_relocalize_stride_ = declare_parameter<int>("auto_relocalize_stride", 10);
    icp_max_correspondence_distance_m_ = declare_parameter<double>("icp_max_correspondence_distance_m", 2.0);
    icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 40);
    icp_fitness_threshold_ = declare_parameter<double>("icp_fitness_threshold", 0.35);
    relocalization_fitness_threshold_ = declare_parameter<double>("relocalization_fitness_threshold", 0.45);
    publish_map_every_n_keyframes_ = declare_parameter<int>("publish_map_every_n_keyframes", 3);
    enable_loop_closure_ = declare_parameter<bool>("enable_loop_closure", true);
    enable_relocalization_mode_ = declare_parameter<bool>("enable_relocalization_mode", false);
    enable_gtsam_backend_param_ = declare_parameter<bool>("enable_gtsam_backend", true);

    map_pub_ = create_publisher<CloudMsg>(global_map_topic_, 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(optimized_path_topic_, 10);
    loop_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(loop_marker_topic_, 10);
    relocalized_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(relocalized_pose_topic_, 10);

    save_map_srv_ = create_service<SaveMapSrv>(
      "~/save_map",
      std::bind(&FastlioGlobalBackend::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    load_map_srv_ = create_service<LoadMapSrv>(
      "~/load_map",
      std::bind(&FastlioGlobalBackend::handleLoadMap, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    relocalize_srv_ = create_service<RelocalizeSrv>(
      "~/relocalize",
      std::bind(&FastlioGlobalBackend::handleRelocalize, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    odom_sub_.subscribe(this, odom_topic_);
    cloud_sub_.subscribe(this, cloud_topic_);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), odom_sub_, cloud_sub_);
    sync_->registerCallback(std::bind(&FastlioGlobalBackend::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));

#ifdef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
    gtsam_enabled_ = enable_gtsam_backend_param_;
    if (gtsam_enabled_) {
      resetGtsamState();
      RCLCPP_INFO(get_logger(), "GTSAM backend enabled.");
    }
#else
    gtsam_enabled_ = false;
    if (enable_gtsam_backend_param_) {
      RCLCPP_WARN(get_logger(), "GTSAM requested but not found at build time. Running without factor graph optimization.");
    }
#endif

    optimized_path_.header.frame_id = map_frame_id_;
    RCLCPP_INFO(
      get_logger(),
      "FAST-LIO global backend listening on odom=%s cloud=%s",
      odom_topic_.c_str(), cloud_topic_.c_str());
  }

private:
  struct Keyframe
  {
    Eigen::Isometry3d odom_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d optimized_pose = Eigen::Isometry3d::Identity();
    CloudT::Ptr local_cloud{new CloudT()};
    Eigen::MatrixXf scan_context;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool loaded_from_map{false};
  };

  struct LoopCandidate
  {
    bool valid{false};
    int candidate_index{-1};
    int sector_shift{0};
    double similarity{-1.0};
  };

  void syncedCallback(const OdomMsg::ConstSharedPtr & odom_msg, const CloudMsg::ConstSharedPtr & cloud_msg)
  {
    CloudT::Ptr incoming(new CloudT());
    pcl::fromROSMsg(*cloud_msg, *incoming);
    if (incoming->empty()) {
      return;
    }

    const Eigen::Isometry3d odom_pose = poseFromOdometry(*odom_msg);
    CloudT::Ptr local_cloud = cloud_is_world_frame_ ? transformCloud(incoming, odom_pose.inverse()) : incoming;
    local_cloud = downsampleCloud(local_cloud, keyframe_voxel_leaf_m_);

    latest_odom_pose_ = odom_pose;
    latest_local_cloud_ = local_cloud;
    latest_stamp_ = odom_msg->header.stamp;
    has_latest_scan_ = true;
    ++callback_count_;

    if (enable_relocalization_mode_ && has_loaded_map_ && !has_relocalized_ &&
        callback_count_ % std::max(1, auto_relocalize_stride_) == 0) {
      std::string reason;
      Eigen::Isometry3d estimated_pose = Eigen::Isometry3d::Identity();
      attemptRelocalization(reason, estimated_pose);
    }

    if (enable_relocalization_mode_ && has_loaded_map_ && !has_relocalized_) {
      return;
    }

    if (!shouldCreateKeyframe(odom_pose)) {
      return;
    }

    Keyframe keyframe;
    keyframe.odom_pose = odom_pose;
    keyframe.optimized_pose = globalPoseFromOdom(odom_pose);
    keyframe.local_cloud = local_cloud;
    keyframe.scan_context = buildScanContext(*local_cloud);
    keyframe.stamp = odom_msg->header.stamp;
    keyframe.loaded_from_map = false;

    const int new_index = static_cast<int>(keyframes_.size());
    keyframes_.push_back(keyframe);

    addOdometryFactor(new_index);
    optimizePoseGraph();

    if (enable_loop_closure_ && new_index >= loop_recent_exclusion_count_ &&
        new_index % std::max(1, loop_detection_stride_) == 0) {
      const LoopCandidate loop = detectLoopCandidate(keyframes_[new_index].scan_context, new_index, loop_recent_exclusion_count_);
      if (loop.valid) {
        tryAddLoopClosure(new_index, loop, icp_fitness_threshold_);
      }
    }

    updatePublishedPath();
    if (publish_map_every_n_keyframes_ > 0 && new_index % publish_map_every_n_keyframes_ == 0) {
      publishGlobalMap();
    }
  }

  bool shouldCreateKeyframe(const Eigen::Isometry3d & pose) const
  {
    if (keyframes_.empty()) {
      return true;
    }

    int last_live_index = -1;
    for (int i = static_cast<int>(keyframes_.size()) - 1; i >= 0; --i) {
      if (!keyframes_[i].loaded_from_map) {
        last_live_index = i;
        break;
      }
    }
    if (last_live_index < 0) {
      return true;
    }

    const Eigen::Vector3d delta_t = pose.translation() - keyframes_[last_live_index].odom_pose.translation();
    const double translation = delta_t.norm();

    const Eigen::Matrix3d delta_r = keyframes_[last_live_index].odom_pose.rotation().transpose() * pose.rotation();
    const Eigen::AngleAxisd aa(delta_r);
    const double rotation_deg = std::abs(aa.angle()) * kRadToDeg;

    return translation >= keyframe_translation_thresh_m_ || rotation_deg >= keyframe_rotation_thresh_deg_;
  }

  Eigen::Isometry3d globalPoseFromOdom(const Eigen::Isometry3d & odom_pose) const
  {
    if (has_relocalized_) {
      return map_to_odom_ * odom_pose;
    }
    return odom_pose;
  }

  Eigen::MatrixXf buildScanContext(const CloudT & cloud) const
  {
    Eigen::MatrixXf descriptor = Eigen::MatrixXf::Zero(scan_context_rings_, scan_context_sectors_);
    for (const auto & pt : cloud.points) {
      const double range_xy = std::hypot(pt.x, pt.y);
      if (range_xy < 1e-3 || range_xy > scan_context_max_radius_m_) {
        continue;
      }

      const double angle = std::atan2(pt.y, pt.x);
      const double angle_deg = std::fmod((angle * 180.0 / M_PI) + 360.0, 360.0);
      const int ring = std::min(
        scan_context_rings_ - 1,
        static_cast<int>(range_xy / scan_context_max_radius_m_ * static_cast<double>(scan_context_rings_)));
      const int sector = std::min(
        scan_context_sectors_ - 1,
        static_cast<int>(angle_deg / 360.0 * static_cast<double>(scan_context_sectors_)));
      descriptor(ring, sector) = std::max(descriptor(ring, sector), pt.z);
    }
    return descriptor;
  }

  double shiftedSimilarity(const Eigen::MatrixXf & lhs, const Eigen::MatrixXf & rhs, int shift) const
  {
    double sum = 0.0;
    int valid_cols = 0;
    for (int col = 0; col < scan_context_sectors_; ++col) {
      const int rhs_col = (col + shift) % scan_context_sectors_;
      const Eigen::VectorXf a = lhs.col(col);
      const Eigen::VectorXf b = rhs.col(rhs_col);
      const double denom = a.norm() * b.norm();
      if (denom < 1e-6) {
        continue;
      }
      sum += a.dot(b) / denom;
      ++valid_cols;
    }
    return valid_cols == 0 ? -1.0 : sum / static_cast<double>(valid_cols);
  }

  LoopCandidate detectLoopCandidate(const Eigen::MatrixXf & query_descriptor, int query_index, int recent_exclusion) const
  {
    LoopCandidate best;
    for (int idx = 0; idx < static_cast<int>(keyframes_.size()); ++idx) {
      if (idx == query_index) {
        continue;
      }
      if (query_index >= 0 && idx >= query_index - recent_exclusion && idx <= query_index) {
        continue;
      }
      if (query_index < 0 && !keyframes_[idx].loaded_from_map) {
        continue;
      }

      double best_for_idx = -1.0;
      int best_shift = 0;
      for (int shift = 0; shift < scan_context_sectors_; ++shift) {
        const double sim = shiftedSimilarity(query_descriptor, keyframes_[idx].scan_context, shift);
        if (sim > best_for_idx) {
          best_for_idx = sim;
          best_shift = shift;
        }
      }

      if (best_for_idx > best.similarity) {
        best.valid = best_for_idx >= scan_context_similarity_threshold_;
        best.candidate_index = idx;
        best.sector_shift = best_shift;
        best.similarity = best_for_idx;
      }
    }
    return best;
  }

  std::vector<Eigen::Isometry3d> effectivePoses() const
  {
    std::vector<Eigen::Isometry3d> poses;
    poses.reserve(keyframes_.size());
    for (const auto & keyframe : keyframes_) {
      poses.push_back(keyframe.optimized_pose);
    }
    return poses;
  }

  CloudT::Ptr buildLocalSubmap(int center_index, const std::vector<Eigen::Isometry3d> & poses) const
  {
    CloudT::Ptr submap(new CloudT());
    const int begin = std::max(0, center_index - loop_submap_half_width_);
    const int end = std::min(static_cast<int>(keyframes_.size()) - 1, center_index + loop_submap_half_width_);
    const Eigen::Isometry3d ref_pose = poses[center_index];

    for (int i = begin; i <= end; ++i) {
      CloudT::Ptr transformed = transformCloud(keyframes_[i].local_cloud, ref_pose.inverse() * poses[i]);
      *submap += *transformed;
    }
    return downsampleCloud(submap, submap_voxel_leaf_m_);
  }

  std::optional<Eigen::Isometry3d> runIcpRegistration(
    const CloudT::Ptr & source,
    const CloudT::Ptr & target,
    double fitness_threshold,
    double * fitness_score = nullptr) const
  {
    if (source == nullptr || target == nullptr || source->empty() || target->empty()) {
      return std::nullopt;
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_m_);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    CloudT aligned;
    icp.align(aligned);
    if (!icp.hasConverged()) {
      return std::nullopt;
    }

    const double score = icp.getFitnessScore();
    if (fitness_score != nullptr) {
      *fitness_score = score;
    }
    if (score > fitness_threshold) {
      return std::nullopt;
    }

    return Eigen::Isometry3d(icp.getFinalTransformation().cast<double>());
  }

  void tryAddLoopClosure(int current_index, const LoopCandidate & loop, double fitness_threshold)
  {
    const auto poses = effectivePoses();
    CloudT::Ptr source = buildLocalSubmap(current_index, poses);
    CloudT::Ptr target = buildLocalSubmap(loop.candidate_index, poses);

    double fitness = std::numeric_limits<double>::infinity();
    const auto relative_transform = runIcpRegistration(source, target, fitness_threshold, &fitness);
    if (!relative_transform.has_value()) {
      return;
    }

    addLoopFactor(loop.candidate_index, current_index, relative_transform.value());
    optimizePoseGraph();
    publishLoopMarker(loop.candidate_index, current_index);
    RCLCPP_INFO(
      get_logger(),
      "Loop closure accepted: current=%d candidate=%d similarity=%.3f fitness=%.4f",
      current_index, loop.candidate_index, loop.similarity, fitness);
  }

  bool attemptRelocalization(std::string & message, Eigen::Isometry3d & estimated_pose)
  {
    if (!has_loaded_map_) {
      message = "No loaded map is available.";
      return false;
    }
    if (!has_latest_scan_ || latest_local_cloud_ == nullptr || latest_local_cloud_->empty()) {
      message = "No latest scan available for relocalization.";
      return false;
    }

    const Eigen::MatrixXf latest_descriptor = buildScanContext(*latest_local_cloud_);
    const LoopCandidate loop = detectLoopCandidate(latest_descriptor, -1, 0);
    if (!loop.valid || loop.candidate_index < 0) {
      message = "Scan Context did not find a confident relocalization candidate.";
      return false;
    }

    const auto poses = effectivePoses();
    CloudT::Ptr target = buildLocalSubmap(loop.candidate_index, poses);
    double fitness = std::numeric_limits<double>::infinity();
    const auto target_from_source = runIcpRegistration(latest_local_cloud_, target, relocalization_fitness_threshold_, &fitness);
    if (!target_from_source.has_value()) {
      std::ostringstream oss;
      oss << "ICP relocalization failed or exceeded threshold. fitness=" << fitness;
      message = oss.str();
      return false;
    }

    estimated_pose = poses[loop.candidate_index] * target_from_source.value();
    map_to_odom_ = estimated_pose * latest_odom_pose_.inverse();
    has_relocalized_ = true;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.header.stamp = now();
    pose_msg.pose = poseMsgFromEigen(estimated_pose);
    relocalized_pose_pub_->publish(pose_msg);

    publishLoopMarker(loop.candidate_index, loop.candidate_index);

    std::ostringstream oss;
    oss << "Relocalization succeeded. candidate=" << loop.candidate_index
        << " similarity=" << std::fixed << std::setprecision(3) << loop.similarity
        << " fitness=" << std::setprecision(4) << fitness;
    message = oss.str();
    return true;
  }

  void handleSaveMap(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<SaveMapSrv::Request> request,
    std::shared_ptr<SaveMapSrv::Response> response)
  {
    const std::string directory = request->directory.empty() ? map_save_directory_ : expandUserPath(request->directory);
    const double resolution = request->resolution > 0.0f ? request->resolution : save_keyframe_leaf_m_;
    std::string message;
    response->success = saveMap(directory, resolution, message);
    response->message = message;
  }

  void handleLoadMap(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<LoadMapSrv::Request> request,
    std::shared_ptr<LoadMapSrv::Response> response)
  {
    const std::string directory = request->directory.empty() ? map_load_directory_ : expandUserPath(request->directory);
    std::string message;
    response->success = loadMap(directory, message);
    response->message = message;
  }

  void handleRelocalize(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<RelocalizeSrv::Request> request,
    std::shared_ptr<RelocalizeSrv::Response> response)
  {
    (void)request;
    Eigen::Isometry3d estimated_pose = Eigen::Isometry3d::Identity();
    std::string message;
    response->success = attemptRelocalization(message, estimated_pose);
    response->message = message;
    if (response->success) {
      response->estimated_pose = poseMsgFromEigen(estimated_pose);
    }
  }

  bool saveMap(const std::string & directory, double resolution, std::string & message)
  {
    if (keyframes_.empty()) {
      message = "No keyframes available to save.";
      return false;
    }

    const std::filesystem::path root(directory);
    const std::filesystem::path keyframe_dir = root / "keyframes";
    std::error_code ec;
    std::filesystem::create_directories(keyframe_dir, ec);
    if (ec) {
      message = "Failed to create map directory: " + ec.message();
      return false;
    }

    std::ofstream metadata(root / "metadata.csv", std::ios::trunc);
    if (!metadata.is_open()) {
      message = "Failed to open metadata.csv for writing.";
      return false;
    }
    metadata << "index,timestamp_ns,x,y,z,qx,qy,qz,qw,loaded_from_map,cloud_file\n";

    CloudT::Ptr global_map(new CloudT());
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      const auto & keyframe = keyframes_[i];
      const std::string cloud_file = "keyframes/keyframe_" + zeroPad(i) + ".pcd";
      CloudT::Ptr cloud_to_save = downsampleCloud(keyframe.local_cloud, resolution);
      pcl::io::savePCDFileBinary((root / cloud_file).string(), *cloud_to_save);

      const Eigen::Quaterniond q(keyframe.optimized_pose.rotation());
      metadata << i << ','
               << keyframe.stamp.nanoseconds() << ','
               << keyframe.optimized_pose.translation().x() << ','
               << keyframe.optimized_pose.translation().y() << ','
               << keyframe.optimized_pose.translation().z() << ','
               << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
               << (keyframe.loaded_from_map ? 1 : 0) << ','
               << cloud_file << '\n';

      CloudT::Ptr global_cloud = transformCloud(cloud_to_save, keyframe.optimized_pose);
      *global_map += *global_cloud;
    }

    CloudT::Ptr global_map_ds = downsampleCloud(global_map, resolution);
    pcl::io::savePCDFileBinary((root / "GlobalMap.pcd").string(), *global_map_ds);

    message = "Map saved to " + root.string();
    return true;
  }

  bool loadMap(const std::string & directory, std::string & message)
  {
    const std::filesystem::path root(directory);
    std::ifstream metadata(root / "metadata.csv");
    if (!metadata.is_open()) {
      message = "metadata.csv not found in " + root.string();
      return false;
    }

    clearRuntimeState();

    std::string line;
    std::getline(metadata, line);
    while (std::getline(metadata, line)) {
      if (line.empty()) {
        continue;
      }

      const std::vector<std::string> fields = splitCsvLine(line);
      if (fields.size() < 11) {
        message = "Malformed metadata.csv line: " + line;
        clearRuntimeState();
        return false;
      }

      Keyframe keyframe;
      keyframe.optimized_pose = poseFromComponents(
        std::stod(fields[2]), std::stod(fields[3]), std::stod(fields[4]),
        std::stod(fields[5]), std::stod(fields[6]), std::stod(fields[7]), std::stod(fields[8]));
      keyframe.odom_pose = keyframe.optimized_pose;
      keyframe.loaded_from_map = true;
      keyframe.stamp = rclcpp::Time(std::stoll(fields[1]), RCL_ROS_TIME);
      keyframe.local_cloud = std::make_shared<CloudT>();
      const std::filesystem::path cloud_path = root / fields[10];
      if (pcl::io::loadPCDFile<PointT>(cloud_path.string(), *keyframe.local_cloud) != 0) {
        message = "Failed to load keyframe cloud: " + cloud_path.string();
        clearRuntimeState();
        return false;
      }
      keyframe.scan_context = buildScanContext(*keyframe.local_cloud);
      keyframes_.push_back(keyframe);
    }

    loaded_keyframe_count_ = static_cast<int>(keyframes_.size());
    has_loaded_map_ = loaded_keyframe_count_ > 0;
    has_relocalized_ = !enable_relocalization_mode_;
    map_to_odom_.setIdentity();

    rebuildPoseGraphFromKeyframes();
    updatePublishedPath();
    publishGlobalMap();

    std::ostringstream oss;
    oss << "Loaded " << loaded_keyframe_count_ << " keyframes from " << root.string();
    message = oss.str();
    return true;
  }

  void clearRuntimeState()
  {
    keyframes_.clear();
    loop_edges_.clear();
    optimized_path_.poses.clear();
    has_loaded_map_ = false;
    has_relocalized_ = false;
    loaded_keyframe_count_ = 0;
    callback_count_ = 0;
    has_latest_scan_ = false;
    latest_local_cloud_.reset();
    map_to_odom_.setIdentity();
    resetGtsamState();
  }

  std::string zeroPad(std::size_t index) const
  {
    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << index;
    return oss.str();
  }

#ifdef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
  void resetGtsamState()
  {
    if (!gtsam_enabled_ && enable_gtsam_backend_param_) {
      return;
    }
    if (gtsam_enabled_) {
      gtsam::ISAM2Params params;
      params.relinearizeThreshold = 0.1;
      params.relinearizeSkip = 1;
      isam_ = std::make_unique<gtsam::ISAM2>(params);
      graph_factors_.resize(0);
      initial_estimates_.clear();
      latest_estimate_.clear();
    }
  }
#else
  void resetGtsamState() {}
#endif

  void rebuildPoseGraphFromKeyframes()
  {
    if (!gtsam_enabled_ || keyframes_.empty()) {
      return;
    }

    resetGtsamState();
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      addPoseFactorFromStoredState(static_cast<int>(i));
    }
    optimizePoseGraph();
  }

  void addPoseFactorFromStoredState(int index)
  {
#ifndef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
    (void)index;
#endif
#ifdef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
    if (!gtsam_enabled_) {
      return;
    }

    if (index == 0) {
      gtsam::Vector prior_sigmas(6);
      prior_sigmas << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2;
      graph_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Symbol('x', 0),
        pose3FromEigen(keyframes_[0].optimized_pose),
        gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas)));
      initial_estimates_.insert(gtsam::Symbol('x', 0), pose3FromEigen(keyframes_[0].optimized_pose));
      return;
    }

    const Eigen::Isometry3d previous = keyframes_[index - 1].optimized_pose;
    const Eigen::Isometry3d current = keyframes_[index].optimized_pose;
    const Eigen::Isometry3d delta = previous.inverse() * current;
    gtsam::Vector odom_sigmas(6);
    odom_sigmas << 0.05, 0.05, 0.05, 0.15, 0.15, 0.15;
    graph_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('x', index - 1),
      gtsam::Symbol('x', index),
      pose3FromEigen(delta),
      gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas)));
    initial_estimates_.insert(gtsam::Symbol('x', index), pose3FromEigen(current));
#endif
  }

  void addOdometryFactor(int index)
  {
#ifndef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
    (void)index;
#endif
#ifdef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
    if (!gtsam_enabled_) {
      return;
    }

    if (index == 0) {
      addPoseFactorFromStoredState(0);
      return;
    }

    const Eigen::Isometry3d delta = keyframes_[index - 1].odom_pose.inverse() * keyframes_[index].odom_pose;
    gtsam::Vector odom_sigmas(6);
    odom_sigmas << 0.05, 0.05, 0.05, 0.15, 0.15, 0.15;
    graph_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('x', index - 1),
      gtsam::Symbol('x', index),
      pose3FromEigen(delta),
      gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas)));
    initial_estimates_.insert(gtsam::Symbol('x', index), pose3FromEigen(keyframes_[index].optimized_pose));
#endif
  }

  void addLoopFactor(int from_index, int to_index, const Eigen::Isometry3d & relative_transform)
  {
#ifdef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
    if (!gtsam_enabled_) {
      keyframes_[to_index].optimized_pose = keyframes_[from_index].optimized_pose * relative_transform;
      return;
    }

    gtsam::Vector noise_vec(6);
    noise_vec << 0.2, 0.2, 0.2, 0.5, 0.5, 0.5;
    graph_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('x', from_index),
      gtsam::Symbol('x', to_index),
      pose3FromEigen(relative_transform),
      gtsam::noiseModel::Diagonal::Sigmas(noise_vec)));
#else
    keyframes_[to_index].optimized_pose = keyframes_[from_index].optimized_pose * relative_transform;
#endif
  }

  void optimizePoseGraph()
  {
#ifdef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
    if (!gtsam_enabled_ || graph_factors_.empty() || initial_estimates_.empty()) {
      return;
    }

    isam_->update(graph_factors_, initial_estimates_);
    isam_->update();
    latest_estimate_ = isam_->calculateEstimate();
    graph_factors_.resize(0);
    initial_estimates_.clear();

    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      if (latest_estimate_.exists(gtsam::Symbol('x', i))) {
        keyframes_[i].optimized_pose = eigenFromPose3(
          latest_estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', i)));
      }
    }
#endif
  }

  void updatePublishedPath()
  {
    optimized_path_.header.stamp = now();
    optimized_path_.header.frame_id = map_frame_id_;
    optimized_path_.poses.clear();
    for (const auto & keyframe : keyframes_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = optimized_path_.header;
      pose.pose = poseMsgFromEigen(keyframe.optimized_pose);
      optimized_path_.poses.push_back(pose);
    }
    path_pub_->publish(optimized_path_);
  }

  void publishGlobalMap()
  {
    if (keyframes_.empty()) {
      return;
    }

    CloudT::Ptr map(new CloudT());
    for (const auto & keyframe : keyframes_) {
      CloudT::Ptr global_cloud = transformCloud(keyframe.local_cloud, keyframe.optimized_pose);
      *map += *global_cloud;
    }
    map = downsampleCloud(map, submap_voxel_leaf_m_);

    CloudMsg msg;
    pcl::toROSMsg(*map, msg);
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_id_;
    map_pub_->publish(msg);
  }

  void publishLoopMarker(int from_index, int to_index)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = now();
    marker.ns = "loop_edges";
    marker.id = static_cast<int>(loop_edges_.size());
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.08;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.1;

    geometry_msgs::msg::Point p0;
    const auto & pose0 = keyframes_[from_index].optimized_pose.translation();
    p0.x = pose0.x();
    p0.y = pose0.y();
    p0.z = pose0.z();

    geometry_msgs::msg::Point p1;
    const auto & pose1 = keyframes_[to_index].optimized_pose.translation();
    p1.x = pose1.x();
    p1.y = pose1.y();
    p1.z = pose1.z();

    marker.points.push_back(p0);
    marker.points.push_back(p1);
    loop_edges_.push_back(marker);

    visualization_msgs::msg::MarkerArray array;
    array.markers = loop_edges_;
    loop_marker_pub_->publish(array);
  }

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string global_map_topic_;
  std::string optimized_path_topic_;
  std::string loop_marker_topic_;
  std::string relocalized_pose_topic_;
  std::string map_frame_id_;
  std::string map_save_directory_;
  std::string map_load_directory_;
  bool cloud_is_world_frame_{true};
  double keyframe_translation_thresh_m_{0.5};
  double keyframe_rotation_thresh_deg_{10.0};
  double keyframe_voxel_leaf_m_{0.2};
  double submap_voxel_leaf_m_{0.3};
  double save_keyframe_leaf_m_{0.15};
  int scan_context_rings_{20};
  int scan_context_sectors_{60};
  double scan_context_max_radius_m_{50.0};
  double scan_context_similarity_threshold_{0.82};
  int loop_recent_exclusion_count_{30};
  int loop_submap_half_width_{5};
  int loop_detection_stride_{5};
  int auto_relocalize_stride_{10};
  double icp_max_correspondence_distance_m_{2.0};
  int icp_max_iterations_{40};
  double icp_fitness_threshold_{0.35};
  double relocalization_fitness_threshold_{0.45};
  int publish_map_every_n_keyframes_{3};
  bool enable_loop_closure_{true};
  bool enable_relocalization_mode_{false};
  bool enable_gtsam_backend_param_{true};
  bool gtsam_enabled_{false};
  bool has_loaded_map_{false};
  bool has_relocalized_{false};
  bool has_latest_scan_{false};
  int loaded_keyframe_count_{0};
  int callback_count_{0};

  Eigen::Isometry3d map_to_odom_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d latest_odom_pose_{Eigen::Isometry3d::Identity()};
  CloudT::Ptr latest_local_cloud_;
  rclcpp::Time latest_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<CloudMsg>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr relocalized_pose_pub_;
  rclcpp::Service<SaveMapSrv>::SharedPtr save_map_srv_;
  rclcpp::Service<LoadMapSrv>::SharedPtr load_map_srv_;
  rclcpp::Service<RelocalizeSrv>::SharedPtr relocalize_srv_;
  message_filters::Subscriber<OdomMsg> odom_sub_;
  message_filters::Subscriber<CloudMsg> cloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  std::vector<Keyframe> keyframes_;
  nav_msgs::msg::Path optimized_path_;
  std::vector<visualization_msgs::msg::Marker> loop_edges_;

#ifdef FASTLIO_GLOBAL_SLAM_HAS_GTSAM
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::NonlinearFactorGraph graph_factors_;
  gtsam::Values initial_estimates_;
  gtsam::Values latest_estimate_;
#endif
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioGlobalBackend>());
  rclcpp::shutdown();
  return 0;
}
