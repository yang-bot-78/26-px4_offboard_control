#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <race_msgs/msg/flight_altitude_reference.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "race_ego_bridge/frame_utils.hpp"
#include "race_ego_bridge/safety_utils.hpp"

class EgoGoalBridge : public rclcpp::Node
{
public:
  EgoGoalBridge()
  : Node("race_ego_goal_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/goal_pose");
    ego_topic_ = declare_parameter<std::string>("ego_topic", "/race/ego/goal_input");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/race/ego/goal");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/race/ego/odom");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    release_height_ = declare_parameter<double>("release_height", 0.50);
    // Kept for launch-file compatibility; the handover gate now uses the
    // configured min_height/max_height safety window instead of flight_height.
    stable_height_tolerance_m_ = declare_parameter<double>("stable_height_tolerance_m", 0.05);
    stable_horizontal_speed_mps_ = declare_parameter<double>("stable_horizontal_speed_mps", 0.08);
    stable_vertical_speed_mps_ = declare_parameter<double>("stable_vertical_speed_mps", 0.05);
    stable_duration_sec_ = declare_parameter<double>("stable_duration_sec", 0.50);
    retry_goal_on_ego_failure_ = declare_parameter<bool>(
      "retry_goal_on_ego_failure", false);
    ego_failure_retry_period_sec_ = declare_parameter<double>(
      "ego_failure_retry_period_sec", 0.75);
    duplicate_goal_position_tolerance_m_ = declare_parameter<double>(
      "duplicate_goal_position_tolerance_m", 0.03);
    duplicate_goal_time_window_sec_ = declare_parameter<double>(
      "duplicate_goal_time_window_sec", 1.0);
    flight_mode_ = declare_parameter<std::string>("flight_mode", "flat");
    flight_height_ = declare_parameter<double>("flight_height", 0.78);
    min_height_ = declare_parameter<double>("min_height", 0.50);
    max_height_ = declare_parameter<double>("max_height", 0.90);
    min_x_ = declare_parameter<double>("min_x", -7.5);
    max_x_ = declare_parameter<double>("max_x", 7.5);
    min_y_ = declare_parameter<double>("min_y", -7.0);
    max_y_ = declare_parameter<double>("max_y", 7.0);
    if (flight_mode_ != "flat" && flight_mode_ != "3d") {
      throw std::runtime_error("flight_mode must be flat or 3d");
    }
    if (stable_height_tolerance_m_ < 0.0 || stable_horizontal_speed_mps_ < 0.0 ||
      stable_vertical_speed_mps_ < 0.0 || stable_duration_sec_ < 0.0 ||
      min_height_ > max_height_)
    {
      throw std::runtime_error(
              "EGO takeoff stability parameters are invalid: height range or limits");
    }

    ego_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(ego_topic_, 10);
    debug_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(debug_topic_, 10);
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10, std::bind(&EgoGoalBridge::callback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&EgoGoalBridge::odomCallback, this, std::placeholders::_1));
    state_subscription_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::SensorDataQoS(),
      std::bind(&EgoGoalBridge::stateCallback, this, std::placeholders::_1));
    altitude_reference_subscription_ =
      create_subscription<race_msgs::msg::FlightAltitudeReference>(
      "/race/flight_altitude_reference", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&EgoGoalBridge::altitudeReferenceCallback, this, std::placeholders::_1));
    status_subscription_ = create_subscription<std_msgs::msg::String>(
      "/race/ego/status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&EgoGoalBridge::statusCallback, this, std::placeholders::_1));
  }

private:
  double currentAgl() const
  {
    return altitude_reference_valid_ ? current_height_ - ground_z_map_ : current_height_;
  }

  double effectiveMinHeight() const
  {
    return (altitude_reference_valid_ ? ground_z_map_ : 0.0) + min_height_;
  }

  double effectiveMaxHeight() const
  {
    return (altitude_reference_valid_ ? ground_z_map_ : 0.0) + max_height_;
  }

  void altitudeReferenceCallback(
    const race_msgs::msg::FlightAltitudeReference::SharedPtr msg)
  {
    if (!msg->valid) {
      if (altitude_reference_valid_ && msg->flight_id == altitude_reference_flight_id_) {
        altitude_reference_valid_ = false;
        stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        have_pending_goal_ = false;
        have_active_goal_ = false;
        flight_handover_released_ = false;
      }
      return;
    }
    if (!std::isfinite(msg->ground_z_map) || !std::isfinite(msg->target_z_map) ||
      !std::isfinite(msg->target_agl_m) || !std::isfinite(msg->min_agl_m) ||
      !std::isfinite(msg->max_agl_m) ||
      std::abs(msg->target_agl_m - flight_height_) > 1.0e-3 ||
      std::abs(msg->min_agl_m - min_height_) > 1.0e-3 ||
      std::abs(msg->max_agl_m - max_height_) > 1.0e-3 ||
      std::abs((msg->target_z_map - msg->ground_z_map) - msg->target_agl_m) > 1.0e-3)
    {
      RCLCPP_ERROR(
        get_logger(), "[GOAL_BRIDGE_ALTITUDE_REFERENCE_REJECT] flight_id=%lu",
        static_cast<unsigned long>(msg->flight_id));
      return;
    }
    if (altitude_reference_valid_ && msg->flight_id < altitude_reference_flight_id_) {
      return;
    }
    altitude_reference_flight_id_ = msg->flight_id;
    ground_z_map_ = msg->ground_z_map;
    target_z_map_ = msg->target_z_map;
    altitude_reference_valid_ = true;
    stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    RCLCPP_WARN(
      get_logger(),
      "[GOAL_BRIDGE_ALTITUDE_REFERENCE_ACCEPTED] flight_id=%lu ground_map=%.3f "
      "target_map=%.3f",
      static_cast<unsigned long>(msg->flight_id), ground_z_map_, target_z_map_);
  }

  void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
      RCLCPP_ERROR(
        get_logger(), "FRAME_MISMATCH goal=%s expected=%s; no implicit TF fallback",
        msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    auto output = *msg;
    output.header.stamp = now();
    output.header.frame_id = frame_id_;
    if (flight_mode_ == "flat") {
      if (!altitude_reference_valid_) {
        RCLCPP_ERROR(
          get_logger(),
          "[GOAL_BRIDGE_REJECT] no valid /race/flight_altitude_reference for this flight");
        return;
      }
      output.pose.position.z = target_z_map_;
    }
    const race_ego_bridge::Vec3 goal{
      output.pose.position.x, output.pose.position.y, output.pose.position.z};
    if (!race_ego_bridge::insideGeofence(
        goal, min_x_, max_x_, min_y_, max_y_, effectiveMinHeight(), effectiveMaxHeight()))
    {
      RCLCPP_ERROR(
        get_logger(), "OUT_OF_GEOFENCE goal=(%.2f,%.2f,%.2f)", goal.x, goal.y, goal.z);
      return;
    }
    if (have_last_input_goal_ && output.header.frame_id == last_input_goal_.header.frame_id &&
      std::hypot(
        output.pose.position.x - last_input_goal_.pose.position.x,
        output.pose.position.y - last_input_goal_.pose.position.y) <=
      duplicate_goal_position_tolerance_m_ &&
      std::abs(output.pose.position.z - last_input_goal_.pose.position.z) <=
      duplicate_goal_position_tolerance_m_ &&
      (now() - last_input_goal_time_).seconds() <= duplicate_goal_time_window_sec_)
    {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignore duplicate EGO goal within %.2fm/%.2fs",
        duplicate_goal_position_tolerance_m_, duplicate_goal_time_window_sec_);
      return;
    }
    last_input_goal_ = output;
    last_input_goal_time_ = now();
    have_last_input_goal_ = true;
    if (flight_handover_released_) {
      publishGoal(output, "rolling local goal after takeoff handover");
      return;
    }
    if (isVehicleStable()) {
      flight_handover_released_ = true;
      publishGoal(output, "vehicle stable within safety height range");
      return;
    }
    pending_goal_ = output;
    ++pending_goal_seq_;
    have_pending_goal_ = true;
    RCLCPP_INFO(
      get_logger(), "[GOAL_BRIDGE_TAKEOFF_GATE] goal_seq=%lu current_agl=%.3f "
      "safety_height=[%.3f,%.3f] stable=%s cached=true",
      static_cast<unsigned long>(pending_goal_seq_), currentAgl(), min_height_, max_height_,
      isVehicleStable() ? "true" : "false");
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
      return;
    }
    have_odom_ = true;
    current_height_ = msg->pose.pose.position.z;
    current_horizontal_speed_ = std::hypot(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    current_vertical_speed_ = std::abs(msg->twist.twist.linear.z);
    // The handover gate accepts any stable altitude inside the configured
    // safety window.  flight_height_ remains the commanded altitude for flat
    // goals, but it must not be used to reject a manually established hover.
    const double current_agl = currentAgl();
    const bool stable_sample = altitude_reference_valid_ &&
      current_agl >= min_height_ && current_agl <= max_height_ &&
      current_horizontal_speed_ <= stable_horizontal_speed_mps_ &&
      current_vertical_speed_ <= stable_vertical_speed_mps_;
    if (stable_sample) {
      if (stable_since_.nanoseconds() == 0) {
        stable_since_ = now();
      }
    } else {
      stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    if (have_pending_goal_ && isVehicleStable()) {
      RCLCPP_INFO(
        get_logger(), "[GOAL_BRIDGE_RELEASE] goal_seq=%lu current_agl=%.3f target=(%.3f,%.3f,%.3f) "
        "reason=SAFETY_HEIGHT_RANGE_STABLE", static_cast<unsigned long>(pending_goal_seq_), current_agl,
        pending_goal_.pose.position.x, pending_goal_.pose.position.y,
        pending_goal_.pose.position.z);
      flight_handover_released_ = true;
      publishGoal(pending_goal_, "vehicle stable within safety height range");
      have_pending_goal_ = false;
    }
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    if (race_ego_bridge::shouldResetFlightHandover(armed_, msg->armed)) {
      flight_handover_released_ = false;
      stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      have_pending_goal_ = false;
      RCLCPP_WARN(
        get_logger(),
        "[GOAL_BRIDGE_HANDOVER_RESET] vehicle disarmed; next flight must pass the takeoff gate");
    }
    armed_ = msg->armed;
  }

  bool isVehicleStable() const
  {
    return have_odom_ && stable_since_.nanoseconds() != 0 &&
           (now() - stable_since_).seconds() >= stable_duration_sec_;
  }

  void publishGoal(geometry_msgs::msg::PoseStamped output, const char * reason)
  {
    output.header.stamp = now();
    active_goal_ = output;
    have_active_goal_ = true;
    ego_publisher_->publish(output);
    debug_publisher_->publish(output);
    RCLCPP_INFO(
      get_logger(), "Release EGO goal=(%.2f,%.2f,%.2f): %s",
      output.pose.position.x, output.pose.position.y, output.pose.position.z, reason);
  }

  void statusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!retry_goal_on_ego_failure_ || !have_active_goal_ ||
      (!flight_handover_released_ && !isVehicleStable()))
    {
      return;
    }
    const bool needs_recovery = msg->data == "NO_SAFE_TRAJECTORY" ||
      msg->data == "SETPOINT_INVALID" || msg->data == "OUT_OF_GEOFENCE";
    if (!needs_recovery) {
      return;
    }
    if (last_recovery_time_.nanoseconds() != 0 &&
      (now() - last_recovery_time_).seconds() < ego_failure_retry_period_sec_)
    {
      return;
    }
    last_recovery_time_ = now();
    publishGoal(active_goal_, "persistent EGO failure; reinitialize from current odometry");
  }

  std::string input_topic_;
  std::string ego_topic_;
  std::string debug_topic_;
  std::string odom_topic_;
  std::string frame_id_;
  std::string flight_mode_;
  double flight_height_{0.78};
  double min_height_{0.50};
  double max_height_{0.90};
  double min_x_{-7.5};
  double max_x_{7.5};
  double min_y_{-7.0};
  double max_y_{7.0};
  double release_height_{0.50};
  double stable_height_tolerance_m_{0.05};
  double stable_horizontal_speed_mps_{0.08};
  double stable_vertical_speed_mps_{0.05};
  double stable_duration_sec_{0.50};
  double ego_failure_retry_period_sec_{0.75};
  double duplicate_goal_position_tolerance_m_{0.03};
  double duplicate_goal_time_window_sec_{1.0};
  uint64_t altitude_reference_flight_id_{0};
  double ground_z_map_{0.0};
  double target_z_map_{0.0};
  double current_height_{0.0};
  double current_horizontal_speed_{0.0};
  double current_vertical_speed_{0.0};
  bool have_odom_{false};
  bool have_pending_goal_{false};
  uint64_t pending_goal_seq_{0};
  bool have_active_goal_{false};
  bool flight_handover_released_{false};
  bool armed_{false};
  bool retry_goal_on_ego_failure_{false};
  bool have_last_input_goal_{false};
  bool altitude_reference_valid_{false};
  geometry_msgs::msg::PoseStamped pending_goal_;
  geometry_msgs::msg::PoseStamped active_goal_;
  geometry_msgs::msg::PoseStamped last_input_goal_;
  rclcpp::Time last_recovery_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_input_goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ego_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Subscription<race_msgs::msg::FlightAltitudeReference>::SharedPtr
    altitude_reference_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoGoalBridge>());
  rclcpp::shutdown();
  return 0;
}
