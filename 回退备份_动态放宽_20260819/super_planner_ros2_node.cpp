#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <race_msgs/msg/global_planner_status.hpp>
#include <race_msgs/msg/flight_altitude_reference.hpp>
#include <race_msgs/msg/local_path_reference.hpp>
#include <race_msgs/msg/navigation_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <traj_utils/msg/bspline.hpp>

#include "race_super_planner_ros2/path_fallback_policy.hpp"
#include "race_super_planner_ros2/takeoff_path_policy.hpp"
#include "race_super_planner_ros2/global_grid_policy.hpp"
#include "race_super_planner_ros2/narrow_corridor_policy.hpp"
#include "race_super_planner_ros2/obstacle_aware_smoothing_policy.hpp"
#include "race_super_planner_ros2/continuous_segment_policy.hpp"
#include "race_super_planner_ros2/local_goal_lifecycle_policy.hpp"
#include "race_super_planner_ros2/pending_path_gate_policy.hpp"

using namespace std::chrono_literals;

namespace
{
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class SegmentSafetyFailure
{
  NONE,
  NON_FINITE,
  CONTINUOUS_CLEARANCE,
  GRID_CLEARANCE
};

struct PointSafetyResult
{
  bool safe{false};
  SegmentSafetyFailure failure{SegmentSafetyFailure::NONE};
  Vec3 query_point;
  Vec3 nearest_obstacle;
  double clearance{std::numeric_limits<double>::infinity()};
};

struct SegmentSafetyResult
{
  bool safe{false};
  SegmentSafetyFailure failure{SegmentSafetyFailure::NONE};
  std::size_t sample_index{0};
  std::size_t sample_count{0};
  double sample_step{0.0};
  Vec3 failure_point;
  Vec3 nearest_obstacle;
  double clearance{std::numeric_limits<double>::infinity()};
  double minimum_clearance{std::numeric_limits<double>::infinity()};
};

struct PathProjection
{
  Vec3 point;
  std::size_t segment_index{0};
  double segment_ratio{0.0};
  double cross_track_error{0.0};
};

struct PathSample
{
  Vec3 point;
  std::size_t path_index{0};
  double lookahead{0.0};
};

struct Cell
{
  int x{0};
  int y{0};

  bool operator==(const Cell & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct CellHash
{
  std::size_t operator()(const Cell & cell) const
  {
    const auto hx = std::hash<int>{}(cell.x);
    const auto hy = std::hash<int>{}(cell.y);
    return hx ^ (hy + 0x9e3779b9 + (hx << 6) + (hx >> 2));
  }
};

double distance2d(const Vec3 & a, const Vec3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool finite_vec(const Vec3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

class SuperPlannerRos2Node : public rclcpp::Node
{
public:
  SuperPlannerRos2Node()
  : Node("super_planner_ros2_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    // Global planning and EGO consume the same already-aligned project map.
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/race/ego/odom");
    fallback_odom_topic_ = declare_parameter<std::string>(
      "fallback_odom_topic",
      "/race/ego/odom");
    odom_input_frame_ = declare_parameter<std::string>("odom_input_frame", "map");
    // MAVROS reports link health on /mavros/state instead of per-sample estimator flags.
    mavros_state_topic_ =
      declare_parameter<std::string>("mavros_state_topic", "/mavros/state");
    require_mavros_connected_ = declare_parameter<bool>("require_mavros_connected", false);
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    fallback_cloud_topic_ = declare_parameter<std::string>(
      "fallback_cloud_topic",
      "/fast_lio/cloud_registered");
    navigation_setpoint_topic_ =
      declare_parameter<std::string>("navigation_setpoint_topic", "/race/navigation_setpoint");
    path_topic_ = declare_parameter<std::string>("path_topic", "/rm_mpc_trajectory_vis_path");
    global_path_topic_ =
      declare_parameter<std::string>("global_path_topic", "/race/global_path");
    ego_local_goal_topic_ =
      declare_parameter<std::string>("ego_local_goal_topic", "/race/ego/local_goal");
    ego_reference_path_topic_ = declare_parameter<std::string>(
      "ego_reference_path_topic", "/race/ego/local_path_reference");
    raw_path_topic_ = declare_parameter<std::string>(
      "raw_path_topic",
      "/race/super_planner/raw_path");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic",
      "/race/super_planner/debug_markers");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    setpoint_output_frame_ = declare_parameter<std::string>("setpoint_output_frame", "px4_ned");

    control_rate_ = declare_parameter<double>("control_rate", 20.0);
    replan_rate_ = declare_parameter<double>("replan_rate", 2.0);
    replan_while_tracking_ = declare_parameter<bool>("replan_while_tracking", true);
    tracking_lookahead_distance_ = declare_parameter<double>("tracking_lookahead_distance", 0.50);
    vehicle_collision_radius_ = declare_parameter<double>("vehicle_collision_radius", 0.384);
    narrow_corridor_tracking_margin_ =
      declare_parameter<double>("narrow_corridor_tracking_margin", 0.05);
    required_center_clearance_ =
      declare_parameter<double>("required_center_clearance", 0.466);
    narrow_corridor_lookahead_distance_ =
      declare_parameter<double>("narrow_corridor_lookahead_distance", 0.15);
    narrow_corridor_centering_tolerance_ =
      declare_parameter<double>("narrow_corridor_centering_tolerance", 0.05);
    heading_lookahead_distance_ = declare_parameter<double>("heading_lookahead_distance", 0.80);
    max_path_tracking_error_ = declare_parameter<double>("mission_corridor_max_error_m", 1.50);
    yaw_smoothing_time_constant_ = declare_parameter<double>("yaw_smoothing_time_constant", 0.35);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 1.0);
    publish_yaw_rate_feedforward_ = declare_parameter<bool>("publish_yaw_rate_feedforward", true);
    min_safe_height_ = declare_parameter<double>("min_safe_height", 0.75);
    max_safe_height_ = declare_parameter<double>("max_safe_height", 0.90);
    fixed_flight_height_ = declare_parameter<double>("fixed_flight_height", 0.78);
    use_fixed_flight_height_ = declare_parameter<bool>("use_fixed_flight_height", true);
    final_goal_position_tolerance_m_ =
      declare_parameter<double>("final_goal_position_tolerance_m", 0.10);
    final_goal_velocity_tolerance_mps_ =
      declare_parameter<double>("final_goal_velocity_tolerance_mps", 0.08);
    final_goal_confirmation_cycles_ =
      declare_parameter<int>("final_goal_confirmation_cycles", 10);
    enable_output_ = declare_parameter<bool>("enable_output", false);
    global_only_mode_ = declare_parameter<bool>("global_only_mode", false);
    publish_global_path_ = declare_parameter<bool>("publish_global_path", false);
    publish_ego_local_goal_ = declare_parameter<bool>("publish_ego_local_goal", false);
    ego_local_goal_lookahead_m_ =
      declare_parameter<double>("ego_local_goal_lookahead_m", 1.20);
    ego_local_goal_min_distance_m_ =
      declare_parameter<double>("ego_local_goal_min_distance_m", 0.60);
    ego_local_goal_max_distance_m_ =
      declare_parameter<double>("ego_local_goal_max_distance_m", 1.60);
    ego_reference_preview_distance_m_ =
      declare_parameter<double>("ego_reference_preview_distance_m", 0.60);
    ego_local_goal_reached_radius_m_ =
      declare_parameter<double>("ego_local_goal_reached_radius_m", 0.30);
    local_goal_update_min_interval_sec_ =
      declare_parameter<double>("local_goal_update_min_interval_sec", 0.10);
    local_goal_update_distance_m_ =
      declare_parameter<double>("local_goal_update_distance_m", 0.20);
    goal_update_position_tolerance_m_ =
      declare_parameter<double>("goal_update_position_tolerance_m", 0.05);
    pending_path_max_cross_track_error_m_ = declare_parameter<double>(
      "pending_path_max_cross_track_error_m", 0.15);
    pending_path_max_replans_ = declare_parameter<int>("pending_path_max_replans", 1);
    ego_status_topic_ = declare_parameter<std::string>("ego_status_topic", "/race/ego/status");
    validated_bspline_topic_ = declare_parameter<std::string>(
      "validated_bspline_topic", "/race/ego/validated_bspline");
    command_local_goal_seq_topic_ = declare_parameter<std::string>(
      "command_local_goal_seq_topic", "/race/ego/command_local_goal_seq");
    local_planner_failure_replan_sec_ =
      declare_parameter<double>("local_planner_failure_replan_sec", 1.0);
    trajectory_prefetch_sec_ = declare_parameter<double>("trajectory_prefetch_sec", 1.5);
    trajectory_stall_timeout_sec_ = declare_parameter<double>(
      "trajectory_stall_timeout_sec", 1.0);
    trajectory_recovery_confirmation_sec_ = declare_parameter<double>(
      "trajectory_recovery_confirmation_sec", 0.5);
    resolution_ = declare_parameter<double>("grid_resolution", 0.20);
    planning_resolution_ = declare_parameter<double>("planning_resolution", resolution_);
    local_range_xy_ = declare_parameter<double>("local_range_xy", 10.0);
    obstacle_min_height_ = declare_parameter<double>("obstacle_min_height", 0.30);
    obstacle_max_height_ = declare_parameter<double>("obstacle_max_height", 2.00);
    inflation_radius_ = declare_parameter<double>("obstacle_inflation_radius", 0.36);
    soft_obstacle_cost_radius_ = declare_parameter<double>("soft_obstacle_cost_radius", 0.52);
    clearance_cost_weight_ = declare_parameter<double>("clearance_cost_weight", 3.5);
    max_cloud_points_ = declare_parameter<int>("max_cloud_points", 200000);
    cloud_timeout_sec_ = declare_parameter<double>("cloud_timeout_sec", 600.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 1.0);
    goal_timeout_sec_ = declare_parameter<double>("goal_timeout_sec", 600.0);
    enable_path_smoothing_ = declare_parameter<bool>("enable_path_smoothing", true);
    smoothing_method_ = declare_parameter<std::string>("smoothing_method", "chaikin");
    smoothing_iterations_ = declare_parameter<int>("smoothing_iterations", 2);
    path_sample_resolution_ = declare_parameter<double>("path_sample_resolution", 0.10);
    smoothing_collision_check_ = declare_parameter<bool>("smoothing_collision_check", true);
    smoothing_max_deviation_ = declare_parameter<double>("smoothing_max_deviation", 0.8);
    smoothing_min_clearance_ = declare_parameter<double>("smoothing_min_clearance", 0.30);
    min_planning_inflation_radius_ =
      declare_parameter<double>("min_planning_inflation_radius", 0.36);
    allow_direct_path_ = declare_parameter<bool>("allow_direct_path", false);
    enable_path_shortcut_ = declare_parameter<bool>("enable_path_shortcut", false);
    publish_debug_markers_ = declare_parameter<bool>("publish_debug_markers", true);
    debug_marker_height_ = declare_parameter<double>("debug_marker_height", 0.20);
    debug_max_cells_ = declare_parameter<int>("debug_max_cells", 25000);
    debug_marker_period_sec_ = declare_parameter<double>("debug_marker_period_sec", 0.50);
    shared_bounds_x_min_ = declare_parameter<double>("shared_bounds/x_min", -7.5);
    shared_bounds_x_max_ = declare_parameter<double>("shared_bounds/x_max", 7.5);
    shared_bounds_y_min_ = declare_parameter<double>("shared_bounds/y_min", -7.0);
    shared_bounds_y_max_ = declare_parameter<double>("shared_bounds/y_max", 7.0);
    shared_bounds_z_min_ = declare_parameter<double>("shared_bounds/z_min", 0.50);
    shared_bounds_z_max_ = declare_parameter<double>("shared_bounds/z_max", 0.90);
    fault_position_max_speed_mps_ =
      declare_parameter<double>("fault_envelope/max_speed_mps", 4.0);
    fault_position_jump_allowance_m_ =
      declare_parameter<double>("fault_envelope/jump_allowance_m", 0.40);
    fault_planning_grid_padding_m_ =
      declare_parameter<double>("fault_envelope/planning_grid_padding_m", 3.0);
    if (shared_bounds_x_min_ >= shared_bounds_x_max_ ||
      shared_bounds_y_min_ >= shared_bounds_y_max_ ||
      shared_bounds_z_min_ >= shared_bounds_z_max_ ||
      fixed_flight_height_ < shared_bounds_z_min_ || fixed_flight_height_ > shared_bounds_z_max_)
    {
      throw std::runtime_error(
              "FAULT_ENVELOPE_MISMATCH: invalid fault envelope or fixed_flight_height");
    }

    tracking_lookahead_distance_ = std::max(0.10, tracking_lookahead_distance_);
    resolution_ = std::max(0.01, resolution_);
    planning_resolution_ = std::clamp(planning_resolution_, 0.01, resolution_);
    vehicle_collision_radius_ = std::max(0.10, vehicle_collision_radius_);
    narrow_corridor_tracking_margin_ = std::max(0.0, narrow_corridor_tracking_margin_);
    required_center_clearance_ = std::max(0.01, required_center_clearance_);
    narrow_corridor_lookahead_distance_ = std::clamp(
      narrow_corridor_lookahead_distance_, 0.05, tracking_lookahead_distance_);
    narrow_corridor_centering_tolerance_ = std::clamp(
      narrow_corridor_centering_tolerance_, 0.02, max_path_tracking_error_);
    heading_lookahead_distance_ =
      std::max(tracking_lookahead_distance_, heading_lookahead_distance_);
    max_path_tracking_error_ = std::max(0.20, max_path_tracking_error_);
    fault_position_max_speed_mps_ = std::max(0.10, fault_position_max_speed_mps_);
    fault_position_jump_allowance_m_ = std::max(0.05, fault_position_jump_allowance_m_);
    fault_planning_grid_padding_m_ = std::max(0.50, fault_planning_grid_padding_m_);
    control_rate_ = std::max(1.0, control_rate_);
    replan_rate_ = std::clamp(replan_rate_, 0.2, control_rate_);
    yaw_smoothing_time_constant_ = std::max(0.0, yaw_smoothing_time_constant_);
    max_yaw_rate_ = std::max(0.1, max_yaw_rate_);
    ego_local_goal_min_distance_m_ = std::max(0.10, ego_local_goal_min_distance_m_);
    ego_local_goal_max_distance_m_ = std::max(
      ego_local_goal_min_distance_m_, ego_local_goal_max_distance_m_);
    ego_local_goal_lookahead_m_ = std::clamp(
      ego_local_goal_lookahead_m_, ego_local_goal_min_distance_m_,
      ego_local_goal_max_distance_m_);
    ego_reference_preview_distance_m_ = std::clamp(
      ego_reference_preview_distance_m_, 0.20, 2.00);
    ego_local_goal_reached_radius_m_ = std::max(0.05, ego_local_goal_reached_radius_m_);
    final_goal_position_tolerance_m_ = std::max(0.02, final_goal_position_tolerance_m_);
    final_goal_velocity_tolerance_mps_ = std::max(0.01, final_goal_velocity_tolerance_mps_);
    final_goal_confirmation_cycles_ = std::max(1, final_goal_confirmation_cycles_);
    local_goal_update_min_interval_sec_ = std::max(0.02, local_goal_update_min_interval_sec_);
    local_goal_update_distance_m_ = std::clamp(local_goal_update_distance_m_, 0.15, 0.25);
    goal_update_position_tolerance_m_ = std::max(0.01, goal_update_position_tolerance_m_);
    pending_path_max_cross_track_error_m_ = std::clamp(
      pending_path_max_cross_track_error_m_, 0.05, 0.50);
    pending_path_max_replans_ = std::clamp(pending_path_max_replans_, 0, 1);
    local_planner_failure_replan_sec_ = std::max(0.25, local_planner_failure_replan_sec_);
    trajectory_prefetch_sec_ = std::max(0.2, trajectory_prefetch_sec_);
    trajectory_stall_timeout_sec_ = std::max(0.5, trajectory_stall_timeout_sec_);
    trajectory_recovery_confirmation_sec_ = std::clamp(
      trajectory_recovery_confirmation_sec_, 0.2, trajectory_stall_timeout_sec_);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, std::bind(&SuperPlannerRos2Node::goalCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {odomCallback(msg, odom_topic_);});
    if (fallback_odom_topic_ != odom_topic_) {
      fallback_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        fallback_odom_topic_, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {odomCallback(msg, fallback_odom_topic_);});
    }
    mavros_state_sub_ = create_subscription<mavros_msgs::msg::State>(
      mavros_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SuperPlannerRos2Node::mavrosStateCallback, this, std::placeholders::_1));
    altitude_reference_sub_ = create_subscription<race_msgs::msg::FlightAltitudeReference>(
      "/race/flight_altitude_reference", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &SuperPlannerRos2Node::altitudeReferenceCallback, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {cloudCallback(msg, cloud_topic_);});
    if (fallback_cloud_topic_ != cloud_topic_) {
      fallback_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        fallback_cloud_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          cloudCallback(msg, fallback_cloud_topic_);
        });
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(raw_path_topic_, 10);
    if (!global_only_mode_) {
      navigation_setpoint_pub_ =
        create_publisher<race_msgs::msg::NavigationSetpoint>(navigation_setpoint_topic_, 10);
    }
    if (publish_global_path_) {
      global_path_pub_ = create_publisher<nav_msgs::msg::Path>(global_path_topic_, 10);
    }
    if (publish_ego_local_goal_) {
      ego_local_goal_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(ego_local_goal_topic_, 10);
      ego_reference_path_pub_ = create_publisher<race_msgs::msg::LocalPathReference>(
        ego_reference_path_topic_, rclcpp::QoS(10).reliable().transient_local());
      ego_status_sub_ = create_subscription<std_msgs::msg::String>(
        ego_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&SuperPlannerRos2Node::egoStatusCallback, this, std::placeholders::_1));
      validated_bspline_sub_ = create_subscription<traj_utils::msg::Bspline>(
        validated_bspline_topic_, rclcpp::QoS(10).reliable(),
        std::bind(
          &SuperPlannerRos2Node::validatedBsplineCallback, this, std::placeholders::_1));
      command_local_goal_seq_sub_ = create_subscription<std_msgs::msg::UInt64>(
        command_local_goal_seq_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::UInt64::SharedPtr msg) {
          last_validated_local_goal_seq_ = msg->data;
        });
    }
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/race/planner/status", rclcpp::QoS(1).reliable().transient_local());
    global_status_pub_ = create_publisher<race_msgs::msg::GlobalPlannerStatus>(
      "/race/global_planner/status", rclcpp::QoS(1).reliable().transient_local());
    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SuperPlannerRos2Node::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "MODE_INIT enable_output=%s setpoint_topic=%s setpoint_frame=%s control_rate=%.1f replan_rate=%.1f tracking_lookahead=%.2f heading_lookahead=%.2f max_yaw_rate=%.2f odom_topic=%s odom_input_frame=%s require_mavros_connected=%s mavros_state_topic=%s cloud_topic=%s PLANNER_INTERNAL_FRAME=%s hard_inflation=%.2f soft_cost_radius=%.2f clearance_cost_weight=%.2f smoothing=%s method=%s iterations=%d sample_resolution=%.2f allow_direct_path=%s enable_path_shortcut=%s",
      enable_output_ ? "true" : "false", navigation_setpoint_topic_.c_str(),
      setpoint_output_frame_.c_str(), control_rate_, replan_rate_,
      tracking_lookahead_distance_, heading_lookahead_distance_,
      max_yaw_rate_,
      odom_topic_.c_str(), odom_input_frame_.c_str(),
      require_mavros_connected_ ? "true" : "false",
      mavros_state_topic_.c_str(), cloud_topic_.c_str(), world_frame_.c_str(),
      inflation_radius_, soft_obstacle_cost_radius_, clearance_cost_weight_,
      enable_path_smoothing_ ? "true" : "false", smoothing_method_.c_str(), smoothing_iterations_,
      path_sample_resolution_, allow_direct_path_ ? "true" : "false",
      enable_path_shortcut_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "[SUPER_TRACKING_POLICY] replan_while_tracking=%s",
      replan_while_tracking_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "[SUPER_PENDING_PATH_GATE] first_commit_only=true max_path_error=%.3fm "
      "max_stale_start_replans=%d require_post_plan_odom=true",
      pending_path_max_cross_track_error_m_, pending_path_max_replans_);
    RCLCPP_INFO(
      get_logger(),
      "GLOBAL_MODE global_only=%s control_output=%s global_path=%s local_goal=%s",
      global_only_mode_ ? "true" : "false",
      controlOutputEnabled() ? "true" : "false",
      publish_global_path_ ? global_path_topic_.c_str() : "disabled",
      publish_ego_local_goal_ ? ego_local_goal_topic_.c_str() : "disabled");
    RCLCPP_INFO(
      get_logger(), "[SUPER_MODE] global_only=%s max_planning_distance=%.3f segment_chaining=false",
      global_only_mode_ ? "true" : "false", std::max(0.5, local_range_xy_ * 0.90));
    // Keep the operationally safe default explicit in every startup log.
    // DIRECT is an opt-in branch and is not part of the A* baseline.
    RCLCPP_INFO(
      get_logger(), "[SUPER_DIRECT_MODE] enabled=%s",
      allow_direct_path_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "[FAULT_ENVELOPE] x=[%.2f,%.2f] y=[%.2f,%.2f] z=[%.2f,%.2f] "
      "jump=max(%.2fm/s*dt)+%.2fm grid_padding=%.2fm corridor=%.2fm",
      shared_bounds_x_min_, shared_bounds_x_max_, shared_bounds_y_min_, shared_bounds_y_max_,
      shared_bounds_z_min_, shared_bounds_z_max_, fault_position_max_speed_mps_,
      fault_position_jump_allowance_m_, fault_planning_grid_padding_m_, max_path_tracking_error_);
  }

private:
  enum class Mode
  {
    IDLE,
    PLANNING,
    FOLLOWING,
    HOLD,
    BLOCKED_UNSAFE,
    NO_ODOM,
    NO_MAP
  };

  double heightAboveGround(double map_z) const
  {
    return altitude_reference_valid_ ? map_z - ground_z_map_ : map_z;
  }

  double effectiveMinSafeHeight() const
  {
    return (altitude_reference_valid_ ? ground_z_map_ : 0.0) + min_safe_height_;
  }

  double effectiveMaxSafeHeight() const
  {
    return (altitude_reference_valid_ ? ground_z_map_ : 0.0) + max_safe_height_;
  }

  double effectiveFixedFlightHeight() const
  {
    return altitude_reference_valid_ ? target_z_map_ : fixed_flight_height_;
  }

  double effectiveSharedMinHeight() const
  {
    return (altitude_reference_valid_ ? ground_z_map_ : 0.0) + shared_bounds_z_min_;
  }

  double effectiveSharedMaxHeight() const
  {
    return (altitude_reference_valid_ ? ground_z_map_ : 0.0) + shared_bounds_z_max_;
  }

  void altitudeReferenceCallback(
    const race_msgs::msg::FlightAltitudeReference::SharedPtr msg)
  {
    if (!msg->valid) {
      if (altitude_reference_valid_ && msg->flight_id == altitude_reference_flight_id_) {
        altitude_reference_valid_ = false;
        have_goal_ = false;
        active_path_.clear();
        clearPendingGlobalPath();
        resetLocalGoalProgress();
        RCLCPP_WARN(
          get_logger(), "[SUPER_ALTITUDE_REFERENCE_RESET] flight_id=%lu",
          static_cast<unsigned long>(msg->flight_id));
      }
      return;
    }
    const bool finite = std::isfinite(msg->ground_z_map) &&
      std::isfinite(msg->target_z_map) && std::isfinite(msg->target_agl_m) &&
      std::isfinite(msg->min_agl_m) && std::isfinite(msg->max_agl_m) &&
      std::isfinite(msg->ground_z_local_ned) && std::isfinite(msg->target_z_local_ned);
    if (!finite || msg->target_agl_m < shared_bounds_z_min_ - 1.0e-3 ||
      msg->target_agl_m > shared_bounds_z_max_ + 1.0e-3 ||
      std::abs(msg->min_agl_m - shared_bounds_z_min_) > 1.0e-3 ||
      std::abs(msg->max_agl_m - shared_bounds_z_max_) > 1.0e-3 ||
      std::abs((msg->target_z_map - msg->ground_z_map) - msg->target_agl_m) > 1.0e-3 ||
      std::abs(
        (msg->ground_z_local_ned - msg->target_z_local_ned) -
        msg->target_agl_m) > 1.0e-3)
    {
      RCLCPP_ERROR(
        get_logger(),
        "[SUPER_ALTITUDE_REFERENCE_REJECT] flight_id=%lu target_agl=%.3f safe_range=[%.3f,%.3f]",
        static_cast<unsigned long>(msg->flight_id), msg->target_agl_m,
        shared_bounds_z_min_, shared_bounds_z_max_);
      return;
    }
    if (altitude_reference_valid_ && msg->flight_id < altitude_reference_flight_id_) {
      return;
    }
    altitude_reference_flight_id_ = msg->flight_id;
    ground_z_map_ = msg->ground_z_map;
    target_z_map_ = msg->target_z_map;
    ground_z_local_ned_ = msg->ground_z_local_ned;
    target_z_local_ned_ = msg->target_z_local_ned;
    altitude_reference_valid_ = true;
    RCLCPP_WARN(
      get_logger(),
      "[SUPER_ALTITUDE_REFERENCE_ACCEPTED] flight_id=%lu ground_map=%.3f "
      "target_map=%.3f target_agl=%.3f",
      static_cast<unsigned long>(msg->flight_id), ground_z_map_, target_z_map_,
      msg->target_agl_m);
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (use_fixed_flight_height_ && !altitude_reference_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "[SUPER_GOAL_REJECT] no valid /race/flight_altitude_reference for this flight");
      return;
    }
    Vec3 goal{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    if (!finite_vec(goal)) {
      latchFault("FAULT_GOAL_NON_FINITE");
      RCLCPP_ERROR(get_logger(), "[FAULT_GOAL_NON_FINITE] source=%s", goal_topic_.c_str());
      return;
    }
    goal.z = clampHeight(goal.z);
    if (!insideSharedBounds(goal)) {
      latchFault("FAULT_GOAL_OUTSIDE_ENVELOPE");
      RCLCPP_ERROR(
        get_logger(),
        "[FAULT_GOAL_OUTSIDE_ENVELOPE] goal=(%.3f,%.3f,%.3f) envelope=x=[%.3f,%.3f] y=[%.3f,%.3f] z=[%.3f,%.3f]",
        goal.x, goal.y, goal.z, shared_bounds_x_min_, shared_bounds_x_max_,
        shared_bounds_y_min_, shared_bounds_y_max_, effectiveSharedMinHeight(),
        effectiveSharedMaxHeight());
      return;
    }
    if (fault_latched_) {
      RCLCPP_WARN(
        get_logger(), "[FAULT_LATCH_CLEARED] reason=%s by explicit valid final goal",
        fault_reason_.c_str());
      fault_latched_ = false;
      fault_reason_.clear();
    }
    if (have_goal_ && distance2d(goal, latest_goal_) <= goal_update_position_tolerance_m_ &&
      std::abs(goal.z - latest_goal_.z) <= goal_update_position_tolerance_m_)
    {
      if (pending_path_retry_exhausted_) {
        pending_path_retry_exhausted_ = false;
        pending_path_completed_replans_ = 0;
        active_path_.clear();
        clearPendingGlobalPath();
        last_goal_time_ = now();
        RCLCPP_WARN(
          get_logger(),
          "[SUPER_PENDING_PATH_REARM] explicit repeated goal id=%lu accepted after retry exhaustion",
          global_goal_id_);
        return;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignore duplicate final goal within %.2fm", goal_update_position_tolerance_m_);
      return;
    }
    latest_goal_ = goal;
    ++global_goal_id_;
    have_goal_ = true;
    clearFinalGoalReached("new_global_goal");
    last_goal_time_ = now();
    active_path_.clear();
    clearPendingGlobalPath();
    pending_path_completed_replans_ = 0;
    pending_path_retry_exhausted_ = false;
    resetLocalGoalProgress();
    RCLCPP_INFO(
      get_logger(), "Received goal id=%lu ENU/map=(%.2f, %.2f, %.2f)",
      global_goal_id_, latest_goal_.x, latest_goal_.y, latest_goal_.z);
  }

  void egoStatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const bool failed = msg->data == "NO_SAFE_TRAJECTORY" ||
      msg->data == "SETPOINT_INVALID" || msg->data == "TRAJECTORY_TIMEOUT" ||
      msg->data == "EMERGENCY_HOLD" ||
      msg->data == "REPLAN_SWITCH_DISCONTINUITY";
    // TRACKING and LOCAL_GOAL_REACHED are observations, not proof of recovery.
    // Only a newer bridge-validated trajectory that remains healthy for the
    // confirmation window clears the failure below.
    if (!failed) {return;}
    recovery_required_after_trajectory_id_ = last_validated_trajectory_id_;
    trajectory_recovery_candidate_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (local_planner_failure_start_time_.nanoseconds() == 0) {
      local_planner_failure_start_time_ = now();
    }
  }

  void validatedBsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
  {
    if (msg->traj_id <= last_validated_trajectory_id_) {return;}
    last_validated_trajectory_id_ = msg->traj_id;
    last_validated_trajectory_time_ = now();
    if (race_super_planner_ros2::shouldStartRecoveryConfirmation(
        msg->traj_id, recovery_required_after_trajectory_id_,
        trajectory_recovery_candidate_since_.nanoseconds() != 0))
    {
      // Start once on the first post-failure validated trajectory. Later
      // healthy trajectory updates must not perpetually restart the window.
      trajectory_recovery_candidate_since_ = now();
    }
    double duration = 0.0;
    const std::size_t order = msg->order > 0 ? static_cast<std::size_t>(msg->order) : 0U;
    if (msg->knots.size() > msg->pos_pts.size() && order < msg->knots.size()) {
      duration = msg->knots[msg->pos_pts.size()] - msg->knots[order];
    }
    if (!std::isfinite(duration) || duration <= 0.0) {
      duration = trajectory_prefetch_sec_;
    }
    const rclcpp::Time trajectory_start(msg->start_time, get_clock()->get_clock_type());
    validated_trajectory_lease_deadline_ =
      trajectory_start + rclcpp::Duration::from_seconds(duration);
  }

  double validatedTrajectoryLeaseRemaining()
  {
    if (validated_trajectory_lease_deadline_.nanoseconds() == 0) {
      return -std::numeric_limits<double>::infinity();
    }
    return (validated_trajectory_lease_deadline_ - now()).seconds();
  }

  void updateLocalPlannerWatchdog()
  {
    const bool recovery_candidate_is_stable =
      trajectory_recovery_candidate_since_.nanoseconds() != 0 &&
      last_validated_trajectory_id_ > recovery_required_after_trajectory_id_ &&
      ageSeconds(trajectory_recovery_candidate_since_) >=
      trajectory_recovery_confirmation_sec_;
    if (recovery_candidate_is_stable) {
      local_planner_failure_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      local_planner_failure_replan_latched_ = false;
      trajectory_recovery_candidate_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    const double trajectory_age = last_validated_trajectory_time_.nanoseconds() == 0 ?
      std::numeric_limits<double>::infinity() : ageSeconds(last_validated_trajectory_time_);
    const bool goal_waiting_for_trajectory = have_local_goal_ &&
      local_goal_seq_ > last_validated_local_goal_seq_ &&
      ageSeconds(last_local_goal_publish_time_) > trajectory_stall_timeout_sec_;
    const bool objective_stall = race_super_planner_ros2::localPlanningStalled(
      global_only_mode_ && have_goal_ && !final_goal_reached_latched_ &&
      heightAboveGround(latest_odom_.z) >= min_safe_height_,
      local_goal_reached_logged_, goal_waiting_for_trajectory, trajectory_age,
      validatedTrajectoryLeaseRemaining(), trajectory_stall_timeout_sec_);
    const bool reported_failure_persisted =
      local_planner_failure_start_time_.nanoseconds() != 0 &&
      ageSeconds(local_planner_failure_start_time_) >= local_planner_failure_replan_sec_;
    if (!local_planner_failure_replan_latched_ &&
      (objective_stall || reported_failure_persisted))
    {
      if (objective_stall && local_planner_failure_start_time_.nanoseconds() == 0) {
        recovery_required_after_trajectory_id_ = last_validated_trajectory_id_;
        trajectory_recovery_candidate_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      local_planner_failure_replan_latched_ = true;
      local_planner_replan_requested_ = true;
      RCLCPP_WARN(
        get_logger(),
        "[EGO_PROGRESS_STALLED] trajectory_id=%ld trajectory_age=%.3f "
        "lease_remaining=%.3f local_goal_seq=%lu validated_goal_seq=%lu "
        "local_goal_reached=%s; request one global replan",
        last_validated_trajectory_id_, trajectory_age,
        validatedTrajectoryLeaseRemaining(), local_goal_seq_,
        last_validated_local_goal_seq_, local_goal_reached_logged_ ? "true" : "false");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string & source)
  {
    Vec3 odom{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    if (odom_input_frame_ == "px4_ned") {
      odom = nedToEnu(odom);
    } else if (odom_input_frame_ == "map_enu") {
      // Scheme 1 uses the standard ENU map contract from 坐标转换.md.  The
      // MAVROS local pose is already expressed in that same world frame.
      // Keep this branch explicit because the launch interface still names
      // the source ``map_enu``.
      odom = Vec3{odom.x, odom.y, odom.z};
    }
    if (!finite_vec(odom)) {
      latchFault("FAULT_POSITION_NON_FINITE");
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 1000, "[FAULT_POSITION_NON_FINITE] source=%s", source.c_str());
      return;
    }
    if (!insideFaultEnvelopeXY(odom)) {
      latchFault("FAULT_POSITION_OUTSIDE_ENVELOPE");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[FAULT_POSITION_OUTSIDE_ENVELOPE] source=%s position=(%.3f,%.3f,%.3f) "
        "envelope_xy=x=[%.3f,%.3f] y=[%.3f,%.3f]",
        source.c_str(), odom.x, odom.y, odom.z, shared_bounds_x_min_, shared_bounds_x_max_,
        shared_bounds_y_min_, shared_bounds_y_max_);
      return;
    }
    const auto received_at = now();
    if (have_fault_checked_odom_) {
      const double dt_sec = (received_at - last_fault_checked_odom_time_).seconds();
      const double displacement_m = std::hypot(
        std::hypot(odom.x - latest_odom_.x, odom.y - latest_odom_.y), odom.z - latest_odom_.z);
      const double allowed_m = fault_position_max_speed_mps_ * std::max(0.0, dt_sec) +
        fault_position_jump_allowance_m_;
      if (dt_sec > 0.0 && displacement_m > allowed_m) {
        latchFault("FAULT_POSITION_JUMP");
        RCLCPP_ERROR(
          get_logger(), "[FAULT_POSITION_JUMP] source=%s displacement=%.3fm dt=%.3fs "
          "allowed=%.3fm previous=(%.3f,%.3f,%.3f) candidate=(%.3f,%.3f,%.3f)",
          source.c_str(), displacement_m, dt_sec, allowed_m, latest_odom_.x, latest_odom_.y,
          latest_odom_.z, odom.x, odom.y, odom.z);
        return;
      }
    }
    latest_odom_ = odom;
    last_fault_checked_odom_time_ = received_at;
    have_fault_checked_odom_ = true;
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    if (std::isfinite(vx) && std::isfinite(vy)) {
      latest_horizontal_speed_ = std::hypot(vx, vy);
    }
    const double odom_yaw = tf2::getYaw(msg->pose.pose.orientation);
    if (std::isfinite(odom_yaw)) {
      if (odom_input_frame_ == "px4_ned") {
        current_yaw_ned_ = wrapAngle(odom_yaw);
      } else if (odom_input_frame_ == "map_enu") {
        // Standard MAVROS ENU yaw -> PX4 NED yaw.
        current_yaw_ned_ = wrapAngle(M_PI_2 - odom_yaw);
      } else {
        const Vec3 heading_ned = enuToNed(Vec3{std::cos(odom_yaw), std::sin(odom_yaw), 0.0});
        current_yaw_ned_ = std::atan2(heading_ned.y, heading_ned.x);
      }
      have_current_yaw_ = true;
    }
    have_odom_ = true;
    last_odom_time_ = now();
    ++odom_generation_;
    last_odom_source_stamp_ns_ =
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
      static_cast<int64_t>(msg->header.stamp.nanosec);
    if (mavros_armed_ && !takeoff_path_released_ &&
      heightAboveGround(latest_odom_.z) >= min_safe_height_ - 1.0e-3)
    {
      takeoff_path_released_ = true;
      RCLCPP_INFO(
        get_logger(),
        "[SUPER_TAKEOFF_PATH_RELEASED] height=%.3f release_height=%.3f; "
        "later altitude excursions will not reset path progress",
        heightAboveGround(latest_odom_.z), min_safe_height_);
    }
    if (active_odom_source_ != source) {
      active_odom_source_ = source;
      RCLCPP_WARN(
        get_logger(),
        "Using odom source: %s odom_input_frame=%s planner_map=(%.2f, %.2f, %.2f) "
        "current_yaw_ned=%.3f",
        source.c_str(), odom_input_frame_.c_str(), odom.x, odom.y, odom.z,
        have_current_yaw_ ? current_yaw_ned_ : std::numeric_limits<double>::quiet_NaN());
    }
  }

  void mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    // MAVROS has no per-sample estimator validity flags.  Link health is the only
    // signal available here; odom finiteness/freshness is checked in odomCallback.
    mavros_connected_ = msg->connected;
    if (mavros_armed_ && !msg->armed) {
      takeoff_path_released_ = false;
      RCLCPP_INFO(
        get_logger(),
        "[SUPER_TAKEOFF_PATH_RESET] vehicle disarmed; next flight must release the takeoff path");
    }
    mavros_armed_ = msg->armed;
    last_mavros_state_time_ = now();
    if (!mavros_connected_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MAVROS reports connected=false; FCU link is down.");
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string & source)
  {
    latest_cloud_ = msg;
    have_cloud_ = true;
    last_cloud_time_ = now();
    if (active_cloud_source_ != source) {
      active_cloud_source_ = source;
      RCLCPP_WARN(
        get_logger(), "Using cloud source: %s frame_id=%s",
        source.c_str(), msg->header.frame_id.c_str());
    }
  }

  void clearFinalGoalReached(const char * trigger)
  {
    if (final_goal_reached_latched_ || final_goal_confirmation_count_ > 0) {
      RCLCPP_INFO(
        get_logger(),
        "Clear final-goal latch: trigger=%s global_goal_id=%lu global_path_id=%lu "
        "local_goal_seq=%lu",
        trigger, global_goal_id_, global_path_id_, local_goal_seq_);
    }
    final_goal_reached_latched_ = false;
    final_goal_confirmation_count_ = 0;
  }

  bool updateFinalGoalReached()
  {
    distance_to_final_ = distance2d(latest_odom_, latest_goal_);
    const bool position_ok = distance_to_final_ <= final_goal_position_tolerance_m_;
    const bool speed_ok = latest_horizontal_speed_ <= final_goal_velocity_tolerance_mps_;
    if (position_ok && speed_ok) {
      final_goal_confirmation_count_ = std::min(
        final_goal_confirmation_count_ + 1, final_goal_confirmation_cycles_);
    } else {
      final_goal_confirmation_count_ = 0;
    }

    if (!final_goal_reached_latched_ &&
      final_goal_confirmation_count_ >= final_goal_confirmation_cycles_)
    {
      final_goal_reached_latched_ = true;
      RCLCPP_WARN(
        get_logger(),
        "[GOAL_REACHED_HOLD] trigger_source=super_final_goal global_goal_id=%lu "
        "distance=%.3f speed=%.3f",
        global_goal_id_, distance_to_final_, latest_horizontal_speed_);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "[FINAL_GOAL_CHECK] global_goal_id=%lu distance_to_final=%.3f speed=%.3f "
      "confirm=%d/%d final_goal_reached=%s",
      global_goal_id_, distance_to_final_, latest_horizontal_speed_,
      final_goal_confirmation_count_, final_goal_confirmation_cycles_,
      final_goal_reached_latched_ ? "true" : "false");
    return final_goal_reached_latched_;
  }

  void clearPendingGlobalPath()
  {
    pending_global_path_.clear();
    pending_raw_path_.clear();
    pending_global_path_valid_ = false;
    pending_route_goal_id_ = 0;
    pending_plan_start_odom_ = Vec3{};
    pending_plan_start_odom_generation_ = 0;
    pending_plan_start_odom_source_stamp_ns_ = 0;
    pending_plan_completion_odom_generation_ = 0;
    pending_plan_completion_odom_source_stamp_ns_ = 0;
    pending_plan_completion_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  race_super_planner_ros2::PendingPathGateInput pendingPathGateInput(
    bool projection_valid, double cross_track_error) const
  {
    race_super_planner_ros2::PendingPathGateInput input;
    input.pending_goal_id = pending_route_goal_id_;
    input.current_goal_id = global_goal_id_;
    input.plan_completion_odom_generation = pending_plan_completion_odom_generation_;
    input.current_odom_generation = odom_generation_;
    input.plan_completion_odom_source_stamp_ns =
      pending_plan_completion_odom_source_stamp_ns_;
    input.current_odom_source_stamp_ns = last_odom_source_stamp_ns_;
    input.projection_valid = projection_valid;
    input.path_cross_track_error_m = cross_track_error;
    input.maximum_path_cross_track_error_m = pending_path_max_cross_track_error_m_;
    input.completed_retries = pending_path_completed_replans_;
    input.maximum_retries = pending_path_max_replans_;
    return input;
  }

  bool pendingPathNeedsFreshOdom() const
  {
    return pending_global_path_valid_ &&
           !race_super_planner_ros2::hasPostPlanFreshOdom(
      pendingPathGateInput(false, std::numeric_limits<double>::infinity()));
  }

  std::vector<Vec3> trimPendingPathAtProjection(const PathProjection & projection) const
  {
    std::vector<Vec3> trimmed;
    if (pending_global_path_.size() < 2 ||
      projection.segment_index + 1 >= pending_global_path_.size())
    {
      return trimmed;
    }

    const auto append_if_new = [&trimmed](const Vec3 & point) {
        if (trimmed.empty() || distance2d(trimmed.back(), point) > 1.0e-6 ||
          std::abs(trimmed.back().z - point.z) > 1.0e-6)
        {
          trimmed.push_back(point);
        }
      };

    Vec3 current_start = latest_odom_;
    current_start.z = clampHeight(current_start.z);
    append_if_new(current_start);
    append_if_new(projection.point);
    for (std::size_t index = projection.segment_index + 1;
      index < pending_global_path_.size(); ++index)
    {
      append_if_new(pending_global_path_[index]);
    }

    // Keep the two-point path contract even when the fresh odom is already at
    // the final endpoint. The normal final-goal confirmation then owns HOLD.
    if (trimmed.size() == 1) {
      trimmed.push_back(pending_global_path_.back());
    }
    return trimmed;
  }

  bool rejectPendingGlobalPath(const char * reason, double cross_track_error)
  {
    const double old_start_distance = pending_global_path_.empty() ?
      std::numeric_limits<double>::infinity() :
      distance2d(latest_odom_, pending_global_path_.front());
    const bool retry_allowed =
      pending_path_completed_replans_ < pending_path_max_replans_;
    const uint64_t rejected_goal_id = pending_route_goal_id_;
    clearPendingGlobalPath();
    active_path_.clear();
    selected_setpoint_ = latest_odom_;
    selected_setpoint_.z = clampHeight(selected_setpoint_.z);

    if (retry_allowed) {
      ++pending_path_completed_replans_;
      setMode(Mode::PLANNING);
      publish_reason_ = "PENDING_PATH_REPLAN_FROM_FRESH_ODOM";
      RCLCPP_WARN(
        get_logger(),
        "[SUPER_PENDING_PATH_REJECT] goal_id=%lu reason=%s d_path=%.3f threshold=%.3f "
        "d_start_diagnostic=%.3f retry=%d/%d; no path was committed",
        rejected_goal_id, reason, cross_track_error,
        pending_path_max_cross_track_error_m_, old_start_distance,
        pending_path_completed_replans_, pending_path_max_replans_);
      logStatus();
      return true;
    }

    pending_path_retry_exhausted_ = true;
    setMode(Mode::HOLD);
    publish_reason_ = "PENDING_PATH_RETRY_EXHAUSTED";
    RCLCPP_ERROR(
      get_logger(),
      "[SUPER_PENDING_PATH_RETRY_EXHAUSTED] goal_id=%lu reason=%s d_path=%.3f "
      "threshold=%.3f d_start_diagnostic=%.3f; wait for a new explicit goal",
      rejected_goal_id, reason, cross_track_error,
      pending_path_max_cross_track_error_m_, old_start_distance);
    publishHoldIfEnabled("HOLD: pending global path stayed stale after one retry");
    logStatus();
    return true;
  }

  bool handlePendingGlobalPath()
  {
    if (!pending_global_path_valid_) {
      return false;
    }

    const auto projection = projectOntoPath(pending_global_path_, latest_odom_);
    const bool projection_valid = projection.has_value() &&
      projection->segment_index + 1 < pending_global_path_.size() &&
      std::isfinite(projection->segment_ratio) && projection->segment_ratio >= 0.0 &&
      projection->segment_ratio <= 1.0 && finite_vec(projection->point);
    const double cross_track_error = projection_valid ? projection->cross_track_error :
      std::numeric_limits<double>::infinity();
    const auto decision = race_super_planner_ros2::decidePendingPathGate(
      pendingPathGateInput(projection_valid, cross_track_error));

    if (decision == race_super_planner_ros2::PendingPathGateDecision::WAIT_FOR_FRESH_ODOM) {
      setMode(Mode::PLANNING);
      publish_reason_ = "PENDING_FRESH_ODOM";
      logStatus();
      return true;
    }
    if (decision == race_super_planner_ros2::PendingPathGateDecision::DISCARD_GOAL_CHANGED) {
      RCLCPP_WARN(
        get_logger(),
        "[SUPER_PENDING_PATH_DISCARD] pending_goal_id=%lu current_goal_id=%lu reason=GOAL_CHANGED",
        pending_route_goal_id_, global_goal_id_);
      clearPendingGlobalPath();
      return false;
    }
    if (decision ==
      race_super_planner_ros2::PendingPathGateDecision::RETRY_FROM_LATEST_ODOM ||
      decision == race_super_planner_ros2::PendingPathGateDecision::HOLD_RETRY_EXHAUSTED)
    {
      return rejectPendingGlobalPath(
        projection_valid ? "PATH_CROSS_TRACK" : "INVALID_FORWARD_PROJECTION",
        cross_track_error);
    }

    std::vector<Vec3> committed_path = trimPendingPathAtProjection(projection.value());
    const auto validation = validateCompletePath(committed_path, "PENDING_TRIMMED");
    if (!validation.valid()) {
      return rejectPendingGlobalPath("TRIMMED_CONNECTOR_UNSAFE", cross_track_error);
    }

    const double old_start_distance = distance2d(latest_odom_, pending_global_path_.front());
    const Vec3 raw_endpoint = pending_raw_path_.empty() ?
      committed_path.back() : pending_raw_path_.back();
    active_path_ = std::move(committed_path);
    const std::vector<Vec3> raw_path = pending_raw_path_;
    const uint64_t committed_goal_id = pending_route_goal_id_;
    const std::size_t projection_segment = projection->segment_index;
    const double pending_wait_sec = ageSeconds(pending_plan_completion_time_);
    clearPendingGlobalPath();
    pending_path_completed_replans_ = 0;
    pending_path_retry_exhausted_ = false;

    publishRawPath(raw_path);
    publishPath(active_path_);
    startNewGlobalPath(active_path_);
    last_plan_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "[SUPER_PENDING_PATH_COMMIT] goal_id=%lu d_path=%.3f threshold=%.3f "
      "d_start_diagnostic=%.3f projection_segment=%zu committed_points=%zu "
      "fresh_odom_wait=%.3fs",
      committed_goal_id, cross_track_error, pending_path_max_cross_track_error_m_,
      old_start_distance, projection_segment, active_path_.size(), pending_wait_sec);
    RCLCPP_INFO(
      get_logger(),
      "[SUPER_GLOBAL_PATH_COMPLETE] goal_id=%lu raw_endpoint=(%.3f,%.3f,%.3f) "
      "published_endpoint=(%.3f,%.3f,%.3f) final_goal=(%.3f,%.3f,%.3f) endpoint_error=%.6f",
      committed_goal_id, raw_endpoint.x, raw_endpoint.y, raw_endpoint.z,
      active_path_.back().x, active_path_.back().y, active_path_.back().z,
      latest_goal_.x, latest_goal_.y, latest_goal_.z,
      distance2d(active_path_.back(), latest_goal_));
    return false;
  }

  void timerCallback()
  {
    // A synchronous A* blocks the single-threaded executor. Do not let the
    // timer classify that expected gap as NO_ODOM, and do not plan again while
    // the completed route is waiting for a callback that occurred afterwards.
    if (pendingPathNeedsFreshOdom()) {
      setMode(Mode::PLANNING);
      publish_reason_ = "PENDING_FRESH_ODOM";
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[SUPER_PENDING_PATH_WAIT] goal_id=%lu plan_odom_generation=%lu "
        "current_odom_generation=%lu reason=PENDING_FRESH_ODOM",
        pending_route_goal_id_, pending_plan_completion_odom_generation_, odom_generation_);
      logStatus();
      return;
    }

    const bool odom_ok = have_odom_ && ageSeconds(last_odom_time_) <= odom_timeout_sec_;
    if (!odom_ok) {
      setMode(Mode::NO_ODOM);
      publish_reason_ = "NO_ODOM";
      logStatus();
      return;
    }

    if (fault_latched_) {
      active_path_.clear();
      clearPendingGlobalPath();
      resetLocalGoalProgress();
      selected_setpoint_ = latest_odom_;
      selected_setpoint_.z = clampHeight(selected_setpoint_.z);
      setMode(Mode::HOLD);
      publish_reason_ = fault_reason_;
      publishHoldIfEnabled("HOLD: fault envelope/corridor protection latched");
      logStatus();
      return;
    }

    if (use_fixed_flight_height_ && !altitude_reference_valid_) {
      active_path_.clear();
      clearPendingGlobalPath();
      selected_setpoint_ = latest_odom_;
      setMode(Mode::IDLE);
      publish_reason_ = "WAIT_ALTITUDE_REFERENCE";
      logStatus();
      return;
    }

    const bool goal_ok = have_goal_ &&
      (goal_timeout_sec_ <= 0.0 || ageSeconds(last_goal_time_) <= goal_timeout_sec_);
    if (!goal_ok) {
      active_path_.clear();
      clearPendingGlobalPath();
      selected_setpoint_ = latest_odom_;
      selected_setpoint_.z = clampHeight(selected_setpoint_.z);
      setMode(Mode::IDLE);
      publish_reason_ = "NO_GOAL";
      logStatus();
      return;
    }

    if (pending_path_retry_exhausted_) {
      active_path_.clear();
      selected_setpoint_ = latest_odom_;
      selected_setpoint_.z = clampHeight(selected_setpoint_.z);
      setMode(Mode::HOLD);
      publish_reason_ = "PENDING_PATH_RETRY_EXHAUSTED";
      publishHoldIfEnabled("HOLD: waiting for an explicit goal after stale-path retries");
      logStatus();
      return;
    }

    if (updateFinalGoalReached()) {
      active_path_.clear();
      clearPendingGlobalPath();
      selected_setpoint_ = latest_odom_;
      selected_setpoint_.z = clampHeight(selected_setpoint_.z);
      setMode(Mode::HOLD);
      publish_reason_ = "GOAL_REACHED_HOLD";
      publishHoldIfEnabled("GOAL_REACHED: holding current position");
      logStatus();
      return;
    }

    const bool cloud_ok = have_cloud_ && ageSeconds(last_cloud_time_) <= cloud_timeout_sec_;
    if (!cloud_ok) {
      active_path_.clear();
      clearPendingGlobalPath();
      selected_setpoint_ = latest_odom_;
      selected_setpoint_.z = clampHeight(selected_setpoint_.z);
      setMode(Mode::NO_MAP);
      publish_reason_ = "NO_MAP";
      publishHoldIfEnabled("NO_MAP: no valid point cloud/map");
      logStatus();
      return;
    }

    const bool had_pending_global_path = pending_global_path_valid_;
    if (handlePendingGlobalPath()) {
      publishDebugMarkers();
      return;
    }

    if (!had_pending_global_path) {
      updateLocalPlannerWatchdog();
    }

    const bool freeze_global_path_for_takeoff =
      race_super_planner_ros2::shouldFreezeGlobalPathDuringTakeoff(
      global_only_mode_, takeoff_path_released_, heightAboveGround(latest_odom_.z),
      min_safe_height_);

    if (freeze_global_path_for_takeoff && local_planner_replan_requested_) {
      local_planner_replan_requested_ = false;
      local_planner_failure_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      local_planner_failure_replan_latched_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[SUPER_TAKEOFF_REPLAN_SUPPRESSED] height=%.3f release_height=%.3f",
        heightAboveGround(latest_odom_.z), min_safe_height_);
    } else if (global_only_mode_ && local_planner_replan_requested_) {
      active_path_.clear();
      resetLocalGoalProgress();
      local_planner_replan_requested_ = false;
      publish_reason_ = "EGO_FAILURE_REPLAN";
    }

    const bool replan_due = active_path_.empty() ||
      (!global_only_mode_ && replan_while_tracking_ &&
      ageSeconds(last_plan_time_) >= (1.0 / replan_rate_));
    if (replan_due) {
      setMode(Mode::PLANNING);
      const Vec3 plan_start_odom = latest_odom_;
      const uint64_t plan_start_goal_id = global_goal_id_;
      const uint64_t plan_start_odom_generation = odom_generation_;
      const int64_t plan_start_odom_source_stamp_ns = last_odom_source_stamp_ns_;
      std::vector<Vec3> raw_path;
      if (!plan(raw_path) || raw_path.empty()) {
        active_path_.clear();
        selected_setpoint_ = latest_odom_;
        selected_setpoint_.z = clampHeight(selected_setpoint_.z);
        publishEmptyPath();
        publishRawPath({});
        setMode(Mode::HOLD);
        publish_reason_ = "NO_PATH";
        publishHoldIfEnabled("HOLD: planner failed");
        publishDebugMarkers();
        logStatus();
        return;
      }

      std::vector<Vec3> processed_path;
      if (!processPath(raw_path, processed_path)) {
        active_path_.clear();
        selected_setpoint_ = latest_odom_;
        selected_setpoint_.z = clampHeight(selected_setpoint_.z);
        publishEmptyPath();
        setMode(Mode::HOLD);
        publish_reason_ = "NO_SAFE_PATH_AFTER_SMOOTHING";
        publishHoldIfEnabled("HOLD: raw and smoothed paths failed safety validation");
        publishDebugMarkers();
        logStatus();
        return;
      }

      pending_global_path_ = std::move(processed_path);
      pending_raw_path_ = std::move(raw_path);
      pending_global_path_valid_ = true;
      pending_route_goal_id_ = plan_start_goal_id;
      pending_plan_start_odom_ = plan_start_odom;
      pending_plan_start_odom_generation_ = plan_start_odom_generation;
      pending_plan_start_odom_source_stamp_ns_ = plan_start_odom_source_stamp_ns;
      pending_plan_completion_odom_generation_ = odom_generation_;
      pending_plan_completion_odom_source_stamp_ns_ = last_odom_source_stamp_ns_;
      pending_plan_completion_time_ = now();
      setMode(Mode::PLANNING);
      publish_reason_ = "PENDING_FRESH_ODOM";
      RCLCPP_INFO(
        get_logger(),
        "[SUPER_PENDING_PATH_READY] goal_id=%lu points=%zu plan_start=(%.3f,%.3f,%.3f) "
        "plan_start_odom_generation=%lu completion_odom_generation=%lu "
        "plan_start_source_stamp_ns=%ld completion_source_stamp_ns=%ld; "
        "wait for post-plan odom before first commit",
        pending_route_goal_id_, pending_global_path_.size(),
        pending_plan_start_odom_.x, pending_plan_start_odom_.y, pending_plan_start_odom_.z,
        pending_plan_start_odom_generation_, pending_plan_completion_odom_generation_,
        pending_plan_start_odom_source_stamp_ns_,
        pending_plan_completion_odom_source_stamp_ns_);
      publishDebugMarkers();
      logStatus();
      return;
    }
    publishDebugMarkers();

    if (freeze_global_path_for_takeoff) {
      cross_track_error_ = distance2d(latest_odom_, active_path_.front());
      bool initial_local_goal_ok = true;
      if (publish_ego_local_goal_ && !have_local_goal_) {
        const PathProjection start_projection{active_path_.front(), 0, 0.0, cross_track_error_};
        initial_local_goal_ok = publishRollingLocalGoal(active_path_, start_projection);
      }
      if (!initial_local_goal_ok) {
        setMode(Mode::HOLD);
        publish_reason_ = "TAKEOFF_INITIAL_LOCAL_GOAL_BLOCKED";
        logStatus();
        return;
      }
      setMode(Mode::FOLLOWING);
      publish_reason_ = "TAKEOFF_GLOBAL_PATH_FROZEN";
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[SUPER_TAKEOFF_PATH_FREEZE] path_id=%lu height=%.3f release_height=%.3f "
        "cross_track_ignored=%.3f local_goal_seq=%lu",
        global_path_id_, heightAboveGround(latest_odom_.z), min_safe_height_, cross_track_error_,
        local_goal_seq_);
      logStatus();
      return;
    }

    const auto projection = projectOntoPath(active_path_, latest_odom_);
    cross_track_error_ = projection.has_value() ? projection->cross_track_error :
      std::numeric_limits<double>::infinity();
    if (!projection.has_value() || projection->cross_track_error > max_path_tracking_error_) {
      latchFault("FAULT_MISSION_CORRIDOR_DEVIATION");
      setMode(Mode::HOLD);
      publish_reason_ = fault_reason_;
      publishHoldIfEnabled("HOLD: mission corridor deviation; automatic replan disabled");
      logStatus();
      return;
    }

    const std::size_t previous_progress_index = tracking_progress_index_;
    tracking_progress_index_ = std::max(
      tracking_progress_index_, projection->segment_index);

    bool rolling_local_goal_ok = true;
    if (publish_ego_local_goal_) {
      rolling_local_goal_ok = publishRollingLocalGoal(active_path_, projection.value());
    }

    if (global_only_mode_) {
      if (!rolling_local_goal_ok) {
        active_path_.clear();
        resetLocalGoalProgress();
        setMode(Mode::HOLD);
        publish_reason_ = "EGO_LOCAL_GOAL_BLOCKED";
        logStatus();
        return;
      }
      setMode(Mode::FOLLOWING);
      publish_reason_ = "GLOBAL_PATH_TO_EGO";
      logStatus();
      return;
    }
    if (!rolling_local_goal_ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[EGO_SHADOW_LOCAL_GOAL_BLOCKED] Keep Super navigation control unchanged");
    }

    bool narrow_corridor = false;
    bool centering_only = false;
    double corridor_clearance = std::numeric_limits<double>::infinity();
    const auto safe_tracking_target = selectSafeTrackingTarget(
      active_path_, projection.value(), narrow_corridor, centering_only, corridor_clearance);
    if (narrow_corridor) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[NARROW_CORRIDOR_TRACKING] path_clearance=%.3f threshold=%.3f "
        "cross_track=%.3f centering_only=%s lookahead=%.3f",
        corridor_clearance,
        vehicle_collision_radius_ + narrow_corridor_tracking_margin_ + resolution_ * 0.5,
        projection->cross_track_error, centering_only ? "true" : "false",
        centering_only ? 0.0 : narrow_corridor_lookahead_distance_);
    }
    if (!safe_tracking_target.has_value()) {
      selected_setpoint_ = latest_odom_;
      selected_setpoint_.z = clampHeight(selected_setpoint_.z);
      setMode(Mode::HOLD);
      publish_reason_ = "TRACKING_CHORD_BLOCKED";
      publishHoldIfEnabled("HOLD: direct tracking chord intersects inflated obstacle");
      logStatus();
      return;
    }
    selected_setpoint_ = safe_tracking_target->point;
    selected_setpoint_.z = clampHeight(selected_setpoint_.z);
    updateYawCommand(active_path_, projection.value());
    setMode(Mode::FOLLOWING);
    publish_reason_ = controlOutputEnabled() ?
      "NAVIGATION_SETPOINT_PENDING" : "ENABLE_OUTPUT_FALSE";
    publishNavigationSetpointIfEnabled(selected_setpoint_);
    logPathProgress(
      projection.value(), previous_progress_index, safe_tracking_target.value());
    logStatus();
  }

  bool plan(std::vector<Vec3> & path)
  {
    path.clear();
    const Vec3 start{latest_odom_.x, latest_odom_.y, clampHeight(latest_odom_.z)};
    const Vec3 final_goal{latest_goal_.x, latest_goal_.y, clampHeight(latest_goal_.z)};
    const auto goal_selection = race_super_planner_ros2::selectPlanningGoal(
      global_only_mode_, start.x, start.y, final_goal.x, final_goal.y,
      std::max(0.5, local_range_xy_ * 0.90));
    Vec3 goal{goal_selection.x, goal_selection.y, final_goal.z};
    RCLCPP_INFO(
      get_logger(),
      "[SUPER_GOAL_SELECTION] mode=%s final_goal=(%.3f,%.3f,%.3f) "
      "planning_goal=(%.3f,%.3f,%.3f) distance=%.3f clipped=%s",
      global_only_mode_ ? "GLOBAL_FULL_GOAL" : "LOCAL_DISTANCE_LIMIT",
      final_goal.x, final_goal.y, final_goal.z, goal.x, goal.y, goal.z,
      distance2d(start, final_goal), goal_selection.clipped ? "true" : "false");

    const double min_attempt_inflation =
      std::min(inflation_radius_, std::max(0.0, min_planning_inflation_radius_));
    std::vector<double> inflation_attempts{inflation_radius_};
    const std::vector<double> ratios{0.85, 0.70, 0.55};
    for (const double ratio : ratios) {
      const double candidate = inflation_radius_ * ratio;
      if (candidate >= min_attempt_inflation - 1.0e-6) {
        inflation_attempts.push_back(candidate);
      }
    }
    if (std::abs(inflation_attempts.back() - min_attempt_inflation) > 1.0e-6) {
      inflation_attempts.push_back(min_attempt_inflation);
    }
    inflation_attempts.erase(
      std::unique(inflation_attempts.begin(), inflation_attempts.end()),
      inflation_attempts.end());

    for (double attempt_inflation : inflation_attempts) {
      active_inflation_radius_ = attempt_inflation;
      if (!buildGrid(start, goal)) {
        return false;
      }

      const Cell start_cell = worldToCell(start);
      if (isOccupied(start_cell)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[GRID_START_OCCUPIED] position=(%.3f,%.3f) cell=(%d,%d) "
          "raw_obstacle_clearance=%.3f inflation=%.2f",
          start.x, start.y, start_cell.x, start_cell.y,
          distanceToNearestRawObstacle(start), active_inflation_radius_);
      }
      Cell goal_cell = worldToCell(goal);
      if (!isPlanningCell(start_cell)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Start outside planning grid");
        return false;
      }
      if (!isPlanningCell(goal_cell)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Goal outside centre-safe planning grid after selection");
        return false;
      }
      if (isOccupied(goal_cell)) {
        if (global_only_mode_) {
          RCLCPP_WARN(
            get_logger(),
            "Global final-goal cell is occupied; reject rather than snapping endpoint goal=(%.3f,%.3f,%.3f)",
            goal.x, goal.y, goal.z);
          return false;
        }
        const auto nearest_goal = findNearestFreeCell(goal_cell, 2.0);
        if (nearest_goal.has_value()) {
          goal_cell = *nearest_goal;
          goal = cellToWorld(goal_cell);
          goal.z = clampHeight(latest_goal_.z);
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Goal cell occupied, snapped to nearest free cell with inflation=%.2f",
            active_inflation_radius_);
        }
      }

      const double direct_path_clearance = std::max(
        smoothing_min_clearance_,
        min_planning_inflation_radius_);
      if (allow_direct_path_ && !isOccupied(goal_cell) &&
        isSegmentContinuouslySafe(start, goal, direct_path_clearance).safe)
      {
        path = {start, goal};
        if (attempt_inflation != inflation_radius_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Planner recovered with reduced inflation %.2f -> %.2f (direct path)",
            inflation_radius_, attempt_inflation);
        }
        return true;
      }

      std::vector<Cell> cell_path;
      if (!astar(start_cell, goal_cell, cell_path)) {
        continue;
      }

      path.reserve(cell_path.size());
      for (const auto & cell : cell_path) {
        Vec3 point = cellToWorld(cell);
        point.z =
          interpolateHeight(
          start, goal,
          distance2d(start, point) / std::max(0.01, distance2d(start, goal)));
        path.push_back(point);
      }
      if (!path.empty()) {
        path.front() = start;
        path.back() = goal;
      }
      if (attempt_inflation != inflation_radius_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Planner recovered with reduced inflation %.2f -> %.2f (A*)",
          inflation_radius_, attempt_inflation);
      }
      return true;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "A* failed: no safe path in local grid with inflation >= %.2f", min_attempt_inflation);
    return false;
  }

  bool buildGrid(const Vec3 & center, const Vec3 & goal)
  {
    if (global_only_mode_) {
      const double grid_min_x = std::max(
        shared_bounds_x_min_, std::min(center.x, goal.x) - fault_planning_grid_padding_m_);
      const double grid_max_x = std::min(
        shared_bounds_x_max_, std::max(center.x, goal.x) + fault_planning_grid_padding_m_);
      const double grid_min_y = std::max(
        shared_bounds_y_min_, std::min(center.y, goal.y) - fault_planning_grid_padding_m_);
      const double grid_max_y = std::min(
        shared_bounds_y_max_, std::max(center.y, goal.y) + fault_planning_grid_padding_m_);
      const auto grid = race_super_planner_ros2::makeSharedBoundsGrid(
        grid_min_x, grid_max_x, grid_min_y, grid_max_y, planning_resolution_);
      grid_origin_x_ = grid.origin_x;
      grid_origin_y_ = grid.origin_y;
      grid_width_ = grid.width;
      grid_height_ = grid.height;
      const Cell start_cell = worldToCell(center);
      const Cell goal_cell = worldToCell(goal);
      RCLCPP_INFO(
        get_logger(),
        "[SUPER_GLOBAL_GRID] min_x=%.3f max_x=%.3f min_y=%.3f max_y=%.3f "
        "resolution=%.3f width=%d height=%d start_index=(%d,%d) goal_index=(%d,%d)",
        grid_min_x, grid_max_x, grid_min_y, grid_max_y,
        planning_resolution_, grid_width_, grid_height_, start_cell.x, start_cell.y, goal_cell.x,
        goal_cell.y);
    } else {
      // Keep local-mode obstacle quantization on a static lattice around the vehicle.
      grid_origin_x_ =
        std::floor((center.x - local_range_xy_) / planning_resolution_) * planning_resolution_;
      grid_origin_y_ =
        std::floor((center.y - local_range_xy_) / planning_resolution_) * planning_resolution_;
      grid_width_ = std::max(
        3, static_cast<int>(std::ceil((2.0 * local_range_xy_) / planning_resolution_)));
      grid_height_ = grid_width_;
    }
    occupied_.clear();
    raw_obstacles_.clear();
    inflated_costs_.clear();
    raw_obstacle_points_->clear();

    if (!latest_cloud_) {
      return false;
    }

    const std::string cloud_frame = latest_cloud_->header.frame_id;
    if (cloud_frame.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject point cloud with empty frame_id; expected %s or a TF-connected source",
        world_frame_.c_str());
      return false;
    }
    bool can_transform = cloud_frame == world_frame_;
    geometry_msgs::msg::TransformStamped transform;
    if (!can_transform) {
      try {
        transform = tf_buffer_.lookupTransform(
          world_frame_, cloud_frame, tf2::TimePointZero, tf2::durationFromSec(0.02));
        can_transform = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Reject cloud: no TF %s -> %s. error=%s",
          cloud_frame.c_str(), world_frame_.c_str(), ex.what());
      }
    }
    if (!can_transform) {
      return false;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud_, "z");

    const double cost_radius = std::max(active_inflation_radius_, soft_obstacle_cost_radius_);
    const int cost_cells = std::max(
      0, static_cast<int>(std::ceil(cost_radius / planning_resolution_)));
    int used_points = 0;
    int point_index = 0;
    const int stride =
      std::max(
      1,
      static_cast<int>((latest_cloud_->width * latest_cloud_->height) /
      std::max(1, max_cloud_points_)));

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
      if (point_index % stride != 0) {
        continue;
      }
      Vec3 point{*iter_x, *iter_y, *iter_z};
      if (!finite_vec(point)) {
        continue;
      }
      if (cloud_frame != world_frame_ && can_transform) {
        geometry_msgs::msg::PointStamped in;
        in.header = latest_cloud_->header;
        in.point.x = point.x;
        in.point.y = point.y;
        in.point.z = point.z;
        geometry_msgs::msg::PointStamped out;
        tf2::doTransform(in, out, transform);
        point = Vec3{out.point.x, out.point.y, out.point.z};
      }
      if (point.z < obstacle_min_height_ || point.z > obstacle_max_height_) {
        continue;
      }
      if (!global_only_mode_ &&
        (std::abs(point.x - center.x) > local_range_xy_ ||
        std::abs(point.y - center.y) > local_range_xy_))
      {
        continue;
      }
      const Cell cell = worldToCell(point);
      if (!isInside(cell)) {
        continue;
      }
      raw_obstacle_points_->push_back(
        pcl::PointXYZ(
          static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)));
      raw_obstacles_.insert(cell);
      for (int dx = -cost_cells; dx <= cost_cells; ++dx) {
        for (int dy = -cost_cells; dy <= cost_cells; ++dy) {
          const double dist = std::hypot(dx, dy) * planning_resolution_;
          if (dist > cost_radius) {
            continue;
          }
          Cell inflated{cell.x + dx, cell.y + dy};
          if (isInside(inflated)) {
            if (dist <= active_inflation_radius_) {
              occupied_.insert(inflated);
            }
            const double ratio =
              std::max(
              0.0, std::min(
                1.0, 1.0 - dist / std::max(cost_radius, planning_resolution_)));
            const uint8_t cost = static_cast<uint8_t>(std::round(20.0 + ratio * 80.0));
            auto it = inflated_costs_.find(inflated);
            if (it == inflated_costs_.end() || cost > it->second) {
              inflated_costs_[inflated] = cost;
            }
          }
        }
      }
      ++used_points;
    }
    raw_obstacle_kdtree_.setInputCloud(raw_obstacle_points_);
    RCLCPP_DEBUG(
      get_logger(), "Built grid with %d cloud points, occupied cells=%zu", used_points,
      occupied_.size());
    return true;
  }

  bool astar(const Cell & start, const Cell & goal, std::vector<Cell> & path)
  {
    struct Node
    {
      Cell cell;
      double f{0.0};
      double g{0.0};
    };
    struct Compare
    {
      bool operator()(const Node & a, const Node & b) const {return a.f > b.f;}
    };

    auto heuristic = [](const Cell & a, const Cell & b) {
        return std::hypot(static_cast<double>(a.x - b.x), static_cast<double>(a.y - b.y));
      };

    std::priority_queue<Node, std::vector<Node>, Compare> open;
    std::unordered_map<Cell, double, CellHash> g_score;
    std::unordered_map<Cell, Cell, CellHash> came_from;
    std::unordered_set<Cell, CellHash> closed;

    open.push(Node{start, heuristic(start, goal), 0.0});
    g_score[start] = 0.0;

    const std::vector<Cell> dirs = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    while (!open.empty()) {
      const Node current = open.top();
      open.pop();
      if (closed.count(current.cell) > 0) {
        continue;
      }
      if (current.cell == goal) {
        reconstructPath(came_from, current.cell, path);
        return true;
      }
      closed.insert(current.cell);

      for (const auto & dir : dirs) {
        Cell next{current.cell.x + dir.x, current.cell.y + dir.y};
        if (!isPlanningCell(next) ||
          !continuouslySafe(cellToWorld(next)) || closed.count(next) > 0)
        {
          continue;
        }
        if (dir.x != 0 && dir.y != 0 &&
          (!isPlanningCell(Cell{current.cell.x + dir.x, current.cell.y}) ||
          !isPlanningCell(Cell{current.cell.x, current.cell.y + dir.y})))
        {
          continue;
        }
        if (!isSegmentSafeForAstar(cellToWorld(current.cell), cellToWorld(next))) {
          continue;
        }
        const double step = (dir.x != 0 && dir.y != 0) ? std::sqrt(2.0) : 1.0;
        const double soft_cost = clearanceCost(next);
        const double tentative_g = current.g + step + soft_cost;
        const auto found = g_score.find(next);
        if (found != g_score.end() && tentative_g >= found->second) {
          continue;
        }
        came_from[next] = current.cell;
        g_score[next] = tentative_g;
        open.push(Node{next, tentative_g + heuristic(next, goal), tentative_g});
      }
    }
    return false;
  }

  void reconstructPath(
    const std::unordered_map<Cell, Cell, CellHash> & came_from,
    Cell current,
    std::vector<Cell> & path)
  {
    path.clear();
    path.push_back(current);
    while (came_from.count(current) > 0) {
      current = came_from.at(current);
      path.push_back(current);
    }
    std::reverse(path.begin(), path.end());
  }

  bool lineFree(const Cell & a, const Cell & b) const
  {
    int x0 = a.x;
    int y0 = a.y;
    const int x1 = b.x;
    const int y1 = b.y;
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
      if (isOccupied(Cell{x0, y0})) {
        return false;
      }
      if (x0 == x1 && y0 == y1) {
        return true;
      }
      const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }
    }
  }

  std::optional<PathProjection> projectOntoPath(
    const std::vector<Vec3> & path, const Vec3 & position) const
  {
    if (path.empty()) {
      return std::nullopt;
    }
    if (path.size() == 1) {
      return PathProjection{path.front(), 0, 0.0, distance2d(path.front(), position)};
    }

    PathProjection best;
    best.cross_track_error = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const Vec3 & a = path[i];
      const Vec3 & b = path[i + 1];
      const double vx = b.x - a.x;
      const double vy = b.y - a.y;
      const double length_squared = vx * vx + vy * vy;
      const double ratio = length_squared <= 1.0e-9 ? 0.0 : std::clamp(
        ((position.x - a.x) * vx + (position.y - a.y) * vy) / length_squared, 0.0, 1.0);
      const Vec3 projected = interpolatePoint(a, b, ratio);
      const double error = distance2d(position, projected);
      if (error < best.cross_track_error) {
        best = PathProjection{projected, i, ratio, error};
      }
    }

    return best;
  }

  Vec3 samplePathFromProjection(
    const std::vector<Vec3> & path, const PathProjection & projection,
    double target_distance) const
  {
    if (path.size() < 2) {
      return path.empty() ? latest_odom_ : path.back();
    }

    double remaining = std::max(0.0, target_distance);
    Vec3 segment_start = projection.point;
    for (std::size_t i = projection.segment_index + 1; i < path.size(); ++i) {
      const Vec3 & segment_end = path[i];
      const double segment_length = distance2d(segment_start, segment_end);
      if (remaining <= segment_length) {
        return interpolatePoint(
          segment_start, segment_end, remaining / std::max(0.01, segment_length));
      }
      remaining -= segment_length;
      segment_start = segment_end;
    }
    return path.back();
  }

  PathSample samplePathTarget(
    const std::vector<Vec3> & path, const PathProjection & projection,
    double target_distance) const
  {
    PathSample sample;
    sample.point = path.empty() ? latest_odom_ : path.back();
    sample.path_index = path.empty() ? 0 : path.size() - 1;
    sample.lookahead = std::max(0.0, target_distance);
    if (path.size() < 2) {
      return sample;
    }

    double remaining = sample.lookahead;
    Vec3 segment_start = projection.point;
    for (std::size_t i = projection.segment_index + 1; i < path.size(); ++i) {
      const double segment_length = distance2d(segment_start, path[i]);
      if (remaining <= segment_length) {
        sample.point = interpolatePoint(
          segment_start, path[i], remaining / std::max(0.01, segment_length));
        sample.path_index = i;
        return sample;
      }
      remaining -= segment_length;
      segment_start = path[i];
    }
    return sample;
  }

  double cornerAwareLookahead(
    const std::vector<Vec3> & path, const PathProjection & projection) const
  {
    double traveled = distance2d(projection.point, path[projection.segment_index + 1]);
    Vec3 previous = projection.point;
    Vec3 direction{
      path[projection.segment_index + 1].x - previous.x,
      path[projection.segment_index + 1].y - previous.y, 0.0};
    double previous_yaw = std::atan2(direction.y, direction.x);
    for (std::size_t i = projection.segment_index + 2; i < path.size(); ++i) {
      const double segment_length = distance2d(path[i - 1], path[i]);
      if (traveled > ego_local_goal_lookahead_m_) {
        break;
      }
      const double yaw = std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
      if (std::abs(wrapAngle(yaw - previous_yaw)) > 0.70) {
        return std::clamp(
          traveled, ego_local_goal_min_distance_m_, ego_local_goal_lookahead_m_);
      }
      traveled += segment_length;
      previous_yaw = yaw;
    }
    return ego_local_goal_lookahead_m_;
  }

  bool publishRollingLocalGoal(
    const std::vector<Vec3> & path, const PathProjection & projection)
  {
    if (!publish_ego_local_goal_ || !ego_local_goal_pub_ || path.size() < 2) {
      return !publish_ego_local_goal_;
    }
    if (have_local_goal_ && !local_goal_reached_logged_) {
      const double local_goal_distance = distance2d(latest_odom_, last_local_goal_);
      if (local_goal_distance <= ego_local_goal_reached_radius_m_) {
        local_goal_reached_logged_ = true;
        RCLCPP_INFO(
          get_logger(),
          "[LOCAL_GOAL_REACHED] local_goal_seq=%lu distance=%.3f",
          local_goal_seq_, local_goal_distance);
      }
    }
    if (ageSeconds(last_local_goal_publish_time_) < local_goal_update_min_interval_sec_) {
      return true;
    }

    PathProjection forward_projection = projection;
    if (forward_projection.segment_index < local_goal_progress_index_) {
      forward_projection.segment_index = std::min(local_goal_progress_index_, path.size() - 2);
      forward_projection.segment_ratio = 0.0;
      forward_projection.point = path[forward_projection.segment_index];
    }
    local_goal_progress_index_ = std::max(
      local_goal_progress_index_, forward_projection.segment_index);

    const double remaining_path = remainingPathDistance(path, forward_projection);
    const bool force_final_approach =
      have_local_goal_ && local_goal_reached_logged_ &&
      race_super_planner_ros2::shouldPublishFinalApproach(
      remaining_path, local_goal_update_distance_m_,
      distance2d(last_local_goal_, path.back()), goal_update_position_tolerance_m_,
      final_goal_reached_latched_, final_approach_published_);
    const bool trajectory_prefetch_due = have_local_goal_ &&
      last_validated_local_goal_seq_ >= local_goal_seq_ &&
      validatedTrajectoryLeaseRemaining() <= trajectory_prefetch_sec_;

    const double preferred = cornerAwareLookahead(path, forward_projection);
    std::optional<PathSample> selected;
    for (double distance = preferred;
      distance >= ego_local_goal_min_distance_m_ - 1.0e-6; distance -= 0.15)
    {
      PathSample candidate = samplePathTarget(path, forward_projection, distance);
      candidate.point.z = clampHeight(candidate.point.z);
      if (isPointContinuouslySafe(candidate.point, 0.0).safe &&
        isSegmentContinuouslySafe(latest_odom_, candidate.point, 0.0).safe)
      {
        selected = candidate;
        break;
      }
    }
    if (!selected.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No collision-free EGO local goal in [%.2f, %.2f] m",
        ego_local_goal_min_distance_m_, preferred);
      return false;
    }

    std::vector<Vec3> reference_path;
    reference_path.reserve(selected->path_index - forward_projection.segment_index + 2);
    reference_path.push_back(forward_projection.point);
    for (std::size_t index = forward_projection.segment_index + 1;
      index < selected->path_index && index < path.size(); ++index)
    {
      reference_path.push_back(path[index]);
    }
    if (reference_path.empty() ||
      distance2d(reference_path.back(), selected->point) > 1.0e-6)
    {
      reference_path.push_back(selected->point);
    }

    double reference_arc_length = 0.0;
    double reference_minimum_clearance = std::numeric_limits<double>::infinity();
    bool reference_safe = reference_path.size() >= 2;
    for (std::size_t index = 0; reference_safe && index < reference_path.size(); ++index) {
      if (!insideSharedBounds(reference_path[index])) {
        reference_safe = false;
        break;
      }
      const auto point_check = isPointContinuouslySafe(reference_path[index], 0.0);
      reference_minimum_clearance = std::min(
        reference_minimum_clearance, point_check.clearance);
      reference_safe = point_check.safe;
      if (index == 0) {
        continue;
      }
      reference_arc_length += distance2d(reference_path[index - 1], reference_path[index]);
      const auto segment_check = isSegmentContinuouslySafe(
        reference_path[index - 1], reference_path[index], 0.0);
      reference_minimum_clearance = std::min(
        reference_minimum_clearance, segment_check.minimum_clearance);
      reference_safe = segment_check.safe;
    }
    if (!reference_safe) {
      RCLCPP_ERROR(
        get_logger(),
        "[SUPER_LOCAL_REFERENCE_REJECT] path_id=%lu next_seq=%lu points=%zu "
        "minimum_clearance=%.3f",
        global_path_id_, local_goal_seq_ + 1, reference_path.size(),
        reference_minimum_clearance);
      return false;
    }

    // Recovery must follow the real global polyline after the rolling goal.
    // Keep this continuation separate so normal EGO initialization still ends
    // exactly at selected->point and cannot overshoot then backtrack.
    std::vector<Vec3> continuation_path;
    continuation_path.push_back(selected->point);
    double preview_remaining = ego_reference_preview_distance_m_;
    Vec3 preview_start = selected->point;
    for (std::size_t index = selected->path_index;
      index < path.size() && preview_remaining > 1.0e-6; ++index)
    {
      const Vec3 & preview_end = path[index];
      const double segment_length = distance2d(preview_start, preview_end);
      if (segment_length <= 1.0e-9) {
        preview_start = preview_end;
        continue;
      }
      if (segment_length <= preview_remaining + 1.0e-9) {
        continuation_path.push_back(preview_end);
        preview_remaining -= segment_length;
        preview_start = preview_end;
        continue;
      }
      continuation_path.push_back(
        interpolatePoint(
          preview_start, preview_end, preview_remaining / segment_length));
      preview_remaining = 0.0;
    }

    selected->path_index = std::max(selected->path_index, local_goal_progress_index_);
    const bool first_for_path = !have_local_goal_;
    const double selected_progress = pathDistanceToIndex(path, selected->path_index);
    const bool index_advanced = selected->path_index > last_local_goal_index_ &&
      selected_progress - last_local_goal_path_distance_ >= local_goal_update_distance_m_;
    const bool position_changed =
      distance2d(selected->point, last_local_goal_) >= local_goal_update_distance_m_;
    const bool prefetch_advanced = trajectory_prefetch_due &&
      distance2d(selected->point, last_local_goal_) >= 0.05;
    const bool advanced = first_for_path || index_advanced || position_changed ||
      force_final_approach || prefetch_advanced;
    if (!advanced) {
      return true;
    }

    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = world_frame_;
    message.pose.position.x = selected->point.x;
    message.pose.position.y = selected->point.y;
    message.pose.position.z = selected->point.z;
    const double yaw = pathYawEnu(path, selected->path_index);
    message.pose.orientation.z = std::sin(yaw * 0.5);
    message.pose.orientation.w = std::cos(yaw * 0.5);

    const uint64_t next_local_goal_seq = local_goal_seq_ + 1;
    race_msgs::msg::LocalPathReference reference_message;
    reference_message.header = message.header;
    reference_message.global_path_id = global_path_id_;
    reference_message.local_goal_seq = next_local_goal_seq;
    reference_message.local_goal = message.pose.position;
    reference_message.arc_length = reference_arc_length;
    reference_message.minimum_clearance = reference_minimum_clearance;
    reference_message.points.reserve(reference_path.size());
    for (const auto & point : reference_path) {
      geometry_msgs::msg::Point output;
      output.x = point.x;
      output.y = point.y;
      output.z = point.z;
      reference_message.points.push_back(output);
    }
    reference_message.continuation_points.reserve(continuation_path.size());
    for (const auto & point : continuation_path) {
      geometry_msgs::msg::Point output;
      output.x = point.x;
      output.y = point.y;
      output.z = point.z;
      reference_message.continuation_points.push_back(output);
    }
    ego_reference_path_pub_->publish(reference_message);
    ego_local_goal_pub_->publish(message);

    const double previous_goal_to_end = have_local_goal_ ?
      distance2d(last_local_goal_, path.back()) :
      std::numeric_limits<double>::infinity();
    last_local_goal_index_ = selected->path_index;
    last_local_goal_path_distance_ = selected_progress;
    last_local_goal_ = selected->point;
    have_local_goal_ = true;
    ++local_goal_seq_;
    local_goal_reached_logged_ = false;
    if (force_final_approach) {
      final_approach_published_ = true;
      RCLCPP_WARN(
        get_logger(),
        "[FINAL_APPROACH] path_id=%lu seq=%lu remaining_path=%.3f "
        "previous_goal_to_end=%.3f",
        global_path_id_, local_goal_seq_, remaining_path,
        previous_goal_to_end);
    }
    clearFinalGoalReached("new_local_goal");
    last_local_goal_publish_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "[LOCAL_GOAL] path_id=%lu seq=%lu index=%zu position=(%.3f,%.3f,%.3f) lookahead=%.2f",
      global_path_id_, local_goal_seq_, selected->path_index,
      selected->point.x, selected->point.y, selected->point.z, selected->lookahead);
    RCLCPP_INFO(
      get_logger(),
      "[SUPER_LOCAL_REFERENCE] global_path_id=%lu local_goal_seq=%lu points=%zu "
      "continuation_points=%zu arc_length=%.3f minimum_clearance=%.3f",
      global_path_id_, local_goal_seq_, reference_path.size(), continuation_path.size(),
      reference_arc_length, reference_minimum_clearance);
    return true;
  }

  double pathDistanceToIndex(const std::vector<Vec3> & path, std::size_t index) const
  {
    double distance = 0.0;
    const std::size_t end = std::min(index, path.size() - 1);
    for (std::size_t i = 1; i <= end; ++i) {
      distance += distance2d(path[i - 1], path[i]);
    }
    return distance;
  }

  double remainingPathDistance(
    const std::vector<Vec3> & path, const PathProjection & projection) const
  {
    if (path.size() < 2 || projection.segment_index + 1 >= path.size()) {
      return 0.0;
    }
    double remaining = distance2d(
      projection.point, path[projection.segment_index + 1]);
    for (std::size_t index = projection.segment_index + 2; index < path.size(); ++index) {
      remaining += distance2d(path[index - 1], path[index]);
    }
    return remaining;
  }

  double pathLength(const std::vector<Vec3> & path) const
  {
    double length = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index) {
      length += distance2d(path[index - 1], path[index]);
    }
    return length;
  }

  std::optional<PathSample> selectSafeTrackingTarget(
    const std::vector<Vec3> & path, const PathProjection & projection,
    bool & narrow_corridor, bool & centering_only, double & corridor_clearance) const
  {
    const PathSample nominal_target =
      samplePathTarget(path, projection, tracking_lookahead_distance_);
    corridor_clearance = distanceToNearestRawObstacle(nominal_target.point);
    const double corridor_threshold =
      vehicle_collision_radius_ + narrow_corridor_tracking_margin_ + resolution_ * 0.5;
    narrow_corridor = corridor_clearance <= corridor_threshold;
    centering_only = narrow_corridor &&
      projection.cross_track_error > narrow_corridor_centering_tolerance_;
    const double preferred_lookahead = centering_only ? 0.0 :
      (narrow_corridor ? narrow_corridor_lookahead_distance_ : tracking_lookahead_distance_);
    const std::array<double, 5> lookahead_candidates{
      preferred_lookahead,
      preferred_lookahead * 0.75,
      preferred_lookahead * 0.50,
      preferred_lookahead * 0.25,
      0.0};

    double previous_distance = -1.0;
    for (const double candidate_distance : lookahead_candidates) {
      const double distance = std::max(0.0, candidate_distance);
      if (std::abs(distance - previous_distance) < 1.0e-6) {
        continue;
      }
      previous_distance = distance;
      PathSample candidate = samplePathTarget(path, projection, distance);
      candidate.point.z = clampHeight(candidate.point.z);
      if (isSegmentContinuouslySafe(latest_odom_, candidate.point, 0.0).safe) {
        return candidate;
      }
    }
    return std::nullopt;
  }

  std::optional<double> desiredPathYawNed(
    const std::vector<Vec3> & path, const PathProjection & projection) const
  {
    if (path.size() < 2) {
      return std::nullopt;
    }

    const Vec3 heading_point =
      samplePathFromProjection(path, projection, heading_lookahead_distance_);
    Vec3 tangent{
      heading_point.x - projection.point.x, heading_point.y - projection.point.y, 0.0};

    if (std::hypot(tangent.x, tangent.y) < 0.05) {
      tangent = Vec3{
        path.back().x - projection.point.x, path.back().y - projection.point.y, 0.0};
    }
    if (std::hypot(tangent.x, tangent.y) < 0.05) {
      return std::nullopt;
    }

    const Vec3 tangent_ned = enuToNed(tangent);
    return std::atan2(tangent_ned.y, tangent_ned.x);
  }

  void updateYawCommand(const std::vector<Vec3> & path, const PathProjection & projection)
  {
    const auto desired_yaw = desiredPathYawNed(path, projection);
    if (!desired_yaw.has_value()) {
      commanded_yaw_rate_ = 0.0;
      return;
    }

    const rclcpp::Time update_time = now();
    if (!have_commanded_yaw_) {
      commanded_yaw_ned_ = have_current_yaw_ ? current_yaw_ned_ : desired_yaw.value();
      have_commanded_yaw_ = true;
      last_yaw_update_time_ = update_time;
    }

    double dt = (update_time - last_yaw_update_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = 1.0 / control_rate_;
    }
    dt = std::min(dt, 0.2);

    const double yaw_error = wrapAngle(desired_yaw.value() - commanded_yaw_ned_);
    const double alpha = yaw_smoothing_time_constant_ <= 1.0e-6 ?
      1.0 : dt / (yaw_smoothing_time_constant_ + dt);
    const double filtered_step = alpha * yaw_error;
    commanded_yaw_rate_ = std::clamp(filtered_step / dt, -max_yaw_rate_, max_yaw_rate_);
    commanded_yaw_ned_ = wrapAngle(commanded_yaw_ned_ + commanded_yaw_rate_ * dt);
    desired_yaw_ned_ = desired_yaw.value();
    last_yaw_update_time_ = update_time;
  }

  const char * pathValidationFailureName(
    const race_super_planner_ros2::PathValidationFailure failure) const
  {
    using race_super_planner_ros2::PathValidationFailure;
    switch (failure) {
      case PathValidationFailure::NONE:
        return "NONE";
      case PathValidationFailure::EMPTY:
        return "EMPTY";
      case PathValidationFailure::NON_FINITE:
        return "NON_FINITE";
      case PathValidationFailure::OUT_OF_BOUNDS:
        return "OUT_OF_BOUNDS";
      case PathValidationFailure::COLLISION:
        return "COLLISION";
      case PathValidationFailure::SMOOTHING_VALIDATION:
        return "SMOOTHING_VALIDATION";
    }
    return "UNKNOWN";
  }

  race_super_planner_ros2::PathValidationResult validateCompletePath(
    const std::vector<Vec3> & path, const char * path_type) const
  {
    using race_super_planner_ros2::PathValidationFailure;
    using race_super_planner_ros2::PathValidationResult;

    if (path.size() < 2) {
      return {PathValidationFailure::EMPTY, 0};
    }
    for (std::size_t index = 0; index < path.size(); ++index) {
      const Vec3 & point = path[index];
      if (!finite_vec(point)) {
        return {PathValidationFailure::NON_FINITE, index};
      }
      if (!insideSharedBounds(point)) {
        return {PathValidationFailure::OUT_OF_BOUNDS, index};
      }
      if (index == 0) {
        const PointSafetyResult start_check =
          isPointContinuouslySafe(point, smoothing_min_clearance_);
        if (!start_check.safe) {
          if (std::string(path_type) == "RAW") {
            SegmentSafetyResult start_failure;
            start_failure.failure = start_check.failure;
            start_failure.failure_point = start_check.query_point;
            start_failure.nearest_obstacle = start_check.nearest_obstacle;
            start_failure.clearance = start_check.clearance;
            start_failure.minimum_clearance = start_check.clearance;
            start_failure.sample_count = 0;
            start_failure.sample_step = 0.0;
            logRawStartPointCollision(point, start_failure);
          }
          return {PathValidationFailure::COLLISION, index};
        }
        if (!isPlanningCell(worldToCell(point))) {
          return {PathValidationFailure::COLLISION, index};
        }
      }
      if (index > 0) {
        const SegmentSafetyResult segment = isSegmentContinuouslySafe(
          path[index - 1], point, smoothing_min_clearance_);
        if (!segment.safe) {
          logPathSegmentCollision(path, path_type, index, segment);
          return {PathValidationFailure::COLLISION, index};
        }
        if (!isSegmentSafeForAstar(path[index - 1], point)) {
          return {PathValidationFailure::COLLISION, index};
        }
      }
    }
    return {};
  }

  void logPathSegmentCollision(
    const std::vector<Vec3> & path, const char * path_type, std::size_t segment_index,
    const SegmentSafetyResult & result) const
  {
    const Vec3 & start = path[segment_index - 1];
    const Vec3 & end = path[segment_index];
    const bool raw_path = std::string(path_type) == "RAW";
    const char * source = raw_path ? "ASTAR_ADJACENT_EDGE" : "PATH_SIMPLIFICATION_EDGE";
    bool checked_during_astar = raw_path;
    if (raw_path && segment_index == 1) {
      source = "START_ENDPOINT_EDGE";
      checked_during_astar = false;
    } else if (raw_path && segment_index + 1 == path.size()) {
      source = "FINAL_GOAL_ENDPOINT_EDGE";
      checked_during_astar = false;
    }
    RCLCPP_WARN(
      get_logger(),
      "[SUPER_SEGMENT_COLLISION] path_type=%s segment_index=%zu "
      "segment_start=(%.3f,%.3f,%.3f) segment_end=(%.3f,%.3f,%.3f) "
      "segment_length=%.3f sample_index=%zu sample_position=(%.3f,%.3f,%.3f) "
      "nearest_obstacle=(%.3f,%.3f,%.3f) clearance=%.3f required_clearance=%.3f "
      "sample_step=%.3f source=%s astar_checked=%s astar_checker_same=true",
      path_type, segment_index, start.x, start.y, start.z, end.x, end.y, end.z,
      distance2d(start, end), result.sample_index, result.failure_point.x,
      result.failure_point.y, result.failure_point.z, result.nearest_obstacle.x,
      result.nearest_obstacle.y, result.nearest_obstacle.z, result.clearance,
      requiredContinuousClearance(), result.sample_step, source,
      checked_during_astar ? "true" : "false");
  }

  void logRawStartPointCollision(const Vec3 & point, const SegmentSafetyResult & result) const
  {
    RCLCPP_WARN(
      get_logger(),
      "[SUPER_SEGMENT_COLLISION] path_type=RAW segment_index=0 "
      "segment_start=(%.3f,%.3f,%.3f) segment_end=(%.3f,%.3f,%.3f) "
      "segment_length=0.000 sample_index=0 sample_position=(%.3f,%.3f,%.3f) "
      "nearest_obstacle=(%.3f,%.3f,%.3f) clearance=%.3f required_clearance=%.3f "
      "sample_step=0.000 source=START_ENDPOINT_EDGE astar_checked=false astar_checker_same=true",
      point.x, point.y, point.z, point.x, point.y, point.z,
      result.failure_point.x, result.failure_point.y, result.failure_point.z,
      result.nearest_obstacle.x, result.nearest_obstacle.y, result.nearest_obstacle.z,
      result.clearance, requiredContinuousClearance());
  }

  void logSmoothedPathReject(
    const race_super_planner_ros2::PathValidationResult & validation,
    const std::vector<Vec3> & path)
  {
    using race_super_planner_ros2::PathValidationFailure;
    // The smoothing branch leaves smoothed_validation at its SMOOTHING_VALIDATION
    // default and smoothed_path empty whenever obstacleAwareSmooth() produced no
    // change (result.changed == false), which is the ordinary "already smooth"
    // outcome, not a geometric rejection. Reporting a synthesised Vec3{} for that
    // case printed point=(0.000,0.000,0.000) and read as a rejection at the path
    // origin. Say plainly that no candidate exists instead.
    if (path.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "[SUPER_SMOOTHED_PATH_REJECT] reason=%s no_smoothed_candidate=true "
        "(smoothing produced no path; raw path is used unchanged)",
        pathValidationFailureName(validation.failure));
      return;
    }
    const Vec3 point = validation.index < path.size() ? path[validation.index] : Vec3{};
    if (validation.failure == PathValidationFailure::OUT_OF_BOUNDS) {
      const char * axis = "X";
      double bound = shared_bounds_x_max_;
      double overshoot = 0.0;
      if (point.x < shared_bounds_x_min_) {
        bound = shared_bounds_x_min_;
        overshoot = shared_bounds_x_min_ - point.x;
      } else if (point.x > shared_bounds_x_max_) {
        bound = shared_bounds_x_max_;
        overshoot = point.x - shared_bounds_x_max_;
      } else if (point.y < shared_bounds_y_min_) {
        axis = "Y";
        bound = shared_bounds_y_min_;
        overshoot = shared_bounds_y_min_ - point.y;
      } else if (point.y > shared_bounds_y_max_) {
        axis = "Y";
        bound = shared_bounds_y_max_;
        overshoot = point.y - shared_bounds_y_max_;
      } else if (point.z < effectiveSharedMinHeight()) {
        axis = "Z";
        bound = effectiveSharedMinHeight();
        overshoot = effectiveSharedMinHeight() - point.z;
      } else {
        axis = "Z";
        bound = effectiveSharedMaxHeight();
        overshoot = point.z - effectiveSharedMaxHeight();
      }
      RCLCPP_WARN(
        get_logger(),
        "[SUPER_SMOOTHED_PATH_REJECT] reason=OUT_OF_BOUNDS index=%zu point=(%.3f,%.3f,%.3f) "
        "axis=%s bound=%.3f overshoot=%.3f",
        validation.index, point.x, point.y, point.z, axis, bound, overshoot);
      return;
    }
    if (validation.failure == PathValidationFailure::COLLISION) {
      RCLCPP_WARN(
        get_logger(),
        "[SUPER_SMOOTHED_PATH_REJECT] reason=COLLISION index=%zu point=(%.3f,%.3f,%.3f)",
        validation.index, point.x, point.y, point.z);
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "[SUPER_SMOOTHED_PATH_REJECT] reason=%s index=%zu point=(%.3f,%.3f,%.3f)",
      pathValidationFailureName(validation.failure), validation.index, point.x, point.y, point.z);
  }

  std::vector<race_super_planner_ros2::smoothing::Point3> toSmoothingPath(
    const std::vector<Vec3> & path) const
  {
    std::vector<race_super_planner_ros2::smoothing::Point3> output;
    output.reserve(path.size());
    for (const auto & point : path) {
      output.push_back({point.x, point.y, point.z});
    }
    return output;
  }

  std::vector<Vec3> fromSmoothingPath(
    const std::vector<race_super_planner_ros2::smoothing::Point3> & path) const
  {
    std::vector<Vec3> output;
    output.reserve(path.size());
    for (const auto & point : path) {
      output.push_back({point.x, point.y, point.z});
    }
    return output;
  }

  race_super_planner_ros2::smoothing::ClearanceAssessment assessSmoothingCandidate(
    const std::vector<Vec3> & raw_baseline,
    const std::vector<Vec3> & candidate,
    const double numeric_tolerance) const
  {
    const auto clearance = [this](
      const race_super_planner_ros2::smoothing::Point3 & point)
      {
        return distanceToNearestRawObstacle(
          Vec3{point.x, point.y, clampHeight(effectiveFixedFlightHeight())});
      };
    const auto segment_safe = [this](
      const race_super_planner_ros2::smoothing::Point3 & start,
      const race_super_planner_ros2::smoothing::Point3 & finish)
      {
        return isSegmentSafeForAstar(
          Vec3{start.x, start.y, start.z}, Vec3{finish.x, finish.y, finish.z});
      };
    return race_super_planner_ros2::smoothing::assess(
      toSmoothingPath(raw_baseline), toSmoothingPath(candidate),
      clearance, segment_safe, std::min(planning_resolution_ * 0.5, 0.02),
      numeric_tolerance);
  }

  race_super_planner_ros2::smoothing::SmoothingResult obstacleAwareSmooth(
    const std::vector<Vec3> & raw_baseline,
    const int iterations,
    const double numeric_tolerance) const
  {
    const auto clearance = [this](
      const race_super_planner_ros2::smoothing::Point3 & point)
      {
        return distanceToNearestRawObstacle(
          Vec3{point.x, point.y, clampHeight(effectiveFixedFlightHeight())});
      };
    const auto segment_safe = [this](
      const race_super_planner_ros2::smoothing::Point3 & start,
      const race_super_planner_ros2::smoothing::Point3 & finish)
      {
        return isSegmentSafeForAstar(
          Vec3{start.x, start.y, start.z}, Vec3{finish.x, finish.y, finish.z});
      };
    return race_super_planner_ros2::smoothing::acceptedStepSmooth(
      toSmoothingPath(raw_baseline), iterations, clearance, segment_safe,
      std::min(planning_resolution_ * 0.5, 0.02), numeric_tolerance);
  }

  double maximumHeadingJump(const std::vector<Vec3> & path) const
  {
    double maximum = 0.0;
    if (path.size() < 3) {
      return maximum;
    }
    double previous_heading = std::atan2(
      path[1].y - path[0].y, path[1].x - path[0].x);
    for (std::size_t index = 2; index < path.size(); ++index) {
      const double heading = std::atan2(
        path[index].y - path[index - 1].y,
        path[index].x - path[index - 1].x);
      maximum = std::max(maximum, std::abs(wrapAngle(heading - previous_heading)));
      previous_heading = heading;
    }
    return maximum;
  }

  bool processPath(const std::vector<Vec3> & raw_path, std::vector<Vec3> & final_path)
  {
    using race_super_planner_ros2::PathSource;
    using race_super_planner_ros2::PathValidationFailure;
    using race_super_planner_ros2::PathValidationResult;

    final_path.clear();
    constexpr double clearance_numeric_tolerance = 1.0e-6;
    const PathValidationResult raw_validation = validateCompletePath(raw_path, "RAW");
    // The reconstructed A* cell chain is the immutable safety baseline.  A
    // shortcut is only an optional smoothing proposal and cannot silently
    // replace RAW when its clearance is lower.
    std::vector<Vec3> simplified_path = raw_path;
    bool shortcut_generated = false;
    bool shortcut_clearance_preserved = true;
    PathValidationResult simplified_validation = raw_validation;
    if (enable_path_shortcut_) {
      shortcut_generated = true;
      simplified_path = visibilitySimplifyPath(raw_path);
      simplified_validation = validateCompletePath(simplified_path, "SIMPLIFIED_RAW");
      const auto shortcut_assessment = assessSmoothingCandidate(
        raw_path, simplified_path, clearance_numeric_tolerance);
      shortcut_clearance_preserved = shortcut_assessment.accepted;
      if (!shortcut_clearance_preserved) {
        simplified_validation.failure = PathValidationFailure::SMOOTHING_VALIDATION;
      }
    }
    const std::vector<Vec3> & safe_raw_path =
      enable_path_shortcut_ && simplified_validation.valid() ? simplified_path : raw_path;
    const PathValidationResult & safe_raw_validation =
      enable_path_shortcut_ && simplified_validation.valid() ?
      simplified_validation : raw_validation;
    std::vector<Vec3> smoothed_path;
    PathValidationResult smoothed_validation{PathValidationFailure::SMOOTHING_VALIDATION, 0};
    race_super_planner_ros2::smoothing::ClearanceAssessment smoothing_assessment;
    bool smoothing_generated = false;
    bool smoothing_geometry_improved = false;
    std::size_t smoothing_updates = 0;
    std::size_t smoothing_iterations_accepted = 0;
    std::string fallback_reason = "SMOOTHING_DISABLED";

    if (enable_path_smoothing_) {
      const auto result = obstacleAwareSmooth(
        safe_raw_path, smoothing_iterations_, clearance_numeric_tolerance);
      smoothing_updates = result.accepted_updates;
      smoothing_iterations_accepted = result.accepted_iterations;
      smoothing_generated = result.changed;
      const std::vector<Vec3> smoothing_result_path = fromSmoothingPath(result.path);
      smoothing_assessment = assessSmoothingCandidate(
        raw_path, smoothing_result_path, clearance_numeric_tolerance);
      if (smoothing_generated) {
        smoothed_path = smoothing_result_path;
        smoothed_validation = validateCompletePath(smoothed_path, "SMOOTHED");
        if (smoothed_validation.valid() &&
          (!smoothing_assessment.accepted ||
          !validateSmoothedPath(raw_path, smoothed_path)))
        {
          smoothed_validation.failure = PathValidationFailure::SMOOTHING_VALIDATION;
        }
        smoothing_geometry_improved =
          pathLength(smoothed_path) + 1.0e-6 < pathLength(safe_raw_path) ||
          maximumHeadingJump(smoothed_path) + 1.0e-6 <
          maximumHeadingJump(safe_raw_path);
        if (!smoothing_geometry_improved) {
          smoothed_validation.failure = PathValidationFailure::SMOOTHING_VALIDATION;
          fallback_reason = "NO_GEOMETRIC_IMPROVEMENT";
        } else if (!smoothing_assessment.accepted) {
          fallback_reason = "CLEARANCE_NOT_PRESERVED";
        } else if (!smoothed_validation.valid()) {
          fallback_reason = "SMOOTHED_PATH_UNSAFE";
        } else {
          fallback_reason = "NONE";
        }
      } else {
        fallback_reason = "NO_CLEARANCE_PRESERVING_UPDATE";
      }
    } else {
      last_smoothing_result_ = "disabled";
    }

    RCLCPP_INFO(
      get_logger(),
      "[SUPER_CONTINUOUS_PATH_CHECK] source=RAW_ASTAR min_clearance=%.3f required_clearance=%.3f valid=%s",
      minimumContinuousClearance(raw_path), requiredContinuousClearance(),
      raw_validation.valid() ? "true" : "false");
    if (shortcut_generated) {
      RCLCPP_INFO(
        get_logger(),
        "[SUPER_ASTAR_VISIBILITY_SIMPLIFY] raw_points=%zu simplified_points=%zu "
        "raw_length=%.3f simplified_length=%.3f minimum_clearance=%.3f accepted=%s "
        "clearance_preserved=%s",
        raw_path.size(), simplified_path.size(), pathLength(raw_path), pathLength(simplified_path),
        minimumContinuousClearance(simplified_path),
        simplified_validation.valid() ? "true" : "false",
        shortcut_clearance_preserved ? "true" : "false");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "[SUPER_ASTAR_VISIBILITY_SIMPLIFY] raw_points=%zu simplified_points=%zu "
        "raw_length=%.3f simplified_length=%.3f minimum_clearance=%.3f accepted=false "
        "clearance_preserved=true reason=DISABLED",
        raw_path.size(), raw_path.size(), pathLength(raw_path), pathLength(raw_path),
        minimumContinuousClearance(raw_path));
    }
    if (!smoothed_path.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "[SUPER_CONTINUOUS_PATH_CHECK] source=SMOOTHED min_clearance=%.3f required_clearance=%.3f valid=%s",
        minimumContinuousClearance(smoothed_path), requiredContinuousClearance(),
        smoothed_validation.valid() ? "true" : "false");
    }

    const PathSource source = race_super_planner_ros2::selectValidatedPath(
      smoothed_validation, safe_raw_validation);
    if (source == PathSource::SMOOTHED) {
      final_path = smoothed_path;
      last_smoothing_result_ = "accepted";
      fallback_reason = "NONE";
      RCLCPP_INFO(
        get_logger(), "[SUPER_PATH_SELECT] source=SMOOTHED points=%zu goal_id=%lu",
        final_path.size(), global_goal_id_);
    } else if (source == PathSource::RAW_ASTAR) {
      if (enable_path_smoothing_ && smoothing_method_ == "chaikin") {
        logSmoothedPathReject(smoothed_validation, smoothed_path);
      }
      final_path = safe_raw_path;
      last_smoothing_result_ = "fallback_raw_astar";
      RCLCPP_WARN(
        get_logger(), "[SUPER_PATH_FALLBACK] source=RAW_ASTAR raw_points=%zu goal_id=%lu",
        safe_raw_path.size(), global_goal_id_);
    } else {
      RCLCPP_ERROR(
        get_logger(), "[SUPER_PATH_FAILURE] smoothed_reason=%s raw_reason=%s goal_id=%lu",
        pathValidationFailureName(smoothed_validation.failure),
        pathValidationFailureName(raw_validation.failure), global_goal_id_);
      last_smoothing_result_ = "raw_and_smoothed_rejected";
    }

    const char * selected_name =
      source == PathSource::SMOOTHED ? "SMOOTHED" :
      source == PathSource::RAW_ASTAR ? "RAW" : "NONE";
    const bool selected_topology_safe =
      source != PathSource::NONE &&
      (source == PathSource::RAW_ASTAR || smoothing_assessment.accepted);
    RCLCPP_INFO(
      get_logger(),
      "[CLEARANCE_PRESERVATION_POLICY] raw_global_min=%.9f "
      "smoothed_global_min=%.9f global_clearance_loss=%.9f "
      "local_comparison_method=NORMALIZED_ARC_PROGRESS_0.02M "
      "local_maximum_loss=%.9f numeric_tolerance=%.9f",
      minimumContinuousClearance(raw_path),
      smoothed_path.empty() ? std::numeric_limits<double>::quiet_NaN() :
      minimumContinuousClearance(smoothed_path),
      smoothed_path.empty() ? std::numeric_limits<double>::quiet_NaN() :
      minimumContinuousClearance(raw_path) - minimumContinuousClearance(smoothed_path),
      smoothing_assessment.maximum_progress_matched_loss,
      clearance_numeric_tolerance);
    RCLCPP_INFO(
      get_logger(),
      "[PATH_TOPOLOGY_GUARD] raw_obstacle_side=BASELINE "
      "smoothed_obstacle_side=CONTINUOUS_DEFORMATION same_homotopy=%s",
      selected_topology_safe ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "[SUPER_PATH_SELECTION] candidate_smoothing_generated=%s "
      "candidate_smoothing_safe=%s clearance_preserved=%s "
      "accepted_updates=%zu accepted_iterations=%zu selected=%s fallback_reason=%s",
      smoothing_generated ? "true" : "false",
      smoothed_validation.valid() ? "true" : "false",
      smoothing_assessment.accepted ? "true" : "false",
      smoothing_updates, smoothing_iterations_accepted, selected_name,
      source == PathSource::SMOOTHED ? "NONE" : fallback_reason.c_str());

    bool selected_finite = !final_path.empty();
    double selected_maximum_segment_length = 0.0;
    for (std::size_t index = 0; index < final_path.size(); ++index) {
      selected_finite = selected_finite && finite_vec(final_path[index]);
      if (index > 0) {
        selected_maximum_segment_length = std::max(
          selected_maximum_segment_length,
          distance2d(final_path[index - 1], final_path[index]));
      }
    }
    const PathValidationResult selected_validation =
      final_path.empty() ?
      PathValidationResult{PathValidationFailure::EMPTY, 0} :
    validateCompletePath(final_path, "SELECTED");
    RCLCPP_INFO(
      get_logger(),
      "[FINAL_SELECTED_PATH_VALIDATION] selected_type=%s min_clearance=%.9f "
      "inflated_collision=%s continuous_clearance_pass=%s finite=%s "
      "maximum_segment_length=%.6f",
      selected_name,
      final_path.empty() ? std::numeric_limits<double>::quiet_NaN() :
      minimumContinuousClearance(final_path),
      selected_validation.valid() ? "false" : "true",
      selected_validation.valid() ? "true" : "false",
      selected_finite ? "true" : "false", selected_maximum_segment_length);

    last_raw_path_size_ = raw_path.size();
    last_shortcut_path_size_ = simplified_path.size();
    last_smoothed_path_size_ = smoothed_path.size();
    RCLCPP_INFO(
      get_logger(),
      "path_pipeline raw=%zu shortcut=%zu smoothed=%zu smoothing=%s",
      raw_path.size(), simplified_path.size(), smoothed_path.size(),
      last_smoothing_result_.c_str());
    return source != PathSource::NONE;
  }

  std::vector<Vec3> visibilitySimplifyPath(const std::vector<Vec3> & raw_path) const
  {
    if (raw_path.size() <= 2) {
      return raw_path;
    }

    std::vector<Vec3> shortcut;
    shortcut.push_back(raw_path.front());
    std::size_t current = 0;
    while (current < raw_path.size() - 1) {
      std::size_t best = current + 1;
      for (std::size_t candidate = raw_path.size() - 1; candidate > current; --candidate) {
        if (isSegmentSafeForAstar(raw_path[current], raw_path[candidate])) {
          best = candidate;
          break;
        }
      }
      shortcut.push_back(raw_path[best]);
      current = best;
    }
    return shortcut;
  }

  std::vector<Vec3> densifyPath(const std::vector<Vec3> & path) const
  {
    if (path.size() <= 1 || path_sample_resolution_ <= 1.0e-6) {
      return path;
    }

    std::vector<Vec3> dense;
    dense.push_back(path.front());
    for (std::size_t i = 1; i < path.size(); ++i) {
      const Vec3 & a = path[i - 1];
      const Vec3 & b = path[i];
      const double segment_len = distance2d(a, b);
      const int steps =
        std::max(1, static_cast<int>(std::ceil(segment_len / path_sample_resolution_)));
      for (int step = 1; step <= steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(steps);
        dense.push_back(interpolatePoint(a, b, t));
      }
    }
    return dense;
  }

  std::vector<Vec3> chaikinSmooth(const std::vector<Vec3> & path) const
  {
    if (path.size() <= 2) {
      return path;
    }

    std::vector<Vec3> current = path;
    const int iterations = std::max(0, smoothing_iterations_);
    for (int iter = 0; iter < iterations; ++iter) {
      if (current.size() <= 2) {
        break;
      }
      std::vector<Vec3> next;
      next.reserve(current.size() * 2);
      next.push_back(current.front());
      for (std::size_t i = 0; i + 1 < current.size(); ++i) {
        const Vec3 & p0 = current[i];
        const Vec3 & p1 = current[i + 1];
        next.push_back(interpolatePoint(p0, p1, 0.25));
        next.push_back(interpolatePoint(p0, p1, 0.75));
      }
      next.push_back(current.back());
      current = densifyPath(next);
    }
    return current;
  }

  bool validateSmoothedPath(
    const std::vector<Vec3> & reference_path,
    const std::vector<Vec3> & smoothed_path)
  {
    if (smoothed_path.size() < 2) {
      last_smoothing_result_ = "rejected: too short";
      return false;
    }

    for (const auto & point : smoothed_path) {
      if (!isPointContinuouslySafe(point, smoothing_min_clearance_).safe) {
        last_smoothing_result_ = "rejected: point collision";
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(), 1000, "Reject smoothed path: point collision");
        return false;
      }
    }

    if (smoothing_collision_check_) {
      for (std::size_t i = 1; i < smoothed_path.size(); ++i) {
        if (!isSegmentContinuouslySafe(
            smoothed_path[i - 1], smoothed_path[i],
            smoothing_min_clearance_).safe)
        {
          last_smoothing_result_ = "rejected: segment collision";
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(), 1000, "Reject smoothed path: segment collision");
          return false;
        }
      }
    }

    for (const auto & point : smoothed_path) {
      double min_dist = std::numeric_limits<double>::infinity();
      for (const auto & ref : reference_path) {
        min_dist = std::min(min_dist, distance2d(point, ref));
      }
      if (min_dist > smoothing_max_deviation_) {
        last_smoothing_result_ = "rejected: deviation too large";
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(), 1000, "Reject smoothed path: deviation %.2f > %.2f", min_dist,
          smoothing_max_deviation_);
        return false;
      }
    }

    last_smoothing_result_ = "accepted";
    return true;
  }

  Vec3 interpolatePoint(const Vec3 & a, const Vec3 & b, double t) const
  {
    t = std::max(0.0, std::min(1.0, t));
    return Vec3{
      a.x + (b.x - a.x) * t,
      a.y + (b.y - a.y) * t,
      a.z + (b.z - a.z) * t};
  }

  PointSafetyResult isPointContinuouslySafe(
    const Vec3 & point, double requested_clearance) const
  {
    PointSafetyResult result;
    result.query_point = point;
    result.query_point.z = clampHeight(effectiveFixedFlightHeight());
    if (!finite_vec(point)) {
      result.failure = SegmentSafetyFailure::NON_FINITE;
      return result;
    }
    result.nearest_obstacle = nearestRawObstacle(result.query_point);
    result.clearance = distanceToNearestRawObstacle(result.query_point);
    if (result.clearance < std::max(requiredContinuousClearance(), requested_clearance)) {
      result.failure = SegmentSafetyFailure::CONTINUOUS_CLEARANCE;
      return result;
    }
    result.safe = true;
    return result;
  }

  SegmentSafetyResult isSegmentContinuouslySafe(
    const Vec3 & a, const Vec3 & b, double requested_clearance) const
  {
    const double dist = distance2d(a, b);
    const double maximum_step = std::min(planning_resolution_ * 0.5, 0.02);
    const race_super_planner_ros2::policy::SegmentPoint2D start{a.x, a.y};
    const race_super_planner_ros2::policy::SegmentPoint2D end{b.x, b.y};
    const std::size_t sample_count =
      race_super_planner_ros2::policy::segmentSampleCount(start, end, maximum_step);
    SegmentSafetyResult result;
    result.sample_count = sample_count;
    result.sample_step = dist / static_cast<double>(sample_count);
    for (std::size_t sample_index = 0; sample_index <= sample_count; ++sample_index) {
      const auto sample_2d =
        race_super_planner_ros2::policy::segmentSample(start, end, sample_index, sample_count);
      const Vec3 sample = interpolatePoint(
        a, b, static_cast<double>(sample_index) / static_cast<double>(sample_count));
      const PointSafetyResult point = isPointContinuouslySafe(
        Vec3{sample_2d.x, sample_2d.y, sample.z}, requested_clearance);
      result.minimum_clearance = std::min(result.minimum_clearance, point.clearance);
      if (!point.safe) {
        result.failure = point.failure;
        result.sample_index = sample_index;
        result.failure_point = point.query_point;
        result.nearest_obstacle = point.nearest_obstacle;
        result.clearance = point.clearance;
        return result;
      }
    }
    result.safe = true;
    return result;
  }

  std::optional<Cell> findNearestFreeCell(const Cell & center, double max_radius_m) const
  {
    const int max_radius_cells =
      std::max(1, static_cast<int>(std::ceil(max_radius_m / planning_resolution_)));
    if (isInside(center) && !isOccupied(center)) {
      return center;
    }
    for (int radius = 1; radius <= max_radius_cells; ++radius) {
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          if (std::max(std::abs(dx), std::abs(dy)) != radius) {
            continue;
          }
          Cell candidate{center.x + dx, center.y + dy};
          if (!isInside(candidate) || isOccupied(candidate)) {
            continue;
          }
          return candidate;
        }
      }
    }
    return std::nullopt;
  }

  double pathYawEnu(const std::vector<Vec3> & path, std::size_t index) const
  {
    if (path.size() < 2) {
      return 0.0;
    }
    const Vec3 & before = path[index == 0 ? 0 : index - 1];
    const Vec3 & after = path[index + 1 < path.size() ? index + 1 : index];
    return std::atan2(after.y - before.y, after.x - before.x);
  }

  void publishPath(const std::vector<Vec3> & path)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_;
    msg.poses.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
      const auto & point = path[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      const double yaw = pathYawEnu(path, i);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      msg.poses.push_back(pose);
    }
    path_pub_->publish(msg);
    last_path_size_ = path.size();
  }

  void startNewGlobalPath(const std::vector<Vec3> & path)
  {
    ++global_path_id_;
    tracking_progress_index_ = 0;
    clearFinalGoalReached("new_global_path");
    resetLocalGoalProgress();
    if (!publish_global_path_ || !global_path_pub_) {
      return;
    }
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = world_frame_;
    message.poses.reserve(path.size());
    double length = 0.0;
    double min_x = path.empty() ? 0.0 : path.front().x;
    double max_x = min_x;
    double min_y = path.empty() ? 0.0 : path.front().y;
    double max_y = min_y;
    for (std::size_t i = 0; i < path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = path[i].x;
      pose.pose.position.y = path[i].y;
      pose.pose.position.z = path[i].z;
      const double yaw = pathYawEnu(path, i);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      message.poses.push_back(pose);
      min_x = std::min(min_x, path[i].x);
      max_x = std::max(max_x, path[i].x);
      min_y = std::min(min_y, path[i].y);
      max_y = std::max(max_y, path[i].y);
      if (i > 0) {
        length += distance2d(path[i - 1], path[i]);
      }
    }
    global_path_pub_->publish(message);
    RCLCPP_INFO(
      get_logger(),
      "[GLOBAL_PATH] id=%lu points=%zu length=%.3f bounds_x=[%.3f,%.3f] "
      "bounds_y=[%.3f,%.3f] hard_clearance=%.2f soft_radius=%.2f",
      global_path_id_, path.size(), length, min_x, max_x, min_y, max_y,
      active_inflation_radius_, soft_obstacle_cost_radius_);
  }

  void resetLocalGoalProgress()
  {
    local_goal_progress_index_ = 0;
    last_local_goal_index_ = 0;
    last_local_goal_path_distance_ = 0.0;
    have_local_goal_ = false;
    local_goal_reached_logged_ = false;
    final_approach_published_ = false;
  }

  bool controlOutputEnabled() const
  {
    return enable_output_ && !global_only_mode_;
  }

  bool insideSharedBounds(const Vec3 & point) const
  {
    return finite_vec(point) && point.x >= shared_bounds_x_min_ &&
           point.x <= shared_bounds_x_max_ &&
           point.y >= shared_bounds_y_min_ && point.y <= shared_bounds_y_max_ &&
           point.z >= effectiveSharedMinHeight() && point.z <= effectiveSharedMaxHeight();
  }

  bool insideFaultEnvelopeXY(const Vec3 & point) const
  {
    return finite_vec(point) && point.x >= shared_bounds_x_min_ &&
           point.x <= shared_bounds_x_max_ && point.y >= shared_bounds_y_min_ &&
           point.y <= shared_bounds_y_max_;
  }

  void latchFault(const std::string & reason)
  {
    if (fault_latched_) {
      return;
    }
    fault_latched_ = true;
    fault_reason_ = reason;
    active_path_.clear();
    clearPendingGlobalPath();
    pending_path_retry_exhausted_ = false;
    resetLocalGoalProgress();
    if (finite_vec(latest_odom_)) {
      selected_setpoint_ = latest_odom_;
      selected_setpoint_.z = clampHeight(selected_setpoint_.z);
    }
    RCLCPP_ERROR(
      get_logger(), "[SUPER_FAULT_LATCH] reason=%s; hold and automatic planning disabled "
      "until an explicit valid final goal arrives", fault_reason_.c_str());
  }

  void publishRawPath(const std::vector<Vec3> & path)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_;
    msg.poses.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
      const auto & point = path[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      const double yaw = pathYawEnu(path, i);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      msg.poses.push_back(pose);
    }
    raw_path_pub_->publish(msg);
  }

  void publishEmptyPath()
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_;
    path_pub_->publish(msg);
    last_path_size_ = 0;
  }

  void publishHoldIfEnabled(const std::string & reason)
  {
    if (!controlOutputEnabled()) {
      return;
    }
    if (!have_output_setpoint_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s; no previous safe navigation setpoint is available for HOLD.", reason.c_str());
      return;
    }
    Vec3 hold = latest_odom_;
    hold.z = clampHeight(hold.z);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "%s; publishing safe hold",
      reason.c_str());
    commanded_yaw_rate_ = 0.0;
    const std::string hold_reason = publish_reason_;
    publishNavigationSetpointIfEnabled(hold);
    publish_reason_ = hold_reason;
  }

  void publishNavigationSetpointIfEnabled(const Vec3 & setpoint_enu)
  {
    if (!controlOutputEnabled() || !navigation_setpoint_pub_) {
      publish_reason_ = "ENABLE_OUTPUT_FALSE";
      return;
    }
    if (!have_odom_ || ageSeconds(last_odom_time_) > odom_timeout_sec_) {
      setMode(Mode::NO_ODOM);
      publish_reason_ = "NO_ODOM";
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "NO VALID ODOM, BLOCK navigation setpoint");
      return;
    }
    if (require_mavros_connected_ && !mavros_connected_) {
      setMode(Mode::NO_ODOM);
      publish_reason_ = "MAVROS_LINK_DOWN";
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MAVROS link is down, BLOCK navigation setpoint instead of chasing stale odom.");
      return;
    }

    Vec3 safe = setpoint_enu;
    safe.z = clampHeight(safe.z);

    if (!isSafeEnuGoal(safe)) {
      setMode(Mode::BLOCKED_UNSAFE);
      publish_reason_ = "BLOCKED_UNSAFE";
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "BLOCKED_UNSAFE: reject ENU navigation setpoint=(%.2f, %.2f, %.2f)",
        safe.x, safe.y, safe.z);
      return;
    }

    Vec3 ned = enuToNed(safe);
    if (altitude_reference_valid_ && use_fixed_flight_height_) {
      ned.z = target_z_local_ned_;
    }
    if (!isSafeNedGoal(ned)) {
      setMode(Mode::BLOCKED_UNSAFE);
      publish_reason_ = "BLOCKED_UNSAFE";
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "BLOCKED_UNSAFE: reject NED navigation setpoint=(%.2f, %.2f, %.2f)",
        ned.x, ned.y, ned.z);
      return;
    }

    if (!have_commanded_yaw_) {
      commanded_yaw_ned_ = have_current_yaw_ ? current_yaw_ned_ : 0.0;
      commanded_yaw_rate_ = 0.0;
      have_commanded_yaw_ = true;
    }

    race_msgs::msg::NavigationSetpoint msg;
    msg.header.stamp = now();
    msg.header.frame_id = setpoint_output_frame_;
    msg.position.x = ned.x;
    msg.position.y = ned.y;
    msg.position.z = ned.z;
    msg.velocity_valid = false;
    msg.yaw = commanded_yaw_ned_;
    msg.yaw_rate_valid = publish_yaw_rate_feedforward_;
    msg.yaw_rate = publish_yaw_rate_feedforward_ ? commanded_yaw_rate_ : 0.0;
    navigation_setpoint_pub_->publish(msg);
    last_output_setpoint_ned_ = ned;
    have_output_setpoint_ = true;
    publish_reason_ = "NAVIGATION_SETPOINT_PUBLISHED";
  }

  bool isSafeEnuGoal(const Vec3 & goal) const
  {
    if (!finite_vec(goal)) {
      return false;
    }
    const double max_expected_distance = tracking_lookahead_distance_ + 2.0 * resolution_;
    if (distance2d(latest_odom_, goal) > max_expected_distance) {
      return false;
    }
    if (goal.z < effectiveMinSafeHeight() - 1.0e-3 ||
      goal.z > effectiveMaxSafeHeight() + 1.0e-3)
    {
      return false;
    }
    return std::abs(goal.x) <= 20.0 && std::abs(goal.y) <= 20.0 && std::abs(goal.z) <= 5.0;
  }

  bool isSafeNedGoal(const Vec3 & goal) const
  {
    if (!finite_vec(goal)) {
      return false;
    }
    if (std::abs(goal.x) > 20.0 || std::abs(goal.y) > 20.0 || std::abs(goal.z) > 5.0) {
      return false;
    }
    const double height = altitude_reference_valid_ ?
      ground_z_local_ned_ - goal.z : -goal.z;
    return height >= min_safe_height_ - 1.0e-3 && height <= max_safe_height_ + 1.0e-3;
  }

  double clampHeight(double height) const
  {
    if (use_fixed_flight_height_) {
      return std::min(
        std::max(effectiveFixedFlightHeight(), effectiveMinSafeHeight()),
        effectiveMaxSafeHeight());
    }
    if (!std::isfinite(height) || height < 0.05) {
      return (effectiveMinSafeHeight() + effectiveMaxSafeHeight()) * 0.5;
    }
    return std::min(std::max(height, effectiveMinSafeHeight()), effectiveMaxSafeHeight());
  }

  double interpolateHeight(const Vec3 & start, const Vec3 & goal, double ratio) const
  {
    ratio = std::min(1.0, std::max(0.0, ratio));
    return clampHeight(start.z + (goal.z - start.z) * ratio);
  }

  Vec3 enuToNed(const Vec3 & enu) const
  {
    // Standard ROS ENU -> PX4 NED conversion from 坐标转换.md.
    return Vec3{enu.y, enu.x, -enu.z};
  }

  Vec3 nedToEnu(const Vec3 & ned) const
  {
    return Vec3{ned.y, ned.x, -ned.z};
  }

  Cell worldToCell(const Vec3 & point) const
  {
    return Cell{
      static_cast<int>(std::floor((point.x - grid_origin_x_) / planning_resolution_)),
      static_cast<int>(std::floor((point.y - grid_origin_y_) / planning_resolution_))};
  }

  Vec3 cellToWorld(const Cell & cell) const
  {
    return Vec3{
      grid_origin_x_ + (static_cast<double>(cell.x) + 0.5) * planning_resolution_,
      grid_origin_y_ + (static_cast<double>(cell.y) + 0.5) * planning_resolution_,
      effectiveMinSafeHeight()};
  }

  bool isInside(const Cell & cell) const
  {
    return cell.x >= 0 && cell.y >= 0 && cell.x < grid_width_ && cell.y < grid_height_;
  }

  bool isCellCenterInsideSharedBounds(const Cell & cell) const
  {
    if (!isInside(cell)) {
      return false;
    }
    const race_super_planner_ros2::GlobalGridGeometry grid{
      grid_origin_x_, grid_origin_y_, planning_resolution_, grid_width_, grid_height_};
    return race_super_planner_ros2::cellCenterInsideSharedBounds(
      cell.x, cell.y, grid, shared_bounds_x_min_, shared_bounds_x_max_,
      shared_bounds_y_min_, shared_bounds_y_max_);
  }

  bool isPlanningCell(const Cell & cell) const
  {
    return isCellCenterInsideSharedBounds(cell) && !isOccupied(cell);
  }

  bool isSegmentSafeForAstar(const Vec3 & start, const Vec3 & end) const
  {
    if (!insideSharedBounds(start) || !insideSharedBounds(end)) {
      return false;
    }
    if (!isSegmentContinuouslySafe(start, end, 0.0).safe) {
      return false;
    }
    const double maximum_step = std::min(planning_resolution_ * 0.5, 0.02);
    const std::size_t samples = race_super_planner_ros2::policy::segmentSampleCount(
      {start.x, start.y}, {end.x, end.y}, maximum_step);
    for (std::size_t index = 0; index <= samples; ++index) {
      const auto sample = race_super_planner_ros2::policy::segmentSample(
        {start.x, start.y}, {end.x, end.y}, index, samples);
      const Cell cell = worldToCell(Vec3{sample.x, sample.y, start.z});
      if (!isPlanningCell(cell)) {
        return false;
      }
    }
    return true;
  }

  bool isOccupied(const Cell & cell) const
  {
    return occupied_.count(cell) > 0;
  }

  double clearanceCost(const Cell & cell) const
  {
    const auto it = inflated_costs_.find(cell);
    const double continuous_clearance = distanceToNearestRawObstacle(cellToWorld(cell));
    const double centre_bias = std::isfinite(continuous_clearance) ?
      clearance_cost_weight_ * requiredContinuousClearance() /
      std::max(continuous_clearance, requiredContinuousClearance()) : 0.0;
    if (it == inflated_costs_.end()) {
      return centre_bias;
    }
    return centre_bias + clearance_cost_weight_ * static_cast<double>(it->second) / 100.0;
  }

  double distanceToNearestRawObstacle(const Vec3 & point) const
  {
    if (!raw_obstacle_points_ || raw_obstacle_points_->empty()) {
      return std::numeric_limits<double>::infinity();
    }
    pcl::PointXYZ query;
    query.x = static_cast<float>(point.x);
    query.y = static_cast<float>(point.y);
    query.z = static_cast<float>(point.z);
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    if (raw_obstacle_kdtree_.nearestKSearch(query, 1, indices, squared_distances) != 1) {
      return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(static_cast<double>(squared_distances.front()));
  }

  Vec3 nearestRawObstacle(const Vec3 & point) const
  {
    if (!raw_obstacle_points_ || raw_obstacle_points_->empty()) {
      return Vec3{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    }
    pcl::PointXYZ query;
    query.x = static_cast<float>(point.x);
    query.y = static_cast<float>(point.y);
    query.z = static_cast<float>(point.z);
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    if (raw_obstacle_kdtree_.nearestKSearch(query, 1, indices, squared_distances) != 1) {
      return Vec3{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    }
    const auto & obstacle = raw_obstacle_points_->at(static_cast<std::size_t>(indices.front()));
    return Vec3{obstacle.x, obstacle.y, obstacle.z};
  }

  double requiredContinuousClearance() const
  {
    return required_center_clearance_;
  }

  bool continuouslySafe(const Vec3 & point) const
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return false;
    }
    // Grid cells are 2-D and cellToWorld carries min_safe_height_. Continuous
    // collision clearance must instead be evaluated at the actual fixed flight
    // height used by the emitted global path.
    Vec3 flight_point = point;
    flight_point.z = clampHeight(effectiveFixedFlightHeight());
    return distanceToNearestRawObstacle(flight_point) >= requiredContinuousClearance();
  }

  double minimumContinuousClearance(const std::vector<Vec3> & path) const
  {
    double minimum = std::numeric_limits<double>::infinity();
    if (path.empty()) {
      return minimum;
    }
    minimum = distanceToNearestRawObstacle(
      Vec3{path.front().x, path.front().y, clampHeight(effectiveFixedFlightHeight())});
    for (std::size_t index = 1; index < path.size(); ++index) {
      const SegmentSafetyResult segment = isSegmentContinuouslySafe(
        path[index - 1], path[index],
        0.0);
      minimum = std::min(minimum, segment.minimum_clearance);
    }
    return minimum;
  }

  void logPathProgress(
    const PathProjection & projection, std::size_t previous_progress_index,
    const PathSample & target)
  {
    if (last_path_progress_log_time_.nanoseconds() != 0 &&
      ageSeconds(last_path_progress_log_time_) < 0.2)
    {
      return;
    }
    last_path_progress_log_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "[PATH_PROGRESS] plan_id=%lu vehicle_position=(%.3f,%.3f) closest_index=%zu "
      "previous_progress_index=%zu selected_progress_index=%zu lookahead_index=%zu "
      "lookahead_arc_distance=%.3f navigation_setpoint=(%.3f,%.3f) "
      "setpoint_clearance=%.3f control_mode=FOLLOWING",
      global_path_id_, latest_odom_.x, latest_odom_.y, projection.segment_index,
      previous_progress_index, tracking_progress_index_, target.path_index,
      target.lookahead, selected_setpoint_.x, selected_setpoint_.y,
      distanceToNearestRawObstacle(selected_setpoint_));
  }

  void publishDebugMarkers()
  {
    if (!publish_debug_markers_) {
      return;
    }
    if (std::isfinite(ageSeconds(last_debug_marker_time_)) &&
      ageSeconds(last_debug_marker_time_) < debug_marker_period_sec_)
    {
      return;
    }
    last_debug_marker_time_ = now();

    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = world_frame_;
    clear.header.stamp = now();
    clear.ns = "planner_debug";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    visualization_msgs::msg::Marker costmap;
    costmap.header = clear.header;
    costmap.ns = "inflated_costmap";
    costmap.id = 1;
    costmap.type = visualization_msgs::msg::Marker::CUBE_LIST;
    costmap.action = visualization_msgs::msg::Marker::ADD;
    costmap.pose.orientation.w = 1.0;
    costmap.scale.x = planning_resolution_;
    costmap.scale.y = planning_resolution_;
    costmap.scale.z = debug_marker_height_;
    costmap.frame_locked = false;

    const std::size_t stride =
      std::max<std::size_t>(1, inflated_costs_.size() / std::max(1, debug_max_cells_));
    std::size_t index = 0;
    for (const auto & entry : inflated_costs_) {
      if ((index++ % stride) != 0) {
        continue;
      }
      const Vec3 world = cellToWorld(entry.first);
      geometry_msgs::msg::Point point;
      point.x = world.x;
      point.y = world.y;
      point.z = debug_marker_height_ * 0.5;
      costmap.points.push_back(point);

      std_msgs::msg::ColorRGBA color;
      const double t = static_cast<double>(entry.second) / 100.0;
      color.r = static_cast<float>(0.2 + 0.8 * t);
      color.g = static_cast<float>(0.8 * (1.0 - t));
      color.b = 0.1f;
      color.a = static_cast<float>(0.18 + 0.45 * t);
      costmap.colors.push_back(color);
    }
    array.markers.push_back(costmap);

    visualization_msgs::msg::Marker footprint;
    footprint.header = clear.header;
    footprint.ns = "planner_debug";
    footprint.id = 2;
    footprint.type = visualization_msgs::msg::Marker::CYLINDER;
    footprint.action = visualization_msgs::msg::Marker::ADD;
    footprint.pose.position.x = latest_odom_.x;
    footprint.pose.position.y = latest_odom_.y;
    footprint.pose.position.z = debug_marker_height_ * 0.5;
    footprint.pose.orientation.w = 1.0;
    footprint.scale.x = active_inflation_radius_ * 2.0;
    footprint.scale.y = active_inflation_radius_ * 2.0;
    footprint.scale.z = debug_marker_height_ * 1.25;
    footprint.color.r = 0.1f;
    footprint.color.g = 0.6f;
    footprint.color.b = 1.0f;
    footprint.color.a = 0.30f;
    array.markers.push_back(footprint);

    visualization_msgs::msg::Marker start_goal;
    start_goal.header = clear.header;
    start_goal.ns = "planner_debug";
    start_goal.id = 3;
    start_goal.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    start_goal.action = visualization_msgs::msg::Marker::ADD;
    start_goal.pose.orientation.w = 1.0;
    start_goal.scale.x = 0.22;
    start_goal.scale.y = 0.22;
    start_goal.scale.z = 0.22;

    geometry_msgs::msg::Point start_pt;
    start_pt.x = latest_odom_.x;
    start_pt.y = latest_odom_.y;
    start_pt.z = clampHeight(latest_odom_.z);
    start_goal.points.push_back(start_pt);
    std_msgs::msg::ColorRGBA start_color;
    start_color.r = 0.1f;
    start_color.g = 1.0f;
    start_color.b = 0.2f;
    start_color.a = 0.95f;
    start_goal.colors.push_back(start_color);

    if (have_goal_) {
      geometry_msgs::msg::Point goal_pt;
      goal_pt.x = latest_goal_.x;
      goal_pt.y = latest_goal_.y;
      goal_pt.z = clampHeight(latest_goal_.z);
      start_goal.points.push_back(goal_pt);
      std_msgs::msg::ColorRGBA goal_color;
      goal_color.r = 1.0f;
      goal_color.g = 0.15f;
      goal_color.b = 0.15f;
      goal_color.a = 0.95f;
      start_goal.colors.push_back(goal_color);
    }
    array.markers.push_back(start_goal);

    marker_pub_->publish(array);
  }

  void setMode(Mode mode)
  {
    mode_ = mode;
  }

  std::string modeName() const
  {
    switch (mode_) {
      case Mode::IDLE: return "IDLE";
      case Mode::PLANNING: return "PLANNING";
      case Mode::FOLLOWING: return "FOLLOWING";
      case Mode::HOLD: return "HOLD";
      case Mode::BLOCKED_UNSAFE: return "BLOCKED_UNSAFE";
      case Mode::NO_ODOM: return "NO_ODOM";
      case Mode::NO_MAP: return "NO_MAP";
    }
    return "UNKNOWN";
  }

  void logStatus()
  {
    std_msgs::msg::String status;
    status.data = "planner=super mode=" + modeName() + " reason=" + publish_reason_ +
      " goal=" + (have_goal_ ? "active" : "none") +
      " global_goal_id=" + std::to_string(global_goal_id_) +
      " global_path_id=" + std::to_string(global_path_id_) +
      " local_goal_seq=" + std::to_string(local_goal_seq_) +
      " final_goal_reached=" + (final_goal_reached_latched_ ? "true" : "false");
    status_pub_->publish(status);

    race_msgs::msg::GlobalPlannerStatus global_status;
    global_status.header.stamp = now();
    global_status.header.frame_id = world_frame_;
    global_status.global_goal_id = global_goal_id_;
    global_status.global_path_id = global_path_id_;
    global_status.local_goal_seq = local_goal_seq_;
    global_status.goal_active = have_goal_;
    global_status.final_goal_reached = final_goal_reached_latched_;
    global_status.distance_to_final = have_goal_ ?
      distance2d(latest_odom_, latest_goal_) : std::numeric_limits<double>::infinity();
    global_status.horizontal_speed = latest_horizontal_speed_;
    global_status.mode = modeName();
    global_status.reason = publish_reason_;
    global_status_pub_->publish(global_status);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(), 1000,
      "SUPER_ROS2 mode=%s enable_output=%s reason=%s odom=(%.2f,%.2f,%.2f) goal=%s tracking_setpoint_enu=(%.2f,%.2f,%.2f) "
      "setpoint_ned=%s yaw(desired/cmd/rate)=(%.2f/%.2f/%.2f) cross_track=%.2f frame_id=%s path_points=%zu odom_source=%s cloud_source=%s mavros_connected=%s",
      modeName().c_str(),
      controlOutputEnabled() ? "true" : "false",
      publish_reason_.c_str(),
      latest_odom_.x, latest_odom_.y, latest_odom_.z,
      have_goal_ ? "set" : "none",
      selected_setpoint_.x, selected_setpoint_.y, selected_setpoint_.z,
      have_output_setpoint_ ? formatVec(last_output_setpoint_ned_).c_str() : "none",
      desired_yaw_ned_, commanded_yaw_ned_, commanded_yaw_rate_,
      cross_track_error_,
      setpoint_output_frame_.c_str(),
      last_path_size_,
      active_odom_source_.empty() ? "none" : active_odom_source_.c_str(),
      active_cloud_source_.empty() ? "none" : active_cloud_source_.c_str(),
      mavros_connected_ ? "true" : "false");
  }

  std::string formatVec(const Vec3 & v) const
  {
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), "(%.2f,%.2f,%.2f)", v.x, v.y, v.z);
    return std::string(buffer);
  }

  rclcpp::Time now()
  {
    return get_clock()->now();
  }

  double ageSeconds(const rclcpp::Time & stamp)
  {
    if (stamp.nanoseconds() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return (now() - stamp).seconds();
  }

  std::string goal_topic_;
  std::string odom_topic_;
  std::string fallback_odom_topic_;
  std::string odom_input_frame_;
  std::string mavros_state_topic_;
  std::string cloud_topic_;
  std::string fallback_cloud_topic_;
  std::string navigation_setpoint_topic_;
  std::string path_topic_;
  std::string global_path_topic_;
  std::string ego_local_goal_topic_;
  std::string ego_reference_path_topic_;
  std::string ego_status_topic_;
  std::string validated_bspline_topic_;
  std::string command_local_goal_seq_topic_;
  std::string raw_path_topic_;
  std::string marker_topic_;
  std::string world_frame_;
  std::string body_frame_;
  std::string setpoint_output_frame_;

  double control_rate_{20.0};
  double replan_rate_{2.0};
  bool replan_while_tracking_{true};
  double tracking_lookahead_distance_{0.50};
  double vehicle_collision_radius_{0.384};
  double narrow_corridor_tracking_margin_{0.05};
  double required_center_clearance_{0.466};
  double narrow_corridor_lookahead_distance_{0.15};
  double narrow_corridor_centering_tolerance_{0.05};
  double heading_lookahead_distance_{0.80};
  double max_path_tracking_error_{0.80};
  double yaw_smoothing_time_constant_{0.35};
  double max_yaw_rate_{1.0};
  double min_safe_height_{0.8};
  double max_safe_height_{2.0};
  double fixed_flight_height_{0.78};
  double final_goal_position_tolerance_m_{0.10};
  double final_goal_velocity_tolerance_mps_{0.08};
  double ego_local_goal_lookahead_m_{1.20};
  double ego_local_goal_min_distance_m_{0.60};
  double ego_local_goal_max_distance_m_{1.60};
  double ego_reference_preview_distance_m_{0.60};
  double ego_local_goal_reached_radius_m_{0.30};
  double local_goal_update_min_interval_sec_{0.10};
  double local_goal_update_distance_m_{0.20};
  double goal_update_position_tolerance_m_{0.05};
  double pending_path_max_cross_track_error_m_{0.15};
  int pending_path_max_replans_{1};
  double local_planner_failure_replan_sec_{1.0};
  double trajectory_prefetch_sec_{1.5};
  double trajectory_stall_timeout_sec_{1.0};
  double trajectory_recovery_confirmation_sec_{0.5};
  double shared_bounds_x_min_{-7.5};
  double shared_bounds_x_max_{7.5};
  double shared_bounds_y_min_{-7.0};
  double shared_bounds_y_max_{7.0};
  double shared_bounds_z_min_{0.50};
  double shared_bounds_z_max_{0.90};
  double fault_position_max_speed_mps_{4.0};
  double fault_position_jump_allowance_m_{0.40};
  double fault_planning_grid_padding_m_{3.0};
  uint64_t altitude_reference_flight_id_{0};
  double ground_z_map_{0.0};
  double target_z_map_{0.0};
  double ground_z_local_ned_{0.0};
  double target_z_local_ned_{0.0};
  double resolution_{0.20};
  double planning_resolution_{0.20};
  double local_range_xy_{10.0};
  double obstacle_min_height_{0.30};
  double obstacle_max_height_{2.00};
  double inflation_radius_{0.36};
  double active_inflation_radius_{0.36};
  double soft_obstacle_cost_radius_{0.52};
  double clearance_cost_weight_{3.5};
  double cloud_timeout_sec_{600.0};
  double odom_timeout_sec_{1.0};
  double goal_timeout_sec_{600.0};
  double path_sample_resolution_{0.10};
  double smoothing_max_deviation_{0.8};
  double smoothing_min_clearance_{0.30};
  double min_planning_inflation_radius_{0.36};
  double debug_marker_period_sec_{0.50};
  int max_cloud_points_{200000};
  int smoothing_iterations_{2};
  int debug_max_cells_{25000};
  int final_goal_confirmation_cycles_{10};
  bool enable_output_{false};
  bool global_only_mode_{false};
  bool publish_global_path_{false};
  bool publish_ego_local_goal_{false};
  bool publish_yaw_rate_feedforward_{true};
  bool enable_path_smoothing_{true};
  bool publish_debug_markers_{true};
  bool require_mavros_connected_{false};
  bool smoothing_collision_check_{true};
  bool use_fixed_flight_height_{true};
  bool altitude_reference_valid_{false};
  bool allow_direct_path_{false};
  bool enable_path_shortcut_{false};
  std::string smoothing_method_{"chaikin"};
  double debug_marker_height_{0.20};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fallback_odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  rclcpp::Subscription<race_msgs::msg::FlightAltitudeReference>::SharedPtr
    altitude_reference_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr fallback_cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ego_status_sub_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr validated_bspline_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr command_local_goal_seq_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ego_local_goal_pub_;
  rclcpp::Publisher<race_msgs::msg::LocalPathReference>::SharedPtr ego_reference_path_pub_;
  rclcpp::Publisher<race_msgs::msg::NavigationSetpoint>::SharedPtr navigation_setpoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<race_msgs::msg::GlobalPlannerStatus>::SharedPtr global_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Vec3 latest_odom_;
  Vec3 latest_goal_;
  Vec3 selected_setpoint_;
  Vec3 last_output_setpoint_ned_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  bool have_odom_{false};
  bool have_goal_{false};
  bool have_cloud_{false};
  bool have_output_setpoint_{false};
  bool have_fault_checked_odom_{false};
  bool fault_latched_{false};
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_goal_time_;
  rclcpp::Time last_cloud_time_;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_marker_time_;
  rclcpp::Time last_path_progress_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mavros_state_time_;
  rclcpp::Time last_fault_checked_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_goal_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time local_planner_failure_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_validated_trajectory_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time validated_trajectory_lease_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trajectory_recovery_candidate_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pending_plan_completion_time_{0, 0, RCL_ROS_TIME};
  std::string active_odom_source_;
  std::string active_cloud_source_;
  std::string fault_reason_;
  bool mavros_connected_{false};
  bool mavros_armed_{false};
  bool takeoff_path_released_{false};
  bool have_current_yaw_{false};
  bool have_commanded_yaw_{false};
  double current_yaw_ned_{0.0};
  double desired_yaw_ned_{0.0};
  double commanded_yaw_ned_{0.0};
  double commanded_yaw_rate_{0.0};
  double cross_track_error_{0.0};
  double latest_horizontal_speed_{std::numeric_limits<double>::infinity()};
  double distance_to_final_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_yaw_update_time_{0, 0, RCL_ROS_TIME};
  std::string publish_reason_{"INIT"};

  double grid_origin_x_{0.0};
  double grid_origin_y_{0.0};
  int grid_width_{0};
  int grid_height_{0};
  std::unordered_set<Cell, CellHash> occupied_;
  std::unordered_set<Cell, CellHash> raw_obstacles_;
  std::unordered_map<Cell, uint8_t, CellHash> inflated_costs_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_obstacle_points_{
    new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::KdTreeFLANN<pcl::PointXYZ> raw_obstacle_kdtree_;
  std::vector<Vec3> active_path_;
  std::vector<Vec3> pending_global_path_;
  std::vector<Vec3> pending_raw_path_;
  Vec3 pending_plan_start_odom_;
  Vec3 last_local_goal_;
  uint64_t global_goal_id_{0};
  uint64_t global_path_id_{0};
  uint64_t local_goal_seq_{0};
  uint64_t odom_generation_{0};
  uint64_t pending_route_goal_id_{0};
  uint64_t pending_plan_start_odom_generation_{0};
  uint64_t pending_plan_completion_odom_generation_{0};
  int64_t last_odom_source_stamp_ns_{0};
  int64_t pending_plan_start_odom_source_stamp_ns_{0};
  int64_t pending_plan_completion_odom_source_stamp_ns_{0};
  uint64_t last_validated_local_goal_seq_{0};
  int64_t last_validated_trajectory_id_{-1};
  int64_t recovery_required_after_trajectory_id_{-1};
  std::size_t local_goal_progress_index_{0};
  std::size_t tracking_progress_index_{0};
  std::size_t last_local_goal_index_{0};
  double last_local_goal_path_distance_{0.0};
  bool have_local_goal_{false};
  bool pending_global_path_valid_{false};
  bool pending_path_retry_exhausted_{false};
  int pending_path_completed_replans_{0};
  bool local_planner_replan_requested_{false};
  bool local_planner_failure_replan_latched_{false};
  bool local_goal_reached_logged_{false};
  bool final_approach_published_{false};
  bool final_goal_reached_latched_{false};
  int final_goal_confirmation_count_{0};
  Mode mode_{Mode::IDLE};
  std::size_t last_path_size_{0};
  std::size_t last_raw_path_size_{0};
  std::size_t last_shortcut_path_size_{0};
  std::size_t last_smoothed_path_size_{0};
  std::string last_smoothing_result_{"none"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SuperPlannerRos2Node>());
  rclcpp::shutdown();
  return 0;
}
