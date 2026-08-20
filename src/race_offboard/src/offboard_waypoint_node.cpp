#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <race_msgs/msg/flight_altitude_reference.hpp>
#include <race_msgs/msg/global_planner_status.hpp>
#include <race_msgs/msg/navigation_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "race_offboard/takeoff_handover_policy.hpp"
#include "race_offboard/global_planner_status_policy.hpp"
#include "race_offboard/ego_command_guard.hpp"
#include "race_offboard/ev_health_tracker.hpp"
#include "race_offboard/local_origin_rebase_guard.hpp"
#include "race_offboard/mavros_frame_utils.hpp"

using namespace std::chrono_literals;

namespace
{
bool finitePosition(const race_msgs::msg::NavigationSetpoint & msg)
{
  return std::isfinite(msg.position.x) && std::isfinite(msg.position.y) &&
         std::isfinite(msg.position.z);
}

// EGO's command held in this node's internal NED convention.
//
// The bridge now sends mavros_msgs/PositionTarget in ENU, but every guard here
// (height limits, over-height freeze, lead-distance check) is written against
// NED, and the over-height soft guard relies on marking a single ignored axis.
// Converting once at the callback keeps all of that logic untouched, and NaN
// still means "this axis carries no feed-forward".
struct EgoCommandNed
{
  std::array<double, 3> position{
    {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  std::array<double, 3> velocity{
    {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  std::array<double, 3> acceleration{
    {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  double yaw{std::numeric_limits<double>::quiet_NaN()};
  double yaw_rate{std::numeric_limits<double>::quiet_NaN()};
};

bool finiteEgoPosition(const EgoCommandNed & command)
{
  return std::isfinite(command.position[0]) && std::isfinite(command.position[1]) &&
         std::isfinite(command.position[2]) && std::isfinite(command.yaw);
}

bool finiteOrAllNan(const std::array<double, 3> & value)
{
  const bool all_finite = std::isfinite(value[0]) && std::isfinite(value[1]) &&
    std::isfinite(value[2]);
  const bool all_nan = std::isnan(value[0]) && std::isnan(value[1]) && std::isnan(value[2]);
  return all_finite || all_nan;
}

// A masked axis conveys "no feed-forward", which this node represents as NaN.
std::array<double, 3> egoAxisOrNan(
  const geometry_msgs::msg::Vector3 & value, bool ignore_x, bool ignore_y, bool ignore_z)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  return {ignore_x ? nan : value.x, ignore_y ? nan : value.y, ignore_z ? nan : value.z};
}

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

class OffboardWaypointNode : public rclcpp::Node
{
public:
  OffboardWaypointNode()
  : Node("offboard_waypoint_node")
  {
    control_source_ = declare_parameter<std::string>("control_source", "navigation");
    navigation_setpoint_topic_ =
      declare_parameter<std::string>("navigation_setpoint_topic", "/race/navigation_setpoint");
    ego_setpoint_topic_ =
      declare_parameter<std::string>("ego_setpoint_topic", "/race/ego/trajectory_setpoint");
    ego_setpoint_timeout_sec_ = declare_parameter<double>("ego_setpoint_timeout_sec", 0.20);
    if (control_source_ == "local_goal" || control_source_ == "super") {
      control_source_ = "navigation";
    }
    if (control_source_ != "navigation" && control_source_ != "ego") {
      throw std::runtime_error(
              "control_source must be navigation (aliases: super/local_goal) or ego");
    }
    required_setpoint_frame_ = declare_parameter<std::string>("required_setpoint_frame", "px4_ned");
    map_odom_topic_ = declare_parameter<std::string>("map_odom_topic", "/race/odom");
    require_map_local_alignment_ =
      declare_parameter<bool>("require_map_local_alignment", false);
    map_local_alignment_stabilization_sec_ =
      declare_parameter<double>("map_local_alignment_stabilization_sec", 1.0);
    map_local_alignment_max_spread_m_ =
      declare_parameter<double>("map_local_alignment_max_spread_m", 0.08);
    map_local_alignment_max_yaw_spread_rad_ =
      declare_parameter<double>("map_local_alignment_max_yaw_spread_rad", 0.10);
    setpoint_timeout_sec_ = declare_parameter<double>("setpoint_timeout_sec", 0.8);
    allow_dead_reckoning_takeoff_ = declare_parameter<bool>("allow_dead_reckoning_takeoff", false);
    require_strict_local_position_health_ =
      declare_parameter<bool>("require_strict_local_position_health", true);
    // External-vision health gate.  The simulation node had none; on the real
    // aircraft EKF2's position comes from FAST-LIO via MAVROS vision_pose.
    require_ev_health_ = declare_parameter<bool>("require_ev_health", true);
    ev_health_topic_ = declare_parameter<std::string>("ev_health_topic", "/ev_health/status");
    ev_flight_ready_topic_ = declare_parameter<std::string>(
      "ev_flight_ready_topic", "/ev_health/flight_ready");
    ev_health_required_s_ = declare_parameter<double>("ev_health_required_s", 7.5);
    ev_health_freshness_s_ = declare_parameter<double>("ev_health_freshness_s", 0.5);
    ev_fault_auto_land_ = declare_parameter<bool>("ev_fault_auto_land", true);
    ev_health_.configure(0.0, ev_health_freshness_s_);
    ev_flight_ready_.configure(ev_health_required_s_, ev_health_freshness_s_);
    local_origin_rebase_stabilization_sec_ =
      declare_parameter<double>("local_origin_rebase_stabilization_sec", 1.0);
    local_origin_rebase_max_spread_m_ =
      declare_parameter<double>("local_origin_rebase_max_spread_m", 0.08);
    local_origin_rebase_max_speed_mps_ =
      declare_parameter<double>("local_origin_rebase_max_speed_mps", 0.20);
    local_origin_rebase_guard_.configure(
      local_origin_rebase_stabilization_sec_, local_origin_rebase_max_spread_m_,
      local_origin_rebase_max_speed_mps_);
    ekf_xy_wait_timeout_sec_ = declare_parameter<double>("ekf_xy_wait_timeout_sec", 3.0);
    offboard_warmup_sec_ = declare_parameter<double>("offboard_warmup_sec", 2.0);
    manual_handover_ = declare_parameter<bool>("manual_handover", false);
    manual_handover_max_position_error_m_ =
      declare_parameter<double>("manual_handover_max_position_error_m", 0.15);
    manual_handover_max_speed_mps_ =
      declare_parameter<double>("manual_handover_max_speed_mps", 0.20);
    preflight_stabilization_sec_ = declare_parameter<double>("preflight_stabilization_sec", 5.0);
    command_retry_period_sec_ = declare_parameter<double>("command_retry_period_sec", 1.0);
    cruise_altitude_m_ = declare_parameter<double>("cruise_altitude_m", 0.78);
    flight_target_agl_m_ = cruise_altitude_m_;
    takeoff_complete_height_m_ = declare_parameter<double>("takeoff_complete_height_m", 0.55);
    min_command_height_m_ = declare_parameter<double>("min_command_height_m", 0.50);
    max_command_height_m_ = declare_parameter<double>("max_command_height_m", 0.90);
    overheight_guard_margin_m_ = declare_parameter<double>("overheight_guard_margin_m", 0.20);
    emergency_overheight_margin_m_ =
      declare_parameter<double>("emergency_overheight_margin_m", 0.35);
    idle_hold_enabled_ = declare_parameter<bool>("idle_hold_enabled", true);
    trajectory_timeout_sec_ = declare_parameter<double>("trajectory_timeout", 0.50);
    initial_ego_trajectory_wait_sec_ =
      declare_parameter<double>("initial_ego_trajectory_wait_sec", 2.0);
    hold_position_tolerance_ = declare_parameter<double>("hold_position_tolerance", 0.05);
    hold_velocity_tolerance_ = declare_parameter<double>("hold_velocity_tolerance", 0.05);
    goal_reached_position_tolerance_ =
      declare_parameter<double>("goal_reached_position_tolerance", 0.10);
    goal_reached_velocity_tolerance_ =
      declare_parameter<double>("goal_reached_velocity_tolerance", 0.08);
    goal_update_position_tolerance_m_ =
      declare_parameter<double>("goal_update_position_tolerance_m", 0.05);
    same_goal_retry_on_safety_latch_ =
      declare_parameter<bool>("same_goal_retry_on_safety_latch", true);
    require_global_planner_final_goal_ =
      declare_parameter<bool>("require_global_planner_final_goal", false);
    braking_max_acc_ = declare_parameter<double>("braking_max_acc", 0.50);
    hold_replan_distance_ = declare_parameter<double>("hold_replan_distance", 0.08);
    setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 20.0);
    trusted_position_max_speed_mps_ =
      declare_parameter<double>("trusted_position_max_speed_mps", 4.0);
    trusted_position_jump_allowance_m_ =
      declare_parameter<double>("trusted_position_jump_allowance_m", 0.40);
    ego_setpoint_max_lead_m_ = declare_parameter<double>("ego_setpoint_max_lead_m", 2.0);
    ego_yaw_rate_limit_rad_s_ =
      declare_parameter<double>("ego_yaw_rate_limit_rad_s", 0.80);
    initial_position_stabilization_sec_ =
      declare_parameter<double>("initial_position_stabilization_sec", 0.50);
    initial_position_max_spread_m_ =
      declare_parameter<double>("initial_position_max_spread_m", 0.08);
    initial_position_max_speed_mps_ =
      declare_parameter<double>("initial_position_max_speed_mps", 0.20);
    shared_bounds_x_min_ = declare_parameter<double>("shared_bounds/x_min", -7.5);
    shared_bounds_x_max_ = declare_parameter<double>("shared_bounds/x_max", 7.5);
    shared_bounds_y_min_ = declare_parameter<double>("shared_bounds/y_min", -7.0);
    shared_bounds_y_max_ = declare_parameter<double>("shared_bounds/y_max", 7.0);
    shared_bounds_z_min_ = declare_parameter<double>("shared_bounds/z_min", 0.50);
    shared_bounds_z_max_ = declare_parameter<double>("shared_bounds/z_max", 0.90);
    if (shared_bounds_x_min_ >= shared_bounds_x_max_ ||
      shared_bounds_y_min_ >= shared_bounds_y_max_ ||
      shared_bounds_z_min_ >= shared_bounds_z_max_ || setpoint_rate_hz_ <= 0.0 ||
      cruise_altitude_m_ < shared_bounds_z_min_ ||
      cruise_altitude_m_ > shared_bounds_z_max_)
    {
      throw std::runtime_error("SHARED_BOUNDARY_MISMATCH: invalid Offboard shared bounds");
    }
    trajectory_timeout_sec_ = std::max(0.10, trajectory_timeout_sec_);
    initial_ego_trajectory_wait_sec_ = std::max(0.10, initial_ego_trajectory_wait_sec_);
    braking_max_acc_ = std::max(0.10, braking_max_acc_);
    trusted_position_max_speed_mps_ = std::max(0.10, trusted_position_max_speed_mps_);
    trusted_position_jump_allowance_m_ = std::max(0.05, trusted_position_jump_allowance_m_);
    ego_setpoint_max_lead_m_ = std::max(0.10, ego_setpoint_max_lead_m_);
    ego_yaw_rate_limit_rad_s_ = std::max(0.05, ego_yaw_rate_limit_rad_s_);
    initial_position_stabilization_sec_ = std::max(0.10, initial_position_stabilization_sec_);
    initial_position_max_spread_m_ = std::max(0.01, initial_position_max_spread_m_);
    initial_position_max_speed_mps_ = std::max(0.01, initial_position_max_speed_mps_);
    takeoff_complete_height_m_ = std::clamp(
      takeoff_complete_height_m_, min_command_height_m_, cruise_altitude_m_);

    // MAVROS keeps the OFFBOARD heartbeat itself; there is no equivalent of
    // /fmu/in/offboard_control_mode.  What PX4 requires is an uninterrupted
    // setpoint stream above 2 Hz before the mode switch, which the 20 Hz timer
    // plus offboard_warmup_sec already guarantees.
    trajectory_setpoint_publisher_ =
      create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local", rclcpp::SensorDataQoS());
    control_status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/race/control/status", rclcpp::QoS(1).reliable().transient_local());
    flight_altitude_reference_publisher_ =
      create_publisher<race_msgs::msg::FlightAltitudeReference>(
      "/race/flight_altitude_reference", rclcpp::QoS(1).reliable().transient_local());

    arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    local_position_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&OffboardWaypointNode::localPositionCallback, this, std::placeholders::_1));
    map_odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      map_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OffboardWaypointNode::mapOdomCallback, this, std::placeholders::_1));
    local_velocity_subscriber_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
      std::bind(&OffboardWaypointNode::localVelocityCallback, this, std::placeholders::_1));
    const auto ev_latched_qos = rclcpp::QoS(1).reliable().transient_local();
    ev_health_subscriber_ = create_subscription<std_msgs::msg::String>(
      ev_health_topic_, ev_latched_qos,
      std::bind(&OffboardWaypointNode::evHealthCallback, this, std::placeholders::_1));
    ev_flight_ready_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      ev_flight_ready_topic_, ev_latched_qos,
      std::bind(&OffboardWaypointNode::evFlightReadyCallback, this, std::placeholders::_1));
    vehicle_status_subscriber_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::SensorDataQoS(),
      std::bind(&OffboardWaypointNode::vehicleStatusCallback, this, std::placeholders::_1));
    navigation_setpoint_subscriber_ = create_subscription<race_msgs::msg::NavigationSetpoint>(
      navigation_setpoint_topic_, 10,
      std::bind(&OffboardWaypointNode::navigationSetpointCallback, this, std::placeholders::_1));
    ego_setpoint_subscriber_ = create_subscription<mavros_msgs::msg::PositionTarget>(
      ego_setpoint_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OffboardWaypointNode::egoSetpointCallback, this, std::placeholders::_1));
    ego_command_goal_seq_subscriber_ = create_subscription<std_msgs::msg::UInt64>(
      "/race/ego/command_local_goal_seq", rclcpp::SensorDataQoS(),
      std::bind(&OffboardWaypointNode::egoCommandGoalSeqCallback, this, std::placeholders::_1));
    planner_status_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/race/planner/status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&OffboardWaypointNode::plannerStatusCallback, this, std::placeholders::_1));
    global_planner_status_subscriber_ =
      create_subscription<race_msgs::msg::GlobalPlannerStatus>(
      "/race/global_planner/status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &OffboardWaypointNode::globalPlannerStatusCallback, this, std::placeholders::_1));
    ego_status_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/race/ego/status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&OffboardWaypointNode::egoStatusCallback, this, std::placeholders::_1));
    goal_activity_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&OffboardWaypointNode::goalActivityCallback, this, std::placeholders::_1));

    takeoff_service_ = create_service<std_srvs::srv::Trigger>(
      "/race/takeoff",
      std::bind(
        &OffboardWaypointNode::takeoffCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    land_service_ = create_service<std_srvs::srv::Trigger>(
      "/race/land",
      std::bind(
        &OffboardWaypointNode::landCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/race/cancel_navigation",
      std::bind(
        &OffboardWaypointNode::cancelCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Single-setpoint-publisher invariant.  minipc_mavros_offboard.py is the
    // takeoff/hover baseline and publishes the same topic; two publishers make
    // PX4 track whichever message arrived last, which is unflyable.  Refuse to
    // start rather than fight it -- fail closed at startup, not mid-flight.
    if (conflicting_setpoint_publisher_present()) {
      throw std::runtime_error(
              "SETPOINT_PUBLISHER_CONFLICT: another node already publishes "
              "/mavros/setpoint_raw/local (minipc_mavros_offboard?). Stop it first: only one "
              "Offboard node may drive the vehicle.");
    }

    const auto period = std::chrono::duration<double>(1.0 / setpoint_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&OffboardWaypointNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Offboard adapter: control_source=%s navigation_topic=%s ego_topic=%s frame=%s "
      "timeout=%.2fs ego_timeout=%.2fs height=[%.2f, %.2f]m "
      "warmup=%.1fs stabilization=%.1fs takeoff_complete=%.2fm strict_health=%s",
      control_source_.c_str(), navigation_setpoint_topic_.c_str(), ego_setpoint_topic_.c_str(),
      required_setpoint_frame_.c_str(), setpoint_timeout_sec_, ego_setpoint_timeout_sec_,
      min_command_height_m_, max_command_height_m_, offboard_warmup_sec_,
      preflight_stabilization_sec_, takeoff_complete_height_m_,
      require_strict_local_position_health_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "[PLANNER_SOURCE] backend=%s", control_source_.c_str());
    RCLCPP_INFO(
      get_logger(), "[TRAJECTORY_SOURCE] selected=%s timeout=%.2fs",
      control_source_.c_str(), trajectory_timeout_sec_);
  }

private:
  enum class State
  {
    IDLE,
    WARMUP,
    PREFLIGHT,
    TAKEOFF,
    ACTIVE,
    LANDING,
    MANUAL_OVERRIDE,
    FINISHED
  };

  enum class ControlState
  {
    WAITING_FOR_ODOM,
    IDLE_HOLD,
    PLANNING,
    TRACKING,
    BRAKING,
    GOAL_REACHED_HOLD,
    PLANNER_FAILURE_HOLD,
    EMERGENCY_STOP,
    MANUAL_OVERRIDE
  };

  // MAVROS splits pose and velocity across two topics.  Cache the ENU velocity
  // here and convert it to NED alongside the pose so every internal state
  // variable stays in the NED convention the planner contract expects.
  void localVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    const std::array<double, 3> velocity_enu{
      msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
    if (!std::isfinite(velocity_enu[0]) || !std::isfinite(velocity_enu[1]) ||
      !std::isfinite(velocity_enu[2]))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "[POSE_REJECT] non-finite local velocity");
      return;
    }
    const auto velocity_ned = race_offboard::enuToNed(velocity_enu);
    current_vx_ = static_cast<float>(velocity_ned[0]);
    current_vy_ = static_cast<float>(velocity_ned[1]);
    current_vz_ = static_cast<float>(velocity_ned[2]);
    have_local_velocity_ = true;
  }

  void localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const std::array<double, 3> position_enu{
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    const double yaw_enu = tf2::getYaw(msg->pose.orientation);
    if (!std::isfinite(position_enu[0]) || !std::isfinite(position_enu[1]) ||
      !std::isfinite(position_enu[2]) || !std::isfinite(yaw_enu))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "[POSE_REJECT] non-finite local position");
      return;
    }
    // Velocity is required for the stability gate and the braking profile; wait
    // for the companion topic rather than trusting a zero-initialised value.
    if (!have_local_velocity_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[POSE_WAIT] no local velocity sample yet on /mavros/local_position/velocity_local");
      return;
    }

    const auto position_ned = race_offboard::enuToNed(position_enu);
    const double yaw_ned = race_offboard::enuYawToNed(yaw_enu);
    const auto x_ned = static_cast<float>(position_ned[0]);
    const auto y_ned = static_cast<float>(position_ned[1]);
    const auto z_ned = static_cast<float>(position_ned[2]);
    // MAVROS timestamps are header stamps, not the PX4 microsecond counter.
    const rclcpp::Time stamp(msg->header.stamp);

    bool accepted_origin_rebase = false;
    double accepted_pose_dt_sec = std::numeric_limits<double>::quiet_NaN();
    if (!have_local_position_) {
      const double speed_mps = std::hypot(
        std::hypot(static_cast<double>(current_vx_), static_cast<double>(current_vy_)),
        static_cast<double>(current_vz_));
      if (!initial_position_candidate_valid_ ||
        !race_offboard::initialPositionSampleIsStable(
          std::hypot(
            std::hypot(
              static_cast<double>(x_ned - initial_position_x_),
              static_cast<double>(y_ned - initial_position_y_)),
            static_cast<double>(z_ned - initial_position_z_)),
          speed_mps, initial_position_max_spread_m_, initial_position_max_speed_mps_))
      {
        initial_position_x_ = x_ned;
        initial_position_y_ = y_ned;
        initial_position_z_ = z_ned;
        initial_position_candidate_valid_ = true;
        initial_position_stable_since_ = now();
        return;
      }
      if ((now() - initial_position_stable_since_).seconds() <
        initial_position_stabilization_sec_)
      {
        return;
      }
      RCLCPP_INFO(
        get_logger(),
        "[POSE_TRUSTED] stable_for=%.2fs position_ned=(%.3f,%.3f,%.3f)",
        (now() - initial_position_stable_since_).seconds(), x_ned, y_ned, z_ned);
    } else {
      if (have_trusted_position_stamp_ && stamp <= last_trusted_position_stamp_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[POSE_REJECT] non-monotonic stamp=%.6f trusted=%.6f",
          stamp.seconds(), last_trusted_position_stamp_.seconds());
        return;
      }
      const double dt_sec = have_trusted_position_stamp_ ?
        (stamp - last_trusted_position_stamp_).seconds() : 1.0 / setpoint_rate_hz_;
      const double displacement_m = std::hypot(
        std::hypot(
          static_cast<double>(x_ned - current_x_),
          static_cast<double>(y_ned - current_y_)),
        static_cast<double>(z_ned - current_z_));
      const bool plausible_jump = race_offboard::localPositionJumpIsPlausible(
        displacement_m, dt_sec, trusted_position_max_speed_mps_,
        trusted_position_jump_allowance_m_);
      const auto sample_now_ns = now().nanoseconds();
      const double speed_mps = std::hypot(
        std::hypot(static_cast<double>(current_vx_), static_cast<double>(current_vy_)),
        static_cast<double>(current_vz_));

      if (local_origin_rebase_guard_.active()) {
        if (!localOriginRebaseAllowed(sample_now_ns)) {
          local_origin_rebase_guard_.reset();
        } else if (displacement_m <= trusted_position_jump_allowance_m_) {
          local_origin_rebase_guard_.reset();
          RCLCPP_WARN(
            get_logger(),
            "[LOCAL_ORIGIN_REBASE_CANCELLED] local position returned to the trusted frame");
        } else {
          const auto decision = local_origin_rebase_guard_.observe(
            position_ned, speed_mps, sample_now_ns);
          if (decision == race_offboard::LocalOriginRebaseDecision::CandidateReset) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[LOCAL_ORIGIN_REBASE_WAIT] candidate moved; restarting stable confirmation");
          }
          accepted_origin_rebase =
            decision == race_offboard::LocalOriginRebaseDecision::Confirmed;
          if (!accepted_origin_rebase) {
            return;
          }
        }
      }

      if (!plausible_jump && !accepted_origin_rebase) {
        const bool rebase_allowed = localOriginRebaseAllowed(sample_now_ns);
        if (rebase_allowed) {
          const auto decision = local_origin_rebase_guard_.observe(
            position_ned, speed_mps, sample_now_ns);
          if (decision == race_offboard::LocalOriginRebaseDecision::CandidateStarted) {
            RCLCPP_WARN(
              get_logger(),
              "[LOCAL_ORIGIN_REBASE_CANDIDATE] disarmed IDLE jump=%.3fm; "
              "requiring %.2fs stable confirmation",
              displacement_m, local_origin_rebase_stabilization_sec_);
          }
        } else if (armed_ && !planner_safety_failure_latched_) {
          // Armed flight cannot safely reinterpret a large local-frame change
          // as an origin reset. Keep the last trusted hold and let the normal
          // braking path stop the vehicle; no later EGO/global replan may
          // overwrite that decision without an explicit operator recovery.
          planner_safety_failure_latched_ = true;
          goal_active_ = false;
          navigation_cancelled_ = true;
          have_setpoint_ = false;
          have_ego_setpoint_ = false;
          awaiting_initial_ego_trajectory_ = false;
          beginBraking(ControlState::PLANNER_FAILURE_HOLD, "local position jump");
          RCLCPP_ERROR(
            get_logger(), "[POSITION_JUMP_SAFETY_LATCH] braking and holding; "
            "cancel/restart navigation only after localization is verified");
        }
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(), 1000,
          "[POSE_REJECT] jump=%.3fm dt=%.3fs trusted=(%.3f,%.3f,%.3f) received=(%.3f,%.3f,%.3f)",
          displacement_m, dt_sec, current_x_, current_y_, current_z_, x_ned, y_ned, z_ned);
        return;
      }

      if (accepted_origin_rebase) {
        RCLCPP_WARN(
          get_logger(),
          "[LOCAL_ORIGIN_REBASE_CONFIRMED] old=(%.3f,%.3f,%.3f) "
          "new=(%.3f,%.3f,%.3f); disarmed IDLE hold will be replaced",
          current_x_, current_y_, current_z_, x_ned, y_ned, z_ned);
        have_pose_vertical_speed_ = false;
      } else {
        accepted_pose_dt_sec = dt_sec;
      }
    }
    if (std::isfinite(accepted_pose_dt_sec) && accepted_pose_dt_sec >= 0.005 &&
      accepted_pose_dt_sec <= 0.25)
    {
      const double raw_pose_vz_ned =
        static_cast<double>(z_ned - current_z_) / accepted_pose_dt_sec;
      constexpr double kPoseVelocityFilterAlpha = 0.20;
      pose_vertical_speed_ned_ = have_pose_vertical_speed_ ?
        (1.0 - kPoseVelocityFilterAlpha) * pose_vertical_speed_ned_ +
        kPoseVelocityFilterAlpha * raw_pose_vz_ned : raw_pose_vz_ned;
      have_pose_vertical_speed_ = true;
    }
    current_x_ = x_ned;
    current_y_ = y_ned;
    current_z_ = z_ned;
    current_yaw_ = static_cast<float>(yaw_ned);
    // MAVROS pose carries no estimator validity flags.  A finite, monotonic,
    // jump-checked pose is the strongest statement available here, so the flags
    // degrade to "we have a usable sample".  require_strict_local_position_health
    // defaults to false, so this matches the previous effective behaviour; FCU
    // link loss is caught separately via State.connected in localPositionSafe().
    xy_valid_ = true;
    z_valid_ = true;
    heading_good_ = true;
    dead_reckoning_ = false;
    have_local_position_ = true;
    last_trusted_position_stamp_ = stamp;
    have_trusted_position_stamp_ = true;
    if (accepted_origin_rebase) {
      // Kept below with the accepted-pose assignment so no hold point can ever
      // be constructed from a candidate sample.
      hold_position_valid_ = false;
      last_command_valid_ = false;
      map_local_alignment_ready_ = false;
      lockHoldPosition(current_z_, "confirmed disarmed local origin rebase");
      setControlState(ControlState::IDLE_HOLD, "confirmed disarmed local origin rebase");
    }
    if (pending_takeoff_request_ && state_ == State::IDLE) {
      RCLCPP_INFO(
        get_logger(),
        "[OFFBOARD_DEFERRED_TAKEOFF_RELEASE] trusted_position=(%.3f,%.3f,%.3f)",
        current_x_, current_y_, current_z_);
      beginTakeoff("trusted local position available");
    }
    updateMapLocalAlignment();
  }

  void mapOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id != "map") {
      return;
    }
    map_position_enu_ = {
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    map_yaw_enu_ = tf2::getYaw(msg->pose.pose.orientation);
    have_map_odom_ = std::isfinite(map_position_enu_[0]) &&
      std::isfinite(map_position_enu_[1]) && std::isfinite(map_position_enu_[2]) &&
      std::isfinite(map_yaw_enu_);
    updateMapLocalAlignment();
  }

  void updateMapLocalAlignment()
  {
    if (map_local_alignment_ready_ || !have_map_odom_ || !have_local_position_) {
      return;
    }
    if (require_map_local_alignment_ &&
      (armed_ || horizontalSpeed() > initial_position_max_speed_mps_ ||
      !ev_flight_ready_.ready(now().nanoseconds())))
    {
      return;
    }
    const auto local_enu = race_offboard::nedToEnu({current_x_, current_y_, current_z_});
    const double local_yaw_enu = race_offboard::nedYawToEnu(current_yaw_);
    race_offboard::PlanarFrameTransform candidate;
    candidate.yaw = wrapAngle(local_yaw_enu - map_yaw_enu_);
    const auto rotated_map = race_offboard::mapToLocalVector(map_position_enu_, candidate);
    candidate.x = local_enu[0] - rotated_map[0];
    candidate.y = local_enu[1] - rotated_map[1];
    candidate.z = local_enu[2] - rotated_map[2];
    const double spread = std::hypot(
      std::hypot(
        candidate.x - map_to_local_candidate_.x,
        candidate.y - map_to_local_candidate_.y),
      candidate.z - map_to_local_candidate_.z);
    const double yaw_spread = std::fabs(
      wrapAngle(
        candidate.yaw - map_to_local_candidate_.yaw));
    if (!map_local_alignment_candidate_valid_ ||
      spread > map_local_alignment_max_spread_m_ ||
      yaw_spread > map_local_alignment_max_yaw_spread_rad_)
    {
      map_to_local_candidate_ = candidate;
      map_local_alignment_candidate_valid_ = true;
      map_local_alignment_stable_since_ = now();
      return;
    }
    if ((now() - map_local_alignment_stable_since_).seconds() <
      map_local_alignment_stabilization_sec_)
    {
      return;
    }
    map_to_local_ = candidate;
    map_local_alignment_ready_ = true;
    RCLCPP_WARN(
      get_logger(),
      "[MAP_LOCAL_ALIGNMENT_LOCKED] yaw=%.6f translation=(%.3f,%.3f,%.3f)",
      map_to_local_.yaw, map_to_local_.x, map_to_local_.y, map_to_local_.z);
  }

  void vehicleStatusCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool was_offboard = offboard_mode_;
    const bool was_armed = armed_;
    const bool was_position_mode = positionMode(last_vehicle_mode_);
    offboard_mode_ = (msg->mode == mavros_msgs::msg::State::MODE_PX4_OFFBOARD);
    armed_ = msg->armed;
    const bool position_mode = positionMode(msg->mode);
    last_vehicle_mode_ = msg->mode;
    // connected is MAVROS-only and has no PX4 DDS equivalent: it detects the
    // FCU serial link dropping, which the estimator flags never covered.
    connected_ = msg->connected;
    // The horizontal Offboard reference is the point where the pilot began
    // the POSITION/POSCTL takeoff.  Capture it once, before OFFBOARD, rather
    // than following any small lateral drift while the pilot is hovering.
    if (manual_handover_ && armed_ && !offboard_mode_ && position_mode &&
      !was_position_mode && have_local_position_ && finiteCurrentPosition())
    {
      manual_takeoff_x_ = current_x_;
      manual_takeoff_y_ = current_y_;
      manual_takeoff_yaw_ = holdYaw();
      manual_takeoff_position_valid_ = true;
      RCLCPP_INFO(
        get_logger(),
        "[MANUAL_POSITION_REFERENCE_LOCKED] takeoff_xy_ned=(%.3f,%.3f) yaw=%.3f",
        manual_takeoff_x_, manual_takeoff_y_, manual_takeoff_yaw_);
    }
    const bool automation_active = state_ == State::WARMUP || state_ == State::PREFLIGHT ||
      state_ == State::TAKEOFF || state_ == State::ACTIVE;
    if (!pilot_override_latched_ && race_offboard::pilotOverrideRequested(
        automation_active, was_offboard, offboard_mode_, was_armed, armed_))
    {
      const char * reason = was_armed && !armed_ ?
        "pilot disarmed vehicle" : "pilot left OFFBOARD";
      latchPilotOverride(reason);
    }
    if (was_armed && !armed_) {
      resetFlightAltitudeReference("vehicle disarmed");
      manual_takeoff_position_valid_ = false;
    }
  }

  bool positionMode(const std::string & mode) const
  {
    return mode == "POSITION" || mode == "POSCTL";
  }

  void evHealthCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (ev_health_.update(msg->data, now().nanoseconds())) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[EV_HEALTH] state=%s continuous_healthy=%.2fs",
        race_offboard::evHealthStateName(ev_health_.state()),
        ev_health_.healthyDurationS(now().nanoseconds()));
    }
  }

  void evFlightReadyCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const auto now_ns = now().nanoseconds();
    if (ev_flight_ready_.update(msg->data ? "HEALTHY" : "SUSPECT", now_ns)) {
      RCLCPP_INFO(
        get_logger(), "[EV_FLIGHT_READY] ready=%d continuous_ready=%.2fs",
        msg->data, ev_flight_ready_.healthyDurationS(now_ns));
    }
  }

  void plannerStatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    planner_status_ = msg->data;
    last_planner_status_time_ = now();
  }

  void globalPlannerStatusCallback(
    const race_msgs::msg::GlobalPlannerStatus::SharedPtr msg)
  {
    const bool progress_changed = have_global_planner_status_ &&
      (msg->global_path_id != global_planner_status_.global_path_id ||
      msg->local_goal_seq != global_planner_status_.local_goal_seq);
    global_planner_status_ = *msg;
    have_global_planner_status_ = true;
    if (require_global_planner_final_goal_ && progress_changed &&
      control_state_ == ControlState::GOAL_REACHED_HOLD)
    {
      setControlState(ControlState::PLANNING, "new global path/local goal clears final hold");
    }
  }

  void egoStatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    ego_status_ = msg->data;
    last_ego_status_time_ = now();
    if (race_offboard::egoPlannerStatusRequiresPermanentLatch(ego_status_)) {
      planner_safety_failure_latched_ = true;
      have_ego_setpoint_ = false;
      RCLCPP_ERROR(
        get_logger(), "[PLANNER_SAFETY_LATCH] status=%s; cancel navigation before recovery",
        ego_status_.c_str());
    }
  }

  void goalActivityCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (pilot_override_latched_ || state_ == State::MANUAL_OVERRIDE) {
      RCLCPP_WARN(
        get_logger(),
        "[GOAL_REJECTED_MANUAL_OVERRIDE] restart the navigation stack before resuming automation");
      return;
    }
    if (!flight_altitude_reference_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "[FINAL_GOAL_REJECT] no valid flight altitude reference; finish takeoff first");
      return;
    }
    // /goal_pose is an XY user intent in flat mode; validate it at the actual
    // map-frame flight level, not at the AGL parameter value.
    const double goal_z = target_z_map_;
    const double shared_min_z_map = ground_z_map_ + shared_bounds_z_min_;
    const double shared_max_z_map = ground_z_map_ + shared_bounds_z_max_;
    if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y) ||
      msg->pose.position.x < shared_bounds_x_min_ || msg->pose.position.x > shared_bounds_x_max_ ||
      msg->pose.position.y < shared_bounds_y_min_ || msg->pose.position.y > shared_bounds_y_max_ ||
      goal_z < shared_min_z_map || goal_z > shared_max_z_map)
    {
      RCLCPP_ERROR(
        get_logger(), "FINAL_GOAL_OUT_OF_SHARED_BOUNDS map=(%.3f,%.3f,%.3f)",
        msg->pose.position.x, msg->pose.position.y, goal_z);
      return;
    }
    // RViz goal is in the unified map ENU. Keep the internal command in NED
    // only at this Offboard boundary: (x_ned,y_ned,z_ned)=(y_map,x_map,-z_map).
    const float goal_x_ned = static_cast<float>(msg->pose.position.y);
    const float goal_y_ned = static_cast<float>(msg->pose.position.x);
    const bool duplicate_goal = have_goal_identity_ &&
      std::hypot(goal_x_ned - goal_x_ned_, goal_y_ned - goal_y_ned_) <=
      goal_update_position_tolerance_m_;
    if (duplicate_goal &&
      !(planner_safety_failure_latched_ && same_goal_retry_on_safety_latch_))
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignore duplicate final goal within %.2fm", goal_update_position_tolerance_m_);
      return;
    }
    if (duplicate_goal) {
      RCLCPP_INFO(
        get_logger(),
        "[OFFBOARD_SAME_GOAL_RETRY] clearing safety latch for an explicit retry");
    }
    goal_x_ned_ = goal_x_ned;
    goal_y_ned_ = goal_y_ned;
    have_goal_identity_ = true;
    ++active_goal_id_;
    goal_active_ = true;
    navigation_cancelled_ = false;
    if (race_offboard::canResetPlannerLatchForNewFinalGoal(
        true, state_ == State::FINISHED || state_ == State::LANDING))
    {
      // This is an explicit new operator goal, not an automatic recovery of
      // the same failed transaction.  Do not reuse the prior safe/unsafe EGO
      // command; the normal freshness and Bridge-validation gates below must
      // admit a newly received trajectory before TRACKING can resume.
      planner_safety_failure_latched_ = false;
      have_ego_setpoint_ = false;
      RCLCPP_INFO(
        get_logger(),
        "[OFFBOARD_NEW_GOAL_RESET] old safety latch cleared; waiting for a fresh validated EGO trajectory");
    }
    have_global_planner_status_ = false;
    global_planner_status_ = race_msgs::msg::GlobalPlannerStatus{};
    planner_status_.clear();
    ego_status_.clear();
    last_planner_status_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_ego_status_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    awaiting_initial_ego_trajectory_ = control_source_ == "ego";
    minimum_ego_command_goal_seq_ = ego_command_goal_seq_;
    initial_ego_trajectory_wait_start_ = now();
    if (awaiting_initial_ego_trajectory_ && state_ == State::ACTIVE && finiteCurrentPosition()) {
      // Do not reuse the takeoff/previous-goal hold point while the new
      // planner transaction is preparing its first command.
      lockHoldPosition(
        static_cast<float>(race_offboard::fixedAltitudeHoldZ(targetFlightZNed())),
        "awaiting initial EGO trajectory for new final goal");
    }
    setControlState(ControlState::PLANNING, "new final goal received");
    RCLCPP_INFO(
      get_logger(), "Active final goal id=%lu NED=(%.3f,%.3f)",
      active_goal_id_, goal_x_ned_, goal_y_ned_);
    if (control_source_ == "ego") {
      beginTakeoff("new EGO goal");
    }
  }

  void navigationSetpointCallback(const race_msgs::msg::NavigationSetpoint::SharedPtr msg)
  {
    if (control_source_ != "navigation") {
      return;
    }
    if (navigation_cancelled_) {
      return;
    }
    if (require_map_local_alignment_ && !map_local_alignment_ready_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject navigation setpoint until map -> PX4 local alignment is locked");
      return;
    }
    if (msg->header.frame_id != required_setpoint_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Reject navigation setpoint frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), required_setpoint_frame_.c_str());
      return;
    }
    if (!finitePosition(*msg) || !std::isfinite(msg->yaw)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(), 1000, "Reject non-finite navigation setpoint");
      return;
    }
    if (msg->velocity_valid &&
      (!std::isfinite(msg->velocity.x) || !std::isfinite(msg->velocity.y) ||
      !std::isfinite(msg->velocity.z)))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject invalid velocity feed-forward");
      return;
    }
    if (msg->yaw_rate_valid && !std::isfinite(msg->yaw_rate)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject invalid yaw-rate feed-forward");
      return;
    }

    const double commanded_height = heightAglForLocalNed(msg->position.z);
    if (commanded_height < min_command_height_m_ || commanded_height > max_command_height_m_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject navigation height %.2fm outside hard safety range [%.2f, %.2f]m",
        commanded_height, min_command_height_m_, max_command_height_m_);
      return;
    }

    latest_setpoint_ = *msg;
    if (map_local_alignment_ready_) {
      const auto map_enu = race_offboard::nedToEnu(
        {msg->position.x, msg->position.y, msg->position.z});
      const auto local_enu = race_offboard::mapToLocalPosition(map_enu, map_to_local_);
      const auto local_ned = race_offboard::enuToNed(local_enu);
      latest_setpoint_.position.x = local_ned[0];
      latest_setpoint_.position.y = local_ned[1];
      latest_setpoint_.position.z = local_ned[2];
      const double map_yaw_enu = race_offboard::nedYawToEnu(msg->yaw);
      latest_setpoint_.yaw = race_offboard::enuYawToNed(
        race_offboard::mapToLocalYaw(map_yaw_enu, map_to_local_));
      if (msg->velocity_valid) {
        const auto velocity_enu = race_offboard::nedToEnu(
          {msg->velocity.x, msg->velocity.y, msg->velocity.z});
        const auto local_velocity_enu =
          race_offboard::mapToLocalVector(velocity_enu, map_to_local_);
        const auto local_velocity_ned = race_offboard::enuToNed(local_velocity_enu);
        latest_setpoint_.velocity.x = local_velocity_ned[0];
        latest_setpoint_.velocity.y = local_velocity_ned[1];
        latest_setpoint_.velocity.z = local_velocity_ned[2];
      }
    }
    latest_setpoint_.yaw = wrapAngle(latest_setpoint_.yaw);
    have_setpoint_ = true;
    last_setpoint_time_ = now();
    beginTakeoff("navigation_setpoint");

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Navigation setpoint NED=(%.2f, %.2f, %.2f) yaw=%.2f yaw_rate=%s%.2f",
      latest_setpoint_.position.x, latest_setpoint_.position.y, latest_setpoint_.position.z,
      latest_setpoint_.yaw, latest_setpoint_.yaw_rate_valid ? "" : "disabled/",
      latest_setpoint_.yaw_rate);
  }

  void egoSetpointCallback(const mavros_msgs::msg::PositionTarget::SharedPtr raw)
  {
    if (control_source_ != "ego") {
      return;
    }
    if (navigation_cancelled_) {
      return;
    }
    if (require_map_local_alignment_ && !map_local_alignment_ready_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject EGO setpoint until map -> PX4 local alignment is locked");
      return;
    }
    // ENU -> internal NED, preserving per-axis "ignored" as NaN.
    using PT = mavros_msgs::msg::PositionTarget;
    EgoCommandNed command;
    command.position = race_offboard::enuToNed(
      {raw->position.x, raw->position.y, raw->position.z});
    command.velocity = race_offboard::enuToNed(
      egoAxisOrNan(
        raw->velocity, raw->type_mask & PT::IGNORE_VX,
        raw->type_mask & PT::IGNORE_VY, raw->type_mask & PT::IGNORE_VZ));
    command.acceleration = race_offboard::enuToNed(
      egoAxisOrNan(
        raw->acceleration_or_force, raw->type_mask & PT::IGNORE_AFX,
        raw->type_mask & PT::IGNORE_AFY, raw->type_mask & PT::IGNORE_AFZ));
    command.yaw = (raw->type_mask & PT::IGNORE_YAW) ?
      std::numeric_limits<double>::quiet_NaN() :
      race_offboard::enuYawToNed(raw->yaw);
    // ENU yaw rate is about Up, NED about Down: opposite sense.
    command.yaw_rate = (raw->type_mask & PT::IGNORE_YAW_RATE) ?
      std::numeric_limits<double>::quiet_NaN() : -raw->yaw_rate;
    if (map_local_alignment_ready_) {
      const auto local_position_enu = race_offboard::mapToLocalPosition(
        {raw->position.x, raw->position.y, raw->position.z}, map_to_local_);
      command.position = race_offboard::enuToNed(local_position_enu);
      const auto local_velocity_enu = race_offboard::mapToLocalVector(
        {raw->velocity.x, raw->velocity.y, raw->velocity.z}, map_to_local_);
      const auto local_acceleration_enu = race_offboard::mapToLocalVector(
        {raw->acceleration_or_force.x, raw->acceleration_or_force.y,
          raw->acceleration_or_force.z}, map_to_local_);
      command.velocity = race_offboard::enuToNed(
        egoAxisOrNan(
          geometry_msgs::msg::Vector3().set__x(local_velocity_enu[0]).set__y(
            local_velocity_enu[1]).set__z(local_velocity_enu[2]),
          raw->type_mask & PT::IGNORE_VX, raw->type_mask & PT::IGNORE_VY,
          raw->type_mask & PT::IGNORE_VZ));
      command.acceleration = race_offboard::enuToNed(
        egoAxisOrNan(
          geometry_msgs::msg::Vector3().set__x(local_acceleration_enu[0]).set__y(
            local_acceleration_enu[1]).set__z(local_acceleration_enu[2]),
          raw->type_mask & PT::IGNORE_AFX, raw->type_mask & PT::IGNORE_AFY,
          raw->type_mask & PT::IGNORE_AFZ));
      if (!(raw->type_mask & PT::IGNORE_YAW)) {
        command.yaw = race_offboard::enuYawToNed(
          race_offboard::mapToLocalYaw(raw->yaw, map_to_local_));
      }
    }
    const auto msg = &command;
    if (awaiting_initial_ego_trajectory_ &&
      ego_command_goal_seq_ <= minimum_ego_command_goal_seq_)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[OFFBOARD_STALE_EGO_GOAL_REJECT] command_local_goal_seq=%lu required_after=%lu",
        ego_command_goal_seq_, minimum_ego_command_goal_seq_);
      return;
    }
    if (!finiteEgoPosition(*msg) || !finiteOrAllNan(msg->velocity) ||
      !finiteOrAllNan(msg->acceleration) ||
      (!std::isfinite(msg->yaw_rate) && !std::isnan(msg->yaw_rate)))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Reject invalid EGO trajectory setpoint");
      return;
    }
    const double commanded_height = heightAglForLocalNed(msg->position[2]);
    if (commanded_height < min_command_height_m_ || commanded_height > max_command_height_m_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject EGO height %.2fm outside hard safety range [%.2f, %.2f]m",
        commanded_height, min_command_height_m_, max_command_height_m_);
      return;
    }
    if (have_local_position_) {
      const double horizontal_distance_m = std::hypot(
        msg->position[0] - current_x_, msg->position[1] - current_y_);
      if (!race_offboard::egoSetpointIsNearTrustedPosition(
          horizontal_distance_m, ego_setpoint_max_lead_m_))
      {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[EGO_SETPOINT_REJECT] lead=%.3fm limit=%.3fm trusted=(%.3f,%.3f) requested=(%.3f,%.3f)",
          horizontal_distance_m, ego_setpoint_max_lead_m_, current_x_, current_y_,
          msg->position[0], msg->position[1]);
        return;
      }
    }
    latest_ego_setpoint_ = *msg;
    latest_ego_setpoint_.yaw = wrapAngle(latest_ego_setpoint_.yaw);
    have_ego_setpoint_ = true;
    last_ego_setpoint_time_ = now();
    if (awaiting_initial_ego_trajectory_) {
      awaiting_initial_ego_trajectory_ = false;
      RCLCPP_INFO(
        get_logger(),
        "[OFFBOARD_INITIAL_EGO_TRAJECTORY_READY] goal_id=%lu wait=%.3fs",
        active_goal_id_, (now() - initial_ego_trajectory_wait_start_).seconds());
    }
    // A validated EGO stream is the handover input, not a new takeoff
    // request.  Restarting WARMUP here for every incoming setpoint resets
    // TAKEOFF before it can reach the handover gate and repeatedly relocks
    // XY to the moving vehicle.
    if (race_offboard::shouldRestartTakeoffForEgoSetpoint()) {
      beginTakeoff("ego_trajectory_setpoint");
    }
  }

  void egoCommandGoalSeqCallback(const std_msgs::msg::UInt64::SharedPtr msg)
  {
    ego_command_goal_seq_ = msg->data;
  }

  void takeoffCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ == State::FINISHED || state_ == State::LANDING ||
      state_ == State::MANUAL_OVERRIDE || pilot_override_latched_)
    {
      response->success = false;
      response->message =
        "Landing, finished, or pilot override latched; restart the node before another takeoff.";
      return;
    }
    beginTakeoff("takeoff_service");
    response->success = true;
    response->message =
      "Offboard vertical takeoff sequence active; EGO handover remains gated by a fresh validated trajectory.";
  }

  void landCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ == State::IDLE || state_ == State::FINISHED ||
      state_ == State::MANUAL_OVERRIDE || pilot_override_latched_)
    {
      response->success = false;
      response->message =
        "Vehicle is not in an active offboard flight or pilot override owns the vehicle.";
      return;
    }
    state_ = State::LANDING;
    response->success = true;
    response->message = "PX4 land command requested.";
  }

  void cancelCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    goal_active_ = false;
    navigation_cancelled_ = true;
    have_setpoint_ = false;
    have_ego_setpoint_ = false;
    planner_safety_failure_latched_ = false;
    beginBraking(ControlState::IDLE_HOLD, "goal cancelled");
    response->success = true;
    response->message = "Navigation cancelled; braking then locked position hold.";
  }

  const char * controlStateName(ControlState state) const
  {
    switch (state) {
      case ControlState::WAITING_FOR_ODOM: return "WAITING_FOR_ODOM";
      case ControlState::IDLE_HOLD: return "IDLE_HOLD";
      case ControlState::PLANNING: return "PLANNING";
      case ControlState::TRACKING: return "TRACKING";
      case ControlState::BRAKING: return "BRAKING";
      case ControlState::GOAL_REACHED_HOLD: return "GOAL_REACHED_HOLD";
      case ControlState::PLANNER_FAILURE_HOLD: return "PLANNER_FAILURE_HOLD";
      case ControlState::EMERGENCY_STOP: return "EMERGENCY_STOP";
      case ControlState::MANUAL_OVERRIDE: return "MANUAL_OVERRIDE";
    }
    return "UNKNOWN";
  }

  void setControlState(ControlState next, const std::string & reason)
  {
    if (next == control_state_) {
      control_reason_ = reason;
      return;
    }
    RCLCPP_WARN(
      get_logger(), "[CONTROL_STATE] %s -> %s: %s",
      controlStateName(control_state_), controlStateName(next), reason.c_str());
    control_state_ = next;
    control_reason_ = reason;
  }

  double horizontalSpeed() const
  {
    return std::hypot(static_cast<double>(current_vx_), static_cast<double>(current_vy_));
  }

  bool groundReferenceLockAllowed(std::int64_t now_ns) const
  {
    return race_offboard::initialGroundReferenceLockAllowed(
      armed_, require_ev_health_, ev_flight_ready_.ready(now_ns));
  }

  bool localOriginRebaseAllowed(std::int64_t now_ns) const
  {
    return require_ev_health_ && race_offboard::localOriginRebaseAllowed(
      armed_, state_ == State::IDLE, control_state_ == ControlState::IDLE_HOLD,
      ev_flight_ready_.ready(now_ns), hold_position_valid_);
  }

  void lockHoldPosition(float z_command, const std::string & reason)
  {
    if (!finiteCurrentPosition()) {
      return;
    }
    updateHoldPosition(z_command);
    RCLCPP_WARN(
      get_logger(), "[HOLD_POSITION] x=%.3f y=%.3f z=%.3f yaw=%.3f reason=%s",
      hold_x_, hold_y_, hold_z_, hold_yaw_, reason.c_str());
  }

  void updateHoldPosition(float z_command)
  {
    hold_x_ = current_x_;
    hold_y_ = current_y_;
    hold_z_ = z_command;
    hold_yaw_ = holdYaw();
    hold_position_valid_ = true;
  }

  void captureManualTakeoffPositionIfNeeded()
  {
    if (manual_takeoff_position_valid_ || !manual_handover_ || !armed_ ||
      offboard_mode_ || !positionMode(last_vehicle_mode_) || !finiteCurrentPosition())
    {
      return;
    }
    manual_takeoff_x_ = current_x_;
    manual_takeoff_y_ = current_y_;
    manual_takeoff_yaw_ = holdYaw();
    manual_takeoff_position_valid_ = true;
    RCLCPP_INFO(
      get_logger(),
      "[MANUAL_POSITION_REFERENCE_LOCKED] takeoff_xy_ned=(%.3f,%.3f) yaw=%.3f",
      manual_takeoff_x_, manual_takeoff_y_, manual_takeoff_yaw_);
  }

  void updateManualHandoverHoldPosition(float z_command)
  {
    captureManualTakeoffPositionIfNeeded();
    if (!manual_takeoff_position_valid_) {
      updateHoldPosition(z_command);
      return;
    }
    hold_x_ = manual_takeoff_x_;
    hold_y_ = manual_takeoff_y_;
    hold_z_ = z_command;
    hold_yaw_ = manual_takeoff_yaw_;
    hold_position_valid_ = true;
  }

  void lockManualHandoverPosition(float z_command, const std::string & reason)
  {
    if (!finiteCurrentPosition()) {
      return;
    }
    updateManualHandoverHoldPosition(z_command);
    RCLCPP_WARN(
      get_logger(), "[HOLD_POSITION] x=%.3f y=%.3f z=%.3f yaw=%.3f reason=%s",
      hold_x_, hold_y_, hold_z_, hold_yaw_, reason.c_str());
  }

  double holdPositionError() const
  {
    return hold_position_valid_ && finiteCurrentPosition() ?
           std::hypot(
      std::hypot(current_x_ - hold_x_, current_y_ - hold_y_), current_z_ - hold_z_) :
           std::numeric_limits<double>::infinity();
  }

  double currentHeightAgl() const
  {
    return flight_altitude_reference_valid_ ?
           ground_z_local_ned_ - static_cast<double>(current_z_) :
           -static_cast<double>(current_z_);
  }

  double heightAglForLocalNed(double z_ned) const
  {
    return flight_altitude_reference_valid_ ? ground_z_local_ned_ - z_ned : -z_ned;
  }

  float targetFlightZNed() const
  {
    return flight_altitude_reference_valid_ ?
           static_cast<float>(target_z_local_ned_) :
           -static_cast<float>(cruise_altitude_m_);
  }

  void publishFlightAltitudeReference()
  {
    race_msgs::msg::FlightAltitudeReference reference;
    reference.header.stamp = now();
    reference.header.frame_id = "map";
    reference.flight_id = flight_id_;
    reference.valid = flight_altitude_reference_valid_;
    reference.target_agl_m = flight_target_agl_m_;
    reference.min_agl_m = min_command_height_m_;
    reference.max_agl_m = max_command_height_m_;
    reference.ground_z_local_ned = ground_z_local_ned_;
    reference.target_z_local_ned = target_z_local_ned_;
    reference.ground_z_map = ground_z_map_;
    reference.target_z_map = target_z_map_;
    flight_altitude_reference_publisher_->publish(reference);
  }

  bool lockFlightAltitudeReference()
  {
    if (flight_altitude_reference_valid_) {
      return true;
    }
    if (armed_ || !have_local_position_ || !finiteCurrentPosition() ||
      !have_map_odom_ || !map_local_alignment_ready_)
    {
      return false;
    }
    flight_target_agl_m_ = cruise_altitude_m_;
    ground_z_local_ned_ = static_cast<double>(current_z_);
    target_z_local_ned_ = ground_z_local_ned_ - flight_target_agl_m_;
    const auto local_enu = race_offboard::nedToEnu({current_x_, current_y_, current_z_});
    ground_z_map_ = local_enu[2] - map_to_local_.z;
    target_z_map_ = ground_z_map_ + flight_target_agl_m_;
    ++flight_id_;
    flight_altitude_reference_valid_ = true;
    publishFlightAltitudeReference();
    RCLCPP_WARN(
      get_logger(),
      "[FLIGHT_ALTITUDE_REFERENCE_LOCKED] flight_id=%lu ground_local_ned=%.3f "
      "target_local_ned=%.3f ground_map=%.3f target_map=%.3f target_agl=%.3f",
      static_cast<unsigned long>(flight_id_), ground_z_local_ned_, target_z_local_ned_,
      ground_z_map_, target_z_map_, flight_target_agl_m_);
    return true;
  }

  // During a manual airborne handover, PX4's local origin remains the ground
  // reference.  Preserve the measured POSITION altitude exactly instead of
  // commanding the automatic-takeoff cruise height after the mode switch.
  bool lockManualHandoverFlightAltitudeReference()
  {
    if (flight_altitude_reference_valid_) {
      return true;
    }
    if (!have_local_position_ || !finiteCurrentPosition() || !have_map_odom_ ||
      !map_local_alignment_ready_)
    {
      return false;
    }
    flight_target_agl_m_ = -static_cast<double>(current_z_);
    if (!std::isfinite(flight_target_agl_m_) ||
      flight_target_agl_m_ < min_command_height_m_ ||
      flight_target_agl_m_ > max_command_height_m_)
    {
      return false;
    }
    const auto local_enu = race_offboard::nedToEnu({current_x_, current_y_, current_z_});
    target_z_local_ned_ = static_cast<double>(current_z_);
    target_z_map_ = local_enu[2] - map_to_local_.z;
    ground_z_local_ned_ = target_z_local_ned_ + flight_target_agl_m_;
    ground_z_map_ = target_z_map_ - flight_target_agl_m_;
    ++flight_id_;
    flight_altitude_reference_valid_ = true;
    publishFlightAltitudeReference();
    RCLCPP_WARN(
      get_logger(),
      "[MANUAL_FLIGHT_ALTITUDE_REFERENCE_LOCKED] flight_id=%lu ground_local_ned=%.3f "
      "target_local_ned=%.3f ground_map=%.3f target_map=%.3f target_agl=%.3f",
      static_cast<unsigned long>(flight_id_), ground_z_local_ned_, target_z_local_ned_,
      ground_z_map_, target_z_map_, flight_target_agl_m_);
    return true;
  }

  void resetFlightAltitudeReference(const char * reason)
  {
    if (!flight_altitude_reference_valid_) {
      return;
    }
    flight_altitude_reference_valid_ = false;
    publishFlightAltitudeReference();
    RCLCPP_WARN(
      get_logger(), "[FLIGHT_ALTITUDE_REFERENCE_RESET] flight_id=%lu reason=%s",
      static_cast<unsigned long>(flight_id_), reason);
  }

  bool manualHandoverHeightSafe() const
  {
    const double height = currentHeightAgl();
    return std::isfinite(height) && height >= min_command_height_m_ &&
           height <= max_command_height_m_;
  }

  bool manualHandoverReady()
  {
    const bool ev_ready = !require_ev_health_ ||
      ev_flight_ready_.ready(now().nanoseconds());
    const bool altitude_reference_ready = have_map_odom_ && map_local_alignment_ready_;
    return altitude_reference_ready &&
           race_offboard::canAcceptManualHandover(
      manual_handover_, state_ == State::IDLE, armed_, offboard_mode_,
      localPositionSafe(), ev_ready,
      holdPositionError() <= manual_handover_max_position_error_m_,
      horizontalSpeed() <= manual_handover_max_speed_mps_, manualHandoverHeightSafe());
  }

  void acceptManualHandover()
  {
    if (!lockManualHandoverFlightAltitudeReference()) {
      RCLCPP_ERROR(
        get_logger(),
        "[MANUAL_HANDOVER_REJECTED] cannot lock the current POSITION altitude reference");
      return;
    }
    lockManualHandoverPosition(current_z_, "manual handover accepted");
    state_ = State::ACTIVE;
    state_enter_time_ = now();
    manual_handover_accepted_ = true;
    setControlState(ControlState::IDLE_HOLD, "manual handover accepted");
    RCLCPP_WARN(
      get_logger(), "[MANUAL_HANDOVER_ACCEPTED] current NED=(%.3f,%.3f,%.3f)",
      current_x_, current_y_, current_z_);
  }

  void latchPilotOverride(const std::string & reason)
  {
    if (pilot_override_latched_) {
      return;
    }
    pilot_override_latched_ = true;
    manual_handover_accepted_ = false;
    pending_takeoff_request_ = false;
    goal_active_ = false;
    navigation_cancelled_ = true;
    have_setpoint_ = false;
    have_ego_setpoint_ = false;
    awaiting_initial_ego_trajectory_ = false;
    state_ = State::MANUAL_OVERRIDE;
    setControlState(ControlState::MANUAL_OVERRIDE, reason);
    RCLCPP_ERROR(
      get_logger(),
      "[PILOT_OVERRIDE_LATCHED] reason=%s; no more OFFBOARD/arm requests or setpoints "
      "will be sent until the navigation stack is restarted",
      reason.c_str());
  }

  void beginBraking(ControlState destination, const std::string & reason)
  {
    if (control_state_ == ControlState::BRAKING) {
      return;
    }
    const float cruise_z = targetFlightZNed();
    lockHoldPosition(
      static_cast<float>(race_offboard::fixedAltitudeHoldZ(cruise_z)), reason);
    braking_vx_ = std::isfinite(current_vx_) ? current_vx_ : 0.0F;
    braking_vy_ = std::isfinite(current_vy_) ? current_vy_ : 0.0F;
    braking_vz_ = std::isfinite(current_vz_) ? current_vz_ : 0.0F;
    const double speed = std::hypot(
      std::hypot(static_cast<double>(braking_vx_), static_cast<double>(braking_vy_)),
      static_cast<double>(braking_vz_));
    braking_duration_sec_ = std::clamp(speed / braking_max_acc_, 0.20, 1.50);
    braking_start_time_ = now();
    post_braking_state_ = destination;
    setControlState(ControlState::BRAKING, reason);
  }

  void publishSafetySetpoint(float vx, float vy, float vz, float ax, float ay, float az)
  {
    if (!hold_position_valid_) {
      return;
    }
    // Hold and braking always supply finite position, velocity and
    // acceleration, so no feed-forward axis is masked off here.
    const auto position_enu = race_offboard::nedToEnu({hold_x_, hold_y_, hold_z_});
    const auto velocity_enu = race_offboard::nedToEnu({vx, vy, vz});
    const auto acceleration_enu = race_offboard::nedToEnu({ax, ay, az});
    mavros_msgs::msg::PositionTarget msg{};
    msg.header.stamp = now();
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = race_offboard::positionTargetTypeMask(velocity_enu, acceleration_enu, true);
    msg.position.x = position_enu[0];
    msg.position.y = position_enu[1];
    msg.position.z = position_enu[2];
    msg.velocity.x = velocity_enu[0];
    msg.velocity.y = velocity_enu[1];
    msg.velocity.z = velocity_enu[2];
    msg.acceleration_or_force.x = acceleration_enu[0];
    msg.acceleration_or_force.y = acceleration_enu[1];
    msg.acceleration_or_force.z = acceleration_enu[2];
    msg.yaw = static_cast<float>(race_offboard::nedYawToEnu(hold_yaw_));
    msg.yaw_rate = 0.0F;
    trajectory_setpoint_publisher_->publish(msg);
    last_command_x_ = hold_x_;
    last_command_y_ = hold_y_;
    last_command_yaw_ = hold_yaw_;
    last_command_valid_ = true;
  }

  void publishLockedHold()
  {
    publishSafetySetpoint(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    const double error = hold_position_valid_ && finiteCurrentPosition() ?
      std::hypot(
      std::hypot(current_x_ - hold_x_, current_y_ - hold_y_), current_z_ - hold_z_) :
      std::numeric_limits<double>::infinity();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[HOLD_ERROR] position_error=%.3f velocity=%.3f state=%s",
      error, horizontalSpeed(), controlStateName(control_state_));
  }

  void runBraking()
  {
    if (!hold_position_valid_) {
      setControlState(ControlState::WAITING_FOR_ODOM, "cannot brake without odometry");
      return;
    }
    const double elapsed = (now() - braking_start_time_).seconds();
    const double ratio = std::clamp(elapsed / braking_duration_sec_, 0.0, 1.0);
    const float scale = static_cast<float>(1.0 - ratio);
    const float ax = -braking_vx_ / static_cast<float>(braking_duration_sec_);
    const float ay = -braking_vy_ / static_cast<float>(braking_duration_sec_);
    const float az = -braking_vz_ / static_cast<float>(braking_duration_sec_);
    publishSafetySetpoint(
      braking_vx_ * scale, braking_vy_ * scale, braking_vz_ * scale,
      ax, ay, az);
    if (ratio >= 1.0 || horizontalSpeed() <= hold_velocity_tolerance_) {
      lockHoldPosition(hold_z_, "vehicle stopped after braking");
      setControlState(post_braking_state_, "vehicle stopped");
      publishLockedHold();
    }
  }

  void publishControlDiagnostics(const std::string & source)
  {
    const double hold_error = holdPositionError();
    const double current_height = currentHeightAgl();
    const double hold_height = heightAglForLocalNed(hold_z_);
    const bool position_valid = have_local_position_ && finiteCurrentPosition();
    const bool ev_ready = !require_ev_health_ ||
      ev_flight_ready_.ready(now().nanoseconds());
    const bool position_aligned = hold_position_valid_ &&
      hold_error <= manual_handover_max_position_error_m_;
    const bool speed_safe = horizontalSpeed() <= manual_handover_max_speed_mps_;
    const bool height_safe = manualHandoverHeightSafe();
    const bool pre_offboard_ready =
      manual_handover_ && state_ == State::IDLE && armed_ && !offboard_mode_ &&
      position_valid && ev_ready && position_aligned && speed_safe && height_safe;
    std_msgs::msg::String status;
    status.data = "state=" + std::string(controlStateName(control_state_)) +
      " source=" + source + " reason=" + control_reason_ +
      " manual_handover=" + (manual_handover_ ? "1" : "0") +
      " pilot_override=" + (pilot_override_latched_ ? "1" : "0") +
      " armed=" + (armed_ ? "1" : "0") +
      " offboard=" + (offboard_mode_ ? "1" : "0") +
      " handover_ready=" + (pre_offboard_ready ? "1" : "0") +
      " handover_accepted=" + (manual_handover_accepted_ ? "1" : "0") +
      " map_local_alignment=" + (map_local_alignment_ready_ ? "1" : "0") +
      " position_valid=" + (position_valid ? "1" : "0") +
      " ev_ready=" + (ev_ready ? "1" : "0") +
      " position_aligned=" + (position_aligned ? "1" : "0") +
      " speed_safe=" + (speed_safe ? "1" : "0") +
      " height_safe=" + (height_safe ? "1" : "0") +
      " current_height_m=" + std::to_string(current_height) +
      " current_agl_m=" + std::to_string(current_height) +
      " altitude_reference_valid=" + (flight_altitude_reference_valid_ ? "1" : "0") +
      " flight_id=" + std::to_string(flight_id_) +
      " ground_z_local_ned=" + std::to_string(ground_z_local_ned_) +
      " target_z_local_ned=" + std::to_string(target_z_local_ned_) +
      " hold_height_m=" + std::to_string(hold_height) +
      " hold_error_m=" + std::to_string(hold_error) +
      " speed_mps=" + std::to_string(horizontalSpeed()) +
      " vertical_speed_mps=" + std::to_string(std::abs(current_vz_)) +
      " pose_vertical_speed_mps=" +
      (have_pose_vertical_speed_ ? std::to_string(std::abs(pose_vertical_speed_ned_)) : "nan") +
      " vertical_speed_disagreement_mps=" +
      (have_pose_vertical_speed_ ?
      std::to_string(std::abs(static_cast<double>(current_vz_) - pose_vertical_speed_ned_)) :
      "nan");
    control_status_publisher_->publish(status);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[CONTROL_ARBITER] selected_source=%s state=%s",
      source.c_str(), controlStateName(control_state_));

    if (last_publisher_check_time_.nanoseconds() == 0 ||
      (now() - last_publisher_check_time_).seconds() >= 2.0)
    {
      // Watch the topic actually in use.  This used to poll
      // /fmu/in/trajectory_setpoint, which nothing publishes after the MAVROS
      // port, so it reported a permanent false conflict every 2 s while the
      // real setpoint topic went unmonitored.
      const auto publishers =
        get_publishers_info_by_topic("/mavros/setpoint_raw/local");
      if (publishers.size() != 1) {
        RCLCPP_WARN(
          get_logger(),
          "[PUBLISHER_CONFLICT] topic=/mavros/setpoint_raw/local count=%zu expected=1",
          publishers.size());
      }
      last_publisher_check_time_ = now();
    }
  }

  void beginTakeoff(const char * source)
  {
    if (!race_offboard::canStartIndependentTakeoff(
        state_ == State::LANDING || state_ == State::MANUAL_OVERRIDE ||
        state_ == State::FINISHED))
    {
      return;
    }
    if (state_ != State::IDLE) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[OFFBOARD_TAKEOFF_IGNORED] state=%d reason=%s; preserve current flight transaction",
        static_cast<int>(state_), source);
      return;
    }
    if (armed_) {
      local_origin_rebase_guard_.reset();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[OFFBOARD_TAKEOFF_REJECTED] node is IDLE but MAVROS reports armed; "
        "automatic ground-reference locking is forbidden in flight");
      return;
    }
    if (!groundReferenceLockAllowed(now().nanoseconds())) {
      pending_takeoff_request_ = true;
      hold_position_valid_ = false;
      last_command_valid_ = false;
      local_origin_rebase_guard_.reset();
      setControlState(
        ControlState::WAITING_FOR_ODOM,
        "takeoff queued until EV is continuously healthy");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[OFFBOARD_DEFERRED_TAKEOFF_EV] reason=%s; continuous_healthy=%.2fs required=%.2fs",
        source, ev_flight_ready_.healthyDurationS(now().nanoseconds()),
        ev_flight_ready_.requiredS());
      return;
    }
    if (!have_local_position_ || !finiteCurrentPosition()) {
      pending_takeoff_request_ = true;
      setControlState(
        ControlState::WAITING_FOR_ODOM,
        "takeoff queued until trusted local position");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[OFFBOARD_DEFERRED_TAKEOFF] reason=%s; waiting for trusted PX4 local position",
        source);
      return;
    }
    if (!lockFlightAltitudeReference()) {
      pending_takeoff_request_ = true;
      setControlState(
        ControlState::WAITING_FOR_ODOM,
        "takeoff queued until map/local altitude reference is ready");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[OFFBOARD_DEFERRED_ALTITUDE_REFERENCE] reason=%s map_odom=%d alignment=%d",
        source, have_map_odom_, map_local_alignment_ready_);
      return;
    }
    const State previous = state_;
    state_ = State::WARMUP;
    pending_takeoff_request_ = false;
    state_enter_time_ = now();
    position_ready_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_command_request_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    if (have_local_position_ && finiteCurrentPosition()) {
      lockHoldPosition(targetFlightZNed(), "takeoff XY lock");
    }
    RCLCPP_INFO(
      get_logger(),
      "[OFFBOARD_TAKEOFF_TRANSITION] from=%d to=%d current_position=(%.3f,%.3f,%.3f) "
      "takeoff_target=(%.3f,%.3f,%.3f) reason=%s",
      static_cast<int>(previous), static_cast<int>(state_), current_x_, current_y_, current_z_,
      current_x_, current_y_, targetFlightZNed(), source);
  }

  void timerCallback()
  {
    const float cruise_z = targetFlightZNed();
    if (state_ == State::FINISHED) {
      return;
    }
    if (pilot_override_latched_ || state_ == State::MANUAL_OVERRIDE) {
      publishControlDiagnostics("pilot_override");
      return;
    }
    // In-flight EV guard, ahead of the state dispatch.  A FAULT is always
    // reported; AUTO.LAND is optional for procedures that require RC takeover.
    // SUSPECT and status staleness do not by themselves descend, matching what
    // minipc_mavros_offboard.py already flew.
    // The vehicle must be airborne for this to act -- FAULT while still on the
    // ground is handled by the pre-arm gate in runPreflight().
    if (require_ev_health_ && ev_health_.faulted() && armed_ &&
      state_ != State::LANDING && state_ != State::IDLE)
    {
      if (!ev_fault_report_latched_) {
        ev_fault_report_latched_ = true;
        if (ev_fault_auto_land_) {
          RCLCPP_ERROR(
            get_logger(),
            "[EV_HEALTH_FAULT] external vision reported FAULT in flight; requesting AUTO.LAND");
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "[EV_HEALTH_FAULT] external vision reported FAULT in flight; "
            "AUTO.LAND disabled, pilot must take over immediately");
        }
      }
      if (ev_fault_auto_land_) {
        state_ = State::LANDING;
      }
    }
    if (manualHandoverReady()) {
      acceptManualHandover();
    }
    if (state_ == State::IDLE) {
      if (!idle_hold_enabled_) {
        return;
      }
      if (armed_) {
        local_origin_rebase_guard_.reset();
        if (race_offboard::shouldTrackManualHandoverPosition(
            manual_handover_, true, armed_, offboard_mode_,
            have_local_position_ && finiteCurrentPosition()))
        {
          // PX4 requires the stream before entering OFFBOARD. Keep that stream
          // on the takeoff XY and live POSITION height, then freeze it when
          // OFFBOARD is accepted.
          updateManualHandoverHoldPosition(current_z_);
          setControlState(ControlState::IDLE_HOLD, "tracking manual POSITION handover");
        }
        if (!hold_position_valid_) {
          setControlState(
            ControlState::WAITING_FOR_ODOM,
            "armed IDLE without an existing hold; automatic relock forbidden");
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[GROUND_REFERENCE_RELOCK_BLOCKED] armed=true; refusing automatic hold lock");
          publishControlDiagnostics("safety_hold_blocked");
          return;
        }
      } else {
        if (!groundReferenceLockAllowed(now().nanoseconds())) {
          if (hold_position_valid_) {
            RCLCPP_WARN(
              get_logger(),
              "[GROUND_REFERENCE_INVALIDATED] EV is no longer continuously healthy while disarmed");
          }
          hold_position_valid_ = false;
          last_command_valid_ = false;
          local_origin_rebase_guard_.reset();
          setControlState(
            ControlState::WAITING_FOR_ODOM,
            "waiting for continuous EV health before initial hold");
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[EV_HOLD_WAIT] continuous_healthy=%.2fs required=%.2fs",
            ev_flight_ready_.healthyDurationS(now().nanoseconds()),
            ev_flight_ready_.requiredS());
          publishControlDiagnostics("waiting_for_ev_ground_reference");
          return;
        }
        if (!have_local_position_ || !finiteCurrentPosition()) {
          setControlState(ControlState::WAITING_FOR_ODOM, "no local position");
          publishControlDiagnostics("safety_hold");
          return;
        }
        if (!hold_position_valid_) {
          lockHoldPosition(current_z_, "EV-ready startup idle hold");
        }
      }
      setControlState(ControlState::IDLE_HOLD, "no active goal");
      publishLockedHold();
      publishControlDiagnostics("safety_hold");
      return;
    }
    if (state_ == State::LANDING) {
      land();
      state_ = State::FINISHED;
      return;
    }

    if (state_ == State::WARMUP) {
      if (!hold_position_valid_) {
        lockHoldPosition(cruise_z, "offboard warmup");
      }
      publishLockedHold();
      publishControlDiagnostics("safety_hold");
      if ((now() - state_enter_time_).seconds() >= offboard_warmup_sec_) {
        state_ = State::PREFLIGHT;
        state_enter_time_ = now();
      }
      return;
    }
    if (state_ == State::PREFLIGHT) {
      runPreflight(cruise_z);
      return;
    }
    if (state_ == State::TAKEOFF) {
      runTakeoff(cruise_z);
      return;
    }
    runActive(cruise_z);
  }

  void runPreflight(float cruise_z)
  {
    // Manual flight-validation mode: keep the locked takeoff XY plus the live
    // POSITION height in the setpoint stream, but leave arming, takeoff, and
    // OFFBOARD selection to the pilot.
    if (manual_handover_ && armed_ && have_local_position_ && finiteCurrentPosition() &&
      !offboard_mode_)
    {
      lockManualHandoverPosition(current_z_, "manual handover position-mode hold");
    }
    if (!hold_position_valid_) {
      lockHoldPosition(cruise_z, "preflight hold");
    }
    publishLockedHold();
    const bool finite_pose = finiteCurrentPosition();
    const double wait_sec = (now() - state_enter_time_).seconds();
    const bool effective_strict_health =
      race_offboard::shouldRequireStrictLocalPositionHealth(
      require_strict_local_position_health_);
    const bool relaxed_ready = !effective_strict_health &&
      allow_dead_reckoning_takeoff_ && have_local_position_ && finite_pose &&
      z_valid_ && wait_sec >= ekf_xy_wait_timeout_sec_;
    const bool strict_ready = xy_valid_ && z_valid_ && !dead_reckoning_ && heading_good_ &&
      finite_pose;
    const bool normal_ready = xy_valid_ && z_valid_ && finite_pose;
    const bool position_ready =
      effective_strict_health ? strict_ready : (normal_ready || relaxed_ready);

    if (!position_ready) {
      position_ready_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for local position: xy=%d z=%d dead_reckoning=%d wait=%.1fs",
        xy_valid_, z_valid_, dead_reckoning_, wait_sec);
      return;
    }
    if (position_ready_since_.nanoseconds() == 0) {
      position_ready_since_ = now();
    }
    if ((now() - position_ready_since_).seconds() < preflight_stabilization_sec_) {
      return;
    }

    if (manual_handover_) {
      if (!offboard_mode_ || !armed_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[MANUAL_HANDOVER] waiting for pilot: OFFBOARD=%d armed=%d",
          offboard_mode_, armed_);
        return;
      }
      if (!manualHandoverReady()) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[MANUAL_HANDOVER_BLOCKED] setpoint, speed, height, or EV gate is unsafe");
        return;
      }
      acceptManualHandover();
      return;
    }

    if (!offboard_mode_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[PREFLIGHT_WAIT_OFFBOARD] waiting for pilot-selected OFFBOARD; automatic mode "
        "requests are disabled once a takeoff transaction starts");
      return;
    }
    // EV health gates arming, not the mode switch: entering OFFBOARD while
    // disarmed is harmless, but arming on a degrading external-vision estimate
    // is what this guard exists to prevent.  Ported from the flown behaviour of
    // minipc_mavros_offboard.py.
    if (require_ev_health_ && !ev_flight_ready_.ready(now().nanoseconds())) {
      std::string reason;
      ev_flight_ready_.unsafeReason(now().nanoseconds(), &reason);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[EV_HEALTH_WAIT] %s (continuous_healthy=%.2fs required=%.2fs)",
        reason.c_str(), ev_flight_ready_.healthyDurationS(now().nanoseconds()),
        ev_flight_ready_.requiredS());
      return;
    }
    if (!armed_) {
      requestCommandPeriodically([this]() {arm();});
      return;
    }

    takeoff_x_ = current_x_;
    takeoff_y_ = current_y_;
    takeoff_yaw_ = holdYaw();
    state_ = State::TAKEOFF;
    state_enter_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "Offboard armed; vertical takeoff at NED XY=(%.2f, %.2f), yaw=%.2f until height %.2fm.",
      takeoff_x_, takeoff_y_, takeoff_yaw_, takeoff_complete_height_m_);
  }

  void runTakeoff(float cruise_z)
  {
    race_msgs::msg::NavigationSetpoint takeoff;
    takeoff.header.frame_id = required_setpoint_frame_;
    takeoff.position.x = takeoff_x_;
    takeoff.position.y = takeoff_y_;
    takeoff.position.z = cruise_z;
    takeoff.velocity_valid = false;
    takeoff.yaw = takeoff_yaw_;
    takeoff.yaw_rate_valid = false;
    publishTrajectorySetpoint(takeoff);

    if (!have_local_position_ || !z_valid_ || !finiteCurrentPosition()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Vertical takeoff waiting for valid local height before path tracking.");
      return;
    }

    const double current_height = std::max(0.0, currentHeightAgl());
    if (current_height < takeoff_complete_height_m_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Vertical takeoff: height %.2f/%.2fm; planner XY/yaw held back",
        current_height, takeoff_complete_height_m_);
      return;
    }
    if (control_source_ == "ego" &&
      !race_offboard::canHandoverToEgo(
        true, egoSetpointFresh(), selectedPlannerFailed()))
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Takeoff height reached; holding XY until a fresh safe EGO trajectory is ready");
      return;
    }

    state_ = State::ACTIVE;
    state_enter_time_ = now();
    if (!goal_active_) {
      lockHoldPosition(cruise_z, "takeoff complete idle hold reset");
    }
    setControlState(
      goal_active_ ? ControlState::TRACKING : ControlState::IDLE_HOLD,
      goal_active_ ? "takeoff complete with active goal" : "takeoff complete without goal");
    if (control_source_ == "ego") {
      RCLCPP_INFO(
        get_logger(),
        "[OFFBOARD_EGO_HANDOVER] trajectory_id=unknown validated=true fresh=true");
    }
    RCLCPP_INFO(get_logger(), "Takeoff height reached; following planner navigation setpoints.");
  }

  bool selectedTrajectoryFresh()
  {
    if (control_source_ == "ego") {
      return have_ego_setpoint_ &&
             (now() - last_ego_setpoint_time_).seconds() <= trajectory_timeout_sec_;
    }
    return have_setpoint_ &&
           (now() - last_setpoint_time_).seconds() <= trajectory_timeout_sec_;
  }

  std::string selectedPlannerStatus() const
  {
    return control_source_ == "ego" ? ego_status_ : planner_status_;
  }

  bool globalPlannerFailed() const
  {
    return race_offboard::globalPlannerStatusIsFailure(
      have_global_planner_status_, global_planner_status_.goal_active,
      global_planner_status_.global_goal_id, active_goal_id_,
      global_planner_status_.mode, global_planner_status_.reason);
  }

  std::string plannerFailureStatus() const
  {
    if (globalPlannerFailed()) {
      return "global mode=" + global_planner_status_.mode +
             " reason=" + global_planner_status_.reason;
    }
    return selectedPlannerStatus();
  }

  bool selectedPlannerReachedGoal() const
  {
    const double goal_error = std::hypot(
      static_cast<double>(current_x_ - goal_x_ned_),
      static_cast<double>(current_y_ - goal_y_ned_));
    const bool vehicle_stopped_at_goal = goal_active_ &&
      goal_error <= goal_reached_position_tolerance_ &&
      horizontalSpeed() <= goal_reached_velocity_tolerance_;
    if (!require_global_planner_final_goal_) {
      return vehicle_stopped_at_goal;
    }
    return have_global_planner_status_ &&
           global_planner_status_.goal_active &&
           global_planner_status_.final_goal_reached &&
           global_planner_status_.global_goal_id == active_goal_id_ &&
           vehicle_stopped_at_goal;
  }

  bool selectedPlannerFailed() const
  {
    const auto status = selectedPlannerStatus();
    const bool selected_failed = status.find("NO_PATH") != std::string::npos ||
      status.find("BLOCKED") != std::string::npos ||
      status.find("TRAJECTORY_TIMEOUT") != std::string::npos ||
      status.find("NO_SAFE_TRAJECTORY") != std::string::npos ||
      status.find("EGO_TRAJECTORY_COLLISION") != std::string::npos ||
      status.find("EGO_OCCUPANCY_STALE") != std::string::npos ||
      status.find("EGO_REPLAN_REANCHOR_FAILED") != std::string::npos ||
      status.find("EGO_REPLAN_TRANSACTION_EXHAUSTED") != std::string::npos ||
      status.find("EMERGENCY_HOLD") != std::string::npos ||
      status.find("SETPOINT_INVALID") != std::string::npos ||
      status.find("ODOM_UNHEALTHY") != std::string::npos ||
      status.find("WAIT_MAP") != std::string::npos ||
      // The bridge validated nothing because every trajectory sample sat
      // below its minimum tracking height.  Nothing was cleared for
      // flight, so this must brake rather than pass silently.  It is not
      // in the permanent-latch set: a later valid trajectory recovers.
      status.find("NO_TRACKABLE_POINT") != std::string::npos;
    // A fresh EGO command is unsafe once the global path anchoring it has
    // failed; do not continue an orphaned local trajectory.
    return selected_failed || globalPlannerFailed();
  }

  void runActive(float cruise_z)
  {
    if (!localPositionSafe()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LOCAL_POSITION_UNSAFE: xy=%d z=%d dead_reckoning=%d heading_good=%d; hold position",
        xy_valid_, z_valid_, dead_reckoning_, heading_good_);
      if (control_state_ == ControlState::EMERGENCY_STOP) {
        publishLockedHold();
      } else {
        beginBraking(ControlState::EMERGENCY_STOP, "local position unsafe");
        runBraking();
      }
      publishControlDiagnostics("emergency_stop");
      return;
    }

    if (control_state_ == ControlState::BRAKING) {
      runBraking();
      publishControlDiagnostics("safety_braking");
      return;
    }
    if (planner_safety_failure_latched_) {
      if (control_state_ == ControlState::PLANNER_FAILURE_HOLD) {
        publishLockedHold();
      } else {
        beginBraking(ControlState::PLANNER_FAILURE_HOLD, "latched EGO safety failure");
        runBraking();
      }
      publishControlDiagnostics("planner_failure_hold");
      return;
    }
    if (!goal_active_ || navigation_cancelled_) {
      if (control_state_ == ControlState::IDLE_HOLD ||
        control_state_ == ControlState::GOAL_REACHED_HOLD ||
        control_state_ == ControlState::PLANNER_FAILURE_HOLD)
      {
        publishLockedHold();
      } else {
        beginBraking(ControlState::IDLE_HOLD, "no active goal");
        runBraking();
      }
      publishControlDiagnostics("safety_hold");
      return;
    }
    if (control_state_ == ControlState::GOAL_REACHED_HOLD) {
      publishLockedHold();
      publishControlDiagnostics("goal_hold");
      return;
    }
    if (selectedPlannerReachedGoal()) {
      RCLCPP_WARN(
        get_logger(),
        "[GOAL_REACHED_HOLD] trigger_source=%s active_goal_id=%lu global_goal_id=%lu",
        require_global_planner_final_goal_ ? "super_final_goal" : "direct_final_goal",
        active_goal_id_, global_planner_status_.global_goal_id);
      beginBraking(ControlState::GOAL_REACHED_HOLD, "confirmed final goal reached");
      runBraking();
      publishControlDiagnostics("goal_hold");
      return;
    }
    if (control_source_ == "ego" && awaiting_initial_ego_trajectory_) {
      const double wait_sec = (now() - initial_ego_trajectory_wait_start_).seconds();
      if (race_offboard::shouldHoldForInitialEgoTrajectory(
          true, selectedPlannerFailed()))
      {
        if (!hold_position_valid_) {
          lockHoldPosition(
            static_cast<float>(race_offboard::fixedAltitudeHoldZ(cruise_z)),
            "awaiting initial EGO trajectory");
        }
        if (race_offboard::initialEgoTrajectoryWaitIsSlow(
            wait_sec, initial_ego_trajectory_wait_sec_))
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(), 1000,
            "[OFFBOARD_WAIT_INITIAL_EGO_TRAJECTORY_SLOW] goal_id=%lu wait=%.3fs diagnostic_threshold=%.3fs; holding until fresh validated trajectory or explicit planner failure",
            active_goal_id_, wait_sec, initial_ego_trajectory_wait_sec_);
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(), 1000,
            "[OFFBOARD_WAIT_INITIAL_EGO_TRAJECTORY] goal_id=%lu wait=%.3fs diagnostic_threshold=%.3fs",
            active_goal_id_, wait_sec, initial_ego_trajectory_wait_sec_);
        }
        publishLockedHold();
        publishControlDiagnostics("awaiting_initial_ego_trajectory");
        return;
      }
    }
    const bool planner_failed = selectedPlannerFailed();
    const bool trajectory_fresh = selectedTrajectoryFresh();
    if (!trajectory_fresh || planner_failed) {
      const double age = control_source_ == "ego" ?
        (now() - last_ego_setpoint_time_).seconds() : (now() - last_setpoint_time_).seconds();
      if (planner_failed && control_source_ == "ego" &&
        race_offboard::egoPlannerStatusRequiresPermanentLatch(ego_status_) &&
        !planner_safety_failure_latched_)
      {
        planner_safety_failure_latched_ = true;
        have_ego_setpoint_ = false;
        RCLCPP_ERROR(
          get_logger(),
          "[PLANNER_SAFETY_LATCH] status=%s; cancel navigation before recovery",
          ego_status_.c_str());
      }
      if (!planner_failed && control_source_ == "ego") {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(), 1000,
          "[OFFBOARD_AWAIT_EGO_REPLAN] trajectory age=%.3f status=%s; holding until a fresh validated trajectory arrives",
          age, plannerFailureStatus().c_str());
      } else if (planner_failed && control_source_ == "ego" &&
        ego_status_.find("EGO_TRAJECTORY_COLLISION") != std::string::npos)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(), 1000,
          "[OFFBOARD_EGO_REPLAN_HOLD] status=%s; braking until Bridge validates a replacement trajectory",
          plannerFailureStatus().c_str());
      } else {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[STALE_TRAJECTORY] trajectory age=%.3f status=%s",
          age, plannerFailureStatus().c_str());
      }
      if (control_state_ == ControlState::PLANNER_FAILURE_HOLD) {
        publishLockedHold();
      } else {
        beginBraking(
          ControlState::PLANNER_FAILURE_HOLD,
          planner_failed ? "planner failure" : "awaiting EGO replan");
        runBraking();
      }
      publishControlDiagnostics(
        planner_failed ? "planner_failure_hold" : "awaiting_ego_replan");
      return;
    }

    setControlState(ControlState::TRACKING, "fresh validated trajectory");
    if (control_source_ == "ego") {
      runActiveEgo(cruise_z);
      publishControlDiagnostics("ego");
      return;
    }

    auto command = latest_setpoint_;
    const double current_height = currentHeightAgl();
    const double soft_height_limit = max_command_height_m_ + overheight_guard_margin_m_;
    const double emergency_height_limit = soft_height_limit + emergency_overheight_margin_m_;
    if (current_height > emergency_height_limit) {
      command.position.x = current_x_;
      command.position.y = current_y_;
      command.position.z = cruise_z;
      command.velocity_valid = false;
      command.yaw = holdYaw();
      command.yaw_rate_valid = false;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "OVERHEIGHT_EMERGENCY: %.2fm > %.2fm; freeze XY and descend", current_height,
        emergency_height_limit);
    } else if (current_height > soft_height_limit) {
      command.position.z = cruise_z;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "OVERHEIGHT_SOFT: %.2fm > %.2fm; keep XY/yaw and command cruise altitude",
        current_height, soft_height_limit);
    }

    publishTrajectorySetpoint(command);
    publishControlDiagnostics("super");
  }

  void runActiveEgo(float cruise_z)
  {
    auto command = latest_ego_setpoint_;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double current_height = currentHeightAgl();
    const double soft_height_limit = max_command_height_m_ + overheight_guard_margin_m_;
    const double emergency_height_limit = soft_height_limit + emergency_overheight_margin_m_;
    if (current_height > emergency_height_limit) {
      command.position[0] = current_x_;
      command.position[1] = current_y_;
      command.position[2] = cruise_z;
      // Discard every feed-forward axis: hold XY and descend on position alone.
      command.velocity = {nan, nan, nan};
      command.acceleration = {nan, nan, nan};
      command.yaw = holdYaw();
      command.yaw_rate = nan;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "OVERHEIGHT_EMERGENCY(EGO): %.2fm > %.2fm; freeze XY and descend",
        current_height, emergency_height_limit);
    } else if (current_height > soft_height_limit) {
      // Correct altitude only: the Z feed-forward is dropped while the XY
      // feed-forward the local planner still needs stays live.
      command.position[2] = cruise_z;
      command.velocity[2] = nan;
      command.acceleration[2] = nan;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "OVERHEIGHT_SOFT(EGO): %.2fm > %.2fm; preserve XY feed-forward and correct altitude",
        current_height, soft_height_limit);
    }
    publishEgoTrajectorySetpoint(command);
  }

  bool finiteCurrentPosition() const
  {
    return have_local_position_ && std::isfinite(current_x_) && std::isfinite(current_y_) &&
           std::isfinite(current_z_);
  }

  bool egoSetpointFresh()
  {
    return have_ego_setpoint_ &&
           (now() - last_ego_setpoint_time_).seconds() <= ego_setpoint_timeout_sec_;
  }

  bool localPositionSafe() const
  {
    if (!have_local_position_ || !z_valid_ || !finiteCurrentPosition()) {
      return false;
    }
    // MAVROS-only guard replacing the PX4 estimator flags this port gave up:
    // if the FCU serial link drops, the cached pose goes stale silently.
    if (!connected_) {
      return false;
    }
    const bool effective_strict_health =
      race_offboard::shouldRequireStrictLocalPositionHealth(
      require_strict_local_position_health_);
    return !effective_strict_health ||
           (xy_valid_ && !dead_reckoning_ && heading_good_);
  }

  double holdYaw() const
  {
    if (std::isfinite(current_yaw_)) {
      return wrapAngle(current_yaw_);
    }
    return last_command_valid_ ? last_command_yaw_ : 0.0;
  }

  void publishTrajectorySetpoint(const race_msgs::msg::NavigationSetpoint & setpoint)
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double yaw_ned = wrapAngle(setpoint.yaw);
    const auto position_enu = race_offboard::nedToEnu(
      {setpoint.position.x, setpoint.position.y, setpoint.position.z});
    auto velocity_enu = race_offboard::nedToEnu({nan, nan, nan});
    if (setpoint.velocity_valid) {
      velocity_enu = race_offboard::nedToEnu(
        {setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z});
    }
    // The planner never supplies an acceleration feed-forward on this path.
    const auto acceleration_enu = race_offboard::nedToEnu({nan, nan, nan});

    mavros_msgs::msg::PositionTarget msg{};
    msg.header.stamp = now();
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = race_offboard::positionTargetTypeMask(
      velocity_enu, acceleration_enu, setpoint.yaw_rate_valid);
    msg.position.x = position_enu[0];
    msg.position.y = position_enu[1];
    msg.position.z = position_enu[2];
    // Masked axes must still carry a finite payload: MAVLink floats have no
    // "ignored" encoding and a NaN can propagate past the mask.
    const auto safe_velocity = race_offboard::sanitizeIgnoredAxes(velocity_enu);
    const auto safe_acceleration = race_offboard::sanitizeIgnoredAxes(acceleration_enu);
    msg.velocity.x = safe_velocity[0];
    msg.velocity.y = safe_velocity[1];
    msg.velocity.z = safe_velocity[2];
    msg.acceleration_or_force.x = safe_acceleration[0];
    msg.acceleration_or_force.y = safe_acceleration[1];
    msg.acceleration_or_force.z = safe_acceleration[2];
    msg.yaw = static_cast<float>(race_offboard::nedYawToEnu(yaw_ned));
    // A NED yaw rate is a rotation about Down; ENU yaw rate is about Up.
    msg.yaw_rate = setpoint.yaw_rate_valid ?
      static_cast<float>(-setpoint.yaw_rate) : 0.0F;
    trajectory_setpoint_publisher_->publish(msg);

    last_command_x_ = static_cast<float>(setpoint.position.x);
    last_command_y_ = static_cast<float>(setpoint.position.y);
    last_command_yaw_ = yaw_ned;
    last_command_valid_ = true;
  }

  void publishEgoTrajectorySetpoint(EgoCommandNed setpoint)
  {
    const auto publish_time = now();
    if (last_command_valid_) {
      const double elapsed = last_ego_publish_time_.nanoseconds() == 0 ?
        1.0 / setpoint_rate_hz_ :
        std::clamp((publish_time - last_ego_publish_time_).seconds(), 0.0, 0.25);
      const double requested_yaw = wrapAngle(setpoint.yaw);
      setpoint.yaw = race_offboard::slewEgoYaw(
        last_command_yaw_, requested_yaw, ego_yaw_rate_limit_rad_s_, elapsed);
      if (std::isfinite(setpoint.yaw_rate)) {
        setpoint.yaw_rate = std::clamp(
          setpoint.yaw_rate, -ego_yaw_rate_limit_rad_s_, ego_yaw_rate_limit_rad_s_);
      }
    }
    // The soft over-height guard NaNs only the Z feed-forward and keeps XY
    // live, so the mask has to be derived per axis rather than from a constant.
    const auto position_enu = race_offboard::nedToEnu(setpoint.position);
    const auto velocity_enu = race_offboard::nedToEnu(setpoint.velocity);
    const auto acceleration_enu = race_offboard::nedToEnu(setpoint.acceleration);
    const bool yaw_rate_valid = std::isfinite(setpoint.yaw_rate);

    mavros_msgs::msg::PositionTarget msg{};
    msg.header.stamp = publish_time;
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = race_offboard::positionTargetTypeMask(
      velocity_enu, acceleration_enu, yaw_rate_valid);
    msg.position.x = position_enu[0];
    msg.position.y = position_enu[1];
    msg.position.z = position_enu[2];
    const auto safe_velocity = race_offboard::sanitizeIgnoredAxes(velocity_enu);
    const auto safe_acceleration = race_offboard::sanitizeIgnoredAxes(acceleration_enu);
    msg.velocity.x = safe_velocity[0];
    msg.velocity.y = safe_velocity[1];
    msg.velocity.z = safe_velocity[2];
    msg.acceleration_or_force.x = safe_acceleration[0];
    msg.acceleration_or_force.y = safe_acceleration[1];
    msg.acceleration_or_force.z = safe_acceleration[2];
    // PositionTarget has no jerk field; EGO's jerk feed-forward is dropped here.
    msg.yaw = static_cast<float>(race_offboard::nedYawToEnu(setpoint.yaw));
    msg.yaw_rate = yaw_rate_valid ? static_cast<float>(-setpoint.yaw_rate) : 0.0F;
    trajectory_setpoint_publisher_->publish(msg);

    last_command_x_ = static_cast<float>(setpoint.position[0]);
    last_command_y_ = static_cast<float>(setpoint.position[1]);
    last_command_yaw_ = setpoint.yaw;
    last_command_valid_ = true;
    last_ego_publish_time_ = publish_time;
  }

  // PX4 DDS took commands as fire-and-forget uORB publications.  MAVROS uses
  // services, so every request here is asynchronous: calling a service
  // synchronously from the 20 Hz timer callback would deadlock the executor.
  // A pending flag suppresses duplicate requests while one is in flight.
  void arm()
  {
    if (arm_request_pending_) {
      return;
    }
    if (!arming_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "/mavros/cmd/arming is not available yet");
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;
    arm_request_pending_ = true;
    arming_client_->async_send_request(
      request,
      [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        arm_request_pending_ = false;
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(
            get_logger(), "Arming rejected by FCU (result=%u)", response->result);
        }
      });
  }

  void land()
  {
    if (land_request_pending_) {
      return;
    }
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "/mavros/set_mode is unavailable; cannot request AUTO.LAND");
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = "AUTO.LAND";
    land_request_pending_ = true;
    set_mode_client_->async_send_request(
      request,
      [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        land_request_pending_ = false;
        if (future.get()->mode_sent) {
          RCLCPP_INFO(get_logger(), "AUTO.LAND requested.");
        } else {
          RCLCPP_ERROR(get_logger(), "AUTO.LAND request was not accepted");
        }
      });
  }

  template<typename CallbackT>
  void requestCommandPeriodically(CallbackT callback)
  {
    if (last_command_request_time_.nanoseconds() == 0 ||
      (now() - last_command_request_time_).seconds() >= command_retry_period_sec_)
    {
      callback();
      last_command_request_time_ = now();
    }
  }

  rclcpp::Time now()
  {
    return get_clock()->now();
  }

  // Count publishers on the setpoint topic excluding this node's own, which is
  // created before the timer starts.
  bool conflicting_setpoint_publisher_present() const
  {
    const auto publishers = get_publishers_info_by_topic("/mavros/setpoint_raw/local");
    const std::string self = get_fully_qualified_name();
    for (const auto & info : publishers) {
      std::string other = info.node_namespace();
      if (other.empty() || other.back() != '/') {
        other.push_back('/');
      }
      other += info.node_name();
      if (other != self) {
        // Discovery often has not resolved the peer's name yet, reporting
        // _NODE_NAME_UNKNOWN_.  Say so plainly instead of printing a
        // placeholder the operator cannot act on.
        const bool named = other.find("_NODE_NAME_UNKNOWN_") == std::string::npos;
        RCLCPP_ERROR(
          get_logger(),
          "Conflicting publisher on /mavros/setpoint_raw/local: %s",
          named ? other.c_str() :
          "<name not yet resolved by discovery; check 'ros2 node list' for "
          "minipc_mavros_offboard or a second offboard_waypoint_node>");
        return true;
      }
    }
    return false;
  }

  std::string control_source_;
  std::string navigation_setpoint_topic_;
  std::string ego_setpoint_topic_;
  std::string required_setpoint_frame_;
  std::string map_odom_topic_;
  bool require_map_local_alignment_{false};
  double map_local_alignment_stabilization_sec_{1.0};
  double map_local_alignment_max_spread_m_{0.08};
  double map_local_alignment_max_yaw_spread_rad_{0.10};
  double setpoint_timeout_sec_{0.8};
  double ego_setpoint_timeout_sec_{0.20};
  double ekf_xy_wait_timeout_sec_{3.0};
  double offboard_warmup_sec_{2.0};
  bool manual_handover_{false};
  double manual_handover_max_position_error_m_{0.15};
  double manual_handover_max_speed_mps_{0.20};
  double preflight_stabilization_sec_{5.0};
  double command_retry_period_sec_{1.0};
  double cruise_altitude_m_{0.78};
  double flight_target_agl_m_{0.78};
  double takeoff_complete_height_m_{0.55};
  double min_command_height_m_{0.50};
  double max_command_height_m_{0.90};
  double overheight_guard_margin_m_{0.20};
  double emergency_overheight_margin_m_{0.35};
  double trajectory_timeout_sec_{0.50};
  double initial_ego_trajectory_wait_sec_{2.0};
  double hold_position_tolerance_{0.05};
  double hold_velocity_tolerance_{0.05};
  double goal_reached_position_tolerance_{0.10};
  double goal_reached_velocity_tolerance_{0.08};
  double goal_update_position_tolerance_m_{0.05};
  bool same_goal_retry_on_safety_latch_{true};
  double braking_max_acc_{0.50};
  double hold_replan_distance_{0.08};
  double setpoint_rate_hz_{20.0};
  double trusted_position_max_speed_mps_{4.0};
  double trusted_position_jump_allowance_m_{0.40};
  double ego_setpoint_max_lead_m_{2.0};
  double ego_yaw_rate_limit_rad_s_{0.80};
  double initial_position_stabilization_sec_{0.50};
  double initial_position_max_spread_m_{0.08};
  double initial_position_max_speed_mps_{0.20};
  double local_origin_rebase_stabilization_sec_{1.0};
  double local_origin_rebase_max_spread_m_{0.08};
  double local_origin_rebase_max_speed_mps_{0.20};
  double shared_bounds_x_min_{-7.5};
  double shared_bounds_x_max_{7.5};
  double shared_bounds_y_min_{-7.0};
  double shared_bounds_y_max_{7.0};
  double shared_bounds_z_min_{0.50};
  double shared_bounds_z_max_{0.90};
  bool allow_dead_reckoning_takeoff_{false};
  bool require_strict_local_position_health_{true};
  bool idle_hold_enabled_{true};
  bool require_global_planner_final_goal_{false};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_status_publisher_;
  rclcpp::Publisher<race_msgs::msg::FlightAltitudeReference>::SharedPtr
    flight_altitude_reference_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr map_odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_velocity_subscriber_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr vehicle_status_subscriber_;
  rclcpp::Subscription<race_msgs::msg::NavigationSetpoint>::SharedPtr
    navigation_setpoint_subscriber_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr ego_setpoint_subscriber_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr ego_command_goal_seq_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_status_subscriber_;
  rclcpp::Subscription<race_msgs::msg::GlobalPlannerStatus>::SharedPtr
    global_planner_status_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ego_status_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_activity_subscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;

  race_msgs::msg::NavigationSetpoint latest_setpoint_;
  bool have_setpoint_{false};
  rclcpp::Time last_setpoint_time_{0, 0, RCL_ROS_TIME};
  EgoCommandNed latest_ego_setpoint_;
  bool have_ego_setpoint_{false};
  rclcpp::Time last_ego_setpoint_time_{0, 0, RCL_ROS_TIME};
  uint64_t ego_command_goal_seq_{0};
  uint64_t minimum_ego_command_goal_seq_{0};
  State state_{State::IDLE};
  ControlState control_state_{ControlState::WAITING_FOR_ODOM};
  ControlState post_braking_state_{ControlState::IDLE_HOLD};
  std::string control_reason_{"startup"};
  std::string planner_status_;
  std::string ego_status_;
  race_msgs::msg::GlobalPlannerStatus global_planner_status_;
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time position_ready_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_planner_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ego_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time initial_ego_trajectory_wait_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time braking_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publisher_check_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ego_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time initial_position_stable_since_{0, 0, RCL_ROS_TIME};

  float current_x_{0.0F};
  float current_y_{0.0F};
  float current_z_{0.0F};
  float current_vx_{0.0F};
  float current_vy_{0.0F};
  float current_vz_{0.0F};
  double pose_vertical_speed_ned_{0.0};
  bool have_pose_vertical_speed_{false};
  float current_yaw_{std::numeric_limits<float>::quiet_NaN()};
  std::array<double, 3> map_position_enu_{{0.0, 0.0, 0.0}};
  double map_yaw_enu_{0.0};
  race_offboard::PlanarFrameTransform map_to_local_;
  race_offboard::PlanarFrameTransform map_to_local_candidate_;
  uint64_t flight_id_{0};
  double ground_z_local_ned_{0.0};
  double target_z_local_ned_{0.0};
  double ground_z_map_{0.0};
  double target_z_map_{0.0};
  bool flight_altitude_reference_valid_{false};
  rclcpp::Time map_local_alignment_stable_since_{0, 0, RCL_ROS_TIME};
  bool have_map_odom_{false};
  bool map_local_alignment_candidate_valid_{false};
  bool map_local_alignment_ready_{false};
  float takeoff_x_{0.0F};
  float takeoff_y_{0.0F};
  double takeoff_yaw_{0.0};
  float manual_takeoff_x_{0.0F};
  float manual_takeoff_y_{0.0F};
  double manual_takeoff_yaw_{0.0};
  bool manual_takeoff_position_valid_{false};
  float hold_x_{0.0F};
  float hold_y_{0.0F};
  float hold_z_{0.0F};
  double hold_yaw_{0.0};
  float goal_x_ned_{0.0F};
  float goal_y_ned_{0.0F};
  float braking_vx_{0.0F};
  float braking_vy_{0.0F};
  float braking_vz_{0.0F};
  double braking_duration_sec_{0.20};
  bool hold_position_valid_{false};
  bool manual_handover_accepted_{false};
  bool pilot_override_latched_{false};
  bool goal_active_{false};
  bool have_goal_identity_{false};
  bool have_global_planner_status_{false};
  bool navigation_cancelled_{false};
  bool planner_safety_failure_latched_{false};
  bool awaiting_initial_ego_trajectory_{false};
  bool pending_takeoff_request_{false};
  bool have_local_position_{false};
  bool xy_valid_{false};
  bool z_valid_{false};
  bool heading_good_{false};
  bool dead_reckoning_{false};
  bool initial_position_candidate_valid_{false};
  float initial_position_x_{0.0F};
  float initial_position_y_{0.0F};
  float initial_position_z_{0.0F};
  rclcpp::Time last_trusted_position_stamp_;
  bool have_trusted_position_stamp_{false};
  bool have_local_velocity_{false};
  race_offboard::EvHealthTracker ev_health_;
  race_offboard::EvHealthTracker ev_flight_ready_;
  race_offboard::LocalOriginRebaseGuard local_origin_rebase_guard_;
  std::string ev_health_topic_;
  std::string ev_flight_ready_topic_;
  bool require_ev_health_{true};
  double ev_health_required_s_{7.5};
  double ev_health_freshness_s_{0.5};
  bool ev_fault_auto_land_{true};
  bool ev_fault_report_latched_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ev_health_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ev_flight_ready_subscriber_;
  bool arm_request_pending_{false};
  bool land_request_pending_{false};
  bool offboard_mode_{false};
  bool armed_{false};
  bool connected_{false};
  std::string last_vehicle_mode_;
  uint64_t active_goal_id_{0};

  bool last_command_valid_{false};
  float last_command_x_{0.0F};
  float last_command_y_{0.0F};
  float last_command_yaw_{0.0F};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardWaypointNode>());
  rclcpp::shutdown();
  return 0;
}
