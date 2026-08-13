#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <bspline_opt/uniform_bspline.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <race_msgs/msg/flight_altitude_reference.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "race_ego_bridge/frame_utils.hpp"
#include "race_ego_bridge/safety_utils.hpp"
#include "plan_env/raw_obstacle_distance_index.h"

class EgoTrajectoryBridge : public rclcpp::Node
{
public:
  EgoTrajectoryBridge()
  : Node("race_ego_trajectory_bridge"), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    position_command_topic_ = declare_parameter<std::string>(
      "position_command_topic", "/race/ego/position_command");
    bspline_topic_ = declare_parameter<std::string>("bspline_topic", "/race/ego/bspline");
    validated_bspline_topic_ = declare_parameter<std::string>(
      "validated_bspline_topic", "/race/ego/validated_bspline");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/race/ego/trajectory_setpoint");
    publish_setpoint_ = declare_parameter<bool>("publish_setpoint", true);
    control_enabled_ = declare_parameter<bool>("control_enabled", false);
    require_strict_local_position_health_ =
      declare_parameter<bool>("require_strict_local_position_health", false);
    map_source_ = declare_parameter<std::string>("map_source", "static");
    occupancy_topic_ = declare_parameter<std::string>(
      "occupancy_topic", "/race/ego/occupancy_inflate");
    occupancy_is_inflated_ = declare_parameter<bool>("occupancy_is_inflated", true);
    occupancy_resolution_ = declare_parameter<double>("occupancy_resolution", 0.15);
    trajectory_sample_spacing_ = declare_parameter<double>("trajectory_sample_spacing", 0.04);
    occupancy_timeout_sec_ = declare_parameter<double>("occupancy_timeout_sec", 1.0);
    active_recheck_history_sec_ = declare_parameter<double>("active_recheck_history_sec", 0.5);
    active_recheck_history_sec_ = std::clamp(active_recheck_history_sec_, 0.0, 2.0);
    flight_mode_ = declare_parameter<std::string>("flight_mode", "flat");
    flight_height_ = declare_parameter<double>("flight_height", 0.78);
    max_velocity_ = declare_parameter<double>("max_velocity", 0.60);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 0.80);
    braking_deceleration_mps2_ = declare_parameter<double>(
      "braking_deceleration_mps2", 0.80);
    reaction_time_sec_ = declare_parameter<double>("reaction_time_sec", 0.15);
    minimum_reliable_detection_range_m_ = declare_parameter<double>(
      "minimum_reliable_detection_range_m", 0.725);
    max_yaw_rate_rad_s_ = declare_parameter<double>("max_yaw_rate_rad_s", 0.80);
    feasibility_tolerance_ = declare_parameter<double>("feasibility_tolerance", 1.10);
    // Allow one normal EGO replan handoff to replace a transiently invalid
    // command.  This remains bounded; occupancy or timeout failures still
    // enter the existing safety latch immediately.
    dynamic_invalid_grace_sec_ = declare_parameter<double>(
      "dynamic_invalid_grace_sec", 0.75);
    replan_hold_timeout_sec_ = declare_parameter<double>("replan_hold_timeout_sec", 3.0);
    replan_hold_timeout_sec_ = std::max(0.10, replan_hold_timeout_sec_);
    switch_position_tolerance_m_ = declare_parameter<double>(
      "switch_position_tolerance_m", 0.05);
    switch_velocity_tolerance_mps_ = declare_parameter<double>(
      "switch_velocity_tolerance_mps", 0.10);
    switch_acceleration_tolerance_mps2_ = declare_parameter<double>(
      "switch_acceleration_tolerance_mps2", 0.20);
    if (switch_position_tolerance_m_ <= 0.0 || switch_velocity_tolerance_mps_ <= 0.0 ||
      switch_acceleration_tolerance_mps2_ <= 0.0)
    {
      throw std::runtime_error("trajectory switch tolerances must be > 0");
    }
    rejection_clearance_ = declare_parameter<double>("obstacle_clearance", 0.30);
    if (!std::isfinite(rejection_clearance_) || rejection_clearance_ <= 0.0) {
      throw std::runtime_error("obstacle_clearance must be finite and > 0");
    }
    constrained_clearance_enabled_ = declare_parameter<bool>(
      "constrained_clearance_enable", false);
    constrained_clearance_ = declare_parameter<double>(
      "constrained_clearance_m", 0.25);
    if (!std::isfinite(constrained_clearance_) || constrained_clearance_ <= 0.0 ||
      constrained_clearance_ >= rejection_clearance_)
    {
      throw std::runtime_error(
              "constrained_clearance_m must be finite, > 0 and < obstacle_clearance");
    }
    RCLCPP_INFO(
      get_logger(),
      "[BRIDGE_CLEARANCE_THRESHOLD] hard_clearance=%.9f",
      rejection_clearance_);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.20);
    bspline_timeout_sec_ = declare_parameter<double>("bspline_timeout_sec", 0.20);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.25);
    min_planning_height_ = declare_parameter<double>("min_planning_height", -0.15);
    min_height_ = declare_parameter<double>("min_height", 0.50);
    bootstrap_release_height_ = declare_parameter<double>("bootstrap_release_height", 0.58);
    max_height_ = declare_parameter<double>("max_height", 0.90);
    min_x_ = declare_parameter<double>("min_x", -7.5);
    max_x_ = declare_parameter<double>("max_x", 7.5);
    min_y_ = declare_parameter<double>("min_y", -7.0);
    max_y_ = declare_parameter<double>("max_y", 7.0);
    // Validated for consistency with the cloud bridge only; this node's own
    // behaviour does not branch on the source.
    if (map_source_ != "static" && map_source_ != "live" && map_source_ != "live_px4" &&
      map_source_ != "static_live_px4")
    {
      throw std::runtime_error(
              "map_source must be static, live, live_px4 or static_live_px4");
    }
    if (flight_mode_ != "flat" && flight_mode_ != "3d") {
      throw std::runtime_error("flight_mode must be flat or 3d");
    }
    if (bspline_timeout_sec_ <= 0.0) {
      throw std::runtime_error("bspline_timeout_sec must be > 0");
    }
    if (max_yaw_rate_rad_s_ <= 0.0) {
      throw std::runtime_error("max_yaw_rate_rad_s must be > 0");
    }
    if (braking_deceleration_mps2_ <= 0.0 || reaction_time_sec_ <= 0.0 ||
      minimum_reliable_detection_range_m_ <= 0.0)
    {
      throw std::runtime_error("braking safety parameters must be > 0");
    }
    const double braking_horizon = rejection_clearance_ +
      max_velocity_ * reaction_time_sec_ +
      max_velocity_ * max_velocity_ / (2.0 * braking_deceleration_mps2_);
    if (braking_horizon > minimum_reliable_detection_range_m_) {
      throw std::runtime_error(
              "maximum-speed braking horizon exceeds reliable obstacle detection range");
    }
    RCLCPP_INFO(
      get_logger(),
      "[BRIDGE_BRAKING_CONTRACT] max_speed=%.3f deceleration=%.3f reaction=%.3f "
      "clearance=%.3f braking_horizon=%.3f detection_range=%.3f",
      max_velocity_, braking_deceleration_mps2_, reaction_time_sec_, rejection_clearance_,
      braking_horizon, minimum_reliable_detection_range_m_);

    output_publisher_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      output_topic_, rclcpp::SensorDataQoS());
    command_goal_seq_publisher_ = create_publisher<std_msgs::msg::UInt64>(
      "/race/ego/command_local_goal_seq", rclcpp::SensorDataQoS());
    validated_bspline_publisher_ = create_publisher<traj_utils::msg::Bspline>(
      validated_bspline_topic_, rclcpp::QoS(10).reliable());
    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    status_publisher_ = create_publisher<std_msgs::msg::String>("/race/ego/status", latched_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/race/ego/path", 10);
    predicted_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/race/ego/predicted_path", 10);
    actual_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/race/ego/actual_path", 10);
    control_points_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/race/ego/control_points", 10);
    setpoint_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/race/ego/setpoint", 10);
    status_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/race/ego/status_marker", latched_qos);

    command_subscription_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      position_command_topic_, 50,
      std::bind(&EgoTrajectoryBridge::commandCallback, this, std::placeholders::_1));
    bspline_subscription_ = create_subscription<traj_utils::msg::Bspline>(
      bspline_topic_, 10,
      std::bind(&EgoTrajectoryBridge::bsplineCallback, this, std::placeholders::_1));
    auto map_qos = rclcpp::QoS(5).reliable();
    map_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      occupancy_topic_, map_qos,
      std::bind(&EgoTrajectoryBridge::mapCallback, this, std::placeholders::_1));
    planner_safety_subscription_ = create_subscription<std_msgs::msg::String>(
      "/race/ego/planner_safety_status", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&EgoTrajectoryBridge::plannerSafetyCallback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/race/ego/odom", 10,
      std::bind(&EgoTrajectoryBridge::odomCallback, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/race/ego/goal", 10,
      std::bind(&EgoTrajectoryBridge::goalCallback, this, std::placeholders::_1));
    // Was /fmu/out/vehicle_local_position_v1, which does not exist on this
    // aircraft (MAVROS, not PX4 DDS), so the health gate never fired.  MAVROS
    // exposes link health on /mavros/state instead of per-sample estimator flags.
    health_subscription_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::SensorDataQoS(),
      std::bind(&EgoTrajectoryBridge::healthCallback, this, std::placeholders::_1));
    altitude_reference_subscription_ =
      create_subscription<race_msgs::msg::FlightAltitudeReference>(
      "/race/flight_altitude_reference", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &EgoTrajectoryBridge::altitudeReferenceCallback, this, std::placeholders::_1));

    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&EgoTrajectoryBridge::statusTimer, this));
    setStatus("WAIT_MAP");
  }

private:
  double effectiveFlightHeight() const
  {
    return altitude_reference_valid_ ? target_z_map_ : flight_height_;
  }

  double effectiveMinHeight() const
  {
    return (altitude_reference_valid_ ? ground_z_map_ : 0.0) + min_height_;
  }

  double effectiveMaxHeight() const
  {
    return (altitude_reference_valid_ ? ground_z_map_ : 0.0) + max_height_;
  }

  double currentAgl() const
  {
    return altitude_reference_valid_ ? current_position_.z - ground_z_map_ : current_position_.z;
  }

  void altitudeReferenceCallback(
    const race_msgs::msg::FlightAltitudeReference::SharedPtr msg)
  {
    if (!msg->valid) {
      if (altitude_reference_valid_ && msg->flight_id == altitude_reference_flight_id_) {
        altitude_reference_valid_ = false;
        have_goal_ = false;
        have_trajectory_ = false;
        have_bootstrap_setpoint_ = false;
        planner_replan_hold_ = false;
        have_last_valid_output_ = false;
        validated_trajectory_id_ = -1;
        setStatus("WAIT_ALTITUDE_REFERENCE");
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
        get_logger(), "[BRIDGE_ALTITUDE_REFERENCE_REJECT] flight_id=%lu",
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
    RCLCPP_WARN(
      get_logger(),
      "[BRIDGE_ALTITUDE_REFERENCE_ACCEPTED] flight_id=%lu ground_map=%.3f target_map=%.3f",
      static_cast<unsigned long>(msg->flight_id), ground_z_map_, target_z_map_);
  }

  void setStatus(const std::string & status)
  {
    if (status == status_) {
      return;
    }
    status_ = status;
    std_msgs::msg::String message;
    message.data = status_;
    status_publisher_->publish(message);
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = "ego_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z = effectiveMaxHeight() + 0.25;
    marker.scale.z = 0.18;
    marker.color.r = 1.0;
    marker.color.g = status_ == "TRACKING" || status_ == "SHADOW_TRACKING" ? 1.0 : 0.35;
    marker.color.a = 1.0;
    marker.text = status_;
    status_marker_publisher_->publish(marker);
    RCLCPP_INFO(
      get_logger(),
      "[EGO_STATUS] state=%s control_enabled=%s map=%s odom=%s goal=%s trajectory=%s",
      status_.c_str(), control_enabled_ ? "true" : "false",
      have_map_ ? "ready" : "missing", have_odom_ ? "ready" : "missing",
      have_goal_ ? "ready" : "missing", have_trajectory_ ? "ready" : "missing");
  }

  void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!race_ego_bridge::frameMatches(msg->header.frame_id, frame_id_)) {
      map_frame_valid_ = false;
      setStatus("FRAME_MISMATCH");
      return;
    }
    pcl::fromROSMsg(*msg, *cloud_);
    if (cloud_->empty()) {
      have_map_ = false;
      setStatus("WAIT_MAP");
      return;
    }
    kdtree_.setInputCloud(cloud_);
    raw_obstacle_distance_index_.rebuild(*cloud_);
    have_map_ = true;
    map_frame_valid_ = true;
    last_map_receive_time_ = now();
    last_map_source_time_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    if (last_map_source_time_.nanoseconds() == 0) {
      last_map_source_time_ = last_map_receive_time_;
    }
    if (have_trajectory_) {
      trajectory_collision_free_ = checkActivePredictedPathCollision();
      if (!trajectory_collision_free_) {
        have_bootstrap_setpoint_ = false;
        logTrajectoryCollision();
        setStatus(collision_reason_);
      }
    }
    if (have_bootstrap_setpoint_ && !pointCollisionFree(
        bootstrap_point_, nullptr, nullptr, nullptr, active_rejection_clearance_))
    {
      have_bootstrap_setpoint_ = false;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[EGO_OCCUPANCY_MAP] topic=%s frame=%s raw_points=%zu map_age=%.3f",
      occupancy_topic_.c_str(), msg->header.frame_id.c_str(), cloud_->size(), occupancyMapAge());
  }

  void plannerSafetyCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "EGO_REPLAN_REANCHOR_RETRY") {
      if (!planner_replan_hold_) {
        planner_replan_hold_ = true;
        planner_replan_hold_since_ = now();
        RCLCPP_WARN(
          get_logger(),
          "[EGO_REPLAN_HOLD] reason=reanchor_retry timeout_sec=%.3f",
          replan_hold_timeout_sec_);
      }
      return;
    }
    if (msg->data == "EGO_TRAJECTORY_COLLISION" || msg->data == "EGO_OCCUPANCY_STALE" ||
      msg->data == "EGO_TRAJECTORY_INVALID" || msg->data == "EGO_REPLAN_REANCHOR_FAILED")
    {
      collision_reason_ = msg->data;
      trajectory_collision_free_ = false;
      have_trajectory_ = false;
      have_bootstrap_setpoint_ = false;
      planner_replan_hold_ = false;
      setStatus(msg->data);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!race_ego_bridge::frameMatches(msg->header.frame_id, frame_id_)) {
      odom_frame_valid_ = false;
      setStatus("FRAME_MISMATCH");
      return;
    }
    odom_frame_valid_ = true;
    have_odom_ = true;
    last_odom_time_ = now();
    current_position_ = {
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    appendActualPath(current_position_);
    // During vertical takeoff Offboard owns XY/Z and the EGO trajectory is
    // validated at flight_height_. Do not treat the local map's virtual
    // flight-band boundary as a vehicle collision before planar tracking.
    const bool position_collision_check_enabled =
      flight_mode_ != "flat" || current_position_.z >= effectiveFlightHeight() - 0.05;
    auto occupancy_check_position = current_position_;
    if (flight_mode_ == "flat") {
      occupancy_check_position.z = effectiveFlightHeight();
    }
    // Before the first validated EGO command the vehicle is still in the
    // Offboard takeoff transaction.  A map clearance check at that point must
    // not manufacture a trajectory collision for id=0/goal=missing.
    const bool have_active_ego_command = have_trajectory_ || have_bootstrap_setpoint_;
    if (have_active_ego_command && have_map_ && position_collision_check_enabled &&
      !pointCollisionFree(
        occupancy_check_position, &collision_nearest_obstacle_, &collision_clearance_,
        &collision_nearby_occupied_))
    {
      trajectory_collision_free_ = false;
      have_bootstrap_setpoint_ = false;
      collision_point_ = occupancy_check_position;
      collision_sample_index_ = 0;
      collision_reason_ = "EGO_TRAJECTORY_COLLISION";
      logTrajectoryCollision();
      setStatus("EGO_TRAJECTORY_COLLISION");
    }
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (flight_mode_ == "flat" && !altitude_reference_valid_) {
      have_goal_ = false;
      setStatus("WAIT_ALTITUDE_REFERENCE");
      return;
    }
    if (!race_ego_bridge::frameMatches(msg->header.frame_id, frame_id_)) {
      goal_frame_valid_ = false;
      setStatus("FRAME_MISMATCH");
      return;
    }
    const race_ego_bridge::Vec3 point{
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    current_goal_ = point;
    goal_frame_valid_ = race_ego_bridge::insideGeofence(
      point, min_x_, max_x_, min_y_, max_y_, effectiveMinHeight(), effectiveMaxHeight());
    have_goal_ = goal_frame_valid_;
    if (have_goal_) {
      ++local_goal_seq_;
    }
    if (!goal_frame_valid_) {
      have_trajectory_ = false;
      planner_replan_hold_ = false;
      trajectory_collision_free_ = false;
      validated_trajectory_id_ = -1;
      have_bootstrap_setpoint_ = false;
      setStatus("OUT_OF_GEOFENCE");
    }
  }

  void healthCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    px4_healthy_ = msg->connected;
    if (px4_healthy_) {
      last_px4_healthy_time_ = now();
    }
  }

  void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
  {
    if (validated_trajectory_id_ >= 0 && msg->traj_id <= validated_trajectory_id_) {
      RCLCPP_WARN(
        get_logger(),
        "[BRIDGE_STALE_CANDIDATE] candidate_id=%ld active_id=%ld",
        static_cast<int64_t>(msg->traj_id), validated_trajectory_id_);
      return;
    }
    if (msg->order < 2 || msg->pos_pts.size() <= static_cast<std::size_t>(msg->order) ||
      msg->knots.size() <= msg->pos_pts.size())
    {
      rejectCandidate("SETPOINT_INVALID", msg->traj_id);
      return;
    }

    Eigen::MatrixXd control_points(3, msg->pos_pts.size());
    for (std::size_t i = 0; i < msg->pos_pts.size(); ++i) {
      control_points(0, i) = msg->pos_pts[i].x;
      control_points(1, i) = msg->pos_pts[i].y;
      control_points(2, i) = msg->pos_pts[i].z;
    }
    Eigen::VectorXd knots(msg->knots.size());
    for (std::size_t i = 0; i < msg->knots.size(); ++i) {
      knots(i) = msg->knots[i];
    }

    ego_planner::UniformBspline trajectory(control_points, msg->order, 0.1);
    trajectory.setKnot(knots);
    const double duration = trajectory.getTimeSum();
    if (!std::isfinite(duration) || duration <= 0.0) {
      rejectCandidate("SETPOINT_INVALID", msg->traj_id);
      return;
    }

    std::vector<race_ego_bridge::Vec3> candidate_points;
    std::vector<double> candidate_sample_times;
    sampleTrajectory(trajectory, duration, candidate_points, candidate_sample_times);
    const auto clearance_selection = race_ego_bridge::selectTrajectoryClearance(
      msg->constrained_clearance, msg->required_clearance,
      constrained_clearance_enabled_, rejection_clearance_, constrained_clearance_);
    if (!clearance_selection.accepted) {
      rejectCandidate("CONSTRAINED_CLEARANCE_METADATA_INVALID", msg->traj_id);
      return;
    }
    const double candidate_clearance = clearance_selection.required_clearance;
    const bool candidate_collision_free = checkPredictedPathCollision(
      candidate_points, candidate_sample_times, 0, candidate_clearance);
    const auto & start = candidate_points.front();
    const auto & finish = candidate_points.back();
    RCLCPP_INFO(
      get_logger(),
      "[EGO_TRAJECTORY] trajectory_id=%ld local_goal_seq=%lu start_time=%.3f points=%zu duration=%.3f start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) collision_free=%s",
      static_cast<int64_t>(msg->traj_id), local_goal_seq_, now().seconds(),
      candidate_points.size(), duration,
      start.x, start.y, start.z, finish.x, finish.y, finish.z,
      candidate_collision_free ? "true" : "false");
    if (!candidate_collision_free) {
      logTrajectoryCollision(
        msg->traj_id, rclcpp::Time(msg->start_time, get_clock()->get_clock_type()),
        candidate_points.size());
      rejectCandidate(collision_reason_, msg->traj_id);
      return;
    }

    // A trajectory can be collision-free and dynamically valid in isolation
    // while still being unsafe to switch to.  Compare its state at "now" with
    // the command currently owned by the bridge.  Large discontinuities were
    // observed as 0.4--0.5 m position-target jumps and must enter the existing
    // measured-position hold/replan path instead of reaching PX4.
    const rclcpp::Time candidate_start(msg->start_time, get_clock()->get_clock_type());
    const bool rebase_candidate_start = planner_replan_hold_;
    if (have_last_valid_output_ && (have_trajectory_ || planner_replan_hold_)) {
      auto velocity_trajectory = trajectory.getDerivative();
      auto acceleration_trajectory = velocity_trajectory.getDerivative();
      const double candidate_time = race_ego_bridge::trajectorySwitchEvaluationTime(
        (now() - candidate_start).seconds(), duration, rebase_candidate_start);
      const auto position_eigen = trajectory.evaluateDeBoorT(candidate_time);
      const auto velocity_eigen = velocity_trajectory.evaluateDeBoorT(candidate_time);
      const auto acceleration_eigen = acceleration_trajectory.evaluateDeBoorT(candidate_time);
      race_ego_bridge::TrajectoryState previous{
        {last_valid_output_.position.x, last_valid_output_.position.y,
          last_valid_output_.position.z},
        {last_valid_output_.velocity.x, last_valid_output_.velocity.y,
          last_valid_output_.velocity.z},
        {last_valid_output_.acceleration_or_force.x,
          last_valid_output_.acceleration_or_force.y,
          last_valid_output_.acceleration_or_force.z}};
      race_ego_bridge::TrajectoryState candidate{
        {position_eigen.x(), position_eigen.y(), position_eigen.z()},
        {velocity_eigen.x(), velocity_eigen.y(), velocity_eigen.z()},
        {acceleration_eigen.x(), acceleration_eigen.y(), acceleration_eigen.z()}};
      if (flight_mode_ == "flat") {
        candidate.position.z = effectiveFlightHeight();
        candidate.velocity.z = 0.0;
        candidate.acceleration.z = 0.0;
      }
      const auto transition = race_ego_bridge::transitionStateContinuous(
        previous, candidate, switch_position_tolerance_m_,
        switch_velocity_tolerance_mps_, switch_acceleration_tolerance_mps2_);
      if (!transition.continuous) {
        if (!planner_replan_hold_) {
          planner_replan_hold_ = true;
          planner_replan_hold_since_ = now();
        }
        RCLCPP_ERROR(
          get_logger(),
          "[BRIDGE_SWITCH_REJECTED] candidate_id=%ld active_id=%ld "
          "errors=(position=%.3f velocity=%.3f acceleration=%.3f) "
          "limits=(%.3f %.3f %.3f); hold measured position and replan",
          static_cast<int64_t>(msg->traj_id), validated_trajectory_id_,
          transition.position_error, transition.velocity_error,
          transition.acceleration_error, switch_position_tolerance_m_,
          switch_velocity_tolerance_mps_, switch_acceleration_tolerance_mps2_);
        setStatus("REPLAN_SWITCH_DISCONTINUITY");
        return;
      }
    }

    predicted_points_ = std::move(candidate_points);
    predicted_sample_times_ = std::move(candidate_sample_times);
    trajectory_id_ = msg->traj_id;
    validated_trajectory_id_ = msg->traj_id;
    validated_local_goal_seq_ = local_goal_seq_;
    auto validated_message = *msg;
    if (rebase_candidate_start) {
      validated_message.start_time = now();
      RCLCPP_WARN(
        get_logger(),
        "[BRIDGE_TRAJECTORY_REBASED] trajectory_id=%ld old_start=%.3f new_start=%.3f "
        "reason=measured_position_hold",
        static_cast<int64_t>(msg->traj_id), candidate_start.seconds(),
        rclcpp::Time(validated_message.start_time, get_clock()->get_clock_type()).seconds());
    }
    trajectory_stamp_ = rclcpp::Time(
      validated_message.start_time, get_clock()->get_clock_type());
    last_trajectory_time_ = now();
    trajectory_collision_free_ = true;
    have_trajectory_ = true;
    planner_replan_hold_ = false;
    active_rejection_clearance_ = candidate_clearance;
    have_bootstrap_setpoint_ = prepareBootstrapSetpoint(candidate_clearance);
    publishPredictedPath();
    publishControlPoints(msg->pos_pts);
    validated_bspline_publisher_->publish(validated_message);
    RCLCPP_INFO(
      get_logger(),
      "[BRIDGE_CANDIDATE_COMMIT] trajectory_id=%ld local_goal_seq=%lu output_topic=%s",
      trajectory_id_, validated_local_goal_seq_, validated_bspline_topic_.c_str());
    setStatus("TRACKING");
  }

  void rejectCandidate(const std::string & reason, int64_t candidate_id)
  {
    // Callers pass collision_reason_ itself here, so `reason` is an alias for a
    // member that the active recheck below overwrites.  Copy it first, or the
    // logged and published reason becomes the recheck's result rather than the
    // reason the candidate was actually rejected for.
    const std::string candidate_reason = reason;
    const bool active_still_safe = have_trajectory_ && checkActivePredictedPathCollision();
    trajectory_collision_free_ = active_still_safe;
    if (active_still_safe) {
      RCLCPP_WARN(
        get_logger(),
        "[BRIDGE_CANDIDATE_REJECTED_ACTIVE_PRESERVED] candidate_id=%ld "
        "active_id=%ld reason=%s",
        candidate_id, validated_trajectory_id_, candidate_reason.c_str());
      setStatus(control_enabled_ ? "TRACKING" : "SHADOW_TRACKING");
      return;
    }
    have_bootstrap_setpoint_ = false;
    // The active recheck just ran and its own failure reason is the more recent
    // and more relevant one; fall back to the candidate's reason when the
    // recheck did not run (no active trajectory) or reported nothing.
    const std::string active_reason =
      collision_reason_.empty() ? candidate_reason : collision_reason_;
    setStatus(active_reason.empty() ? "NO_SAFE_TRAJECTORY" : active_reason);
  }

  void sampleTrajectory(
    ego_planner::UniformBspline & trajectory, double duration,
    std::vector<race_ego_bridge::Vec3> & points,
    std::vector<double> & sample_times)
  {
    points.clear();
    sample_times.clear();
    auto to_executed_point = [this](const Eigen::Vector3d & point) {
        return race_ego_bridge::Vec3{
        point.x(), point.y(), flight_mode_ == "flat" ? effectiveFlightHeight() : point.z()};
      };

    constexpr double base_time_step = 0.02;
    auto previous = to_executed_point(trajectory.evaluateDeBoorT(0.0));
    points.push_back(previous);
    sample_times.push_back(0.0);
    for (double t = base_time_step; t < duration + base_time_step; t += base_time_step) {
      const double sample_time = std::min(t, duration);
      const auto next = to_executed_point(trajectory.evaluateDeBoorT(sample_time));
      const double distance = std::hypot(
        std::hypot(next.x - previous.x, next.y - previous.y), next.z - previous.z);
      const auto subdivisions = race_ego_bridge::spatialSubdivisions(
        distance, trajectory_sample_spacing_);
      for (std::size_t i = 1; i <= subdivisions; ++i) {
        const double ratio = static_cast<double>(i) / subdivisions;
        points.push_back(
        {
          previous.x + (next.x - previous.x) * ratio,
          previous.y + (next.y - previous.y) * ratio,
          previous.z + (next.z - previous.z) * ratio});
        sample_times.push_back(
          (t - base_time_step) + ratio * (sample_time - (t - base_time_step)));
      }
      previous = next;
      if (sample_time >= duration) {
        break;
      }
    }
  }

  bool prepareBootstrapSetpoint(const double required_clearance)
  {
    for (std::size_t i = 0; i < predicted_points_.size(); ++i) {
      const auto & point = predicted_points_[i];
      if (point.z < effectiveMinHeight() ||
        !pointCollisionFree(point, nullptr, nullptr, nullptr, required_clearance))
      {
        continue;
      }
      double yaw_map = 0.0;
      for (std::size_t j = i + 1; j < predicted_points_.size(); ++j) {
        const double dx = predicted_points_[j].x - point.x;
        const double dy = predicted_points_[j].y - point.y;
        if (std::hypot(dx, dy) > 0.05) {
          yaw_map = std::atan2(dy, dx);
          break;
        }
      }
      // PositionTarget carries ENU; the Offboard node converts to its internal
      // NED at the boundary.  Position, velocity and acceleration are all
      // finite here, so no ignore bits are set.
      const auto enu = race_ego_bridge::mapToEnu(point);
      bootstrap_setpoint_ = mavros_msgs::msg::PositionTarget{};
      bootstrap_setpoint_.coordinate_frame =
        mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
      bootstrap_setpoint_.type_mask = 0;
      bootstrap_setpoint_.position.x = enu.x;
      bootstrap_setpoint_.position.y = enu.y;
      bootstrap_setpoint_.position.z = enu.z;
      bootstrap_setpoint_.yaw =
        static_cast<float>(race_ego_bridge::mapYawToEnu(yaw_map));
      bootstrap_setpoint_.yaw_rate = 0.0F;
      bootstrap_point_ = point;
      return true;
    }
    return false;
  }

  double occupancyMapAge() const
  {
    if (last_map_source_time_.nanoseconds() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return (now() - last_map_source_time_).seconds();
  }

  bool pointCollisionFree(
    const race_ego_bridge::Vec3 & point,
    race_ego_bridge::Vec3 * nearest_obstacle = nullptr,
    double * clearance = nullptr,
    int * nearby_occupied = nullptr,
    const double required_clearance = std::numeric_limits<double>::quiet_NaN()) const
  {
    if (!have_map_ || cloud_->empty()) {
      return false;
    }
    pcl::PointXYZ query;
    query.x = point.x;
    query.y = point.y;
    query.z = point.z;
    const double effective_clearance =
      std::isfinite(required_clearance) && required_clearance > 0.0 ?
      required_clearance : rejection_clearance_;
    if (!occupancy_is_inflated_) {
      Eigen::Vector3d nearest;
      double distance = std::numeric_limits<double>::infinity();
      if (!raw_obstacle_distance_index_.nearest(
          Eigen::Vector3d(point.x, point.y, point.z), nearest, distance)) {return false;}
      if (nearest_obstacle != nullptr) {
        *nearest_obstacle = {nearest.x(), nearest.y(), nearest.z()};
      }
      if (clearance != nullptr) {*clearance = distance;}
      if (nearby_occupied != nullptr) {*nearby_occupied = distance < effective_clearance ? 1 : 0;}
      return distance >= effective_clearance;
    }
    std::vector<int> indices;
    std::vector<float> distances;
    const double search_radius = occupancy_is_inflated_ ?
      std::sqrt(3.0) * occupancy_resolution_ * 0.5 + 1e-3 : effective_clearance;
    kdtree_.radiusSearch(query, search_radius, indices, distances);

    int collision_index = -1;
    double collision_distance = std::numeric_limits<double>::infinity();
    const double half_voxel = occupancy_resolution_ * 0.5 + 1e-3;
    for (std::size_t i = 0; i < indices.size(); ++i) {
      const auto & obstacle = cloud_->points[indices[i]];
      const bool collides = occupancy_is_inflated_ ?
        std::abs(obstacle.x - query.x) <= half_voxel &&
        std::abs(obstacle.y - query.y) <= half_voxel &&
        std::abs(obstacle.z - query.z) <= half_voxel : true;
      if (collides && distances[i] < collision_distance) {
        collision_distance = distances[i];
        collision_index = indices[i];
      }
    }
    if (nearby_occupied != nullptr) {
      std::vector<int> nearby_indices;
      std::vector<float> nearby_distances;
      *nearby_occupied = kdtree_.radiusSearch(
        query, std::max(0.30, search_radius), nearby_indices, nearby_distances);
    }
    if (collision_index < 0) {
      return true;
    }
    const auto & obstacle = cloud_->points[collision_index];
    if (nearest_obstacle != nullptr) {
      *nearest_obstacle = {obstacle.x, obstacle.y, obstacle.z};
    }
    if (clearance != nullptr) {
      *clearance = occupancy_is_inflated_ ? 0.0 : std::sqrt(collision_distance);
    }
    return false;
  }

  bool checkActivePredictedPathCollision()
  {
    const double elapsed = (now() - trajectory_stamp_).seconds();
    const std::size_t start_index = race_ego_bridge::activeTrajectoryStartIndex(
      predicted_sample_times_, elapsed, active_recheck_history_sec_);
    return checkPredictedPathCollision(
      predicted_points_, predicted_sample_times_, start_index, active_rejection_clearance_);
  }

  bool checkPredictedPathCollision(
    const std::vector<race_ego_bridge::Vec3> & points,
    const std::vector<double> & sample_times,
    const std::size_t start_index = 0,
    const double required_clearance = std::numeric_limits<double>::quiet_NaN())
  {
    collision_required_clearance_ =
      std::isfinite(required_clearance) && required_clearance > 0.0 ?
      required_clearance : rejection_clearance_;
    collision_reason_ = "EGO_TRAJECTORY_COLLISION";
    collision_sample_index_ = 0;
    collision_point_ = {};
    collision_nearest_obstacle_ = {};
    collision_clearance_ = std::numeric_limits<double>::quiet_NaN();
    collision_nearby_occupied_ = 0;
    collision_sample_time_ = 0.0;
    if (!have_map_ || points.empty()) {
      collision_reason_ = "WAIT_MAP";
      return false;
    }
    if (race_ego_bridge::timedOut(occupancyMapAge(), occupancy_timeout_sec_)) {
      collision_reason_ = "EGO_OCCUPANCY_STALE";
      return false;
    }
    bool have_trackable_point = false;
    for (std::size_t i = std::min(start_index, points.size() - 1); i < points.size(); ++i) {
      const auto & point = points[i];
      if (!race_ego_bridge::finite(point) || point.x < min_x_ || point.x > max_x_ ||
        point.y < min_y_ || point.y > max_y_ || point.z < min_planning_height_ ||
        point.z > effectiveMaxHeight())
      {
        collision_reason_ = "OUT_OF_GEOFENCE";
        collision_sample_index_ = i;
        collision_point_ = point;
        collision_sample_time_ = i < sample_times.size() ? sample_times[i] : 0.0;
        return false;
      }
      // Offboard owns vertical takeoff and never follows EGO commands below
      // min_height_. Validate the actual flight segment, not the ground bootstrap.
      if (point.z < effectiveMinHeight()) {
        continue;
      }
      have_trackable_point = true;
      if (!pointCollisionFree(
          point, &collision_nearest_obstacle_, &collision_clearance_,
          &collision_nearby_occupied_, required_clearance))
      {
        collision_sample_index_ = i;
        collision_point_ = point;
        collision_sample_time_ = i < sample_times.size() ? sample_times[i] : 0.0;
        return false;
      }
    }
    if (!have_trackable_point) {
      // Every sample was below min_height_, so nothing was actually validated.
      // Report that instead of leaving the pessimistic default in place, which
      // would surface a ground bootstrap as a collision.
      collision_reason_ = "NO_TRACKABLE_POINT";
      return false;
    }
    // Clearing on success is what keeps a later rejection from reporting this
    // check's stale reason.  Several callers forward collision_reason_ verbatim,
    // and offboard latches some of its values permanently, so a leftover string
    // can turn an ordinary rejection into a hold that lasts the whole flight.
    collision_reason_.clear();
    return true;
  }

  void logTrajectoryCollision(
    int64_t diagnostic_trajectory_id = -1,
    const rclcpp::Time & diagnostic_trajectory_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME),
    std::size_t diagnostic_sample_count = 0)
  {
    const int64_t logged_trajectory_id = diagnostic_trajectory_id >= 0 ?
      diagnostic_trajectory_id : trajectory_id_;
    const rclcpp::Time logged_trajectory_stamp = diagnostic_trajectory_stamp.nanoseconds() != 0 ?
      diagnostic_trajectory_stamp : trajectory_stamp_;
    const std::size_t logged_sample_count = diagnostic_sample_count > 0 ?
      diagnostic_sample_count : predicted_points_.size();
    if (collision_reason_ == "EGO_OCCUPANCY_STALE") {
      RCLCPP_ERROR(
        get_logger(), "[EGO_OCCUPANCY_STALE] trajectory_id=%ld occupancy_map_age=%.3f frame=%s",
        logged_trajectory_id, occupancyMapAge(), frame_id_.c_str());
      return;
    }
    if (collision_reason_ == "OUT_OF_GEOFENCE") {
      RCLCPP_ERROR(
        get_logger(),
        "[EGO_TRAJECTORY_OUT_OF_GEOFENCE] trajectory_id=%ld local_goal_seq=%lu "
        "sample_index=%zu position=(%.3f,%.3f,%.3f) "
        "bounds=x=[%.3f,%.3f] y=[%.3f,%.3f] z=[%.3f,%.3f] frame=%s",
        logged_trajectory_id, local_goal_seq_, collision_sample_index_, collision_point_.x,
        collision_point_.y, collision_point_.z, min_x_, max_x_, min_y_, max_y_,
        min_planning_height_, effectiveMaxHeight(), frame_id_.c_str());
      return;
    }
    if (collision_reason_ != "EGO_TRAJECTORY_COLLISION") {
      return;
    }
    if (last_logged_collision_trajectory_id_ == logged_trajectory_id) {
      return;
    }
    last_logged_collision_trajectory_id_ = logged_trajectory_id;
    RCLCPP_ERROR(
      get_logger(),
      "[EGO_TRAJECTORY_COLLISION] trajectory_id=%ld local_goal_seq=%lu sample_index=%zu "
      "position=(%.3f,%.3f,%.3f) nearest_obstacle=(%.3f,%.3f,%.3f) "
      "clearance=%.3f occupancy_map_age=%.3f frame=%s nearby_occupied_voxels=%d",
      logged_trajectory_id, local_goal_seq_, collision_sample_index_, collision_point_.x,
      collision_point_.y, collision_point_.z, collision_nearest_obstacle_.x,
      collision_nearest_obstacle_.y, collision_nearest_obstacle_.z, collision_clearance_,
      occupancyMapAge(), frame_id_.c_str(), collision_nearby_occupied_);
    RCLCPP_ERROR(
      get_logger(),
      "[BRIDGE_CONTINUOUS_COLLISION] trajectory_id=%ld trajectory_stamp=%.9f "
      "occupancy_stamp=%.9f occupancy_frame=%s sample_index=%zu sample_time=%.3f "
      "sample_position=(%.3f,%.3f,%.3f) nearest_obstacle=(%.3f,%.3f,%.3f) "
      "clearance=%.6f required_clearance=%.6f sample_dt=%.3f sample_count=%zu",
      logged_trajectory_id, logged_trajectory_stamp.seconds(), last_map_source_time_.seconds(),
      frame_id_.c_str(),
      collision_sample_index_, collision_sample_time_, collision_point_.x, collision_point_.y,
      collision_point_.z, collision_nearest_obstacle_.x, collision_nearest_obstacle_.y,
      collision_nearest_obstacle_.z, collision_clearance_, collision_required_clearance_, 0.02,
      logged_sample_count);
  }

  bool prerequisitesReady()
  {
    if (flight_mode_ == "flat" && !altitude_reference_valid_) {
      setStatus("WAIT_ALTITUDE_REFERENCE"); return false;
    }
    if (!have_map_) {setStatus("WAIT_MAP"); return false;}
    if (!have_odom_) {setStatus("WAIT_ODOM"); return false;}
    if (have_trajectory_ && !trajectory_collision_free_) {
      setStatus(collision_reason_.empty() ? "NO_SAFE_TRAJECTORY" : collision_reason_);
      return false;
    }
    if (!have_goal_) {setStatus("WAIT_GOAL"); return false;}
    if (!map_frame_valid_ || !odom_frame_valid_ || !goal_frame_valid_) {
      setStatus("FRAME_MISMATCH"); return false;
    }
    // PX4 publishes local-position health at a much higher rate than the
    // safety window. Require an unhealthy condition to persist for that
    // window so one transient flag during an estimator reset cannot brake a
    // collision-checked EGO trajectory. A persistent health loss still fails
    // closed through the unchanged ODOM_UNHEALTHY path.
    if (require_strict_local_position_health_ &&
      (last_px4_healthy_time_.nanoseconds() == 0 ||
      race_ego_bridge::timedOut(
        (now() - last_px4_healthy_time_).seconds(), odom_timeout_sec_)))
    {
      setStatus("ODOM_UNHEALTHY"); return false;
    }
    if (race_ego_bridge::timedOut((now() - last_odom_time_).seconds(), odom_timeout_sec_)) {
      setStatus("ODOM_UNHEALTHY"); return false;
    }
    if (!have_trajectory_) {setStatus("PLANNING"); return false;}
    // B-splines are intentionally replanned at a lower rate than traj_server
    // publishes PositionCommand.  The command stream remains tied to the
    // validated trajectory_id and is checked by command_timeout_sec_ below.
    // Treating the B-spline update cadence as a command timeout creates a
    // periodic HOLD between otherwise fresh, collision-checked commands.
    return true;
  }

  void commandCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    last_command_time_ = now();
    if (!race_ego_bridge::frameMatches(msg->header.frame_id, frame_id_)) {
      setStatus("FRAME_MISMATCH");
      return;
    }
    if (static_cast<int64_t>(msg->trajectory_id) != validated_trajectory_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject unvalidated PositionCommand trajectory_id=%u validated_trajectory_id=%ld",
        msg->trajectory_id, validated_trajectory_id_);
      return;
    }
    if (!prerequisitesReady()) {
      return;
    }

    race_ego_bridge::Vec3 position{
      msg->position.x, msg->position.y, msg->position.z};
    race_ego_bridge::Vec3 velocity{
      msg->velocity.x, msg->velocity.y, msg->velocity.z};
    race_ego_bridge::Vec3 acceleration{
      msg->acceleration.x, msg->acceleration.y, msg->acceleration.z};
    if (flight_mode_ == "flat") {
      position.z = effectiveFlightHeight();
      velocity.z = 0.0;
      acceleration.z = 0.0;
    }
    if (!race_ego_bridge::insideGeofence(
        position, min_x_, max_x_, min_y_, max_y_, effectiveMinHeight(), effectiveMaxHeight()))
    {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000, "Reject command outside flight band z=%.3f",
        position.z);
      setStatus("OUT_OF_GEOFENCE");
      return;
    }
    const bool velocity_valid = race_ego_bridge::withinVectorLimit(
      velocity, max_velocity_, feasibility_tolerance_);
    const bool acceleration_valid = race_ego_bridge::withinVectorLimit(
      acceleration, max_acceleration_, feasibility_tolerance_);
    const bool yaw_valid = std::isfinite(msg->yaw);
    const bool yaw_rate_valid = std::isfinite(msg->yaw_dot);
    if (!velocity_valid || !acceleration_valid || !yaw_valid || !yaw_rate_valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Reject command dynamics: speed=%.3f accel=%.3f limits=(%.3f,%.3f)x%.2f "
        "valid=(velocity=%s acceleration=%s yaw=%s yaw_dot=%s) yaw=%.3f yaw_dot=%.3f",
        std::hypot(std::hypot(velocity.x, velocity.y), velocity.z),
        std::hypot(std::hypot(acceleration.x, acceleration.y), acceleration.z),
        max_velocity_, max_acceleration_, feasibility_tolerance_,
        velocity_valid ? "true" : "false", acceleration_valid ? "true" : "false",
        yaw_valid ? "true" : "false", yaw_rate_valid ? "true" : "false",
        msg->yaw, msg->yaw_dot);
      command_dynamics_valid_ = false;
      const auto invalid_since = now();
      if (dynamic_invalid_since_.nanoseconds() == 0) {
        dynamic_invalid_since_ = invalid_since;
        dynamic_invalid_trajectory_id_ = msg->trajectory_id;
      }
      const double invalid_age =
        (invalid_since - dynamic_invalid_since_).seconds();
      if (have_last_valid_output_ && invalid_age < dynamic_invalid_grace_sec_) {
        // Keep the last collision-checked command fresh while a newly accepted
        // trajectory replaces a transiently invalid command stream.
        auto hold_output = last_valid_output_;
        hold_output.velocity.x = 0.0;
        hold_output.velocity.y = 0.0;
        hold_output.velocity.z = 0.0;
        hold_output.acceleration_or_force.x = 0.0;
        hold_output.acceleration_or_force.y = 0.0;
        hold_output.acceleration_or_force.z = 0.0;
        hold_output.header.stamp = now();
        if (publish_setpoint_) {
          output_publisher_->publish(hold_output);
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[EGO_DYNAMICS_GRACE] trajectory_id=%ld hold_last_valid age=%.3f limit=%.3f",
          trajectory_id_, invalid_age, dynamic_invalid_grace_sec_);
        last_valid_output_time_ = now();
        return;
      }
      const bool successor_is_fresh =
        validated_trajectory_id_ > dynamic_invalid_trajectory_id_ &&
        !race_ego_bridge::timedOut(
        (now() - last_trajectory_time_).seconds(), dynamic_invalid_grace_sec_);
      if ((!have_last_valid_output_ || invalid_age >= dynamic_invalid_grace_sec_) &&
        !successor_is_fresh)
      {
        setStatus("SETPOINT_INVALID");
      }
      return;
    }
    if (!pointCollisionFree(
        position, &collision_nearest_obstacle_, &collision_clearance_,
        &collision_nearby_occupied_))
    {
      trajectory_collision_free_ = false;
      collision_point_ = position;
      collision_sample_index_ = 0;
      collision_reason_ = "EGO_TRAJECTORY_COLLISION";
      logTrajectoryCollision();
      setStatus("EGO_TRAJECTORY_COLLISION");
      return;
    }

    // PositionTarget is ENU; the Offboard node converts at its boundary.  The
    // jerk feed-forward has no field in PositionTarget and is dropped -- it was
    // always NaN on this path anyway.
    const auto position_enu = race_ego_bridge::mapToEnu(position);
    const auto velocity_enu = race_ego_bridge::mapToEnu(velocity);
    const auto acceleration_enu = race_ego_bridge::mapToEnu(acceleration);
    mavros_msgs::msg::PositionTarget output{};
    output.header.stamp = now();
    output.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    output.type_mask = 0;
    output.position.x = position_enu.x;
    output.position.y = position_enu.y;
    output.position.z = position_enu.z;
    output.velocity.x = velocity_enu.x;
    output.velocity.y = velocity_enu.y;
    output.velocity.z = velocity_enu.z;
    output.acceleration_or_force.x = acceleration_enu.x;
    output.acceleration_or_force.y = acceleration_enu.y;
    output.acceleration_or_force.z = acceleration_enu.z;
    output.yaw = static_cast<float>(race_ego_bridge::mapYawToEnu(msg->yaw));
    // A map-frame yaw rate is about Up, same sense as ENU, so the sign flip the
    // NED output needed is gone.
    output.yaw_rate = static_cast<float>(std::clamp(
        msg->yaw_dot, -max_yaw_rate_rad_s_, max_yaw_rate_rad_s_));
    if (publish_setpoint_) {
      std_msgs::msg::UInt64 command_goal_seq;
      command_goal_seq.data = validated_local_goal_seq_;
      command_goal_seq_publisher_->publish(command_goal_seq);
      output_publisher_->publish(output);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(), 1000,
      "[SETPOINT_SOURCE] source=EGO trajectory_id=%u control_enabled=%s position_enu=(%.3f,%.3f,%.3f)",
      msg->trajectory_id, control_enabled_ ? "true" : "false",
      output.position.x, output.position.y, output.position.z);
    last_valid_output_time_ = now();
    last_valid_output_ = output;
    have_last_valid_output_ = true;
    dynamic_invalid_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    dynamic_invalid_trajectory_id_ = -1;
    command_dynamics_valid_ = true;
    publishSetpoint(position);
    const double goal_error = std::hypot(
      std::hypot(current_position_.x - current_goal_.x, current_position_.y - current_goal_.y),
      current_position_.z - current_goal_.z);
    const double speed = std::hypot(std::hypot(velocity.x, velocity.y), velocity.z);
    if (goal_error <= 0.15 && speed <= 0.08) {
      const std::string local_goal_status =
        "LOCAL_GOAL_REACHED ego_goal_seq=" + std::to_string(local_goal_seq_);
      if (status_ != local_goal_status) {
        RCLCPP_INFO(
          get_logger(),
          "[EGO_LOCAL_GOAL_REACHED] ego_goal_seq=%lu distance=%.3f",
          local_goal_seq_, goal_error);
      }
      setStatus(local_goal_status);
    } else {
      setStatus(control_enabled_ ? "TRACKING" : "SHADOW_TRACKING");
    }
  }

  void statusTimer()
  {
    if (planner_replan_hold_) {
      const double hold_age = (now() - planner_replan_hold_since_).seconds();
      if (!have_map_ || !have_odom_ || !have_goal_ ||
        race_ego_bridge::timedOut(occupancyMapAge(), occupancy_timeout_sec_) ||
        hold_age > replan_hold_timeout_sec_)
      {
        planner_replan_hold_ = false;
        setStatus("TRAJECTORY_TIMEOUT");
        return;
      }
      race_ego_bridge::Vec3 hold_point = current_position_;
      if (flight_mode_ == "flat") {
        hold_point.z = effectiveFlightHeight();
      }
      if (!pointCollisionFree(
          hold_point, nullptr, nullptr, nullptr, active_rejection_clearance_))
      {
        planner_replan_hold_ = false;
        collision_point_ = hold_point;
        collision_reason_ = "EGO_TRAJECTORY_COLLISION";
        logTrajectoryCollision();
        setStatus(collision_reason_);
        return;
      }
      mavros_msgs::msg::PositionTarget output{};
      output.header.stamp = now();
      output.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
      const auto enu = race_ego_bridge::mapToEnu(hold_point);
      output.position.x = enu.x;
      output.position.y = enu.y;
      output.position.z = enu.z;
      // Braking to a stop: zero feed-forward is meaningful, so keep it active.
      // Yaw is masked off only when no previous command established one.
      output.type_mask = have_last_valid_output_ ?
        0 : mavros_msgs::msg::PositionTarget::IGNORE_YAW;
      output.yaw = have_last_valid_output_ ? last_valid_output_.yaw : 0.0F;
      output.yaw_rate = 0.0F;
      if (publish_setpoint_) {
        std_msgs::msg::UInt64 command_goal_seq;
        command_goal_seq.data = validated_local_goal_seq_;
        command_goal_seq_publisher_->publish(command_goal_seq);
        output_publisher_->publish(output);
      }
      last_valid_output_time_ = now();
      last_valid_output_ = output;
      have_last_valid_output_ = true;
      publishSetpoint(hold_point);
      setStatus("PLANNING_HOLD");
      return;
    }
    if (bootstrapPrerequisitesReady()) {
      bootstrap_setpoint_.header.stamp = now();
      if (publish_setpoint_) {
        output_publisher_->publish(bootstrap_setpoint_);
      }
      last_valid_output_time_ = now();
      publishSetpoint(bootstrap_point_);
      setStatus(control_enabled_ ? "TRACKING" : "SHADOW_TRACKING");
      return;
    }
    if (!prerequisitesReady()) {
      return;
    }
    if (!command_dynamics_valid_ && dynamic_invalid_since_.nanoseconds() != 0) {
      // A valid successor B-spline can arrive before traj_server switches the
      // PositionCommand stream. Keep the last validated output during that
      // bounded handoff; latch only when successor planning also goes stale.
      const bool successor_is_fresh =
        validated_trajectory_id_ > dynamic_invalid_trajectory_id_ &&
        !race_ego_bridge::timedOut(
        (now() - last_trajectory_time_).seconds(), dynamic_invalid_grace_sec_);
      if (!successor_is_fresh &&
        (now() - dynamic_invalid_since_).seconds() >= dynamic_invalid_grace_sec_)
      {
        setStatus("SETPOINT_INVALID");
        return;
      }
    }
    if (last_command_time_.nanoseconds() == 0 || race_ego_bridge::timedOut(
        (now() - last_command_time_).seconds(), command_timeout_sec_))
    {
      setStatus("TRAJECTORY_TIMEOUT");
    }
  }

  bool bootstrapPrerequisitesReady()
  {
    if (!have_bootstrap_setpoint_ || currentAgl() >= bootstrap_release_height_) {
      return false;
    }
    if (!have_map_ || !have_odom_ || !have_goal_ || !map_frame_valid_ ||
      !odom_frame_valid_ || !goal_frame_valid_ ||
      (require_strict_local_position_health_ && !px4_healthy_))
    {
      return false;
    }
    if (race_ego_bridge::timedOut((now() - last_odom_time_).seconds(), odom_timeout_sec_)) {
      return false;
    }
    if (!pointCollisionFree(
        bootstrap_point_, nullptr, nullptr, nullptr, active_rejection_clearance_))
    {
      have_bootstrap_setpoint_ = false;
      setStatus("NO_SAFE_TRAJECTORY");
      return false;
    }
    return true;
  }

  void publishPredictedPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    for (const auto & point : predicted_points_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_publisher_->publish(path);
    predicted_path_publisher_->publish(path);
  }

  void publishControlPoints(const std::vector<geometry_msgs::msg::Point> & points)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = "ego_control_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.12;
    marker.color.r = 1.0;
    marker.color.g = 0.45;
    marker.color.b = 0.05;
    marker.color.a = 0.9;
    marker.points = points;
    control_points_publisher_->publish(marker);
  }

  void publishSetpoint(const race_ego_bridge::Vec3 & point)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = "ego_setpoint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.18;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    setpoint_publisher_->publish(marker);
  }

  void appendActualPath(const race_ego_bridge::Vec3 & point)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.w = 1.0;
    actual_path_.header = pose.header;
    actual_path_.poses.push_back(pose);
    if (actual_path_.poses.size() > 5000) {
      actual_path_.poses.erase(actual_path_.poses.begin(), actual_path_.poses.begin() + 1000);
    }
    actual_path_publisher_->publish(actual_path_);
  }

  std::string frame_id_;
  std::string position_command_topic_;
  std::string bspline_topic_;
  std::string validated_bspline_topic_;
  std::string output_topic_;
  std::string map_source_;
  std::string occupancy_topic_;
  std::string flight_mode_;
  bool occupancy_is_inflated_{true};
  double occupancy_resolution_{0.15};
  double trajectory_sample_spacing_{0.04};
  double occupancy_timeout_sec_{1.0};
  double active_recheck_history_sec_{0.5};
  double flight_height_{0.78};
  double max_velocity_{0.60};
  double max_acceleration_{0.80};
  double braking_deceleration_mps2_{0.80};
  double reaction_time_sec_{0.15};
  double minimum_reliable_detection_range_m_{0.725};
  double max_yaw_rate_rad_s_{0.80};
  double feasibility_tolerance_{1.10};
  double dynamic_invalid_grace_sec_{0.75};
  double replan_hold_timeout_sec_{3.0};
  double switch_position_tolerance_m_{0.05};
  double switch_velocity_tolerance_mps_{0.10};
  double switch_acceleration_tolerance_mps2_{0.20};
  double rejection_clearance_{0.30};
  bool constrained_clearance_enabled_{false};
  double constrained_clearance_{0.25};
  double active_rejection_clearance_{0.30};
  double command_timeout_sec_{0.20};
  double bspline_timeout_sec_{0.20};
  double odom_timeout_sec_{0.25};
  double min_planning_height_{-0.15};
  double min_height_{0.50};
  double bootstrap_release_height_{0.58};
  double max_height_{0.90};
  double min_x_{-7.5};
  double max_x_{7.5};
  double min_y_{-7.0};
  double max_y_{7.0};
  uint64_t altitude_reference_flight_id_{0};
  double ground_z_map_{0.0};
  double target_z_map_{0.0};
  bool publish_setpoint_{false};
  bool control_enabled_{false};
  bool require_strict_local_position_health_{false};
  bool have_map_{false};
  bool have_odom_{false};
  bool have_goal_{false};
  bool have_trajectory_{false};
  bool map_frame_valid_{false};
  bool odom_frame_valid_{false};
  bool goal_frame_valid_{false};
  bool px4_healthy_{false};
  bool trajectory_collision_free_{false};
  bool have_bootstrap_setpoint_{false};
  bool command_dynamics_valid_{true};
  bool planner_replan_hold_{false};
  bool altitude_reference_valid_{false};
  int64_t trajectory_id_{0};
  rclcpp::Time trajectory_stamp_{0, 0, RCL_ROS_TIME};
  int64_t validated_trajectory_id_{-1};
  uint64_t validated_local_goal_seq_{0};
  int64_t dynamic_invalid_trajectory_id_{-1};
  int64_t last_logged_collision_trajectory_id_{-1};
  uint64_t local_goal_seq_{0};
  std::string status_;
  std::string collision_reason_;
  std::size_t collision_sample_index_{0};
  double collision_sample_time_{0.0};
  race_ego_bridge::Vec3 collision_point_;
  race_ego_bridge::Vec3 collision_nearest_obstacle_;
  double collision_clearance_{std::numeric_limits<double>::quiet_NaN()};
  double collision_required_clearance_{0.30};
  int collision_nearby_occupied_{0};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_px4_healthy_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trajectory_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_output_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time dynamic_invalid_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time planner_replan_hold_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_map_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_map_source_time_{0, 0, RCL_ROS_TIME};
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  plan_env::RawObstacleDistanceIndex raw_obstacle_distance_index_;
  std::vector<race_ego_bridge::Vec3> predicted_points_;
  std::vector<double> predicted_sample_times_;
  nav_msgs::msg::Path actual_path_;
  race_ego_bridge::Vec3 current_position_;
  race_ego_bridge::Vec3 current_goal_;
  race_ego_bridge::Vec3 bootstrap_point_;
  mavros_msgs::msg::PositionTarget bootstrap_setpoint_;
  mavros_msgs::msg::PositionTarget last_valid_output_;
  bool have_last_valid_output_{false};
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr output_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr command_goal_seq_publisher_;
  rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr validated_bspline_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_points_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr status_marker_publisher_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_safety_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr health_subscription_;
  rclcpp::Subscription<race_msgs::msg::FlightAltitudeReference>::SharedPtr
    altitude_reference_subscription_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoTrajectoryBridge>());
  rclcpp::shutdown();
  return 0;
}
