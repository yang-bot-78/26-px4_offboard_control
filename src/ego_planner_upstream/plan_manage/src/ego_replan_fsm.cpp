#include <ego_planner/ego_replan_fsm.h>

namespace ego_planner
{

void EGOReplanFSM::init(rclcpp::Node::SharedPtr & node)
{
  node_ = node;

  current_wp_ = 0;
  exec_state_ = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_ = false;
  have_recv_pre_agent_ = false;

  node_->declare_parameter("fsm/flight_type", -1);
  node_->declare_parameter("fsm/thresh_replan_time", -1.0);
  node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
  node_->declare_parameter("fsm/planning_horizon", -1.0);
  node_->declare_parameter("fsm/planning_horizen_time", -1.0);
  node_->declare_parameter("fsm/emergency_time", 1.0);
  node_->declare_parameter("fsm/realworld_experiment", false);
  node_->declare_parameter("fsm/fail_safe", true);
  node_->declare_parameter("fsm/shadow_mode", false);
  node_->declare_parameter("fsm/validation_flat_mode", true);
  node_->declare_parameter("fsm/validation_flight_height", 0.78);
  node_->declare_parameter("fsm/validation_sample_spacing", 0.04);
  node_->declare_parameter("fsm/validation_map_timeout_sec", 1.0);
  node_->declare_parameter("fsm/replan_start_position_error_m", 0.10);
  node_->declare_parameter("fsm/replan_candidate_trials", 3);
  node_->declare_parameter("fsm/validation_failure_retry_limit", 3);
  node_->declare_parameter("fsm/ego_recovery/enable", false);
  node_->declare_parameter("fsm/ego_recovery/diagnostic_only", true);
  node_->declare_parameter("fsm/ego_recovery/max_recovery_attempts", 2);
  node_->declare_parameter("fsm/ego_recovery/cooldown_sec", 2.0);
  node_->declare_parameter("fsm/ego_recovery/exhausted_backoff_sec", 2.0);
  node_->declare_parameter("fsm/ego_recovery/rejoin_search_distance_m", 1.20);
  node_->declare_parameter("fsm/ego_recovery/minimum_forward_progress_m", 0.15);
  node_->declare_parameter("fsm/ego_recovery/extra_clearance_m", 0.06);
  node_->declare_parameter("fsm/ego_recovery/max_reference_deviation_m", 0.65);
  // Default 0.0 keeps the forward-only behaviour for launch files that predate
  // the lateral fallback; the shipped tuning YAML always sets both explicitly.
  node_->declare_parameter("fsm/ego_recovery/max_lateral_offset_m", 0.0);
  node_->declare_parameter("fsm/ego_recovery/lateral_offset_step_m", 0.0);
  node_->declare_parameter("fsm/constrained_clearance/enable", false);
  node_->declare_parameter("fsm/constrained_clearance/clearance_m", 0.25);
  node_->declare_parameter("fsm/constrained_clearance/optimization_m", 0.30);
  node_->declare_parameter("fsm/handover_position_tolerance_m", 0.01);
  node_->declare_parameter("fsm/handover_velocity_tolerance_mps", 0.005);
  node_->declare_parameter("fsm/handover_acceleration_tolerance_mps2", 0.01);
  node_->declare_parameter("fsm/active_preplan_lookahead_m", 0.65);
  node_->declare_parameter(
    "fsm/ego_recovery/reference_path_topic", "/race/ego/local_path_reference");
  node_->declare_parameter("ego_diagnostics.enable", false);
  node_->declare_parameter("min_x", -7.5);
  node_->declare_parameter("max_x", 7.5);
  node_->declare_parameter("min_y", -7.0);
  node_->declare_parameter("max_y", 7.0);
  node_->declare_parameter("min_height", 0.50);
  node_->declare_parameter("max_height", 0.85);

  node_->get_parameter("fsm/flight_type", target_type_);
  node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
  node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
  node_->get_parameter("fsm/planning_horizon", planning_horizen_);
  node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
  node_->get_parameter("fsm/emergency_time", emergency_time_);
  node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
  node_->get_parameter("fsm/fail_safe", enable_fail_safe_);
  node_->get_parameter("fsm/shadow_mode", shadow_mode_);
  node_->get_parameter("fsm/validation_flat_mode", validation_flat_mode_);
  node_->get_parameter("fsm/validation_flight_height", validation_flight_height_);
  node_->get_parameter("fsm/validation_sample_spacing", validation_sample_spacing_);
  node_->get_parameter("fsm/validation_map_timeout_sec", validation_map_timeout_sec_);
  node_->get_parameter("fsm/replan_start_position_error_m", replan_start_position_error_);
  node_->get_parameter("fsm/replan_candidate_trials", replan_candidate_trials_);
  node_->get_parameter("fsm/validation_failure_retry_limit", validation_failure_retry_limit_);
  node_->get_parameter("fsm/ego_recovery/enable", recovery_enabled_);
  node_->get_parameter("fsm/ego_recovery/diagnostic_only", recovery_diagnostic_only_);
  node_->get_parameter("fsm/ego_recovery/max_recovery_attempts", recovery_max_attempts_);
  node_->get_parameter("fsm/ego_recovery/cooldown_sec", recovery_cooldown_sec_);
  node_->get_parameter(
    "fsm/ego_recovery/exhausted_backoff_sec", recovery_exhausted_backoff_sec_);
  node_->get_parameter(
    "fsm/ego_recovery/rejoin_search_distance_m",
    recovery_rejoin_search_distance_m_);
  node_->get_parameter(
    "fsm/ego_recovery/minimum_forward_progress_m",
    recovery_minimum_forward_progress_m_);
  node_->get_parameter(
    "fsm/ego_recovery/extra_clearance_m", recovery_extra_clearance_m_);
  node_->get_parameter(
    "fsm/ego_recovery/max_reference_deviation_m",
    recovery_max_reference_deviation_m_);
  node_->get_parameter(
    "fsm/ego_recovery/max_lateral_offset_m", recovery_max_lateral_offset_m_);
  node_->get_parameter(
    "fsm/ego_recovery/lateral_offset_step_m", recovery_lateral_offset_step_m_);
  node_->get_parameter("fsm/constrained_clearance/enable", constrained_clearance_enabled_);
  node_->get_parameter("fsm/constrained_clearance/clearance_m", constrained_clearance_m_);
  node_->get_parameter(
    "fsm/constrained_clearance/optimization_m", constrained_optimization_clearance_m_);
  node_->get_parameter("fsm/handover_position_tolerance_m", handover_position_tolerance_m_);
  node_->get_parameter("fsm/handover_velocity_tolerance_mps", handover_velocity_tolerance_mps_);
  node_->get_parameter(
    "fsm/handover_acceleration_tolerance_mps2", handover_acceleration_tolerance_mps2_);
  node_->get_parameter("fsm/active_preplan_lookahead_m", active_preplan_lookahead_m_);
  std::string reference_path_topic;
  node_->get_parameter("fsm/ego_recovery/reference_path_topic", reference_path_topic);
  node_->get_parameter("ego_diagnostics.enable", ego_diagnostics_enabled_);
  node_->get_parameter("min_x", shared_bounds_x_min_);
  node_->get_parameter("max_x", shared_bounds_x_max_);
  node_->get_parameter("min_y", shared_bounds_y_min_);
  node_->get_parameter("max_y", shared_bounds_y_max_);
  node_->get_parameter("min_height", shared_bounds_z_min_);
  node_->get_parameter("max_height", shared_bounds_z_max_);
  configured_validation_flight_height_ = validation_flight_height_;
  configured_shared_bounds_z_min_ = shared_bounds_z_min_;
  configured_shared_bounds_z_max_ = shared_bounds_z_max_;
  if (shared_bounds_x_min_ >= shared_bounds_x_max_ ||
    shared_bounds_y_min_ >= shared_bounds_y_max_ ||
    shared_bounds_z_min_ >= shared_bounds_z_max_)
  {
    throw std::runtime_error("SHARED_BOUNDARY_MISMATCH: invalid EGO shared bounds");
  }
  replan_candidate_trials_ = std::max(1, replan_candidate_trials_);
  validation_failure_retry_limit_ = std::max(1, validation_failure_retry_limit_);
  handover_position_tolerance_m_ = std::clamp(
    std::isfinite(handover_position_tolerance_m_) ? handover_position_tolerance_m_ : 0.01,
    0.001, 0.02);
  handover_velocity_tolerance_mps_ = std::clamp(
    std::isfinite(handover_velocity_tolerance_mps_) ? handover_velocity_tolerance_mps_ : 0.005,
    0.001, 0.02);
  handover_acceleration_tolerance_mps2_ = std::clamp(
    std::isfinite(handover_acceleration_tolerance_mps2_) ?
    handover_acceleration_tolerance_mps2_ : 0.01,
    0.005, 0.05);
  active_preplan_lookahead_m_ = std::clamp(
    std::isfinite(active_preplan_lookahead_m_) ? active_preplan_lookahead_m_ : 0.65,
    0.30, 1.50);
  recovery_max_attempts_ = std::max(1, recovery_max_attempts_);
  recovery_cooldown_sec_ = std::max(0.0, recovery_cooldown_sec_);
  recovery_exhausted_backoff_sec_ = std::max(
    recovery_cooldown_sec_, recovery_exhausted_backoff_sec_);
  recovery_rejoin_search_distance_m_ = std::max(
    0.0, std::isfinite(recovery_rejoin_search_distance_m_) ?
    recovery_rejoin_search_distance_m_ : 0.0);
  recovery_minimum_forward_progress_m_ = std::clamp(
    std::isfinite(recovery_minimum_forward_progress_m_) ?
    recovery_minimum_forward_progress_m_ : 0.15,
    0.05, 0.40);
  recovery_extra_clearance_m_ = std::clamp(
    std::isfinite(recovery_extra_clearance_m_) ? recovery_extra_clearance_m_ : 0.06,
    0.0, 0.10);
  recovery_max_reference_deviation_m_ = std::clamp(
    std::isfinite(recovery_max_reference_deviation_m_) ?
    recovery_max_reference_deviation_m_ : 0.65,
    0.45, 1.00);
  if (recovery_minimum_forward_progress_m_ > recovery_rejoin_search_distance_m_) {
    throw std::runtime_error(
      "fsm/ego_recovery/minimum_forward_progress_m must be <= rejoin_search_distance_m");
  }
  recovery_max_lateral_offset_m_ = std::max(
    0.0, std::isfinite(recovery_max_lateral_offset_m_) ?
    recovery_max_lateral_offset_m_ : 0.0);
  recovery_lateral_offset_step_m_ = std::max(
    0.0, std::isfinite(recovery_lateral_offset_step_m_) ?
    recovery_lateral_offset_step_m_ : 0.0);
  // A lateral rejoin point still has to survive the reference-deviation gate in
  // build_recovery_seed, so searching further sideways than that gate allows
  // would only produce candidates that are rejected later.
  if (recovery_max_lateral_offset_m_ > recovery_max_reference_deviation_m_) {
    throw std::runtime_error(
      "fsm/ego_recovery/max_lateral_offset_m must be <= max_reference_deviation_m");
  }
  // Both must be set together: a ladder needs a step to enumerate rungs, and a
  // step without a bound would search sideways without limit.
  if ((recovery_max_lateral_offset_m_ > 0.0) !=
    (recovery_lateral_offset_step_m_ > 0.0))
  {
    throw std::runtime_error(
      "fsm/ego_recovery/max_lateral_offset_m and lateral_offset_step_m must both "
      "be positive to enable the lateral rejoin search, or both be zero");
  }
  if (recovery_lateral_offset_step_m_ > recovery_max_lateral_offset_m_) {
    throw std::runtime_error(
      "fsm/ego_recovery/lateral_offset_step_m must be <= max_lateral_offset_m");
  }
  recovery_failure_policy_.configure(
    recovery_max_attempts_, recovery_cooldown_sec_, recovery_exhausted_backoff_sec_);
  RCLCPP_INFO(
    node_->get_logger(),
    "[EGO_RECOVERY_GEOMETRY_CONFIG] search_distance=%.3f minimum_forward_progress=%.3f "
    "extra_clearance=%.3f max_reference_deviation=%.3f max_lateral_offset=%.3f "
    "lateral_offset_step=%.3f",
    recovery_rejoin_search_distance_m_, recovery_minimum_forward_progress_m_,
    recovery_extra_clearance_m_, recovery_max_reference_deviation_m_,
    recovery_max_lateral_offset_m_, recovery_lateral_offset_step_m_);

  have_trigger_ = !flag_realworld_experiment_;

  node_->declare_parameter("fsm/waypoint_num", -1);
  node_->get_parameter("fsm/waypoint_num", waypoint_num_);

  for (int i = 0; i < waypoint_num_; i++) {
    node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
    node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
    node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

    node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
    node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
    node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
  }

  /* initialize main modules */
  visualization_.reset(new PlanningVisualization(node_));

  planner_manager_.reset(new EGOPlannerManager);

  planner_manager_->initPlanModules(node_, visualization_);

  constexpr double physical_envelope_radius_m = 0.207132;
  const double normal_planning_clearance =
    planner_manager_->grid_map_->getPlanningCenterClearance();
  if (!std::isfinite(constrained_clearance_m_) ||
    constrained_clearance_m_ < physical_envelope_radius_m ||
    constrained_clearance_m_ >= normal_planning_clearance ||
    constrained_clearance_m_ >= planner_manager_->grid_map_->getRequiredCenterClearance() ||
    !std::isfinite(constrained_optimization_clearance_m_) ||
    constrained_optimization_clearance_m_ < constrained_clearance_m_ ||
    constrained_optimization_clearance_m_ >= normal_planning_clearance)
  {
    throw std::runtime_error(
      "fsm/constrained_clearance values must be finite, optimization_m >= clearance_m, "
      "and both below normal grid_map/planning_inflation");
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "[EGO_CONSTRAINED_CLEARANCE_CONFIG] enabled=%s normal=%.3f "
    "constrained=%.3f optimization=%.3f",
    constrained_clearance_enabled_ ? "true" : "false",
    normal_planning_clearance, constrained_clearance_m_, constrained_optimization_clearance_m_);

  planner_manager_->deliverTrajToOptimizer();   // store trajectories
  planner_manager_->setDroneIdtoOpt();

  /* callback*/
  exec_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&EGOReplanFSM::execFSMCallback, this));

  safety_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&EGOReplanFSM::checkCollisionCallback, this));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom_world",
    1,
    [this](const std::shared_ptr<const nav_msgs::msg::Odometry> & msg)
    {
      this->odometryCallback(msg);
    });
  reference_path_sub_ = node_->create_subscription<race_msgs::msg::LocalPathReference>(
    reference_path_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](const std::shared_ptr<const race_msgs::msg::LocalPathReference> & msg)
    {
      this->referencePathCallback(msg);
    });
  altitude_reference_sub_ =
    node_->create_subscription<race_msgs::msg::FlightAltitudeReference>(
    "/race/flight_altitude_reference", rclcpp::QoS(1).reliable().transient_local(),
    [this](const std::shared_ptr<const race_msgs::msg::FlightAltitudeReference> & msg)
    {
      this->altitudeReferenceCallback(msg);
    });
  // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

  if (planner_manager_->pp_.drone_id >= 1) {
    string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) +
      string("_planning/swarm_trajs");
    swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
      sub_topic_name,
      10,
      [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines> & msg)
      {
        this->swarmTrajsCallback(msg);
      });
  }

  // ros2 中topic名字中不能出现负号，单机id是-1需要处理
  // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
  string pub_topic_name;
  if (planner_manager_->pp_.drone_id <= -1) {
    RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
    pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
  } else {
    pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string(
      "_planning/swarm_trajs");
  }

  swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);

  broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>(
    "planning/broadcast_bspline_from_planner", 10);
  broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
    "planning/broadcast_bspline_to_planner",
    100,
    [this](const std::shared_ptr<const traj_utils::msg::Bspline> & msg)
    {
      this->BroadcastBsplineCallback(msg);
    });

  bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
  safety_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/race/ego/planner_safety_status", rclcpp::QoS(1).reliable().transient_local());
  replan_ready_sub_ = node_->create_subscription<std_msgs::msg::UInt64>(
    "/race/ego/replan_ready", rclcpp::QoS(1).reliable().transient_local(),
    [this](const std::shared_ptr<const std_msgs::msg::UInt64> & msg)
    {
      this->replanReadyCallback(msg);
    });
  data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal",
      1,
      [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> & msg)
      {
        this->waypointCallback(msg);
      });
  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/traj_start_trigger",
      1,
      [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> & msg)
      {
        this->triggerCallback(msg);
      });

    RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
    int count = 0;
    while (rclcpp::ok() && count++ < 1000) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

    while (rclcpp::ok() && (!have_odom_ || !have_trigger_)) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    readGivenWps();
  } else {
    cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }
}

void EGOReplanFSM::readGivenWps()
{
  if (waypoint_num_ <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
    return;
  }

  wps_.resize(waypoint_num_);
  for (int i = 0; i < waypoint_num_; i++) {
    wps_[i](0) = waypoints_[i][0];
    wps_[i](1) = waypoints_[i][1];
    wps_[i](2) = waypoints_[i][2];
  }

  // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
  for (size_t i = 0; i < (size_t)waypoint_num_; i++) {
    visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // plan first global waypoint
  wp_id_ = 0;
  planNextWaypoint(wps_[wp_id_]);
}

void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
{
  Eigen::Vector3d planning_start = odom_pos_;
  Eigen::Vector3d planning_start_vel = odom_vel_;
  Eigen::Vector3d planning_goal = next_wp;
  // Flat mode is a two-dimensional planner at a fixed flight altitude.  The
  // vehicle odometry is necessarily still close to the ground while takeoff
  // is in progress; passing that raw Z state into the optimizer creates a
  // vertical boundary condition that contradicts every later flat-map check.
  if (validation_flat_mode_) {
    planning_start.z() = validation_flight_height_;
    planning_start_vel.z() = 0.0;
    planning_goal.z() = validation_flight_height_;
  }

  bool success = false;
  success = planner_manager_->planGlobalTraj(
    planning_start, planning_start_vel,
    Eigen::Vector3d::Zero(), planning_goal, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  if (success) {
    end_pt_ = planning_goal;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++) {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    pending_validation_failure_.clear();
    last_validation_failure_.clear();
    consecutive_validation_failures_ = 0;
    ++local_goal_seq_;
    next_replan_retry_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    recovery_cooldown_waiting_ = false;
    recovery_rejoin_failed_ = false;
    recovery_failure_policy_.resetForNewGoal(local_goal_seq_);

    /*** FSM状态转换 ***/
    if (exec_state_ == WAIT_TARGET) {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    } else if (exec_state_ == EMERGENCY_STOP) {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    } else {
      changeFSMExecState(REPLAN_TRAJ, "TRIG");
    }

    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
  }
}

void EGOReplanFSM::triggerCallback(
  const std::shared_ptr<const geometry_msgs::msg::PoseStamped> & msg)
{
  have_trigger_ = true;
  cout << "Triggered!" << endl;
  init_pt_ = odom_pos_;
}

void EGOReplanFSM::altitudeReferenceCallback(
  const std::shared_ptr<const race_msgs::msg::FlightAltitudeReference> & msg)
{
  if (!msg->valid) {
    if (altitude_reference_valid_ && msg->flight_id == altitude_reference_flight_id_) {
      altitude_reference_valid_ = false;
      have_target_ = false;
      have_latest_reference_path_ = false;
      have_active_reference_path_ = false;
      publishSafetyStatus("WAIT_ALTITUDE_REFERENCE");
    }
    return;
  }
  if (!std::isfinite(msg->ground_z_map) || !std::isfinite(msg->target_z_map) ||
    !std::isfinite(msg->target_agl_m) || !std::isfinite(msg->min_agl_m) ||
    !std::isfinite(msg->max_agl_m) ||
    std::abs(msg->target_agl_m - configured_validation_flight_height_) > 1.0e-3 ||
    std::abs(msg->min_agl_m - configured_shared_bounds_z_min_) > 1.0e-3 ||
    std::abs(msg->max_agl_m - configured_shared_bounds_z_max_) > 1.0e-3 ||
    std::abs((msg->target_z_map - msg->ground_z_map) - msg->target_agl_m) > 1.0e-3)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "[EGO_ALTITUDE_REFERENCE_REJECT] flight_id=%lu",
      static_cast<unsigned long>(msg->flight_id));
    return;
  }
  if (altitude_reference_valid_ && msg->flight_id < altitude_reference_flight_id_) {
    return;
  }
  altitude_reference_flight_id_ = msg->flight_id;
  validation_flight_height_ = msg->target_z_map;
  shared_bounds_z_min_ = msg->ground_z_map + configured_shared_bounds_z_min_;
  shared_bounds_z_max_ = msg->ground_z_map + configured_shared_bounds_z_max_;
  altitude_reference_valid_ = true;
  RCLCPP_WARN(
    node_->get_logger(),
    "[EGO_ALTITUDE_REFERENCE_ACCEPTED] flight_id=%lu ground_map=%.3f target_map=%.3f "
    "bounds_z=[%.3f,%.3f]",
    static_cast<unsigned long>(msg->flight_id), msg->ground_z_map,
    validation_flight_height_, shared_bounds_z_min_, shared_bounds_z_max_);
}

void EGOReplanFSM::waypointCallback(
  const std::shared_ptr<const geometry_msgs::msg::PoseStamped> & msg)
{
  if (validation_flat_mode_ && !altitude_reference_valid_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_GOAL_REJECT] no valid /race/flight_altitude_reference for this flight");
    return;
  }
  if (msg->pose.position.z < -0.1) {
    return;
  }

  cout << "Triggered!" << endl;

  last_local_goal_receive_time_ = node_->now();

  Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  if (end_wp.x() < shared_bounds_x_min_ || end_wp.x() > shared_bounds_x_max_ ||
    end_wp.y() < shared_bounds_y_min_ || end_wp.y() > shared_bounds_y_max_ ||
    end_wp.z() < shared_bounds_z_min_ || end_wp.z() > shared_bounds_z_max_)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "OUT_OF_GEOFENCE EGO goal=(%.2f,%.2f,%.2f) shared_bounds=x=[%.2f,%.2f] y=[%.2f,%.2f] z=[%.2f,%.2f]",
      end_wp.x(), end_wp.y(), end_wp.z(), shared_bounds_x_min_, shared_bounds_x_max_,
      shared_bounds_y_min_, shared_bounds_y_max_, shared_bounds_z_min_, shared_bounds_z_max_);
    publishSafetyStatus("OUT_OF_GEOFENCE");
    return;
  }
  if (validation_flat_mode_) {
    end_wp.z() = validation_flight_height_;
  }

  const bool had_active_reference_path = have_active_reference_path_;
  const uint64_t previous_global_path_id = active_reference_path_.global_path_id;
  const uint64_t previous_reference_goal_seq = active_reference_path_.local_goal_seq;
  const bool latest_reference_matches_goal = have_latest_reference_path_ &&
    (latest_reference_path_.local_goal - end_wp).norm() <= 0.05;
  const bool duplicate_reference_transaction = had_active_reference_path &&
    latest_reference_matches_goal &&
    latest_reference_path_.global_path_id == previous_global_path_id &&
    latest_reference_path_.local_goal_seq == previous_reference_goal_seq;
  if (duplicate_reference_transaction) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[EGO_DUPLICATE_LOCAL_GOAL_IGNORED] global_path_id=%lu local_goal_seq=%lu",
      static_cast<unsigned long>(latest_reference_path_.global_path_id),
      static_cast<unsigned long>(latest_reference_path_.local_goal_seq));
    return;
  }

  init_pt_ = odom_pos_;
  const bool new_global_path = have_active_reference_path_ &&
    have_latest_reference_path_ &&
    latest_reference_path_.global_path_id != active_reference_path_.global_path_id;
  force_new_global_path_session_ = new_global_path;
  have_active_reference_path_ = false;
  if (latest_reference_matches_goal)
  {
    active_reference_path_ = latest_reference_path_;
    // Super publishes the reference from its path projection.  During the
    // first airborne handover the measured vehicle can be slightly off that
    // projection.  Bind the measured start through the same safe-map segment
    // instead of falling back to the old start->goal chord.
    const auto projection = RecoveryPathPolicy::projectToPolyline(
      active_reference_path_.points, odom_pos_);
    if (std::isfinite(projection.distance) &&
      projection.distance > std::max(0.01, validation_sample_spacing_))
    {
      const double distance = (active_reference_path_.points.front() - odom_pos_).norm();
      const std::size_t samples = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(
          distance / std::max(0.01, validation_sample_spacing_))));
      bool connector_safe = true;
      for (std::size_t i = 0; i <= samples; ++i) {
        Eigen::Vector3d point = odom_pos_ +
          (active_reference_path_.points.front() - odom_pos_) *
          (static_cast<double>(i) / samples);
        if (validation_flat_mode_) {
          point.z() = validation_flight_height_;
        }
        Eigen::Vector3d nearest;
        double clearance = std::numeric_limits<double>::infinity();
        const int occupancy = point.allFinite() ?
          planner_manager_->grid_map_->queryContinuousOccupancy(
            point, nearest, clearance) : -1;
        if (occupancy != 0) {
          connector_safe = false;
          break;
        }
      }
      if (connector_safe) {
        Eigen::Vector3d connector_start = odom_pos_;
        if (validation_flat_mode_) {
          connector_start.z() = validation_flight_height_;
        }
        active_reference_path_.points.insert(
          active_reference_path_.points.begin(), connector_start);
      }
    }
    have_active_reference_path_ = active_reference_path_.points.size() >= 2;
    if (have_active_reference_path_ && active_reference_path_.local_goal_seq > 0) {
      // planNextWaypoint increments once for the accepted target.
      local_goal_seq_ = active_reference_path_.local_goal_seq - 1;
    }
    const auto start_projection = RecoveryPathPolicy::projectToPolyline(
      active_reference_path_.points, odom_pos_);
    const auto goal_projection = RecoveryPathPolicy::projectToPolyline(
      active_reference_path_.points, end_wp);
    RCLCPP_INFO(
      node_->get_logger(),
      "[EGO_REFERENCE_PATH] global_path_id=%lu local_goal_seq=%lu points=%zu "
      "arc_length=%.3f start_projection=%.3f local_goal_projection=%.3f "
      "minimum_clearance=%.3f bound=%s",
      static_cast<unsigned long>(active_reference_path_.global_path_id),
      static_cast<unsigned long>(active_reference_path_.local_goal_seq),
      active_reference_path_.points.size(), active_reference_path_.arc_length,
      start_projection.distance, goal_projection.distance,
      active_reference_path_.minimum_clearance,
      have_active_reference_path_ ? "true" : "false");
    if (force_new_global_path_session_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[EGO_NEW_GLOBAL_PATH_SESSION] old_global_path_id=%lu new_global_path_id=%lu "
        "start=current_odom preempt_old_trajectory=true",
        static_cast<unsigned long>(previous_global_path_id),
        static_cast<unsigned long>(latest_reference_path_.global_path_id));
    }
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_REFERENCE_PATH_REJECT] reason=missing_or_goal_mismatch "
      "goal=(%.3f,%.3f,%.3f)",
      end_wp.x(), end_wp.y(), end_wp.z());
  }

  const auto & orientation = msg->pose.orientation;
  const double quaternion_norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (std::isfinite(quaternion_norm) && quaternion_norm > 1e-6) {
    const double normalized_x = orientation.x / quaternion_norm;
    const double normalized_y = orientation.y / quaternion_norm;
    const double normalized_z = orientation.z / quaternion_norm;
    const double normalized_w = orientation.w / quaternion_norm;
    const double yaw = std::atan2(
      2.0 * (normalized_w * normalized_z + normalized_x * normalized_y),
      1.0 - 2.0 * (normalized_y * normalized_y + normalized_z * normalized_z));
    local_goal_heading_ = Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
    have_local_goal_heading_ = local_goal_heading_.allFinite();
  } else {
    have_local_goal_heading_ = false;
  }

  planNextWaypoint(end_wp);
}

void EGOReplanFSM::referencePathCallback(
  const std::shared_ptr<const race_msgs::msg::LocalPathReference> & msg)
{
  if (validation_flat_mode_ && !altitude_reference_valid_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_REFERENCE_PATH_REJECT] reason=missing_altitude_reference");
    return;
  }
  if (msg->header.frame_id != "map" || msg->points.size() < 2 ||
    !std::isfinite(msg->arc_length) || msg->arc_length <= 0.0)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_REFERENCE_PATH_REJECT] reason=invalid_header_or_size "
      "frame=%s points=%zu",
      msg->header.frame_id.c_str(), msg->points.size());
    return;
  }

  LocalPathReference reference;
  reference.global_path_id = msg->global_path_id;
  reference.local_goal_seq = msg->local_goal_seq;
  const Eigen::Vector3d received_local_goal(
    msg->local_goal.x, msg->local_goal.y, msg->local_goal.z);
  reference.arc_length = msg->arc_length;
  reference.minimum_clearance = msg->minimum_clearance;
  reference.received_at = node_->now();
  reference.points.reserve(msg->points.size());
  for (const auto & message_point : msg->points) {
    Eigen::Vector3d point(message_point.x, message_point.y, message_point.z);
    if (!point.allFinite() ||
      point.x() < shared_bounds_x_min_ || point.x() > shared_bounds_x_max_ ||
      point.y() < shared_bounds_y_min_ || point.y() > shared_bounds_y_max_ ||
      point.z() < shared_bounds_z_min_ || point.z() > shared_bounds_z_max_)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[EGO_REFERENCE_PATH_REJECT] reason=invalid_point global_path_id=%lu "
        "local_goal_seq=%lu",
        static_cast<unsigned long>(msg->global_path_id),
        static_cast<unsigned long>(msg->local_goal_seq));
      return;
    }
    if (validation_flat_mode_) {
      point.z() = validation_flight_height_;
    }
    reference.points.push_back(point);
  }
  reference.continuation_points.reserve(msg->continuation_points.size());
  for (const auto & message_point : msg->continuation_points) {
    Eigen::Vector3d point(message_point.x, message_point.y, message_point.z);
    if (!point.allFinite() ||
      point.x() < shared_bounds_x_min_ || point.x() > shared_bounds_x_max_ ||
      point.y() < shared_bounds_y_min_ || point.y() > shared_bounds_y_max_ ||
      point.z() < shared_bounds_z_min_ || point.z() > shared_bounds_z_max_)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[EGO_REFERENCE_PATH_REJECT] reason=invalid_continuation_point "
        "global_path_id=%lu local_goal_seq=%lu",
        static_cast<unsigned long>(msg->global_path_id),
        static_cast<unsigned long>(msg->local_goal_seq));
      return;
    }
    if (validation_flat_mode_) {
      point.z() = validation_flight_height_;
    }
    reference.continuation_points.push_back(point);
  }
  reference.local_goal = received_local_goal;
  if (!reference.local_goal.allFinite() ||
    reference.local_goal.x() < shared_bounds_x_min_ ||
    reference.local_goal.x() > shared_bounds_x_max_ ||
    reference.local_goal.y() < shared_bounds_y_min_ ||
    reference.local_goal.y() > shared_bounds_y_max_ ||
    reference.local_goal.z() < shared_bounds_z_min_ ||
    reference.local_goal.z() > shared_bounds_z_max_ ||
    (validation_flat_mode_ &&
    (validation_flight_height_ < shared_bounds_z_min_ ||
    validation_flight_height_ > shared_bounds_z_max_)) ||
    (reference.points.back() - reference.local_goal).norm() > 0.05)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_REFERENCE_PATH_REJECT] reason=endpoint_mismatch global_path_id=%lu "
      "local_goal_seq=%lu error=%.3f",
      static_cast<unsigned long>(msg->global_path_id),
      static_cast<unsigned long>(msg->local_goal_seq),
      (reference.points.back() - reference.local_goal).norm());
    return;
  }
  if (!reference.continuation_points.empty() &&
    (reference.continuation_points.front() - reference.local_goal).norm() > 0.05)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_REFERENCE_PATH_REJECT] reason=continuation_start_mismatch "
      "global_path_id=%lu local_goal_seq=%lu error=%.3f",
      static_cast<unsigned long>(msg->global_path_id),
      static_cast<unsigned long>(msg->local_goal_seq),
      (reference.continuation_points.front() - reference.local_goal).norm());
    return;
  }
  if (validation_flat_mode_) {
    reference.local_goal.z() = validation_flight_height_;
  }

  latest_reference_path_ = std::move(reference);
  have_latest_reference_path_ = true;
}

void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> & msg)
{
  last_odom_receive_time_ = node_->now();
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  // odom_acc_ = estimateAcc( msg );

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void EGOReplanFSM::BroadcastBsplineCallback(
  const std::shared_ptr<const traj_utils::msg::Bspline> & msg)
{
  size_t id = msg->drone_id;
  if ((int)id == planner_manager_->pp_.drone_id) {
    return;
  }

  const auto msg_time = rclcpp::Time(
    msg->start_time, node_->get_clock()->get_clock_type());
  if (abs((node_->now() - msg_time).seconds()) > 0.25) {
    // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
    RCLCPP_ERROR(
      node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
      msg->drone_id, (node_->now() - msg_time).seconds());
    return;
  }

  // 路径缓冲区初始化
  if (planner_manager_->swarm_trajs_buf_.size() <= id) {
    for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++) {
      OneTrajDataOfSwarm blank;
      blank.drone_id = -1;
      planner_manager_->swarm_trajs_buf_.push_back(blank);
    }
  }

  /* Test distance to the agent */
  Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
  Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
  Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
  Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
  if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f) {
    planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
    return;   // if the current drone is too far to the received agent.
  }

  /* Store data */
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t j = 0; j < msg->knots.size(); ++j) {
    knots(j) = msg->knots[j];
  }
  for (size_t j = 0; j < msg->pos_pts.size(); ++j) {
    pos_pts(0, j) = msg->pos_pts[j].x;
    pos_pts(1, j) = msg->pos_pts[j].y;
    pos_pts(2, j) = msg->pos_pts[j].z;
  }

  planner_manager_->swarm_trajs_buf_[id].drone_id = id;

  // 计算路径持续时间
  if (msg->order % 2) {
    double cutback = (double)msg->order / 2 + 1.5;
    planner_manager_->swarm_trajs_buf_[id].duration_ =
      msg->knots[msg->knots.size() - ceil(cutback)];
  } else {
    double cutback = (double)msg->order / 2 + 1.5;
    planner_manager_->swarm_trajs_buf_[id].duration_ =
      (msg->knots[msg->knots.size() - floor(cutback)] +
      msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
  }

  // 生成bspline并存储
  UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
  pos_traj.setKnot(knots);
  planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

  planner_manager_->swarm_trajs_buf_[id].start_pos_ =
    planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

  planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

  /* Check Collision */
  if (planner_manager_->checkCollision(id)) {
    changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
  }
}

void EGOReplanFSM::swarmTrajsCallback(
  const std::shared_ptr<const traj_utils::msg::MultiBsplines> & msg)
{

  multi_bspline_msgs_buf_.traj.clear();
  multi_bspline_msgs_buf_ = *msg;

  if (!have_odom_) {
    RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
    return;
  }

  if ((int)msg->traj.size() != msg->drone_id_from + 1) { // drone_id must start from 0
    RCLCPP_ERROR(
      node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d",
      (int)msg->traj.size(), msg->drone_id_from + 1);
    return;
  }

  if (msg->traj[0].order != 3) { // only support B-spline order equals 3.
    RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
    return;
  }

  // Step 1. receive the trajectories
  planner_manager_->swarm_trajs_buf_.clear();
  planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

  // 处理每条路径
  for (size_t i = 0; i < msg->traj.size(); i++) {

    Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y,
      msg->traj[i].pos_pts[0].z);
    Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y,
      msg->traj[i].pos_pts[1].z);
    Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y,
      msg->traj[i].pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f) {
      planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
      continue;
    }

    // 存储路径控制点和节点
    Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
    Eigen::VectorXd knots(msg->traj[i].knots.size());
    for (size_t j = 0; j < msg->traj[i].knots.size(); ++j) {
      knots(j) = msg->traj[i].knots[j];
    }
    for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j) {
      pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
      pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
      pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[i].drone_id = i;

    // 计算路径持续时间
    if (msg->traj[i].order % 2) {
      double cutback = (double)msg->traj[i].order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[i].duration_ =
        msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
    } else {
      double cutback = (double)msg->traj[i].order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[i].duration_ =
        (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] +
        msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
    }

    // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
    UniformBspline pos_traj(pos_pts, msg->traj[i].order,
      msg->traj[i].knots[1] - msg->traj[i].knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[i].start_pos_ =
      planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
  }

  have_recv_pre_agent_ = true;
}

void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{

  if (new_state == exec_state_) {
    continously_called_times_++;
  } else {
    continously_called_times_ = 1;
  }

  static string state_str[8] =
  {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP",
    "SEQUENTIAL_START"};
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] <<
    endl;
}

std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
{
  return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
}

void EGOReplanFSM::printFSMExecState()
{
  static string state_str[8] =
  {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP",
    "SEQUENTIAL_START"};

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void EGOReplanFSM::execFSMCallback()
{
  exec_timer_->cancel();   // To avoid blockage

  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) {
      cout << "no odom." << endl;
    }
    if (!have_target_) {
      cout << "wait for goal or trigger." << endl;
    }
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT:
      {
        if (!have_odom_) {
          goto force_return;
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
      }

    case WAIT_TARGET:
      {
        if (!have_target_ || !have_trigger_) {
          goto force_return;
        } else {
          changeFSMExecState(SEQUENTIAL_START, "FSM");
        }
        break;
      }

    case SEQUENTIAL_START: // for swarm
      {
        if (planner_manager_->pp_.drone_id <= 0 ||
          (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
        {
          if (have_odom_ && have_target_ && have_trigger_) {
            bool success = planFromGlobalTraj(10); // zx-todo
            if (success) {
              changeFSMExecState(EXEC_TRAJ, "FSM");

              publishSwarmTrajs(true);
            } else {
              RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
              publishPendingValidationFailure(false);
              changeFSMExecState(SEQUENTIAL_START, "FSM");
            }
          } else {
            RCLCPP_ERROR(
              node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_,
              have_target_);
          }
        }

        break;
      }

    case GEN_NEW_TRAJ:
      {
        if (next_replan_retry_time_.nanoseconds() != 0 &&
          node_->now() < next_replan_retry_time_)
        {
          goto force_return;
        }

        bool success = planFromGlobalTraj(10); // zx-todo
        if (success) {
          next_replan_retry_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
          changeFSMExecState(EXEC_TRAJ, "FSM");
          flag_escape_emergency_ = true;
          publishSwarmTrajs(false);
        } else {
          publishPendingValidationFailure(false);
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
      }

    case REPLAN_TRAJ:
      {
        if (next_replan_retry_time_.nanoseconds() != 0 &&
          node_->now() < next_replan_retry_time_)
        {
          goto force_return;
        }
        if (awaiting_bridge_replan_ready_) {
          goto force_return;
        }
        if (shadow_mode_ && (end_pt_ - odom_pos_).norm() <= no_replan_thresh_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "[EGO_SHADOW_LOCAL_GOAL_REACHED] distance=%.3f threshold=%.3f; wait for next goal",
            (end_pt_ - odom_pos_).norm(), no_replan_thresh_);
          have_target_ = false;
          have_new_target_ = false;
          pending_validation_failure_.clear();
          changeFSMExecState(WAIT_TARGET, "SHADOW");
          break;
        }

        const bool success = bridge_replan_ready_received_ ?
          planFromGlobalTraj(replan_candidate_trials_) :
          planFromCurrentTraj(replan_candidate_trials_);
        bridge_replan_ready_received_ = false;
        if (success) {
          next_replan_retry_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
          changeFSMExecState(EXEC_TRAJ, "FSM");
          publishSwarmTrajs(false);
        } else {
          publishPendingValidationFailure(false);
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }

        break;
      }

    case EXEC_TRAJ:
      {
        /* determine if need to replan */
        LocalTrajData * info = &planner_manager_->local_data_;
        rclcpp::Time time_now = node_->now();
        double t_cur = (time_now - info->start_time_).seconds();
        t_cur = std::min(info->duration_, t_cur);

        Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

        /* && (end_pt_ - pos).norm() < 0.5 */
        if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
        {
          wp_id_++;
          planNextWaypoint(wps_[wp_id_]);
        } else if ((local_target_pt_ - end_pt_).norm() < 1e-3) { // close to the global target
          if (t_cur > info->duration_ - 1e-2) {
            if ((end_pt_ - odom_pos_).norm() <= no_replan_thresh_) {
              have_target_ = false;
              have_trigger_ = false;

              if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
                wp_id_ = 0;
                planNextWaypoint(wps_[wp_id_]);
              }

              changeFSMExecState(WAIT_TARGET, "FSM");
            } else {
              changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            goto force_return;
          } else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_) {
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        } else if (t_cur > replan_thresh_) {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }

        break;
      }

    case EMERGENCY_STOP:
      {

        if (flag_escape_emergency_) { // Avoiding repeated calls
          callEmergencyStop(odom_pos_);
        } else {
          // Do not resume planning while the cloud is still missing: without a
          // fresh map every candidate would be checked against stale occupancy.
          // The latch clears on its own once the cloud recovers.
          if (enable_fail_safe_ && !depth_loss_emergency_latched_ &&
            odom_vel_.norm() < 0.1)
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          }
        }

        flag_escape_emergency_ = false;
        break;
      }
  }

  data_disp_.header.stamp = node_->now();
  data_disp_pub_->publish(data_disp_);

force_return:;
  // exec_timer_.start();
  if (exec_timer_ && exec_timer_->is_canceled()) {
    // 取消状态下无需重新创建，可以复用现有计时器
    exec_timer_->reset();
  }
}

bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/)   // zx-todo
{
  pending_validation_failure_.clear();
  start_pt_ = odom_pos_;
  start_vel_ = odom_vel_;
  start_acc_.setZero();
  if (validation_flat_mode_) {
    start_pt_.z() = validation_flight_height_;
    start_vel_.z() = 0.0;
  }

  bool flag_random_poly_init;
  if (timesOfConsecutiveStateCalls().first == 1) {
    flag_random_poly_init = false;
  } else {
    flag_random_poly_init = true;
  }

  for (int i = 0; i < trial_times; i++) {
    if (callReboundReplan(true, flag_random_poly_init)) {
      return true;
    }
    // Optimizer initialization failures may benefit from another seed in the
    // same cycle. A candidate that reached the final publish gates and was
    // rejected is deterministic for this state/reference pair; repeating it
    // here only creates a high-rate rejection storm. Recovery candidates are
    // still evaluated once below, then the newest goal is retried by the FSM.
    if (!pending_validation_failure_.empty()) {
      break;
    }
  }
  return planWithRecovery("global_trajectory");
}

bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
{
  if (awaiting_bridge_replan_ready_) {
    return false;
  }
  if (force_new_global_path_session_) {
    return planFromGlobalTraj(trial_times);
  }
  pending_validation_failure_.clear();

  LocalTrajData * info = &planner_manager_->local_data_;
  // ros::Time time_now = ros::Time::now();
  const auto time_now = node_->now();
  // double t_cur = (time_now - info->start_time_).toSec();
  double t_cur = (time_now - info->start_time_).seconds();
  t_cur = std::clamp(t_cur, 0.0, info->duration_);

  start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
  start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
  start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

  const double horizontal_start_error =
    (start_pt_.head<2>() - odom_pos_.head<2>()).norm();
  const double effective_reanchor_threshold =
    ReplanHandoverPolicy::effectiveOdomReanchorThreshold(
    replan_start_position_error_, odom_vel_.head<2>().norm());
  const bool previous_trajectory_active = ReplanHandoverPolicy::isTrajectoryActiveAt(
    info->start_time_.seconds(), info->duration_, time_now.seconds());
  const bool tracking_diverged = ReplanHandoverPolicy::requiresOdomReanchor(
    horizontal_start_error, effective_reanchor_threshold);
  if (ReplanHandoverPolicy::requiresMeasuredOdomStart(
      info->start_time_.seconds(), info->duration_, time_now.seconds(),
      horizontal_start_error, effective_reanchor_threshold)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_REPLAN_ODOM_DIVERGENCE] predicted_start=(%.3f,%.3f,%.3f) "
      "odom_start=(%.3f,%.3f,%.3f) horizontal_error=%.3f "
      "base_threshold=%.3f effective_threshold=%.3f horizontal_speed=%.3f "
      "reason=%s",
      start_pt_.x(), start_pt_.y(), start_pt_.z(), odom_pos_.x(), odom_pos_.y(),
      odom_pos_.z(), horizontal_start_error, replan_start_position_error_,
      effective_reanchor_threshold, odom_vel_.head<2>().norm(),
      previous_trajectory_active && tracking_diverged ?
      "TRACKING_DIVERGENCE" : "TRAJECTORY_EXPIRED");

    // The old trajectory no longer represents the vehicle state. Continuing
    // its P/V/A splice would command a return to that virtual state and can
    // create a growing orbit. Plan a fresh local session from measured odom;
    // callReboundReplan's preemption path bypasses the obsolete continuity
    // check and commits only after the normal trajectory validation gates.
    force_new_global_path_session_ = true;
    if (have_active_reference_path_ && active_reference_path_.points.size() >= 2) {
      Eigen::Vector3d reanchor_reference_start = odom_pos_;
      if (validation_flat_mode_) {
        reanchor_reference_start.z() = validation_flight_height_;
      }
      const ReferencePrefixTrimResult trim = ReplanHandoverPolicy::trimReferencePrefix(
        active_reference_path_.points, reanchor_reference_start);
      if (trim.valid) {
        const Eigen::Vector3d connector_end = trim.points.front();
        const std::size_t samples = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(
            (connector_end - reanchor_reference_start).norm() /
            std::max(0.01, validation_sample_spacing_))));
        bool connector_safe = true;
        for (std::size_t index = 0; index <= samples; ++index) {
          const Eigen::Vector3d point = reanchor_reference_start +
            (connector_end - reanchor_reference_start) *
            (static_cast<double>(index) / samples);
          Eigen::Vector3d nearest;
          double clearance = std::numeric_limits<double>::infinity();
          if (planner_manager_->grid_map_->queryContinuousOccupancy(
              point, nearest, clearance) != 0) {
            connector_safe = false;
            break;
          }
        }
        if (connector_safe) {
          // Drop the flown prefix before adding the measured connector.  Just
          // prepending to the complete historical reference leaves a loop in
          // the seed after several replans, which the B-spline parameterizer
          // correctly rejects as a self-intersection.
          active_reference_path_.points = trim.points;
          if ((active_reference_path_.points.front() - reanchor_reference_start).norm() >
            1.0e-4)
          {
            active_reference_path_.points.insert(
              active_reference_path_.points.begin(), reanchor_reference_start);
          }
        }
        RCLCPP_INFO(
          node_->get_logger(),
          "[EGO_REPLAN_REANCHOR_REFERENCE] start=(%.3f,%.3f,%.3f) points=%zu "
          "trimmed_prefix=%.3f connector_safe=%s",
          reanchor_reference_start.x(), reanchor_reference_start.y(),
          reanchor_reference_start.z(), active_reference_path_.points.size(),
          trim.discarded_length, connector_safe ? "true" : "false");
      } else {
        RCLCPP_ERROR(
          node_->get_logger(),
          "[EGO_REPLAN_REANCHOR_REFERENCE_REJECT] reason=invalid_reference "
          "points=%zu",
          active_reference_path_.points.size());
      }
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_REPLAN_REANCHOR] source=odom horizontal_error=%.3f "
      "base_threshold=%.3f effective_threshold=%.3f "
      "start=(%.3f,%.3f,%.3f) velocity=(%.3f,%.3f,%.3f) reason=%s",
      horizontal_start_error, replan_start_position_error_, effective_reanchor_threshold,
      odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), odom_vel_.x(), odom_vel_.y(),
      odom_vel_.z(), previous_trajectory_active ?
      "TRACKING_DIVERGENCE" : "TRAJECTORY_EXPIRED");
    const bool reanchored = planFromGlobalTraj(trial_times);
    if (!reanchored) {
      // Recovery may have found the old trajectory clear on the map, but it
      // starts from a state that has already diverged from measured odometry.
      // Never allow that stale trajectory to remain an output source.
      const bool old_trajectory_was_preserved = candidate_failure_preserved_active_;
      candidate_failure_preserved_active_ =
        ReplanHandoverPolicy::mayPreserveActiveTrajectoryAfterFailure(
          true, old_trajectory_was_preserved);
      // The stale trajectory cannot safely be resumed after this measured
      // position divergence. Ask the bridge to hold the measured position
      // while recovery retries, rather than permanently latching navigation.
      pending_validation_failure_ = "EGO_REPLAN_REANCHOR_RETRY";
      recovery_cooldown_waiting_ = false;
      recovery_rejoin_failed_ = true;
      RCLCPP_WARN(
        node_->get_logger(),
        "[EGO_REPLAN_REANCHOR_RETRY] active_trajectory_id=%d "
        "horizontal_error=%.3f odom=(%.3f,%.3f,%.3f) "
        "old_predicted_start=(%.3f,%.3f,%.3f)",
        info->traj_id_, horizontal_start_error, odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
        start_pt_.x(), start_pt_.y(), start_pt_.z());
      publishSafetyStatus(pending_validation_failure_);
    }
    return reanchored;
  }

  // Small tracking errors use the normal P/V/A splice to avoid unnecessary
  // discontinuities. Large errors took the re-anchor path above.
  bool success = callReboundReplan(false, false);

  if (!success) {
    if (planner_manager_->lastReplanFailureReason() ==
      EGOPlannerManager::ReplanFailureReason::HANDOVER_CONTINUITY ||
      pending_validation_failure_ == "EGO_HANDOVER_CONTINUITY_REJECT")
    {
      return false;
    }
    if (!pending_validation_failure_.empty()) {
      return planWithRecovery("current_trajectory");
    }
    success = callReboundReplan(true, false);
    if (!success) {
      if (planner_manager_->lastReplanFailureReason() ==
        EGOPlannerManager::ReplanFailureReason::HANDOVER_CONTINUITY ||
        pending_validation_failure_ == "EGO_HANDOVER_CONTINUITY_REJECT")
      {
        return false;
      }
      if (!pending_validation_failure_.empty()) {
        return planWithRecovery("current_trajectory");
      }
      for (int i = 0; i < trial_times; i++) {
        success = callReboundReplan(true, true);
        if (success) {
          break;
        }
        if (planner_manager_->lastReplanFailureReason() ==
          EGOPlannerManager::ReplanFailureReason::HANDOVER_CONTINUITY ||
          pending_validation_failure_ == "EGO_HANDOVER_CONTINUITY_REJECT")
        {
          return false;
        }
        if (!pending_validation_failure_.empty()) {
          break;
        }
      }
      if (!success) {
        return planWithRecovery("current_trajectory");
      }
    }
  }

  return true;
}

std::optional<EGOReplanFSM::RecoveryCandidate> EGOReplanFSM::generateRecoveryCandidate() const
{
  Eigen::Vector2d heading = local_goal_heading_;
  if (!have_local_goal_heading_ || !heading.allFinite() || heading.norm() < 1e-6) {
    heading = (end_pt_ - odom_pos_).head<2>();
  }
  if (!heading.allFinite() || heading.norm() < 1e-6) {
    return std::nullopt;
  }

  auto inside_shared_bounds = [this](const Eigen::Vector3d & point) {
      return point.x() >= shared_bounds_x_min_ && point.x() <= shared_bounds_x_max_ &&
             point.y() >= shared_bounds_y_min_ && point.y() <= shared_bounds_y_max_ &&
             point.z() >= shared_bounds_z_min_ && point.z() <= shared_bounds_z_max_;
    };
  auto is_free = [this](const Eigen::Vector3d & query) {
      Eigen::Vector3d point = query;
      if (validation_flat_mode_) {
        point.z() = validation_flight_height_;
      }
      const double recovery_clearance =
        planner_manager_->grid_map_->getPlanningCenterClearance() +
        recovery_extra_clearance_m_;
      return planner_manager_->grid_map_->getInflateOccupancy(
        point, recovery_clearance) == 0;
    };
  Eigen::Vector3d local_goal = end_pt_;
  if (validation_flat_mode_) {
    local_goal.z() = validation_flight_height_;
  }
  constexpr std::size_t required_consecutive_free = 3;
  const double sample_spacing = RecoveryPathPolicy::stableSampleSpacing(
    validation_sample_spacing_, planner_manager_->grid_map_->getResolution(),
    required_consecutive_free);
  const bool have_continuation = have_active_reference_path_ &&
    active_reference_path_.continuation_points.size() >= 2;
  auto rejoin = have_continuation ?
    RecoveryPathPolicy::findForwardRejoinOnPolyline(
      active_reference_path_.continuation_points,
      recovery_rejoin_search_distance_m_, sample_spacing,
      required_consecutive_free, is_free, inside_shared_bounds) :
    RecoveryPathPolicy::findForwardRejoin(
      local_goal, heading, recovery_rejoin_search_distance_m_, sample_spacing,
      required_consecutive_free,
      false,  // Compatibility fallback for an older reference publisher.
      is_free, inside_shared_bounds);

  // Staying on the reference line is preferred, but an obstacle sitting on that
  // line blocks it for its own width plus clearance on both sides -- 1.3m for a
  // 0.5m box at 0.40m recovery clearance.  A forward-only search bounded by
  // rejoin_search_distance_m then has no solution at all, which is what
  // produced 213 consecutive EGO_REJOIN_NOT_FOUND entries in the 2026-08-05 C
  // area log while the vehicle sat still for 140s.  Fall back to searching on
  // lines parallel to the reference: free space beside the obstacle is reachable
  // at a far shorter forward distance.  This picks a reachable target only; the
  // local A* still searches the route and decides which side to pass on.
  double rejoin_lateral_offset = 0.0;
  bool rejoin_used_lateral = false;
  if (!rejoin.found && have_continuation &&
    recovery_lateral_offset_step_m_ > 0.0 && recovery_max_lateral_offset_m_ > 0.0)
  {
    const auto lateral = RecoveryPathPolicy::findLateralRejoinOnPolyline(
      active_reference_path_.continuation_points,
      recovery_rejoin_search_distance_m_, sample_spacing,
      required_consecutive_free,
      RecoveryPathPolicy::lateralOffsetLadder(
        recovery_max_lateral_offset_m_, recovery_lateral_offset_step_m_),
      is_free, inside_shared_bounds);
    if (lateral.found) {
      rejoin.found = true;
      rejoin.point = lateral.point;
      rejoin.forward_distance = lateral.forward_distance;
      rejoin_lateral_offset = lateral.lateral_offset;
      rejoin_used_lateral = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "[EGO_REJOIN_LATERAL] lateral_offset=%.3f forward_distance=%.3f "
        "target=(%.3f,%.3f,%.3f) max_offset=%.3f step=%.3f",
        lateral.lateral_offset, lateral.forward_distance, lateral.point.x(),
        lateral.point.y(), lateral.point.z(), recovery_max_lateral_offset_m_,
        recovery_lateral_offset_step_m_);
    }
  }

  if (!rejoin.found) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_REJOIN_NOT_FOUND] local_goal=(%.3f,%.3f,%.3f) "
      "heading=(%.3f,%.3f) search_distance=%.3f sample_spacing=%.3f source=%s "
      "lateral_attempted=%s max_lateral_offset=%.3f",
      local_goal.x(), local_goal.y(), local_goal.z(), heading.x(), heading.y(),
      recovery_rejoin_search_distance_m_, sample_spacing,
      have_continuation ? "REFERENCE_POLYLINE" : "YAW_FALLBACK",
      (have_continuation && recovery_lateral_offset_step_m_ > 0.0 &&
      recovery_max_lateral_offset_m_ > 0.0) ? "true" : "false",
      recovery_max_lateral_offset_m_);
    return std::nullopt;
  }

  // A recovery candidate that does not move forward only causes another
  // identical replan cycle.  Reject it before invoking the optimizer so a
  // failed recovery cannot turn into an in-place command.
  const Eigen::Vector2d heading_unit = heading.normalized();
  // A lateral rejoin point is off the reference line, so its polyline arc length
  // no longer measures progress away from the vehicle.  Project onto the heading
  // instead, the same way the no-continuation fallback does.
  const double forward_progress = (have_continuation && !rejoin_used_lateral) ?
    rejoin.forward_distance :
    (rejoin.point - odom_pos_).head<2>().dot(heading_unit);
  if (!std::isfinite(forward_progress) ||
    forward_progress < recovery_minimum_forward_progress_m_)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_REJOIN_REJECTED_NO_PROGRESS] forward_progress=%.3f "
      "required=%.3f target=(%.3f,%.3f,%.3f)",
      forward_progress, recovery_minimum_forward_progress_m_,
      rejoin.point.x(), rejoin.point.y(), rejoin.point.z());
    return std::nullopt;
  }

  const std::string type =
    rejoin.forward_distance <= 1.0e-6 ? "LOCAL_ASTAR" : "LOCAL_ASTAR_REJOIN";
  RCLCPP_INFO(
    node_->get_logger(),
    "[EGO_REJOIN_SELECTED] type=%s local_goal=(%.3f,%.3f,%.3f) "
    "forward_distance=%.3f forward_progress=%.3f target=(%.3f,%.3f,%.3f) "
    "consecutive_free=%zu required_clearance=%.3f source=%s lateral_offset=%.3f",
    type.c_str(), local_goal.x(), local_goal.y(), local_goal.z(),
    rejoin.forward_distance, forward_progress,
    rejoin.point.x(), rejoin.point.y(), rejoin.point.z(),
    required_consecutive_free,
    planner_manager_->grid_map_->getPlanningCenterClearance() +
    recovery_extra_clearance_m_,
    rejoin_used_lateral ? "REFERENCE_POLYLINE_LATERAL" :
    (have_continuation ? "REFERENCE_POLYLINE" : "YAW_FALLBACK"),
    rejoin_lateral_offset);
  return RecoveryCandidate{
    type, rejoin.point, rejoin.forward_distance, rejoin_lateral_offset};
}

std::vector<EGOReplanFSM::RecoveryCandidate>
EGOReplanFSM::generateRecoveryCandidates() const
{
  std::vector<RecoveryCandidate> candidates;

  // Preferred: stay on the reference line.  generateRecoveryCandidate already
  // falls back to a lateral point when no on-line point exists at all.
  if (const auto primary = generateRecoveryCandidate()) {
    candidates.push_back(*primary);
  }

  // A found rejoin point is not the same as a usable one: the local A* still has
  // to route to it, and buildLocalAStarSeed rejects the route when any segment
  // fails its clearance check.  The 2026-08-06 C-area log deadlocked exactly
  // here -- the on-line point was found on every one of 1271 attempts, so the
  // lateral fallback above never triggered, and 140 consecutive
  // continuous_segment_collision rejections produced no motion for 559s.  Queue
  // the lateral points as alternatives so an unroutable primary can be replaced
  // instead of ending the whole recovery.
  const bool have_continuation = have_active_reference_path_ &&
    active_reference_path_.continuation_points.size() >= 2;
  if (!have_continuation || recovery_lateral_offset_step_m_ <= 0.0 ||
    recovery_max_lateral_offset_m_ <= 0.0)
  {
    return candidates;
  }

  auto inside_shared_bounds = [this](const Eigen::Vector3d & point) {
      return point.x() >= shared_bounds_x_min_ && point.x() <= shared_bounds_x_max_ &&
             point.y() >= shared_bounds_y_min_ && point.y() <= shared_bounds_y_max_ &&
             point.z() >= shared_bounds_z_min_ && point.z() <= shared_bounds_z_max_;
    };
  auto is_free = [this](const Eigen::Vector3d & query) {
      Eigen::Vector3d point = query;
      if (validation_flat_mode_) {
        point.z() = validation_flight_height_;
      }
      return planner_manager_->grid_map_->getInflateOccupancy(
        point,
        planner_manager_->grid_map_->getPlanningCenterClearance() +
        recovery_extra_clearance_m_) == 0;
    };

  constexpr std::size_t required_consecutive_free = 3;
  const double sample_spacing = RecoveryPathPolicy::stableSampleSpacing(
    validation_sample_spacing_, planner_manager_->grid_map_->getResolution(),
    required_consecutive_free);
  const auto laterals = RecoveryPathPolicy::findAllLateralRejoinsOnPolyline(
    active_reference_path_.continuation_points,
    recovery_rejoin_search_distance_m_, sample_spacing,
    required_consecutive_free,
    RecoveryPathPolicy::lateralOffsetLadder(
      recovery_max_lateral_offset_m_, recovery_lateral_offset_step_m_),
    is_free, inside_shared_bounds);

  const Eigen::Vector2d heading_unit =
    (have_local_goal_heading_ && local_goal_heading_.allFinite() &&
    local_goal_heading_.norm() > 1e-6) ?
    local_goal_heading_.normalized() :
    Eigen::Vector2d((end_pt_ - odom_pos_).head<2>().normalized());

  for (const auto & lateral : laterals) {
    // Off-line points cannot use polyline arc length as progress; project onto
    // the heading exactly as the primary path does.
    const double forward_progress =
      (lateral.point - odom_pos_).head<2>().dot(heading_unit);
    if (!std::isfinite(forward_progress) ||
      forward_progress < recovery_minimum_forward_progress_m_)
    {
      continue;
    }
    // Skip one that is effectively the primary candidate again.
    if (!candidates.empty() &&
      (candidates.front().target - lateral.point).norm() < 1.0e-3)
    {
      continue;
    }
    candidates.push_back(
      RecoveryCandidate{
        "LOCAL_ASTAR_REJOIN", lateral.point, lateral.forward_distance,
        lateral.lateral_offset});
    if (candidates.size() >= recovery_max_candidates_) {
      break;
    }
  }
  return candidates;
}

void EGOReplanFSM::logRecoveryCandidateOnFailure(const char * planning_context)
{
  if (!recovery_enabled_ || !recovery_diagnostic_only_) {
    return;
  }

  constexpr double diagnostic_period_sec = 1.0;
  if (last_recovery_diagnostic_time_.nanoseconds() != 0 &&
    (node_->now() - last_recovery_diagnostic_time_).seconds() < diagnostic_period_sec)
  {
    return;
  }
  last_recovery_diagnostic_time_ = node_->now();

  const auto candidate = generateRecoveryCandidate();
  RCLCPP_WARN(
    node_->get_logger(),
    "[EGO_RECOVERY_DIAGNOSTIC] context=%s current_position=(%.3f,%.3f,%.3f) "
    "current_velocity=(%.3f,%.3f,%.3f) local_goal=(%.3f,%.3f,%.3f) "
    "rejoin_found=%s diagnostic_only=true",
    planning_context, odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), odom_vel_.x(),
    odom_vel_.y(), odom_vel_.z(), end_pt_.x(), end_pt_.y(), end_pt_.z(),
    candidate.has_value() ? "true" : "false");
  if (candidate.has_value()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_RECOVERY_DIAGNOSTIC] type=%s forward_distance=%.3f "
      "target=(%.3f,%.3f,%.3f)",
      candidate->type.c_str(), candidate->forward_distance_m, candidate->target.x(),
      candidate->target.y(), candidate->target.z());
  }
}

bool EGOReplanFSM::planWithRecovery(const char * planning_context)
{
  recovery_cooldown_waiting_ = false;
  recovery_rejoin_failed_ = false;
  if (!recovery_enabled_) {
    return false;
  }
  if (recovery_diagnostic_only_) {
    logRecoveryCandidateOnFailure(planning_context);
    return false;
  }

  const double recovery_now_sec = node_->now().seconds();
  const auto gate = recovery_failure_policy_.gate(recovery_now_sec);
  if (gate == RecoveryGateResult::EXHAUSTED_BACKOFF_WAIT) {
    double active_remaining = 0.0;
    double active_clearance = std::numeric_limits<double>::infinity();
    Eigen::Vector3d active_endpoint;
    candidate_failure_preserved_active_ = activeTrajectoryRemainingSafe(
      planner_manager_->local_data_, active_remaining, active_endpoint, active_clearance);
    recovery_rejoin_failed_ = !candidate_failure_preserved_active_;
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_RECOVERY_FAILED] context=%s reason=max_attempts attempts=%d "
      "rejoin_found=false active_safe=%s active_remaining_time=%.3f",
      planning_context, recovery_failure_policy_.attemptsForGoal(),
      candidate_failure_preserved_active_ ? "true" : "false", active_remaining);
    // Keep optimizer load bounded, but do not wait forever for a rolling goal
    // that itself depends on vehicle progress. Retry the same goal after a
    // longer backoff; a newly received goal still resets the policy immediately.
    const double remaining = recovery_failure_policy_.exhaustedBackoffRemaining(
      recovery_now_sec);
    scheduleReplanRetry("RECOVERY_EXHAUSTED_BACKOFF", remaining);
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_RECOVERY_EXHAUSTED_BACKOFF] goal_seq=%lu attempts=%d "
      "remaining_sec=%.3f active_safe=%s retry_scheduled=true",
      static_cast<unsigned long>(local_goal_seq_),
      recovery_failure_policy_.attemptsForGoal(),
      remaining,
      candidate_failure_preserved_active_ ? "true" : "false");
    return false;
  }
  if (gate == RecoveryGateResult::COOLDOWN_WAIT) {
    recovery_cooldown_waiting_ = true;
    const double remaining = recovery_failure_policy_.cooldownRemaining(recovery_now_sec);
    scheduleReplanRetry("RECOVERY_COOLDOWN", remaining);
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_RECOVERY_COOLDOWN_WAIT] goal_seq=%lu remaining_sec=%.3f retry_scheduled=true",
      static_cast<unsigned long>(local_goal_seq_),
      remaining);
    return false;
  }

  // Same goal, same obstacle layout, already proven unsolvable: rerunning the
  // local A* and a full B-spline optimization can only reach the same verdict.
  // Wait for the map to change instead of spinning the optimizer.
  const uint64_t map_revision = planner_manager_->grid_map_->getMapRevision();
  if (recovery_failure_policy_.layoutKnownUnsolvable(map_revision)) {
    recovery_cooldown_waiting_ = true;
    scheduleReplanRetry("RECOVERY_LAYOUT_UNCHANGED", recovery_cooldown_sec_);
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[EGO_RECOVERY_LAYOUT_UNCHANGED] goal_seq=%lu map_revision=%lu "
      "skipping_reoptimization=true retry_scheduled=true",
      static_cast<unsigned long>(local_goal_seq_),
      static_cast<unsigned long>(map_revision));
    return false;
  }

  auto candidates = generateRecoveryCandidates();
  recovery_failure_policy_.markAttempt(recovery_now_sec);
  RCLCPP_INFO(
    node_->get_logger(),
    "[EGO_RECOVERY_ATTEMPT] goal_seq=%lu rejoin_found=%s candidates=%zu result=STARTED",
    static_cast<unsigned long>(local_goal_seq_),
    candidates.empty() ? "false" : "true", candidates.size());
  // Try each candidate in turn.  A rejoin point that the local A* cannot route to
  // must not end the recovery while other reachable points remain untried.
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    auto & candidate = candidates[index];
    RCLCPP_INFO(
      node_->get_logger(),
      "[EGO_RECOVERY_ATTEMPT] context=%s attempt=%d candidate=%zu/%zu type=%s "
      "forward_distance=%.3f lateral_offset=%.3f target=(%.3f,%.3f,%.3f)",
      planning_context, recovery_failure_policy_.attemptsForGoal(),
      index + 1, candidates.size(), candidate.type.c_str(),
      candidate.forward_distance_m, candidate.lateral_offset_m,
      candidate.target.x(), candidate.target.y(), candidate.target.z());
    // callReboundReplan publishes only after its existing EGO occupancy and
    // feasibility validation succeeds. Failed candidates publish nothing.
    if (callReboundReplan(true, false, &candidate.target, &candidate.type)) {
      const auto * info = &planner_manager_->local_data_;
      RCLCPP_INFO(
        node_->get_logger(),
        "[EGO_RECOVERY_SUCCESS] original_goal=(%.3f,%.3f,%.3f) "
        "selected_candidate=%s candidate_index=%zu/%zu forward_distance=%.3f "
        "lateral_offset=%.3f trajectory_id=%d clearance=%.3f",
        end_pt_.x(), end_pt_.y(), end_pt_.z(), candidate.type.c_str(),
        index + 1, candidates.size(), candidate.forward_distance_m,
        candidate.lateral_offset_m, info->traj_id_, recoveryTrajectoryClearance());
      RCLCPP_INFO(
        node_->get_logger(),
        "[EGO_RECOVERY_ATTEMPT] goal_seq=%lu rejoin_found=true result=SUCCESS",
        static_cast<unsigned long>(local_goal_seq_));
      return true;
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_RECOVERY_CANDIDATE_UNROUTABLE] candidate=%zu/%zu type=%s "
      "lateral_offset=%.3f target=(%.3f,%.3f,%.3f)",
      index + 1, candidates.size(), candidate.type.c_str(),
      candidate.lateral_offset_m, candidate.target.x(), candidate.target.y(),
      candidate.target.z());
  }

  // Every candidate for this layout failed.  Record it so the next tick waits for
  // a map update instead of repeating the identical search and optimization.
  recovery_failure_policy_.markLayoutUnsolvable(map_revision);
  recovery_rejoin_failed_ = true;
  {
    double active_remaining = 0.0;
    double active_clearance = std::numeric_limits<double>::infinity();
    Eigen::Vector3d active_endpoint;
    candidate_failure_preserved_active_ = activeTrajectoryRemainingSafe(
      planner_manager_->local_data_, active_remaining, active_endpoint, active_clearance);
    recovery_rejoin_failed_ = !candidate_failure_preserved_active_;
  }
  scheduleReplanRetry(
    "RECOVERY_REJOIN_REJECTED",
    std::max(0.10, recovery_failure_policy_.cooldownRemaining(node_->now().seconds())));
  RCLCPP_ERROR(
    node_->get_logger(),
    "[EGO_RECOVERY_FAILED] context=%s attempts=%d rejoin_found=%s "
    "candidates_tried=%zu",
    planning_context, recovery_failure_policy_.attemptsForGoal(),
    candidates.empty() ? "false" : "true", candidates.size());
  RCLCPP_ERROR(
    node_->get_logger(),
    "[EGO_RECOVERY_ATTEMPT] goal_seq=%lu rejoin_found=%s candidates_tried=%zu "
    "result=REJOIN_REJECTED",
    static_cast<unsigned long>(local_goal_seq_),
    candidates.empty() ? "false" : "true", candidates.size());
  return false;
}

double EGOReplanFSM::recoveryTrajectoryClearance()
{
  auto * info = &planner_manager_->local_data_;
  const auto map = planner_manager_->grid_map_;
  if (info->duration_ <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  constexpr double sample_period_sec = 0.01;
  double minimum_clearance = std::numeric_limits<double>::infinity();
  for (double time = 0.0; time < info->duration_ + sample_period_sec; time += sample_period_sec) {
    Eigen::Vector3d point = info->position_traj_.evaluateDeBoorT(std::min(time, info->duration_));
    if (validation_flat_mode_) {
      point.z() = validation_flight_height_;
    }
    Eigen::Vector3d nearest;
    double clearance = std::numeric_limits<double>::infinity();
    map->findNearestInflatedObstacle(point, nearest, clearance);
    minimum_clearance = std::min(minimum_clearance, clearance);
  }
  return minimum_clearance;
}

void EGOReplanFSM::logActiveTrajectoryRecheck(const double fsm_timer_gap_ms)
{
  if (!ego_diagnostics_enabled_ || !have_active_trajectory_diagnostic_data_ ||
    !active_trajectory_map_binding_.bound())
  {
    return;
  }

  const auto map = planner_manager_->grid_map_;
  const uint64_t revision = map->getMapRevision();
  if (!active_trajectory_map_binding_.shouldRecheck(revision)) {
    return;
  }

  auto & trajectory = active_trajectory_diagnostic_data_;
  const double now_t = (node_->now() - trajectory.start_time_).seconds();
  const double start_t = std::max(0.0, now_t);
  bool collision_free = true;
  Eigen::Vector3d first_collision = Eigen::Vector3d::Constant(
    std::numeric_limits<double>::quiet_NaN());
  double minimum_clearance = std::numeric_limits<double>::infinity();
  size_t sample_count = 0;
  const auto recheck_start = std::chrono::steady_clock::now();
  const double active_clearance =
    std::isfinite(trajectory.required_clearance_) && trajectory.required_clearance_ > 0.0 ?
    trajectory.required_clearance_ : map->getPlanningCenterClearance();

  constexpr double sample_period_sec = 0.01;
  for (double t = start_t; t <= trajectory.duration_ + sample_period_sec; t += sample_period_sec) {
    const double sample_t = std::min(t, trajectory.duration_);
    Eigen::Vector3d point = trajectory.position_traj_.evaluateDeBoorT(sample_t);
    if (validation_flat_mode_) {
      point.z() = validation_flight_height_;
    }
    ++sample_count;
    Eigen::Vector3d nearest;
    double clearance = std::numeric_limits<double>::infinity();
    map->findNearestInflatedObstacle(point, nearest, clearance);
    minimum_clearance = std::min(minimum_clearance, clearance);
    if (point.allFinite() && map->getInflateOccupancy(point, active_clearance) != 0) {
      collision_free = false;
      first_collision = point;
      break;
    }
    if (sample_t >= trajectory.duration_) {
      break;
    }
  }

  const double active_recheck_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - recheck_start).count();
  const double odom_age_ms = last_odom_receive_time_.nanoseconds() == 0 ?
    std::numeric_limits<double>::infinity() :
    (node_->now() - last_odom_receive_time_).seconds() * 1000.0;
  const double local_goal_age_ms = last_local_goal_receive_time_.nanoseconds() == 0 ?
    std::numeric_limits<double>::infinity() :
    (node_->now() - last_local_goal_receive_time_).seconds() * 1000.0;

  RCLCPP_INFO(
    node_->get_logger(),
    "[EGO_ACTIVE_TRAJECTORY_RECHECK] trajectory_id=%d published_map_revision=%lu "
    "current_map_revision=%lu collision_free=%s first_collision_position=(%.3f,%.3f,%.3f) "
    "minimum_clearance=%.3f",
    active_trajectory_map_binding_.trajectoryId(),
    static_cast<unsigned long>(active_trajectory_map_binding_.publishedMapRevision()),
    static_cast<unsigned long>(revision), collision_free ? "true" : "false",
    first_collision.x(), first_collision.y(), first_collision.z(), minimum_clearance);
  RCLCPP_INFO(
    node_->get_logger(),
    "[EGO_DIAGNOSTIC_TIMING] map_revision=%lu map_update_ms=%.3f active_recheck_ms=%.3f "
    "sample_count=%zu fsm_timer_gap_ms=%.3f odom_age_ms=%.3f local_goal_age_ms=%.3f",
    static_cast<unsigned long>(revision), map->getLastMapUpdateMs(), active_recheck_ms,
    sample_count, fsm_timer_gap_ms, odom_age_ms, local_goal_age_ms);
  active_trajectory_map_binding_.markRechecked(revision);
}

void EGOReplanFSM::logCandidateFailureContext()
{
  if (!ego_diagnostics_enabled_) {
    return;
  }
  const auto map = planner_manager_->grid_map_;
  Eigen::Vector3d current_actual = odom_pos_;
  if (validation_flat_mode_) {
    current_actual.z() = validation_flight_height_;
  }
  Eigen::Vector3d current_predicted = Eigen::Vector3d::Constant(
    std::numeric_limits<double>::quiet_NaN());
  double tracking_error_xy = std::numeric_limits<double>::quiet_NaN();
  if (have_active_trajectory_diagnostic_data_) {
    auto & trajectory = active_trajectory_diagnostic_data_;
    const double current_t = std::clamp(
      (node_->now() - trajectory.start_time_).seconds(), 0.0, trajectory.duration_);
    current_predicted = trajectory.position_traj_.evaluateDeBoorT(current_t);
    if (validation_flat_mode_) {
      current_predicted.z() = validation_flight_height_;
    }
    tracking_error_xy = (current_actual.head<2>() - current_predicted.head<2>()).norm();
  }

  const int raw_occupancy = current_actual.allFinite() ? map->getOccupancy(current_actual) : -1;
  const int inflated_occupancy =
    current_actual.allFinite() ? map->getInflateOccupancy(current_actual) : -1;
  RCLCPP_WARN(
    node_->get_logger(),
    "[EGO_CANDIDATE_FAILURE_CONTEXT] candidate_collision=true active_trajectory_exists=%s "
    "active_trajectory_collision_free=%s current_actual_position=(%.3f,%.3f,%.3f) "
    "current_predicted_position=(%.3f,%.3f,%.3f) tracking_error_xy=%.3f "
    "current_raw_occupancy=%d current_inflated_occupancy=%d",
    have_active_trajectory_diagnostic_data_ ? "true" : "false",
    active_trajectory_map_binding_.bound() ? "true" : "false", current_actual.x(),
    current_actual.y(), current_actual.z(), current_predicted.x(), current_predicted.y(),
    current_predicted.z(), tracking_error_xy, raw_occupancy, inflated_occupancy);
}

void EGOReplanFSM::logFailureSemantics(const std::string & downstream_status)
{
  if (!ego_diagnostics_enabled_ || downstream_status == "EGO_TRAJECTORY_VALID") {
    return;
  }

  const auto map = planner_manager_->grid_map_;
  Eigen::Vector3d current_position = odom_pos_;
  if (validation_flat_mode_) {
    current_position.z() = validation_flight_height_;
  }
  const bool current_position_occupied = current_position.allFinite() &&
    map->getInflateOccupancy(current_position) != 0;
  const bool candidate_failed = !pending_validation_failure_.empty();
  RCLCPP_INFO(
    node_->get_logger(),
    "[EGO_FAILURE_SEMANTICS] candidate_failed=%s active_trajectory_invalidated=false "
    "current_position_occupied=%s downstream_status=%s",
    candidate_failed ? "true" : "false", current_position_occupied ? "true" : "false",
    downstream_status.c_str());
}

void EGOReplanFSM::checkCollisionCallback()
{

  LocalTrajData * info = &planner_manager_->local_data_;
  auto map = planner_manager_->grid_map_;

  if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5) {
    return;
  }

  // Read-only diagnostic: it is deliberately outside all FSM/safety state
  // transitions and is disabled by default.
  if (ego_diagnostics_enabled_) {
    const rclcpp::Time now = node_->now();
    const double fsm_timer_gap_ms = last_diagnostic_fsm_time_.nanoseconds() == 0 ?
      0.0 : (now - last_diagnostic_fsm_time_).seconds() * 1000.0;
    last_diagnostic_fsm_time_ = now;
    logActiveTrajectoryRecheck(fsm_timer_gap_ms);
  }

  /* ---------- check lost of depth ---------- */
  // A cloud dropout is a transient sensor fault, not a permanent loss of the
  // fail-safe capability.  Clearing enable_fail_safe_ here used to be
  // irreversible -- nothing restored it -- so a single dropout left
  // EMERGENCY_STOP with no exit (its only escape at the EMERGENCY_STOP case
  // tests enable_fail_safe_) and the planner never replanned again even after
  // the cloud came back.  Track the dropout in its own latch instead and leave
  // the configured fail-safe switch alone.
  if (map->getOdomDepthTimeout()) {
    if (!depth_loss_emergency_latched_) {
      RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");
      depth_loss_emergency_latched_ = true;
    }
    changeFSMExecState(EMERGENCY_STOP, "SAFETY");
  } else if (depth_loss_emergency_latched_) {
    depth_loss_emergency_latched_ = false;
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_DEPTH_RECOVERED] cloud is fresh again, emergency latch released");
  }

  /* ---------- check trajectory ---------- */
  constexpr double time_step = 0.01;
  // double t_cur = (ros::Time::now() - info->start_time_).toSec();
  double t_cur = (node_->now() - info->start_time_).seconds();

  Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
  const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
  // double t_cur_global = ros::Time::now().toSec();
  double t_cur_global = node_->now().seconds();

  // Check a rolling distance window from the actual current state. The old
  // fixed two-thirds time window depended on trajectory duration and could
  // discover a pillar only after the vehicle was already close to it.
  double checked_distance = 0.0;
  Eigen::Vector3d previous_checked_point = p_cur;
  for (double t = t_cur; t < info->duration_; t += time_step) {
    const Eigen::Vector3d checked_point = info->position_traj_.evaluateDeBoorT(t);
    if (t > t_cur) {
      checked_distance += (checked_point - previous_checked_point).norm();
      if (checked_distance > active_preplan_lookahead_m_) {
        break;
      }
    }
    previous_checked_point = checked_point;

    bool occ = false;
    const double active_clearance =
      std::isfinite(info->required_clearance_) && info->required_clearance_ > 0.0 ?
      info->required_clearance_ : map->getPlanningCenterClearance();
    occ |= map->getInflateOccupancy(checked_point, active_clearance);

    for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++) {
      if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) ||
        (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
      {
        continue;
      }

      double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
      Eigen::Vector3d swarm_pridicted =
        planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
      double dist = (checked_point - swarm_pridicted).norm();

      if (dist < CLEARANCE) {
        occ = true;
        break;
      }
    }

    if (occ) {

      if (planFromCurrentTraj(replan_candidate_trials_)) { // Make a chance
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        publishSwarmTrajs(false);
        return;
      } else {
        // The currently executed trajectory is confirmed to be in collision.
        // Unlike a speculative replan failure, this must stop immediately.
        publishPendingValidationFailure(true);
        if (t - t_cur < emergency_time_) { // 0.8s of emergency time
          RCLCPP_WARN(
            node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f",
            t - t_cur);

          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        } else {
          RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
      break;
    }
  }
}

bool EGOReplanFSM::callReboundReplan(
  bool flag_use_poly_init, bool flag_randomPolyTraj,
  const Eigen::Vector3d * recovery_target, const std::string * recovery_candidate)
{
  getLocalTarget();
  if (recovery_target != nullptr) {
    local_target_pt_ = *recovery_target;
    local_target_vel_.setZero();
  }
  // All planning paths, including recovery candidates and trajectory splices,
  // must use the same state manifold as flat-mode map validation.  Keeping
  // this at the common replan entry point prevents a near-goal recovery from
  // reintroducing a vertical segment after its reference has been flattened.
  if (validation_flat_mode_) {
    start_pt_.z() = validation_flight_height_;
    start_vel_.z() = 0.0;
    start_acc_.z() = 0.0;
    local_target_pt_.z() = validation_flight_height_;
    local_target_vel_.z() = 0.0;
  }
  current_validation_candidate_ = recovery_candidate == nullptr ? "NORMAL" : *recovery_candidate;

  LocalTrajData previous_local_data = planner_manager_->local_data_;
  const bool have_previous_trajectory = ReplanHandoverPolicy::isTrajectoryActiveAt(
    previous_local_data.start_time_.seconds(), previous_local_data.duration_,
    node_->now().seconds());
  LocalTrajData candidate_local_data;
  std::vector<Eigen::Vector3d> reference_seed;
  const bool normal_candidate = recovery_candidate == nullptr;
  bool recovery_seed_ready = true;
  auto build_recovery_seed = [&](const double clearance) {
      if (recovery_target == nullptr) {
        return true;
      }
      std::vector<Eigen::Vector3d> astar_seed;
      if (!planner_manager_->buildLocalAStarSeed(
          start_pt_, *recovery_target, clearance, validation_flat_mode_, astar_seed))
      {
        return false;
      }
      // Keep the recovery-specific deviation bound meaningful even though the
      // optimizer now receives the complete A* polyline as its seed.
      if (have_active_reference_path_ && active_reference_path_.points.size() >= 2) {
        std::vector<Eigen::Vector3d> recovery_reference = active_reference_path_.points;
        for (const auto & continuation_point : active_reference_path_.continuation_points) {
          if (recovery_reference.empty() ||
            (continuation_point - recovery_reference.back()).norm() > 1.0e-6)
          {
            recovery_reference.push_back(continuation_point);
          }
        }
        for (const auto & point : astar_seed) {
          const auto projection = RecoveryPathPolicy::projectToPolyline(
            recovery_reference, point);
          if (!std::isfinite(projection.distance) ||
            projection.distance > recovery_max_reference_deviation_m_ + 1.0e-6)
          {
            RCLCPP_WARN(
              node_->get_logger(),
              "[EGO_RECOVERY_ASTAR_REJECT] reason=reference_deviation "
              "deviation=%.3f limit=%.3f",
              projection.distance, recovery_max_reference_deviation_m_);
            return false;
          }
        }
      }
      reference_seed = std::move(astar_seed);
      return true;
    };
  if (recovery_target == nullptr && have_active_reference_path_ &&
    active_reference_path_.points.size() >= 2)
  {
    reference_seed = active_reference_path_.points;
  } else if (recovery_target != nullptr) {
    recovery_seed_ready = build_recovery_seed(
      planner_manager_->grid_map_->getPlanningCenterClearance());
  }

  const bool new_target_for_attempt = have_new_target_;
  const double normal_clearance = planner_manager_->grid_map_->getPlanningCenterClearance();
  const double attempt_reference_deviation = normal_candidate ?
    std::numeric_limits<double>::quiet_NaN() : recovery_max_reference_deviation_m_;
  current_validation_candidate_ = normal_candidate ? "NORMAL" : *recovery_candidate;
  bool plan_and_refine_success = recovery_seed_ready &&
    planner_manager_->reboundReplan(
    start_pt_, start_vel_, start_acc_, local_target_pt_,
    local_target_vel_, (new_target_for_attempt || flag_use_poly_init),
    flag_randomPolyTraj, reference_seed.size() >= 2 ? &reference_seed : nullptr,
    &candidate_local_data, force_new_global_path_session_, normal_clearance, false,
    std::numeric_limits<double>::quiet_NaN(), attempt_reference_deviation);
  have_new_target_ = false;

  bool constrained_attempted = false;
  // Lowering clearance is meaningful only when the optimizer found no route
  // at the normal clearance. Geometry, dynamics, and handover failures use
  // their own recovery paths and must never be hidden by a smaller margin.
  if (!plan_and_refine_success && constrained_clearance_enabled_ &&
    (normal_candidate || recovery_target != nullptr) &&
    (planner_manager_->lastReplanFailureReason() ==
      EGOPlannerManager::ReplanFailureReason::CLEARANCE_OR_NO_ROUTE ||
      (!recovery_seed_ready && recovery_target != nullptr)))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_CONSTRAINED_REPLAN] normal_clearance=%.3f failed=true "
      "retry_clearance=%.3f optimization_target=%.3f",
      normal_clearance, constrained_clearance_m_, constrained_optimization_clearance_m_);
    candidate_local_data = LocalTrajData();
    if (recovery_target != nullptr) {
      reference_seed.clear();
      recovery_seed_ready = build_recovery_seed(constrained_clearance_m_);
    }
    current_validation_candidate_ = "CONSTRAINED";
    constrained_attempted = true;
    plan_and_refine_success = recovery_seed_ready && planner_manager_->reboundReplan(
      start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
      (new_target_for_attempt || flag_use_poly_init), flag_randomPolyTraj,
      reference_seed.size() >= 2 ? &reference_seed : nullptr, &candidate_local_data,
      force_new_global_path_session_, constrained_clearance_m_, true,
      constrained_optimization_clearance_m_);
    if (plan_and_refine_success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[EGO_CONSTRAINED_REPLAN] generated=true clearance=%.3f "
        "optimization_target=%.3f trajectory_id=%d",
        constrained_clearance_m_, constrained_optimization_clearance_m_,
        candidate_local_data.traj_id_);
    }
  }

  cout << "refine_success=" << plan_and_refine_success << endl;

  if (!plan_and_refine_success &&
    planner_manager_->lastReplanFailureReason() ==
    EGOPlannerManager::ReplanFailureReason::HANDOVER_CONTINUITY)
  {
    pending_validation_failure_ = "EGO_HANDOVER_CONTINUITY_REJECT";
    if (have_previous_trajectory && !force_new_global_path_session_) {
      double remaining_time = 0.0;
      double minimum_clearance = std::numeric_limits<double>::infinity();
      Eigen::Vector3d endpoint;
      candidate_failure_preserved_active_ = activeTrajectoryRemainingSafe(
        previous_local_data, remaining_time, endpoint, minimum_clearance);
      const HandoverRejectAction action =
        ReplanHandoverPolicy::rejectedCandidateAction(candidate_failure_preserved_active_);
      if (action.keep_active) {
        scheduleReplanRetry(
          "HANDOVER_CONTINUITY_ACTIVE_SAFE", action.retry_delay_sec);
      }
      RCLCPP_WARN(
        node_->get_logger(),
        "[EGO_ACTIVE_TRAJECTORY_DECISION] candidate_failed=true "
        "failure=HANDOVER_CONTINUITY active_trajectory_id=%d active_safe=%s "
        "remaining_time=%.3f endpoint=(%.3f,%.3f,%.3f) minimum_clearance=%.3f",
        previous_local_data.traj_id_,
        candidate_failure_preserved_active_ ? "true" : "false",
        remaining_time, endpoint.x(), endpoint.y(), endpoint.z(), minimum_clearance);
    }
    current_validation_candidate_ = "NORMAL";
    return false;
  }

  bool candidate_validated = false;
  while (plan_and_refine_success && !candidate_validated) {
    // Optimization can finish after the old trajectory expires. Re-evaluate
    // at the exact candidate switch time instead of retaining the attempt-time
    // state, otherwise a terminal hold can reject every valid successor.
    const bool previous_active_at_candidate_start =
      ReplanHandoverPolicy::isTrajectoryActiveAt(
      previous_local_data.start_time_.seconds(), previous_local_data.duration_,
      candidate_local_data.start_time_.seconds());
    if (previous_active_at_candidate_start && !force_new_global_path_session_) {
      double old_elapsed =
        (candidate_local_data.start_time_ - previous_local_data.start_time_).seconds();
      old_elapsed = std::clamp(old_elapsed, 0.0, previous_local_data.duration_);
      const Eigen::Vector3d old_position =
        previous_local_data.position_traj_.evaluateDeBoorT(old_elapsed);
      const Eigen::Vector3d old_velocity =
        previous_local_data.velocity_traj_.evaluateDeBoorT(old_elapsed);
      const Eigen::Vector3d old_acceleration =
        previous_local_data.acceleration_traj_.evaluateDeBoorT(old_elapsed);
      const Eigen::Vector3d new_position =
        candidate_local_data.position_traj_.evaluateDeBoorT(0.0);
      const Eigen::Vector3d new_velocity =
        candidate_local_data.velocity_traj_.evaluateDeBoorT(0.0);
      const Eigen::Vector3d new_acceleration =
        candidate_local_data.acceleration_traj_.evaluateDeBoorT(0.0);
      const HandoverContinuityResult continuity = ReplanHandoverPolicy::validate(
        old_position, old_velocity, old_acceleration,
        new_position, new_velocity, new_acceleration,
        handover_position_tolerance_m_, handover_velocity_tolerance_mps_,
        handover_acceleration_tolerance_mps2_);
      if (!continuity.accepted) {
        const int rejected_trajectory_id = candidate_local_data.traj_id_;
        pending_validation_failure_ = "EGO_HANDOVER_CONTINUITY_REJECT";
        double remaining_time = 0.0;
        double minimum_clearance = std::numeric_limits<double>::infinity();
        Eigen::Vector3d endpoint;
        candidate_failure_preserved_active_ = activeTrajectoryRemainingSafe(
          previous_local_data, remaining_time, endpoint, minimum_clearance);
        const HandoverRejectAction action =
          ReplanHandoverPolicy::rejectedCandidateAction(candidate_failure_preserved_active_);
        if (action.keep_active) {
          scheduleReplanRetry(
            "HANDOVER_CONTINUITY_ACTIVE_SAFE", action.retry_delay_sec);
        }
        RCLCPP_ERROR(
          node_->get_logger(),
          "[EGO_HANDOVER_CONTINUITY_REJECT] trajectory_id=%d stage=PRE_PUBLISH "
          "position_error=%.9f velocity_error=%.9f acceleration_error=%.9f",
          rejected_trajectory_id, continuity.position_error,
          continuity.velocity_error, continuity.acceleration_error);
        RCLCPP_WARN(
          node_->get_logger(),
          "[REJECT_ACTIVE_IMMUTABILITY] active_id_before=%d active_id_after=%d "
          "trajectory_same=true start_time_same=true duration_same=true "
          "safety_same=true visual_same=true pass=true rejected_candidate_id=%d "
          "reason=HANDOVER_CONTINUITY",
          previous_local_data.traj_id_, planner_manager_->local_data_.traj_id_,
          rejected_trajectory_id);
        current_validation_candidate_ = "NORMAL";
        return false;
      }
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[EGO_CANDIDATE_STAGED] candidate_id=%d active_id=%d "
      "active_local_goal_seq=%lu candidate_local_goal_seq=%lu",
      candidate_local_data.traj_id_, planner_manager_->local_data_.traj_id_,
      static_cast<unsigned long>(active_local_goal_seq_),
      static_cast<unsigned long>(local_goal_seq_));

    if (!validateTrajectoryForPublish(candidate_local_data)) {
      // The publish gate is part of the same candidate transaction. A normal
      // trajectory that was generated but misses the normal clearance gets
      // exactly one complete constrained retry, including handover and every
      // publish validation. Other validation failures are never relaxed.
      if (constrained_clearance_enabled_ &&
        (normal_candidate || recovery_target != nullptr) &&
        !constrained_attempted &&
        pending_validation_failure_ == "EGO_TRAJECTORY_COLLISION")
      {
        RCLCPP_WARN(
          node_->get_logger(),
          "[EGO_CONSTRAINED_REPLAN] normal_clearance=%.3f "
          "publish_validation_failed=true retry_clearance=%.3f "
          "optimization_target=%.3f",
          normal_clearance, constrained_clearance_m_,
          constrained_optimization_clearance_m_);
        constrained_attempted = true;
        candidate_local_data = LocalTrajData();
        if (recovery_target != nullptr) {
          reference_seed.clear();
          recovery_seed_ready = build_recovery_seed(constrained_clearance_m_);
        }
        current_validation_candidate_ = "CONSTRAINED";
        plan_and_refine_success = recovery_seed_ready && planner_manager_->reboundReplan(
          start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
          (new_target_for_attempt || flag_use_poly_init), flag_randomPolyTraj,
          reference_seed.size() >= 2 ? &reference_seed : nullptr, &candidate_local_data,
          force_new_global_path_session_, constrained_clearance_m_, true,
          constrained_optimization_clearance_m_);
        if (plan_and_refine_success) {
          // Repeat the same handover and publish gates for this new candidate.
          continue;
        }
        if (planner_manager_->lastReplanFailureReason() ==
          EGOPlannerManager::ReplanFailureReason::HANDOVER_CONTINUITY)
        {
          pending_validation_failure_ = "EGO_HANDOVER_CONTINUITY_REJECT";
        }
      }
      if (have_previous_trajectory) {
        double remaining_time = 0.0;
        double minimum_clearance = std::numeric_limits<double>::infinity();
        Eigen::Vector3d endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
        candidate_failure_preserved_active_ = activeTrajectoryRemainingSafe(
          previous_local_data, remaining_time, endpoint, minimum_clearance);
        if (candidate_failure_preserved_active_) {
          scheduleReplanRetry(
            "CANDIDATE_FAILED_ACTIVE_SAFE",
            CandidateTransactionPolicy::kRejectedCandidateRetryDelaySec);
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "[EGO_ACTIVE_TRAJECTORY_DECISION] candidate_failed=true "
          "active_trajectory_id=%d active_safe=%s remaining_time=%.3f "
          "endpoint=(%.3f,%.3f,%.3f) minimum_clearance=%.3f",
          previous_local_data.traj_id_,
          candidate_failure_preserved_active_ ? "true" : "false",
          remaining_time, endpoint.x(), endpoint.y(), endpoint.z(),
          minimum_clearance);
        RCLCPP_WARN(
          node_->get_logger(),
          "[REJECT_ACTIVE_IMMUTABILITY] active_id_before=%d active_id_after=%d "
          "trajectory_same=true start_time_same=true duration_same=true "
          "safety_same=true visual_same=true pass=true rejected_candidate_id=%d",
          previous_local_data.traj_id_, planner_manager_->local_data_.traj_id_,
          candidate_local_data.traj_id_);
      } else {
        scheduleReplanRetry(
          "FIRST_CANDIDATE_REJECT",
          CandidateTransactionPolicy::kRejectedCandidateRetryDelaySec);
        RCLCPP_WARN(
          node_->get_logger(),
          "[FIRST_CANDIDATE_REJECT] candidate_id=%d active_valid=false "
          "active_id=NONE published=false fsm_state=SAFETY_HOLD",
          candidate_local_data.traj_id_);
      }
      current_validation_candidate_ = "NORMAL";
      return false;
    }

    candidate_validated = true;
  }

  if (candidate_validated) {
    if (!CandidateTransactionPolicy::commitIfValidated(
        candidate_local_data, true, planner_manager_->local_data_))
    {
      pending_validation_failure_ = "EGO_CANDIDATE_COMMIT_FAILED";
      current_validation_candidate_ = "NORMAL";
      return false;
    }
    recovery_failure_policy_.markSuccess();
    active_local_goal_seq_ = local_goal_seq_;
    force_new_global_path_session_ = false;
    auto info = &planner_manager_->local_data_;
    if (info->constrained_clearance_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[EGO_CONSTRAINED_REPLAN] accepted=true clearance=%.3f trajectory_id=%d",
        info->required_clearance_, info->traj_id_);
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "[EGO_CANDIDATE_COMMIT] candidate_id=%d active_id=%d "
      "local_goal_seq=%lu all_validations_passed=true",
      candidate_local_data.traj_id_, info->traj_id_,
      static_cast<unsigned long>(active_local_goal_seq_));

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    bspline.constrained_clearance = info->constrained_clearance_;
    bspline.required_clearance = info->required_clearance_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    /* 1. publish traj to traj_server */
    bspline_pub_->publish(bspline);

    if (ego_diagnostics_enabled_) {
      active_trajectory_diagnostic_data_ = *info;
      have_active_trajectory_diagnostic_data_ = true;
      active_trajectory_map_binding_.bind(
        info->traj_id_, planner_manager_->grid_map_->getMapRevision(),
        last_validated_trajectory_minimum_clearance_);
      RCLCPP_INFO(
        node_->get_logger(),
        "[EGO_TRAJECTORY_MAP_BINDING] trajectory_id=%d map_revision=%lu "
        "minimum_clearance=%.3f collision_free=true",
        info->traj_id_,
        static_cast<unsigned long>(active_trajectory_map_binding_.publishedMapRevision()),
        last_validated_trajectory_minimum_clearance_);
    }

    /* 2. publish traj to the next drone of swarm */

    /* 3. publish traj for visualization */
    visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
  }
  current_validation_candidate_ = "NORMAL";
  return candidate_validated;
}

bool EGOReplanFSM::activeTrajectoryRemainingSafe(
  LocalTrajData & trajectory, double & remaining_time,
  Eigen::Vector3d & endpoint, double & minimum_clearance)
{
  remaining_time = 0.0;
  endpoint = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  minimum_clearance = std::numeric_limits<double>::infinity();
  if (trajectory.start_time_.seconds() <= 1e-5 ||
    !std::isfinite(trajectory.duration_) || trajectory.duration_ <= 0.0)
  {
    return false;
  }

  const double elapsed = (node_->now() - trajectory.start_time_).seconds();
  if (!std::isfinite(elapsed)) {
    return false;
  }
  const bool terminal_hold = elapsed >= trajectory.duration_;
  const double start_time = terminal_hold ?
    trajectory.duration_ : std::max(0.0, elapsed);
  remaining_time = trajectory.duration_ - start_time;
  endpoint = trajectory.position_traj_.evaluateDeBoorT(trajectory.duration_);
  const auto map = planner_manager_->grid_map_;
  const double required_clearance =
    std::isfinite(trajectory.required_clearance_) && trajectory.required_clearance_ > 0.0 ?
    trajectory.required_clearance_ : map->getPlanningCenterClearance();
  if (terminal_hold) {
    Eigen::Vector3d actual = odom_pos_;
    if (validation_flat_mode_) {
      actual.z() = validation_flight_height_;
    }
    if (!actual.allFinite() || map->getInflateOccupancy(actual, required_clearance) != 0) {
      return false;
    }
  }
  constexpr double time_step = 0.01;
  for (double time = start_time; time <= trajectory.duration_ + time_step; time += time_step) {
    const double sample_time = std::min(time, trajectory.duration_);
    Eigen::Vector3d point = trajectory.position_traj_.evaluateDeBoorT(sample_time);
    if (validation_flat_mode_) {
      point.z() = validation_flight_height_;
    }
    if (!point.allFinite() || map->getInflateOccupancy(point, required_clearance) != 0) {
      return false;
    }
    Eigen::Vector3d nearest;
    double clearance = std::numeric_limits<double>::infinity();
    map->findNearestInflatedObstacle(point, nearest, clearance);
    minimum_clearance = std::min(minimum_clearance, clearance);
    if (sample_time >= trajectory.duration_) {
      break;
    }
  }
  return true;
}

void EGOReplanFSM::scheduleReplanRetry(const std::string & reason, const double delay_sec)
{
  const double safe_delay = std::max(0.05, std::isfinite(delay_sec) ? delay_sec : 0.25);
  const rclcpp::Time now = node_->now();
  const rclcpp::Time requested = now + rclcpp::Duration::from_seconds(safe_delay);
  if (next_replan_retry_time_.nanoseconds() != 0 &&
    next_replan_retry_time_ >= requested - rclcpp::Duration::from_seconds(0.01))
  {
    return;
  }
  next_replan_retry_time_ = requested;

  double remaining_time = 0.0;
  double minimum_clearance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d endpoint;
  const bool active_safe = activeTrajectoryRemainingSafe(
    planner_manager_->local_data_, remaining_time, endpoint, minimum_clearance);
  RCLCPP_WARN(
    node_->get_logger(),
    "[EGO_REPLAN_RETRY] reason=%s active_safe=%s active_remaining_time=%.3f "
    "retry_delay=%.3f local_goal_seq=%lu",
    reason.c_str(), active_safe ? "true" : "false", remaining_time, safe_delay,
    static_cast<unsigned long>(local_goal_seq_));
}

void EGOReplanFSM::publishSafetyStatus(const std::string & status)
{
  if (status == "EGO_REPLAN_REANCHOR_RETRY") {
    awaiting_bridge_replan_ready_ = true;
    bridge_replan_ready_received_ = false;
  }
  logFailureSemantics(status);
  std_msgs::msg::String message;
  message.data = status;
  safety_status_pub_->publish(message);
}

void EGOReplanFSM::replanReadyCallback(
  const std::shared_ptr<const std_msgs::msg::UInt64> & msg)
{
  if (msg->data <= last_bridge_replan_ready_token_) {
    return;
  }
  last_bridge_replan_ready_token_ = msg->data;
  const bool planner_reanchor_requested = awaiting_bridge_replan_ready_;
  const bool bridge_requested_replan = have_target_ &&
    (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ);
  if (!planner_reanchor_requested && !bridge_requested_replan) {
    return;
  }

  awaiting_bridge_replan_ready_ = false;
  bridge_replan_ready_received_ = true;
  force_new_global_path_session_ = true;
  next_replan_retry_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  if (exec_state_ == EXEC_TRAJ) {
    changeFSMExecState(REPLAN_TRAJ, "BRIDGE_READY");
  }
  RCLCPP_WARN(
    node_->get_logger(),
    "[EGO_REPLAN_READY_ACCEPTED] token=%lu source=%s start=latest_odom",
    static_cast<unsigned long>(msg->data), planner_reanchor_requested ?
    "planner_reanchor" : "bridge_switch");
}

void EGOReplanFSM::publishPendingValidationFailure(bool immediate)
{
  if (pending_validation_failure_.empty()) {
    return;
  }

  if (candidate_failure_preserved_active_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_CANDIDATE_REJECTED_ACTIVE_TRAJECTORY_SAFE] goal_seq=%lu status=%s",
      static_cast<unsigned long>(local_goal_seq_), pending_validation_failure_.c_str());
    candidate_failure_preserved_active_ = false;
    return;
  }

  if (recovery_cooldown_waiting_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[EGO_FAILURE_REPORT] goal_seq=%lu reason=%s deduplicated=true",
      static_cast<unsigned long>(local_goal_seq_), pending_validation_failure_.c_str());
    return;
  }

  if (recovery_rejoin_failed_) {
    const bool publish = recovery_failure_policy_.shouldPublishFailure(
      local_goal_seq_, pending_validation_failure_);
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_FAILURE_REPORT] goal_seq=%lu reason=%s deduplicated=%s",
      static_cast<unsigned long>(local_goal_seq_), pending_validation_failure_.c_str(),
      publish ? "false" : "true");
    if (publish) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[EGO_REPLAN_FAILED] all recovery candidates rejected; status=%s",
        pending_validation_failure_.c_str());
      publishSafetyStatus(pending_validation_failure_);
    }
    return;
  }

  if (pending_validation_failure_ != last_validation_failure_) {
    last_validation_failure_ = pending_validation_failure_;
    consecutive_validation_failures_ = 0;
  }
  ++consecutive_validation_failures_;

  if (!immediate &&
    consecutive_validation_failures_ < validation_failure_retry_limit_)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_REPLAN_RETRY] status=%s consecutive_failures=%d retry_limit=%d",
      pending_validation_failure_.c_str(), consecutive_validation_failures_,
      validation_failure_retry_limit_);
    return;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[EGO_REPLAN_FAILED] all trajectory candidates rejected; status=%s "
    "consecutive_failures=%d immediate=%s",
    pending_validation_failure_.c_str(), consecutive_validation_failures_,
    immediate ? "true" : "false");
  RCLCPP_ERROR(
    node_->get_logger(),
    "[EGO_FAILURE_REPORT] goal_seq=%lu reason=%s deduplicated=false",
    static_cast<unsigned long>(local_goal_seq_), pending_validation_failure_.c_str());
  publishSafetyStatus(pending_validation_failure_);
}

bool EGOReplanFSM::validateTrajectoryForPublish(LocalTrajData & candidate)
{
  auto * info = &candidate;
  auto map = planner_manager_->grid_map_;
  const double map_age = map->getMapAgeSec();
  if (!std::isfinite(map_age) || map_age > validation_map_timeout_sec_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[EGO_OCCUPANCY_STALE] trajectory_id=%d occupancy_map_age=%.3f frame=%s",
      info->traj_id_, map_age, map->getFrameId().c_str());
    pending_validation_failure_ = "EGO_OCCUPANCY_STALE";
    return false;
  }

  const double duration = info->duration_;
  last_validated_trajectory_minimum_clearance_ = std::numeric_limits<double>::infinity();
  if (!std::isfinite(duration) || duration <= 0.0 ||
    validation_sample_spacing_ <= 0.0)
  {
    pending_validation_failure_ = "EGO_TRAJECTORY_INVALID";
    return false;
  }

  if (!validateTrajectoryGeofenceForPublish(candidate)) {
    return false;
  }

  constexpr double base_time_step = 0.01;
  size_t sample_index = 0;
  const bool constrained = info->constrained_clearance_;
  const double trajectory_clearance =
    std::isfinite(info->required_clearance_) && info->required_clearance_ > 0.0 ?
    info->required_clearance_ : map->getPlanningCenterClearance();
  const double hard_clearance = constrained ? trajectory_clearance : map->getRequiredCenterClearance();
  const double planning_clearance = constrained ? trajectory_clearance : map->getPlanningCenterClearance();
  Eigen::Vector3d initial_point = info->position_traj_.evaluateDeBoorT(0.0);
  if (validation_flat_mode_) {
    initial_point.z() = validation_flight_height_;
  }
  Eigen::Vector3d initial_nearest;
  double initial_clearance = std::numeric_limits<double>::infinity();
  const int initial_hard_occupancy = initial_point.allFinite() ?
    map->queryContinuousOccupancy(initial_point, initial_nearest, initial_clearance) : -1;
  if ((!constrained && initial_hard_occupancy != 0) || !std::isfinite(initial_clearance) ||
    !std::isfinite(hard_clearance) || !std::isfinite(planning_clearance))
  {
    pending_validation_failure_ = "EGO_TRAJECTORY_COLLISION";
    return false;
  }
  const double candidate_required_clearance = constrained ? trajectory_clearance :
    CandidateTransactionPolicy::requiredPublishClearance(
      hard_clearance, planning_clearance, initial_clearance);
  RCLCPP_INFO(
    node_->get_logger(),
    "[EGO_CANDIDATE_CLEARANCE_GATE] trajectory_id=%d candidate=%s "
    "start_clearance=%.3f required_publish_clearance=%.3f "
    "planning_clearance=%.3f hard_clearance=%.3f",
    info->traj_id_, current_validation_candidate_.c_str(), initial_clearance,
    candidate_required_clearance, planning_clearance, hard_clearance);
  auto check_point = [&](Eigen::Vector3d point) -> bool
    {
      if (validation_flat_mode_) {
        point.z() = validation_flight_height_;
      }

      Eigen::Vector3d nearest = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
      double clearance = std::numeric_limits<double>::infinity();
      const int occupancy = point.allFinite() ?
        map->queryContinuousOccupancy(point, nearest, clearance) : -1;
      if (ego_diagnostics_enabled_) {
        last_validated_trajectory_minimum_clearance_ = std::min(
          last_validated_trajectory_minimum_clearance_, clearance);
      }
      constexpr double clearance_tolerance = 1.0e-6;
      if ((constrained ? occupancy >= 0 : occupancy == 0) &&
        clearance + clearance_tolerance >= candidate_required_clearance)
      {
        ++sample_index;
        return true;
      }

      const int nearby_occupied = map->countInflatedOccupancy(point, 0.30);
      RCLCPP_ERROR(
        node_->get_logger(),
        "[EGO_TRAJECTORY_COLLISION] trajectory_id=%d local_goal_seq=unavailable "
        "sample_index=%zu position=(%.3f,%.3f,%.3f) "
        "nearest_obstacle=(%.3f,%.3f,%.3f) clearance=%.3f "
        "required_publish_clearance=%.3f planning_clearance=%.3f hard_clearance=%.3f "
        "occupancy_map_age=%.3f frame=%s nearby_occupied_voxels=%d",
        info->traj_id_, sample_index, point.x(), point.y(), point.z(), nearest.x(),
        nearest.y(), nearest.z(), clearance, candidate_required_clearance,
        planning_clearance, hard_clearance, map_age, map->getFrameId().c_str(),
        nearby_occupied);
      pending_validation_failure_ = "EGO_TRAJECTORY_COLLISION";
      logCandidateFailureContext();
      return false;
    };

  Eigen::Vector3d previous = info->position_traj_.evaluateDeBoorT(0.0);
  if (!check_point(previous)) {
    return false;
  }

  for (double t = base_time_step; t < duration + base_time_step; t += base_time_step) {
    const double sample_time = std::min(t, duration);
    const Eigen::Vector3d next = info->position_traj_.evaluateDeBoorT(sample_time);
    const double distance = (next - previous).norm();
    const size_t subdivisions = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(distance / validation_sample_spacing_)));
    for (size_t i = 1; i <= subdivisions; ++i) {
      const Eigen::Vector3d point = previous +
        (next - previous) * (static_cast<double>(i) / subdivisions);
      if (!check_point(point)) {
        return false;
      }
    }
    previous = next;
    if (sample_time >= duration) {
      break;
    }
  }

  if (!pending_validation_failure_.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[EGO_REPLAN_RECOVERED] rejected candidate status=%s "
      "consecutive_failures=%d; publishing safe trajectory_id=%d",
      pending_validation_failure_.c_str(), consecutive_validation_failures_, info->traj_id_);
  }
  pending_validation_failure_.clear();
  candidate_failure_preserved_active_ = false;
  last_validation_failure_.clear();
  consecutive_validation_failures_ = 0;
  recovery_failure_policy_.clearReportedFailure();
  publishSafetyStatus("EGO_TRAJECTORY_VALID");
  return true;
}

bool EGOReplanFSM::validateTrajectoryGeofenceForPublish(
  LocalTrajData & candidate)
{
  auto * info = &candidate;
  constexpr double base_time_step = 0.02;
  const double duration = info->duration_;
  Eigen::Vector3d previous = info->position_traj_.evaluateDeBoorT(0.0);
  if (validation_flat_mode_) {
    previous.z() = validation_flight_height_;
  }
  double previous_time = 0.0;

  auto reject = [this](const Eigen::Vector3d & point, const double time)
    {
      const double values[3] = {point.x(), point.y(), point.z()};
      const double lower[3] = {shared_bounds_x_min_, shared_bounds_y_min_, shared_bounds_z_min_};
      const double upper[3] = {shared_bounds_x_max_, shared_bounds_y_max_, shared_bounds_z_max_};
      const char * axes[3] = {"X", "Y", "Z"};
      for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(values[axis]) || values[axis] < lower[axis] ||
          values[axis] > upper[axis])
        {
          const bool below = !std::isfinite(values[axis]) || values[axis] < lower[axis];
          const double bound = below ? lower[axis] : upper[axis];
          const double overshoot = !std::isfinite(values[axis]) ?
            std::numeric_limits<double>::quiet_NaN() : std::abs(values[axis] - bound);
          RCLCPP_ERROR(
            node_->get_logger(),
            "[EGO_GEOFENCE_REJECT] candidate=%s first_violation_t=%.3f "
            "point=(%.3f,%.3f,%.3f) axis=%s bound=%s=%.3f overshoot=%.3f",
            current_validation_candidate_.c_str(), time, point.x(), point.y(),
            point.z(), axes[axis],
            below ? "min" : "max", bound, overshoot);
          pending_validation_failure_ = "EGO_TRAJECTORY_OUT_OF_GEOFENCE";
          return true;
        }
      }
      return false;
    };

  if (reject(previous, 0.0)) {
    return false;
  }
  for (double time = base_time_step; time < duration + base_time_step; time += base_time_step) {
    const double sample_time = std::min(time, duration);
    Eigen::Vector3d next = info->position_traj_.evaluateDeBoorT(sample_time);
    if (validation_flat_mode_) {
      next.z() = validation_flight_height_;
    }
    const std::size_t subdivisions = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(
        (next - previous).norm() / validation_sample_spacing_)));
    for (std::size_t index = 1; index <= subdivisions; ++index) {
      const double ratio = static_cast<double>(index) / subdivisions;
      const Eigen::Vector3d point = previous + (next - previous) * ratio;
      if (reject(point, previous_time + (sample_time - previous_time) * ratio)) {
        return false;
      }
    }
    previous = next;
    previous_time = sample_time;
    if (sample_time >= duration) {
      break;
    }
  }
  return true;
}

void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
{
  auto info = &planner_manager_->local_data_;

  traj_utils::msg::Bspline bspline;
  bspline.order = 3;
  bspline.start_time = info->start_time_;
  bspline.drone_id = planner_manager_->pp_.drone_id;
  bspline.traj_id = info->traj_id_;
  bspline.constrained_clearance = info->constrained_clearance_;
  bspline.required_clearance = info->required_clearance_;

  Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  bspline.pos_pts.reserve(pos_pts.cols());
  for (int i = 0; i < pos_pts.cols(); ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = pos_pts(0, i);
    pt.y = pos_pts(1, i);
    pt.z = pos_pts(2, i);
    bspline.pos_pts.push_back(pt);
  }

  Eigen::VectorXd knots = info->position_traj_.getKnot();

  bspline.knots.reserve(knots.rows());
  for (int i = 0; i < knots.rows(); ++i) {
    bspline.knots.push_back(knots(i));
  }

  if (startup_pub) {
    multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id;   // zx-todo
    if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1) {
      multi_bspline_msgs_buf_.traj.back() = bspline;
    } else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id) {
      multi_bspline_msgs_buf_.traj.push_back(bspline);
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d",
        (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
      // return plan_and_refine_success;
    }
    // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
    swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
  }

  broadcast_bspline_pub_->publish(bspline);
}

bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
{

  planner_manager_->EmergencyStop(stop_pos);

  auto info = &planner_manager_->local_data_;

  /* publish traj */
  traj_utils::msg::Bspline bspline;
  bspline.order = 3;
  bspline.start_time = info->start_time_;
  bspline.traj_id = info->traj_id_;
  bspline.constrained_clearance = false;
  bspline.required_clearance = planner_manager_->grid_map_->getPlanningCenterClearance();

  Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  bspline.pos_pts.reserve(pos_pts.cols());
  for (int i = 0; i < pos_pts.cols(); ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = pos_pts(0, i);
    pt.y = pos_pts(1, i);
    pt.z = pos_pts(2, i);
    bspline.pos_pts.push_back(pt);
  }

  Eigen::VectorXd knots = info->position_traj_.getKnot();
  bspline.knots.reserve(knots.rows());
  for (int i = 0; i < knots.rows(); ++i) {
    bspline.knots.push_back(knots(i));
  }

  bspline_pub_->publish(bspline);

  return true;
}

void EGOReplanFSM::getLocalTarget()
{
  double t;

  double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
  double dist_min = 9999, dist_min_t = 0.0;
  for (t = planner_manager_->global_data_.last_progress_time_;
    t < planner_manager_->global_data_.global_duration_; t += t_step)
  {
    Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
    double dist = (pos_t - start_pt_).norm();

    if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_) {
      // Important cornor case!
      for (; t < planner_manager_->global_data_.global_duration_; t += t_step) {
        Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
        double dist_temp = (pos_t_temp - start_pt_).norm();
        if (dist_temp < planning_horizen_) {
          pos_t = pos_t_temp;
          dist = (pos_t - start_pt_).norm();
          cout << "Escape cornor case \"getLocalTarget\"" << endl;
          break;
        }
      }
    }

    if (dist < dist_min) {
      dist_min = dist;
      dist_min_t = t;
    }

    if (dist >= planning_horizen_) {
      local_target_pt_ = pos_t;
      planner_manager_->global_data_.last_progress_time_ = dist_min_t;
      break;
    }
  }
  if (t > planner_manager_->global_data_.global_duration_) { // Last global point
    local_target_pt_ = end_pt_;
    planner_manager_->global_data_.last_progress_time_ =
      planner_manager_->global_data_.global_duration_;
  }

  if ((end_pt_ - local_target_pt_).norm() <
    (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) /
    (2 * planner_manager_->pp_.max_acc_))
  {
    local_target_vel_ = Eigen::Vector3d::Zero();
  } else {
    local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
  }
}

} // namespace ego_planner
