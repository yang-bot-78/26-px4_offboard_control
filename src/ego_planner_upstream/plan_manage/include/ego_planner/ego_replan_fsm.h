#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/local_path_reference.hpp"
#include "race_msgs/msg/flight_altitude_reference.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include "visualization_msgs/msg/marker.hpp"

#include "bspline_opt/bspline_optimizer.h"
#include "plan_env/grid_map.h"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/multi_bsplines.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "traj_utils/msg/data_disp.hpp"
#include "ego_planner/candidate_transaction_policy.h"
#include "ego_planner/planner_manager.h"
#include "ego_planner/recovery_path_policy.h"
#include "ego_planner/recovery_failure_policy.h"
#include "ego_planner/trajectory_diagnostic_binding.h"
#include "traj_utils/planning_visualization.h"

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    // Stage 3B recovery candidates. Only a candidate that passes the existing
    // EGO validation path is allowed to reach the existing B-spline publisher.
    struct RecoveryCandidate
    {
      std::string type;
      Eigen::Vector3d target;
      double forward_distance_m;
      // Signed offset from the reference line, left of travel positive.  Zero for
      // an on-line rejoin point.  Diagnostics only; the local A* still decides
      // which way to actually pass the obstacle.
      double lateral_offset_m{0.0};
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::msg::DataDisp data_disp_;
    traj_utils::msg::MultiBsplines multi_bspline_msgs_buf_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_, replan_start_position_error_;
    double waypoints_[50][3];
    int waypoint_num_, wp_id_;
    int replan_candidate_trials_;
    int validation_failure_retry_limit_;
    int recovery_max_attempts_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    // Configured fail-safe switch.  Read once from fsm/fail_safe and never
    // modified at runtime; transient faults use their own latches so that a
    // single dropout cannot permanently disable recovery.
    bool enable_fail_safe_;
    // Set while the occupancy cloud is stale, cleared as soon as it is fresh
    // again.  Blocks the EMERGENCY_STOP exit so planning does not resume
    // against stale occupancy.
    bool depth_loss_emergency_latched_{false};
    bool validation_flat_mode_;
    bool shadow_mode_;
    bool recovery_enabled_;
    bool recovery_diagnostic_only_;
    bool constrained_clearance_enabled_;
    double constrained_clearance_m_;
    double constrained_optimization_clearance_m_;
    double validation_flight_height_;
    double configured_validation_flight_height_;
    double validation_sample_spacing_;
    double validation_map_timeout_sec_;
    double shared_bounds_x_min_, shared_bounds_x_max_;
    double shared_bounds_y_min_, shared_bounds_y_max_;
    double shared_bounds_z_min_, shared_bounds_z_max_;
    double configured_shared_bounds_z_min_, configured_shared_bounds_z_max_;
    uint64_t altitude_reference_flight_id_{0};
    bool altitude_reference_valid_{false};
    double recovery_cooldown_sec_;
    double recovery_exhausted_backoff_sec_;
    double recovery_rejoin_search_distance_m_;
    double recovery_minimum_forward_progress_m_;
    double recovery_extra_clearance_m_;
    double recovery_max_reference_deviation_m_;
    // Lateral rejoin search, used only after the on-line search fails.  Zero on
    // either disables the fallback and restores the forward-only behaviour.
    double recovery_max_lateral_offset_m_{0.0};
    double recovery_lateral_offset_step_m_{0.0};
    // Upper bound on rejoin candidates tried within one recovery attempt.  Each
    // one costs a local A* search plus a full B-spline optimization, so this
    // trades escape breadth against attempt latency.
    std::size_t recovery_max_candidates_{3};
    double handover_position_tolerance_m_;
    double handover_velocity_tolerance_mps_;
    double handover_acceleration_tolerance_mps2_;
    double active_preplan_lookahead_m_{0.65};

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    Eigen::Vector2d local_goal_heading_{Eigen::Vector2d::Zero()};
    bool have_local_goal_heading_{false};
    struct LocalPathReference
    {
      uint64_t global_path_id{0};
      uint64_t local_goal_seq{0};
      Eigen::Vector3d local_goal{Eigen::Vector3d::Zero()};
      std::vector<Eigen::Vector3d> points;
      std::vector<Eigen::Vector3d> continuation_points;
      double arc_length{0.0};
      double minimum_clearance{std::numeric_limits<double>::infinity()};
      rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    };
    LocalPathReference latest_reference_path_;
    LocalPathReference active_reference_path_;
    bool have_latest_reference_path_{false};
    bool have_active_reference_path_{false};
    bool force_new_global_path_session_{false};
    std::vector<Eigen::Vector3d> wps_;
    int current_wp_;

    // init() does not set this, and the first EMERGENCY_STOP entry reads it, so
    // an indeterminate value decided whether the first emergency stop command
    // was actually issued.  Default to true so the stop is always commanded.
    bool flag_escape_emergency_{true};
    std::string pending_validation_failure_;
    bool candidate_failure_preserved_active_{false};
    std::string last_validation_failure_;
    int consecutive_validation_failures_{0};
    uint64_t local_goal_seq_{0};
    uint64_t active_local_goal_seq_{0};
    bool recovery_cooldown_waiting_{false};
    bool recovery_rejoin_failed_{false};
    RecoveryFailurePolicy recovery_failure_policy_;
    TrajectoryDiagnosticBinding active_trajectory_map_binding_;
    LocalTrajData active_trajectory_diagnostic_data_;
    bool have_active_trajectory_diagnostic_data_{false};
    bool ego_diagnostics_enabled_{false};
    double last_validated_trajectory_minimum_clearance_{std::numeric_limits<double>::infinity()};
    std::string current_validation_candidate_{"NORMAL"};
    rclcpp::Time last_recovery_diagnostic_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_diagnostic_fsm_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_odom_receive_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_local_goal_receive_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time next_replan_retry_time_{0, 0, RCL_ROS_TIME};

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<race_msgs::msg::LocalPathReference>::SharedPtr
      reference_path_sub_;
    rclcpp::Subscription<race_msgs::msg::FlightAltitudeReference>::SharedPtr
      altitude_reference_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_sub_;
    rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;

    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_pub_;

    /* helper functions */
    bool callReboundReplan(
      bool flag_use_poly_init, bool flag_randomPolyTraj,
      const Eigen::Vector3d * recovery_target = nullptr,
      const std::string * recovery_candidate = nullptr); // front-end and back-end method
    bool validateTrajectoryForPublish(LocalTrajData & candidate);
    bool validateTrajectoryGeofenceForPublish(LocalTrajData & candidate);
    void publishSafetyStatus(const std::string &status);
    void publishPendingValidationFailure(bool immediate);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromCurrentTraj(const int trial_times = 1);
    std::optional<RecoveryCandidate> generateRecoveryCandidate() const;
    // Ordered rejoin candidates: the on-line point first, then lateral offsets.
    // A rejoin point that is merely *found* is not usable -- the local A* still
    // has to route to it -- so the caller must be able to fall through to the
    // next candidate instead of declaring recovery failed on the first one.
    std::vector<RecoveryCandidate> generateRecoveryCandidates() const;
    void logRecoveryCandidateOnFailure(const char * planning_context);
    bool planWithRecovery(const char * planning_context);
    double recoveryTrajectoryClearance();
    void logActiveTrajectoryRecheck(double fsm_timer_gap_ms);
    void logCandidateFailureContext();
    void logFailureSemantics(const std::string & downstream_status);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps();
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void referencePathCallback(
      const std::shared_ptr<const race_msgs::msg::LocalPathReference> & msg);
    void altitudeReferenceCallback(
      const std::shared_ptr<const race_msgs::msg::FlightAltitudeReference> & msg);
    void triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg);
    void swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg);
    void BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg);

    bool checkCollision();
    void publishSwarmTrajs(bool startup_pub);
    bool activeTrajectoryRemainingSafe(
      LocalTrajData & trajectory, double & remaining_time,
      Eigen::Vector3d & endpoint, double & minimum_clearance);
    void scheduleReplanRetry(const std::string & reason, double delay_sec);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(rclcpp::Node::SharedPtr &node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif
