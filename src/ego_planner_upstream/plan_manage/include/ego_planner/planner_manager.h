#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <limits>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>
#include <ego_planner/replan_handover_policy.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    enum class ReplanFailureReason
    {
      NONE,
      CLEARANCE_OR_NO_ROUTE,
      GEOMETRY_INVALID,
      DYNAMICS_INVALID,
      HANDOVER_CONTINUITY
    };

    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit,
                       bool flag_randomPolyTraj,
                       const std::vector<Eigen::Vector3d> * reference_path = nullptr,
                       LocalTrajData * candidate_output = nullptr,
                       bool preempt_active_trajectory = false,
                       double required_clearance = std::numeric_limits<double>::quiet_NaN(),
                       bool constrained_clearance = false,
                       double optimization_clearance = std::numeric_limits<double>::quiet_NaN(),
                       double max_reference_deviation =
                         std::numeric_limits<double>::quiet_NaN());
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);
    bool buildLocalAStarSeed(
      const Eigen::Vector3d & start, const Eigen::Vector3d & target,
      double required_clearance, bool flat_mode,
      std::vector<Eigen::Vector3d> & path);

    ReplanFailureReason lastReplanFailureReason() const {return last_replan_failure_reason_;}
    const HandoverContinuityResult & lastHandoverContinuityResult() const
    {
      return last_handover_continuity_result_;
    }
    

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr obj_predictor_;    
    SwarmTrajData swarm_trajs_buf_;

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;
    rclcpp::Clock::SharedPtr trajectory_clock_;

    // ros::Publisher obj_pub_; //zx-todo 

    BsplineOptimizer::Ptr bspline_optimizer_;
    // Bound optimized trajectories to a reasonable distance from their
    // reference/A* seed; this is a geometry guard, not an obstacle margin.
    double max_reference_deviation_m_{0.60};

    int continous_failures_count_{0};
    ReplanFailureReason last_replan_failure_reason_{ReplanFailureReason::NONE};
    HandoverContinuityResult last_handover_continuity_result_;

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
