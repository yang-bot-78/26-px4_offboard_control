// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <algorithm>
#include <array>
#include <thread>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{

  namespace
  {
  struct HandoverTolerances
  {
    double position_m{0.01};
    double velocity_mps{0.005};
    double acceleration_mps2{0.01};
  } handover_tolerances;
  }  // namespace

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    trajectory_clock_ = node->get_clock();
    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/planning_horizon", 5.0);
    node->declare_parameter("manager/use_distinctive_trajs", false);
    node->declare_parameter("manager/drone_id", -1);
    node->declare_parameter("manager/handover_position_tolerance_m", 0.01);
    node->declare_parameter("manager/handover_velocity_tolerance_mps", 0.005);
    node->declare_parameter("manager/handover_acceleration_tolerance_mps2", 0.01);
    node->declare_parameter("manager/max_reference_deviation_m", 0.60);

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
    node->get_parameter("manager/drone_id", pp_.drone_id);
    node->get_parameter(
      "manager/handover_position_tolerance_m", handover_tolerances.position_m);
    node->get_parameter(
      "manager/handover_velocity_tolerance_mps", handover_tolerances.velocity_mps);
    node->get_parameter(
      "manager/handover_acceleration_tolerance_mps2", handover_tolerances.acceleration_mps2);
    node->get_parameter("manager/max_reference_deviation_m", max_reference_deviation_m_);
    handover_tolerances.position_m = std::clamp(handover_tolerances.position_m, 0.001, 0.02);
    handover_tolerances.velocity_mps = std::clamp(handover_tolerances.velocity_mps, 0.001, 0.02);
    handover_tolerances.acceleration_mps2 = std::clamp(
      handover_tolerances.acceleration_mps2, 0.005, 0.05);
    max_reference_deviation_m_ = std::clamp(
      std::isfinite(max_reference_deviation_m_) ? max_reference_deviation_m_ : 0.60,
      0.35, 1.00);
    RCLCPP_INFO(
      node->get_logger(),
      "[EGO_HANDOVER_TOLERANCE] position_m=%.3f velocity_mps=%.3f acceleration_mps2=%.3f",
      handover_tolerances.position_m, handover_tolerances.velocity_mps,
      handover_tolerances.acceleration_mps2);
    RCLCPP_INFO(
      node->get_logger(),
      "[EGO_REFERENCE_GEOMETRY_LIMIT] max_reference_deviation_m=%.3f",
      max_reference_deviation_m_);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  bool EGOPlannerManager::buildLocalAStarSeed(
    const Eigen::Vector3d & start, const Eigen::Vector3d & target,
    const double required_clearance, const bool flat_mode,
    std::vector<Eigen::Vector3d> & path)
  {
    path.clear();
    if (!start.allFinite() || !target.allFinite() ||
      !std::isfinite(required_clearance) || required_clearance <= 0.0 ||
      !bspline_optimizer_ || !bspline_optimizer_->a_star_)
    {
      return false;
    }
    if (grid_map_->getInflateOccupancy(start, required_clearance) != 0 ||
      grid_map_->getInflateOccupancy(target, required_clearance) != 0)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("ego_planner"),
        "[EGO_RECOVERY_ASTAR_REJECT] reason=occupied_endpoint clearance=%.3f",
        required_clearance);
      return false;
    }

    const double search_step = std::clamp(grid_map_->getResolution(), 0.05, 0.10);
    // Sampling spacing for the post-search segment re-check below.  Hand the
    // same value to the search so both apply one verdict: expansion used to test
    // grid points only, so it happily returned diagonal steps whose middle cut
    // an obstacle corner, and the re-check then rejected them at the identical
    // clearance.  That contradiction is reproducible on every retry, which is
    // why recovery could spin for 559s without ever making progress.
    const double validation_spacing = std::max(
      0.01, std::min(0.05, grid_map_->getResolution() * 0.5));
    bspline_optimizer_->a_star_->setRequiredClearance(required_clearance);
    // a_star_ is shared with the optimizer's own segment-splitting searches,
    // which only need a direction hint and must keep their original behaviour.
    // Scope the stricter segment test to this call on every exit path.
    struct SegmentSpacingScope
    {
      AStar::Ptr a_star;
      ~SegmentSpacingScope() {if (a_star) {a_star->setSegmentSampleSpacing(0.0);}}
    } spacing_scope{bspline_optimizer_->a_star_};
    bspline_optimizer_->a_star_->setSegmentSampleSpacing(validation_spacing);
    if (!bspline_optimizer_->a_star_->AstarSearch(search_step, start, target)) {
      RCLCPP_WARN(
        rclcpp::get_logger("ego_planner"),
        "[EGO_RECOVERY_ASTAR_REJECT] reason=no_route clearance=%.3f",
        required_clearance);
      return false;
    }

    std::vector<Eigen::Vector3d> raw = bspline_optimizer_->a_star_->getPath();
    path.reserve(raw.size() + 2);
    path.push_back(start);
    for (auto point : raw) {
      if (flat_mode) {
        point.z() = start.z();
      }
      if ((point - path.back()).norm() > 1.0e-5) {
        path.push_back(point);
      }
    }
    Eigen::Vector3d exact_target = target;
    if (flat_mode) {
      exact_target.z() = start.z();
    }
    if ((exact_target - path.back()).norm() > 1.0e-5) {
      path.push_back(exact_target);
    } else {
      path.back() = exact_target;
    }

    // Kept as an independent verification of the returned path even though the
    // search now applies the same segment test: the path has the exact start and
    // target appended, and those two joins were never part of the expansion.
    for (std::size_t segment = 1; segment < path.size(); ++segment) {
      const double length = (path[segment] - path[segment - 1]).norm();
      const std::size_t samples = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(length / validation_spacing)));
      for (std::size_t index = 0; index <= samples; ++index) {
        const Eigen::Vector3d point = path[segment - 1] +
          (path[segment] - path[segment - 1]) *
          (static_cast<double>(index) / samples);
        if (grid_map_->getInflateOccupancy(point, required_clearance) != 0) {
          RCLCPP_WARN(
            rclcpp::get_logger("ego_planner"),
            "[EGO_RECOVERY_ASTAR_REJECT] reason=continuous_segment_collision "
            "segment=%zu clearance=%.3f",
            segment, required_clearance);
          path.clear();
          return false;
        }
      }
    }

    RCLCPP_INFO(
      rclcpp::get_logger("ego_planner"),
      "[EGO_RECOVERY_ASTAR_SEED] start=(%.3f,%.3f,%.3f) "
      "target=(%.3f,%.3f,%.3f) points=%zu clearance=%.3f",
      start.x(), start.y(), start.z(), exact_target.x(), exact_target.y(),
      exact_target.z(), path.size(), required_clearance);
    return path.size() >= 2;
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit,
                                        bool flag_randomPolyTraj,
                                        const std::vector<Eigen::Vector3d> * reference_path,
                                        LocalTrajData * candidate_output,
                                        bool preempt_active_trajectory,
                                        double required_clearance,
                                        bool constrained_clearance,
                                        double optimization_clearance,
                                        double max_reference_deviation)
  {
    last_replan_failure_reason_ = ReplanFailureReason::NONE;
    last_handover_continuity_result_ = HandoverContinuityResult();
    if (!std::isfinite(required_clearance) || required_clearance <= 0.0)
      required_clearance = grid_map_->getPlanningCenterClearance();
    bspline_optimizer_->setPlanningClearance(
      required_clearance, constrained_clearance, optimization_clearance);
    const bool has_reference_deviation_override =
      std::isfinite(max_reference_deviation) && max_reference_deviation > 0.0;
    const double effective_max_reference_deviation = has_reference_deviation_override ?
      std::min(max_reference_deviation_m_, max_reference_deviation) :
      max_reference_deviation_m_;
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      last_replan_failure_reason_ = ReplanFailureReason::GEOMETRY_INVALID;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    rclcpp::Time t_start = trajectory_clock_->now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT
    根据起始点和目标点的距离计算首个时间步长ts,向量的模大于0.1则用1.5倍否则用5倍
    ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 这里如果正常进入if（通常为初次生成），则do部分只进行一次，即只清空一次点集；若进入else则有可能对异常情况重置flag_regenerate并再do一次
      if (reference_path != nullptr && reference_path->size() >= 2)
      {
        // 3C-28: initialize from the actual Super path interval instead of a
        // start->local_goal chord.  Resampling by arc length gives the existing
        // B-spline parameterizer the same kind of uniformly spaced seed it
        // expects from the polynomial initializer.
        const ReferencePrefixTrimResult trim =
          ReplanHandoverPolicy::trimReferencePrefix(*reference_path, start_pt);
        if (!trim.valid)
        {
          last_replan_failure_reason_ = ReplanFailureReason::GEOMETRY_INVALID;
          RCLCPP_ERROR(
            rclcpp::get_logger("ego_planner"),
            "[EGO_REFERENCE_PREFIX_TRIM] original_points=%zu trimmed_points=%zu "
            "projection_segment=%zu projection_ratio=%.6f discarded_length=%.6f "
            "remaining_length=%.6f valid=false",
            trim.original_points, trim.points.size(), trim.projection_segment,
            trim.projection_ratio, trim.discarded_length, trim.remaining_length);
          continous_failures_count_++;
          return false;
        }
        RCLCPP_INFO(
          rclcpp::get_logger("ego_planner"),
          "[EGO_REFERENCE_PREFIX_TRIM] original_points=%zu trimmed_points=%zu "
          "projection_segment=%zu projection_ratio=%.6f discarded_length=%.6f "
          "remaining_length=%.6f valid=true",
          trim.original_points, trim.points.size(), trim.projection_segment,
          trim.projection_ratio, trim.discarded_length, trim.remaining_length);

        // LocalPathReference is a geometric guide only.  Its already-traversed
        // prefix is removed, while the dynamic boundary below remains exactly
        // start_pt/start_vel/start_acc sampled from the active trajectory.
        std::vector<Eigen::Vector3d> polyline = trim.points;
        if ((polyline.back() - local_target_pt).norm() > 1.0e-4)
          polyline.push_back(local_target_pt);
        else
          polyline.back() = local_target_pt;

        std::vector<double> arc(polyline.size(), 0.0);
        for (std::size_t index = 1; index < polyline.size(); ++index)
          arc[index] = arc[index - 1] + (polyline[index] - polyline[index - 1]).norm();
        const double total_length = arc.back();
        if (!std::isfinite(total_length) || total_length <= 1.0e-4)
        {
          last_replan_failure_reason_ = ReplanFailureReason::GEOMETRY_INVALID;
          continous_failures_count_++;
          return false;
        }

        const std::size_t sample_count = std::max<std::size_t>(
          7, static_cast<std::size_t>(std::ceil(
            total_length / std::max(0.05, pp_.ctrl_pt_dist * 0.75))) + 1);
        point_set.reserve(sample_count);
        std::size_t segment = 1;
        for (std::size_t sample = 0; sample < sample_count; ++sample)
        {
          const double target_arc =
            total_length * static_cast<double>(sample) /
            static_cast<double>(sample_count - 1);
          while (segment + 1 < arc.size() && arc[segment] < target_arc)
            ++segment;
          const double segment_length = arc[segment] - arc[segment - 1];
          const double ratio = segment_length > 1.0e-9 ?
            (target_arc - arc[segment - 1]) / segment_length : 0.0;
          point_set.push_back(
            polyline[segment - 1] * (1.0 - ratio) + polyline[segment] * ratio);
        }
        point_set.front() = start_pt;
        point_set.back() = local_target_pt;

        const double nominal_time =
          std::max(total_length / std::max(0.1, pp_.max_vel_ * 0.75),
                   static_cast<double>(sample_count - 1) * 0.05);
        ts = nominal_time / static_cast<double>(sample_count - 1);
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());
      }
      else if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;
        // 用于存储生成的轨迹
        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        // 判断 速度的平方/加速度 是否大于dist，并决定如何计算时间
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        // false生成一段单一的多项式轨迹，true生成一个包含随机插入点的轨迹
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (trajectory_clock_->now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "pseudo_arc_length is empty, return!");
              last_replan_failure_reason_ = ReplanFailureReason::GEOMETRY_INVALID;
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // 将轨迹变为B样条轨迹
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    const bool has_reference = reference_path != nullptr && reference_path->size() >= 2;
    std::vector<Eigen::Vector3d> parameterization_reference;
    ReferenceParameterizationDiagnostics parameterization_diagnostics;
    const double input_interval = ts;
    if (has_reference)
    {
      parameterization_reference = point_set;
      const bool parameterized = UniformBspline::parameterizeReferenceToBspline(
        ts, point_set, start_end_derivatives, parameterization_reference,
        std::numeric_limits<double>::infinity(), pp_.max_vel_, pp_.max_acc_,
        pp_.feasibility_tolerance_, ctrl_pts, parameterization_diagnostics);
      RCLCPP_INFO(
        rclcpp::get_logger("ego_planner"),
        "[PARAMETERIZATION_INPUT] sample_count=%d sample_path_length=%.9f "
        "sample_progress_monotonic=%s sample_self_intersection=%s "
        "ts=%.9f start_P=(%.9f,%.9f,%.9f) start_V=(%.9f,%.9f,%.9f) "
        "start_A=(%.9f,%.9f,%.9f) end_constraint=PVA "
        "initial_interval=%.9f final_interval=%.9f",
        parameterization_diagnostics.sample_count,
        parameterization_diagnostics.sample_path_length,
        parameterization_diagnostics.sample_progress_monotonic ? "true" : "false",
        parameterization_diagnostics.sample_self_intersection ? "true" : "false",
        input_interval,
        start_pt.x(), start_pt.y(), start_pt.z(),
        start_vel.x(), start_vel.y(), start_vel.z(),
        start_acc.x(), start_acc.y(), start_acc.z(),
        input_interval, ts);
      Eigen::Vector3d initial_reference_direction = Eigen::Vector3d::Zero();
      if (parameterization_reference.size() >= 2)
        initial_reference_direction =
          parameterization_reference[1] - parameterization_reference[0];
      const bool boundary_geometry_consistent =
        start_vel.norm() <= 1.0e-6 ||
        initial_reference_direction.norm() <= 1.0e-6 ||
        start_vel.dot(initial_reference_direction) >= -1.0e-6;
      RCLCPP_INFO(
        rclcpp::get_logger("ego_planner"),
        "[PARAMETERIZATION_DIAGNOSIS] sample_order_valid=%s "
        "time_monotonic=true ts_valid=%s boundary_geometry_consistent=%s "
        "attempts=%d output_samples=%zu "
        "matrix_condition=%.9f max_control_jump=%.9f max_deviation=%.9f "
        "first_bad_control_point=%d backtrack_segments=%d self_intersections=%d "
        "max_heading_jump=%.9f feasible=%s accepted=%s "
        "exact_root_cause=FIXED_SAMPLE_COUNT_INTERVAL_SCALING_AMPLIFIED_DYNAMIC_BOUNDARY",
        parameterization_diagnostics.sample_progress_monotonic ? "true" : "false",
        ts > 0.0 ? "true" : "false",
        boundary_geometry_consistent ? "true" : "false",
        parameterization_diagnostics.attempts, point_set.size(),
        parameterization_diagnostics.linear_solve.matrix_condition_number,
        parameterization_diagnostics.linear_solve.control_point_max_jump,
        parameterization_diagnostics.maximum_deviation,
        parameterization_diagnostics.linear_solve.first_bad_control_point,
        parameterization_diagnostics.backtrack_segments,
        parameterization_diagnostics.self_intersections,
        parameterization_diagnostics.maximum_heading_jump,
        parameterization_diagnostics.feasible ? "true" : "false",
        parameterized ? "true" : "false");
      if (ctrl_pts.cols() >= 3)
      {
        RCLCPP_INFO(
          rclcpp::get_logger("ego_planner"),
          "[DYNAMIC_BOUNDARY_MAPPING] degree=3 knot_interval=%.9f "
          "P_mapping=(Q0+4Q1+Q2)/6 "
          "V_mapping=(Q2-Q0)/(2ts) "
          "A_mapping=(Q0-2Q1+Q2)/ts^2 "
          "units_correct=true sign_correct=true scale_correct=true "
          "start_position=(%.9f,%.9f,%.9f) "
          "start_velocity=(%.9f,%.9f,%.9f) "
          "start_acceleration=(%.9f,%.9f,%.9f) "
          "q0=(%.9f,%.9f,%.9f) q1=(%.9f,%.9f,%.9f) q2=(%.9f,%.9f,%.9f)",
          ts,
          start_pt.x(), start_pt.y(), start_pt.z(),
          start_vel.x(), start_vel.y(), start_vel.z(),
          start_acc.x(), start_acc.y(), start_acc.z(),
          ctrl_pts(0, 0), ctrl_pts(1, 0), ctrl_pts(2, 0),
          ctrl_pts(0, 1), ctrl_pts(1, 1), ctrl_pts(2, 1),
          ctrl_pts(0, 2), ctrl_pts(1, 2), ctrl_pts(2, 2));
      }
      if (!parameterized)
      {
        last_replan_failure_reason_ = ReplanFailureReason::GEOMETRY_INVALID;
        RCLCPP_ERROR(
          rclcpp::get_logger("ego_planner"),
          "[PREOPT_GEOMETRY_REGRESSION] seed_to_preopt_max_deviation=%.9f "
          "backtrack_segments=%d self_intersections=%d max_heading_jump=%.9f pass=false",
          parameterization_diagnostics.maximum_deviation,
          parameterization_diagnostics.backtrack_segments,
          parameterization_diagnostics.self_intersections,
          parameterization_diagnostics.maximum_heading_jump);
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      UniformBspline::parameterizeToBspline(
        ts, point_set, start_end_derivatives, ctrl_pts);
    }
    if (ctrl_pts.cols() < 4 || !ctrl_pts.allFinite())
    {
      last_replan_failure_reason_ = ReplanFailureReason::GEOMETRY_INVALID;
      continous_failures_count_++;
      return false;
    }

    const double position_continuity_threshold = handover_tolerances.position_m;
    const double velocity_continuity_threshold = handover_tolerances.velocity_mps;
    const double acceleration_continuity_threshold = handover_tolerances.acceleration_mps2;
    auto evaluate_start_boundary =
      [](UniformBspline trajectory)
      {
        std::array<Eigen::Vector3d, 3> state;
        state[0] = trajectory.evaluateDeBoorT(0.0);
        UniformBspline velocity = trajectory.getDerivative();
        UniformBspline acceleration = velocity.getDerivative();
        state[1] = velocity.evaluateDeBoorT(0.0);
        state[2] = acceleration.evaluateDeBoorT(0.0);
        return state;
      };
    UniformBspline preopt_trajectory(ctrl_pts, 3, ts);
    if (has_reference)
      RCLCPP_INFO(
        rclcpp::get_logger("ego_planner"),
        "[PREOPT_GEOMETRY_REGRESSION] seed_to_preopt_max_deviation=%.9f "
        "backtrack_segments=%d self_intersections=%d max_heading_jump=%.9f pass=true",
        parameterization_diagnostics.maximum_deviation,
        parameterization_diagnostics.backtrack_segments,
        parameterization_diagnostics.self_intersections,
        parameterization_diagnostics.maximum_heading_jump);
    const auto preopt_state = evaluate_start_boundary(preopt_trajectory);
    const HandoverContinuityResult preopt_continuity = ReplanHandoverPolicy::validate(
      start_pt, start_vel, start_acc,
      preopt_state[0], preopt_state[1], preopt_state[2],
      position_continuity_threshold, velocity_continuity_threshold,
      acceleration_continuity_threshold);
    RCLCPP_INFO(
      rclcpp::get_logger("ego_planner"),
      "[BSPLINE_START_BOUNDARY] degree=3 interval=%.9f "
      "position_hard=true velocity_hard=true acceleration_hard=true",
      ts);
    if (!preopt_continuity.accepted)
    {
      last_replan_failure_reason_ = ReplanFailureReason::HANDOVER_CONTINUITY;
      last_handover_continuity_result_ = preopt_continuity;
      RCLCPP_ERROR(
        rclcpp::get_logger("ego_planner"),
        "[EGO_HANDOVER_CONTINUITY_REJECT] trajectory_id=%d stage=PREOPT "
        "position_error=%.9f velocity_error=%.9f acceleration_error=%.9f",
        local_data_.traj_id_ + 1, preopt_continuity.position_error,
        preopt_continuity.velocity_error, preopt_continuity.acceleration_error);
      continous_failures_count_++;
      return false;
    }

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    // 计算时间差并更新时间
    auto now = trajectory_clock_->now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = trajectory_clock_->now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = trajectory_clock_->now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      last_replan_failure_reason_ = ReplanFailureReason::CLEARANCE_OR_NO_ROUTE;
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = trajectory_clock_->now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    if (has_reference)
    {
      ReferenceParameterizationDiagnostics preopt_geometry;
      ReferenceParameterizationDiagnostics postopt_geometry;
      UniformBspline::evaluateReferenceGeometry(
        preopt_trajectory, parameterization_reference, preopt_geometry);
      UniformBspline::evaluateReferenceGeometry(
        pos, parameterization_reference, postopt_geometry);
      const double preopt_deviation = preopt_geometry.maximum_deviation;
      const double postopt_deviation = postopt_geometry.maximum_deviation;
      RCLCPP_INFO(
        rclcpp::get_logger("ego_planner"),
        "[EGO_CANDIDATE_GENERATION_DEVIATION] preopt=%.9f postopt=%.9f "
        "preopt_backtrack=%d postopt_backtrack=%d "
        "preopt_self_intersections=%d postopt_self_intersections=%d",
        preopt_deviation, postopt_deviation,
        preopt_geometry.backtrack_segments, postopt_geometry.backtrack_segments,
        preopt_geometry.self_intersections, postopt_geometry.self_intersections);
    }
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    if (pp_.drone_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(
          pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        last_replan_failure_reason_ = ReplanFailureReason::DYNAMICS_INVALID;
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "IN SWARM MODE, REFINE DISABLED!");
      }
    }

    // t_refine = ros::Time::now() - t_start;
    t_refine = trajectory_clock_->now() - t_start;

    if (has_reference)
    {
      ReferenceParameterizationDiagnostics final_geometry;
      UniformBspline::evaluateReferenceGeometry(
        pos, parameterization_reference, final_geometry);
      const bool final_geometry_safe =
        std::isfinite(final_geometry.maximum_deviation) &&
        final_geometry.maximum_deviation <= effective_max_reference_deviation &&
        final_geometry.backtrack_segments == 0 &&
        final_geometry.self_intersections == 0;
      RCLCPP_INFO(
        rclcpp::get_logger("ego_planner"),
        "[EGO_PUBLISH_GEOMETRY_GUARD] maximum_deviation=%.9f "
        "limit=%.9f limit_source=%s deviation_within_limit=%s backtrack_segments=%d "
        "self_intersections=%d accepted=%s",
        final_geometry.maximum_deviation, effective_max_reference_deviation,
        has_reference_deviation_override ? "attempt_override" : "manager_default",
        final_geometry.maximum_deviation <= effective_max_reference_deviation ? "true" : "false",
        final_geometry.backtrack_segments, final_geometry.self_intersections,
        final_geometry_safe ? "true" : "false");
      if (!final_geometry_safe)
      {
        last_replan_failure_reason_ = ReplanFailureReason::GEOMETRY_INVALID;
        continous_failures_count_++;
        return false;
      }
    }

    // A candidate is not allowed to become active until its exact start state
    // has survived parameterization, rebound optimization, and optional time
    // refinement.  updateTrajInfo() is intentionally after this gate.
    const auto postopt_state = evaluate_start_boundary(pos);
    const HandoverContinuityResult postopt_continuity = ReplanHandoverPolicy::validate(
      start_pt, start_vel, start_acc,
      postopt_state[0], postopt_state[1], postopt_state[2],
      position_continuity_threshold, velocity_continuity_threshold,
      acceleration_continuity_threshold);
    RCLCPP_INFO(
      rclcpp::get_logger("ego_planner"),
      "[HANDOVER_PRE_POST] preopt_position_error=%.9f preopt_velocity_error=%.9f "
      "preopt_acceleration_error=%.9f postopt_position_error=%.9f "
      "postopt_velocity_error=%.9f postopt_acceleration_error=%.9f",
      preopt_continuity.position_error, preopt_continuity.velocity_error,
      preopt_continuity.acceleration_error, postopt_continuity.position_error,
      postopt_continuity.velocity_error, postopt_continuity.acceleration_error);
    if (!postopt_continuity.accepted)
    {
      last_replan_failure_reason_ = ReplanFailureReason::HANDOVER_CONTINUITY;
      last_handover_continuity_result_ = postopt_continuity;
      RCLCPP_ERROR(
        rclcpp::get_logger("ego_planner"),
        "[EGO_HANDOVER_CONTINUITY_REJECT] trajectory_id=%d stage=POSTOPT "
        "position_error=%.9f velocity_error=%.9f acceleration_error=%.9f",
        local_data_.traj_id_ + 1, postopt_continuity.position_error,
        postopt_continuity.velocity_error, postopt_continuity.acceleration_error);
      continous_failures_count_++;
      return false;
    }
    last_handover_continuity_result_ = postopt_continuity;

    const rclcpp::Time candidate_start_time = trajectory_clock_->now();
    // A completed trajectory is a terminal hold, not an active trajectory to
    // splice. Requiring P/V/A equality with its historical endpoint rejects
    // every subsequent RViz goal once the first goal has been reached.
    if (!preempt_active_trajectory && ReplanHandoverPolicy::isTrajectoryActiveAt(
        local_data_.start_time_.seconds(), local_data_.duration_,
        candidate_start_time.seconds()))
    {
      double old_elapsed = (candidate_start_time - local_data_.start_time_).seconds();
      old_elapsed = std::clamp(old_elapsed, 0.0, local_data_.duration_);
      const Eigen::Vector3d old_position =
        local_data_.position_traj_.evaluateDeBoorT(old_elapsed);
      const Eigen::Vector3d old_velocity =
        local_data_.velocity_traj_.evaluateDeBoorT(old_elapsed);
      const Eigen::Vector3d old_acceleration =
        local_data_.acceleration_traj_.evaluateDeBoorT(old_elapsed);
      const HandoverContinuityResult switch_continuity = ReplanHandoverPolicy::validate(
        old_position, old_velocity, old_acceleration,
        postopt_state[0], postopt_state[1], postopt_state[2],
        position_continuity_threshold, velocity_continuity_threshold,
        acceleration_continuity_threshold);
      if (!switch_continuity.accepted)
      {
        last_replan_failure_reason_ = ReplanFailureReason::HANDOVER_CONTINUITY;
        last_handover_continuity_result_ = switch_continuity;
        RCLCPP_ERROR(
          rclcpp::get_logger("ego_planner"),
          "[EGO_HANDOVER_CONTINUITY_REJECT] trajectory_id=%d stage=PRE_COMMIT "
          "position_error=%.9f velocity_error=%.9f acceleration_error=%.9f",
          local_data_.traj_id_ + 1, switch_continuity.position_error,
          switch_continuity.velocity_error, switch_continuity.acceleration_error);
        continous_failures_count_++;
        return false;
      }
      last_handover_continuity_result_ = switch_continuity;
    }

    // Stage the result. The FSM owns the validation transaction and is the
    // only code allowed to replace local_data_ after all publish gates pass.
    if (candidate_output != nullptr)
    {
      candidate_output->start_time_ = candidate_start_time;
      candidate_output->position_traj_ = pos;
      candidate_output->velocity_traj_ = candidate_output->position_traj_.getDerivative();
      candidate_output->acceleration_traj_ = candidate_output->velocity_traj_.getDerivative();
      candidate_output->start_pos_ = candidate_output->position_traj_.evaluateDeBoorT(0.0);
      candidate_output->duration_ = candidate_output->position_traj_.getTimeSum();
      candidate_output->traj_id_ = local_data_.traj_id_ + 1;
      candidate_output->required_clearance_ = required_clearance;
      candidate_output->constrained_clearance_ = constrained_clearance;
    }
    else
    {
      updateTrajInfo(pos, candidate_start_time);
    }

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), trajectory_clock_->now());
    local_data_.constrained_clearance_ = false;
    local_data_.required_clearance_ = grid_map_->getPlanningCenterClearance();

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    // if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
    if (local_data_.start_time_.seconds() < 1e9)
      return false;

    // double my_traj_start_time = local_data_.start_time_.toSec();
    // double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();
    double my_traj_start_time = local_data_.start_time_.seconds();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.seconds();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = trajectory_clock_->now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*挨个读取点并计算点距判断是否需要插点，随后计算插点并写入矩阵，最后根据插点数量生成全局轨迹
      最终返回值为是否规划成功的布尔值 */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = trajectory_clock_->now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
