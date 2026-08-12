#include <gtest/gtest.h>

#include <bspline_opt/uniform_bspline.h>

namespace
{

TEST(UniformBsplineStartBoundary, HistoricalSixToSevenStateIsExact)
{
  const Eigen::Vector3d position(0.49741, 0.08721, 0.78000);
  const Eigen::Vector3d velocity(0.28470, -0.01543, 0.0);
  const Eigen::Vector3d acceleration(0.21296, -0.00970, 0.0);
  const double interval = 0.18;
  std::vector<Eigen::Vector3d> points{
    position,
    {0.55, 0.085, 0.78},
    {0.63, 0.080, 0.78},
    {0.72, 0.070, 0.78},
    {0.82, 0.055, 0.78},
    {0.93, 0.035, 0.78},
    {1.04, 0.010, 0.78}};
  std::vector<Eigen::Vector3d> derivatives{
    velocity, Eigen::Vector3d::Zero(), acceleration, Eigen::Vector3d::Zero()};

  Eigen::MatrixXd controls;
  ego_planner::UniformBspline::parameterizeToBspline(
    interval, points, derivatives, controls);
  ASSERT_EQ(controls.rows(), 3);
  ASSERT_EQ(controls.cols(), static_cast<int>(points.size()) + 2);

  ego_planner::UniformBspline trajectory(controls, 3, interval);
  ego_planner::UniformBspline velocity_trajectory = trajectory.getDerivative();
  ego_planner::UniformBspline acceleration_trajectory = velocity_trajectory.getDerivative();
  const Eigen::Vector3d actual_position = trajectory.evaluateDeBoorT(0.0);
  const Eigen::Vector3d actual_velocity = velocity_trajectory.evaluateDeBoorT(0.0);
  const Eigen::Vector3d actual_acceleration = acceleration_trajectory.evaluateDeBoorT(0.0);

  EXPECT_LE((actual_position - position).norm(), 1.0e-12);
  EXPECT_LE((actual_velocity - velocity).norm(), 1.0e-12);
  EXPECT_LE((actual_acceleration - acceleration).norm(), 1.0e-11);

  const Eigen::Vector3d historical_bad_position(0.42222, 0.08890, 0.78000);
  const double before_position_error = (historical_bad_position - position).head<2>().norm();
  const double after_position_error = (actual_position - position).head<2>().norm();
  EXPECT_NEAR(before_position_error, 0.07520899, 1.0e-6);
  EXPECT_LE(after_position_error, 0.001);
  EXPECT_LE((actual_velocity - velocity).norm(), 0.001);
  EXPECT_LE((actual_acceleration - acceleration).norm(), 0.005);
  std::cout
    << "[HISTORICAL_6_TO_7_REGRESSION] "
    << "before_position_error=" << before_position_error << " "
    << "after_position_error=" << after_position_error << " "
    << "before_wall_jump=0.03172 "
    << "after_wall_jump=0 "
    << "after_velocity_error=" << (actual_velocity - velocity).norm() << " "
    << "after_acceleration_error=" << (actual_acceleration - acceleration).norm()
    << std::endl;
}

TEST(UniformBsplineStartBoundary, CurvedGeometricSamplesCannotMoveDynamicStart)
{
  const Eigen::Vector3d position(0.2, -0.1, 0.78);
  const Eigen::Vector3d velocity(0.3, 0.04, 0.0);
  const Eigen::Vector3d acceleration(-0.1, 0.08, 0.0);
  std::vector<Eigen::Vector3d> points{
    position,
    {0.24, 0.05, 0.78},
    {0.35, 0.20, 0.78},
    {0.55, 0.25, 0.78},
    {0.75, 0.15, 0.78},
    {0.90, 0.02, 0.78},
    {1.00, 0.00, 0.78}};
  std::vector<Eigen::Vector3d> derivatives{
    velocity, Eigen::Vector3d::Zero(), acceleration, Eigen::Vector3d::Zero()};

  Eigen::MatrixXd controls;
  ego_planner::UniformBspline::parameterizeToBspline(0.2, points, derivatives, controls);
  ego_planner::UniformBspline trajectory(controls, 3, 0.2);
  ego_planner::UniformBspline d1 = trajectory.getDerivative();
  ego_planner::UniformBspline d2 = d1.getDerivative();

  EXPECT_LE((trajectory.evaluateDeBoorT(0.0) - position).norm(), 1.0e-12);
  EXPECT_LE((d1.evaluateDeBoorT(0.0) - velocity).norm(), 1.0e-12);
  EXPECT_LE((d2.evaluateDeBoorT(0.0) - acceleration).norm(), 1.0e-11);
}

TEST(UniformBsplineStartBoundary, ReferenceDurationUsesMoreSamplesWithoutGeometricFold)
{
  const Eigen::Vector3d position(0.0, 0.0, 0.78);
  const Eigen::Vector3d velocity(0.10, -0.10, 0.0);
  const Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> reference{
    position,
    {0.20, 0.0, 0.78},
    {0.40, 0.0, 0.78},
    {0.60, 0.0, 0.78},
    {0.80, 0.0, 0.78},
    {1.00, 0.0, 0.78},
    {1.20, 0.0, 0.78}};
  const std::vector<Eigen::Vector3d> derivatives{
    velocity, Eigen::Vector3d::Zero(), acceleration, Eigen::Vector3d::Zero()};

  const double historical_interval = 1.473454967;
  Eigen::MatrixXd historical_controls;
  ego_planner::UniformBspline::parameterizeToBspline(
    historical_interval, reference, derivatives, historical_controls);
  ego_planner::UniformBspline historical(
    historical_controls, 3, historical_interval);
  ego_planner::ReferenceParameterizationDiagnostics historical_geometry;
  ego_planner::UniformBspline::evaluateReferenceGeometry(
    historical, reference, historical_geometry);
  EXPECT_GT(historical_geometry.maximum_deviation, 0.05);

  double interval = historical_interval;
  std::vector<Eigen::Vector3d> samples = reference;
  Eigen::MatrixXd controls;
  ego_planner::ReferenceParameterizationDiagnostics diagnostics;
  const bool accepted =
    ego_planner::UniformBspline::parameterizeReferenceToBspline(
      interval, samples, derivatives, reference,
      0.05, 0.60, 0.80, 0.05, controls, diagnostics);
  std::cout
    << "[REFERENCE_PARAMETERIZATION_DIAGNOSTICS] accepted=" << accepted
    << " attempts=" << diagnostics.attempts
    << " interval=" << interval
    << " samples=" << samples.size()
    << " deviation=" << diagnostics.maximum_deviation
    << " backtrack=" << diagnostics.backtrack_segments
    << " intersections=" << diagnostics.self_intersections
    << " feasible=" << diagnostics.feasible
    << std::endl;
  if (!accepted && controls.cols() >= 4)
  {
    ego_planner::UniformBspline failed(controls, 3, interval);
    failed.setPhysicalLimits(0.60, 0.80, 0.05);
    double failed_ratio = 1.0;
    failed.checkFeasibility(failed_ratio, true);
    std::cout << "[REFERENCE_PARAMETERIZATION_FAILED_RATIO] ratio="
              << failed_ratio << std::endl;
  }
  ASSERT_TRUE(accepted);
  EXPECT_LT(interval, historical_interval);
  EXPECT_GT(samples.size(), reference.size());
  EXPECT_LE(diagnostics.maximum_deviation, 0.05);
  EXPECT_EQ(diagnostics.backtrack_segments, 0);
  EXPECT_EQ(diagnostics.self_intersections, 0);
  EXPECT_TRUE(diagnostics.feasible);

  ego_planner::UniformBspline trajectory(controls, 3, interval);
  ego_planner::UniformBspline d1 = trajectory.getDerivative();
  ego_planner::UniformBspline d2 = d1.getDerivative();
  EXPECT_LE((trajectory.evaluateDeBoorT(0.0) - position).norm(), 0.001);
  EXPECT_LE((d1.evaluateDeBoorT(0.0) - velocity).norm(), 0.001);
  EXPECT_LE((d2.evaluateDeBoorT(0.0) - acceleration).norm(), 0.005);
  std::cout
    << "[REFERENCE_PARAMETERIZATION_REGRESSION] "
    << "old_interval=" << historical_interval << " "
    << "new_interval=" << interval << " "
    << "old_max_deviation=" << historical_geometry.maximum_deviation << " "
    << "new_max_deviation=" << diagnostics.maximum_deviation << " "
    << "samples=" << samples.size() << " "
    << "backtrack=" << diagnostics.backtrack_segments << " "
    << "self_intersections=" << diagnostics.self_intersections
    << std::endl;
}

TEST(UniformBsplineStartBoundary, FirstTrajectoryReferenceRemainsMonotonic)
{
  std::vector<Eigen::Vector3d> reference{
    {0.0, 0.0, 0.78},
    {0.15, 0.01, 0.78},
    {0.30, 0.02, 0.78},
    {0.45, 0.025, 0.78},
    {0.60, 0.025, 0.78},
    {0.75, 0.02, 0.78},
    {0.90, 0.01, 0.78}};
  const Eigen::Vector3d direction = (reference[1] - reference[0]).normalized();
  const std::vector<Eigen::Vector3d> derivatives{
    0.20 * direction, Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  double interval = 0.50;
  std::vector<Eigen::Vector3d> samples = reference;
  Eigen::MatrixXd controls;
  ego_planner::ReferenceParameterizationDiagnostics diagnostics;
  const bool accepted = ego_planner::UniformBspline::parameterizeReferenceToBspline(
    interval, samples, derivatives, reference,
    0.05, 0.60, 0.80, 0.05, controls, diagnostics);
  std::cout << "[FIRST_TRAJECTORY_TEST] accepted=" << accepted
            << " interval=" << interval
            << " attempts=" << diagnostics.attempts
            << " deviation=" << diagnostics.maximum_deviation
            << " backtrack=" << diagnostics.backtrack_segments
            << " intersections=" << diagnostics.self_intersections
            << " feasible=" << diagnostics.feasible << std::endl;
  ASSERT_TRUE(accepted);
  EXPECT_EQ(diagnostics.backtrack_segments, 0);
  EXPECT_EQ(diagnostics.self_intersections, 0);
  EXPECT_LE(diagnostics.maximum_deviation, 0.05);
}

TEST(UniformBsplineStartBoundary, MovingReplanFindsBoundedFeasibleKnotInterval)
{
  // Captured from 20260804_194835. Inflating the 0.374 s knot interval to
  // 1.586 s moved the boundary controls 0.79 m away from this reference.
  const Eigen::Vector3d start(9.394837378, 7.680641804, 0.780);
  const Eigen::Vector3d finish(8.373, 8.377, 0.780);
  std::vector<Eigen::Vector3d> reference;
  for (int index = 0; index < 7; ++index)
    reference.push_back(start + (finish - start) * (static_cast<double>(index) / 6.0));
  const std::vector<Eigen::Vector3d> derivatives{
    {-0.010523558, 0.354686302, 0.0}, Eigen::Vector3d::Zero(),
    {0.088639315, 0.072423481, 0.0}, Eigen::Vector3d::Zero()};

  constexpr double initial_interval = 0.374449790;
  double interval = initial_interval;
  std::vector<Eigen::Vector3d> samples = reference;
  Eigen::MatrixXd controls;
  ego_planner::ReferenceParameterizationDiagnostics diagnostics;
  const bool accepted = ego_planner::UniformBspline::parameterizeReferenceToBspline(
    interval, samples, derivatives, reference,
    0.75, 0.60, 0.80, 0.05, controls, diagnostics);

  std::cout << "[MOVING_REPLAN_PARAMETERIZATION] accepted=" << accepted
            << " interval=" << interval
            << " attempts=" << diagnostics.attempts
            << " samples=" << samples.size()
            << " deviation=" << diagnostics.maximum_deviation
            << " feasible=" << diagnostics.feasible << std::endl;
  ASSERT_TRUE(accepted);
  EXPECT_EQ(samples.size(), reference.size());
  EXPECT_GT(interval, initial_interval);
  EXPECT_LT(interval, 1.50);
  EXPECT_LE(diagnostics.maximum_deviation, 0.75);
  EXPECT_TRUE(diagnostics.feasible);
}

}  // namespace
