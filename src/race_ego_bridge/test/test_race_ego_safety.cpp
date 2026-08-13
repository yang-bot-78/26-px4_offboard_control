#include <limits>

#include <bspline_opt/uniform_bspline.h>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "plan_env/raw_obstacle_distance_index.h"
#include "race_ego_bridge/frame_utils.hpp"
#include "race_ego_bridge/safety_utils.hpp"

using race_ego_bridge::Vec3;

TEST(Coordinates, NedMapRoundTrip)
{
  const Vec3 input{1.2, -2.3, -0.78};
  const auto output = race_ego_bridge::mapToNed(race_ego_bridge::nedToMap(input));
  EXPECT_DOUBLE_EQ(input.x, output.x);
  EXPECT_DOUBLE_EQ(input.y, output.y);
  EXPECT_DOUBLE_EQ(input.z, output.z);
}

TEST(Coordinates, HeightSign)
{
  EXPECT_DOUBLE_EQ(-0.78, race_ego_bridge::mapToNed({0.0, 0.0, 0.78}).z);
}

TEST(Coordinates, MapAxesUseStandardRosEnuConvention)
{
  // Standard ROS ENU -> PX4 NED is (y, x, -z).
  const auto x = race_ego_bridge::mapToNed({1.0, 0.0, 0.0});
  const auto y = race_ego_bridge::mapToNed({0.0, 1.0, 0.0});
  const auto z = race_ego_bridge::mapToNed({0.0, 0.0, 1.0});
  EXPECT_DOUBLE_EQ(0.0, x.x);
  EXPECT_DOUBLE_EQ(1.0, x.y);
  EXPECT_DOUBLE_EQ(1.0, y.x);
  EXPECT_DOUBLE_EQ(0.0, y.y);
  EXPECT_DOUBLE_EQ(-1.0, z.z);
}

TEST(Coordinates, MapAndEnuAreTheSameAlignedFrame)
{
  const Vec3 input{1.2, -2.3, 0.78};
  const auto output = race_ego_bridge::mapToEnu(
    race_ego_bridge::enuToMap(input));
  EXPECT_DOUBLE_EQ(input.x, output.x);
  EXPECT_DOUBLE_EQ(input.y, output.y);
  EXPECT_DOUBLE_EQ(input.z, output.z);
  EXPECT_NEAR(
    0.73, race_ego_bridge::mapYawToEnu(
      race_ego_bridge::enuYawToMap(0.73)), 1e-9);
}

TEST(Coordinates, YawRoundTrip)
{
  const double yaw = 0.73;
  EXPECT_NEAR(yaw, race_ego_bridge::mapYawToNed(race_ego_bridge::nedYawToMap(yaw)), 1e-9);
}

TEST(Coordinates, TimedPoseInterpolatesPositionAndShortestYaw)
{
  const race_ego_bridge::TimedYawPose first{
    10.0, {0.0, 1.0, 0.70}, M_PI - 0.10};
  const race_ego_bridge::TimedYawPose second{
    12.0, {2.0, 3.0, 0.90}, -M_PI + 0.10};
  const auto middle = race_ego_bridge::interpolateYawPose(first, second, 11.0);
  EXPECT_DOUBLE_EQ(1.0, middle.position.x);
  EXPECT_DOUBLE_EQ(2.0, middle.position.y);
  EXPECT_DOUBLE_EQ(0.80, middle.position.z);
  EXPECT_NEAR(M_PI, std::abs(middle.yaw), 1e-9);
}

TEST(Frames, GoalFrame)
{
  EXPECT_TRUE(race_ego_bridge::frameMatches("map", "map"));
  EXPECT_FALSE(race_ego_bridge::frameMatches("camera_init", "map"));
}

TEST(Frames, PointCloudFrame)
{
  EXPECT_FALSE(race_ego_bridge::frameMatches("", "map"));
}

TEST(Trajectory, StartAndEndTime)
{
  EXPECT_DOUBLE_EQ(0.0, race_ego_bridge::clampTrajectoryTime(-0.1, 2.0));
  EXPECT_DOUBLE_EQ(2.0, race_ego_bridge::clampTrajectoryTime(2.5, 2.0));
}

TEST(Trajectory, MeasuredHoldEvaluatesReplacementFromItsStart)
{
  EXPECT_DOUBLE_EQ(
    0.0, race_ego_bridge::trajectorySwitchEvaluationTime(0.42, 2.0, true));
  EXPECT_DOUBLE_EQ(
    0.42, race_ego_bridge::trajectorySwitchEvaluationTime(0.42, 2.0, false));
}

TEST(TakeoffHandover, ResetsOnlyOnDisarmNotOnLowAltitude)
{
  EXPECT_TRUE(race_ego_bridge::shouldResetFlightHandover(true, false));
  EXPECT_FALSE(race_ego_bridge::shouldResetFlightHandover(true, true));
  EXPECT_FALSE(race_ego_bridge::shouldResetFlightHandover(false, false));
}

TEST(Trajectory, SetpointTimeout)
{
  EXPECT_TRUE(race_ego_bridge::timedOut(0.21, 0.20));
  EXPECT_FALSE(race_ego_bridge::timedOut(0.19, 0.20));
}

TEST(Trajectory, RejectNan)
{
  EXPECT_FALSE(race_ego_bridge::finite({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.78}));
}

TEST(Trajectory, VelocityAndAccelerationLimits)
{
  EXPECT_TRUE(race_ego_bridge::withinVectorLimit({0.5, 0.0, 0.0}, 0.6, 1.1));
  EXPECT_FALSE(race_ego_bridge::withinVectorLimit({0.8, 0.0, 0.0}, 0.6, 1.1));
}

TEST(Safety, Geofence)
{
  EXPECT_TRUE(race_ego_bridge::insideGeofence({0.0, 0.0, 0.78}, -7.5, 7.5, -7.0, 7.0, 0.5, 0.85));
  EXPECT_FALSE(race_ego_bridge::insideGeofence({8.0, 0.0, 0.78}, -7.5, 7.5, -7.0, 7.0, 0.5, 0.85));
}

TEST(Safety, NewTrajectoryTimeDoesNotReuseExpiredTime)
{
  EXPECT_DOUBLE_EQ(0.0, race_ego_bridge::clampTrajectoryTime(-2.0, 4.0));
}

TEST(Trajectory, UniformBsplineSamplingIsFinite)
{
  Eigen::MatrixXd points(3, 6);
  points << 0.0, 0.2, 0.4, 0.6, 0.8, 1.0,
    0.0, 0.0, 0.1, 0.2, 0.2, 0.2,
    0.78, 0.78, 0.78, 0.78, 0.78, 0.78;
  ego_planner::UniformBspline trajectory(points, 3, 0.2);
  auto velocity = trajectory.getDerivative();
  auto acceleration = velocity.getDerivative();
  const double duration = trajectory.getTimeSum();

  ASSERT_GT(duration, 0.0);
  for (double t = 0.0; t <= duration; t += duration / 10.0) {
    EXPECT_TRUE(trajectory.evaluateDeBoorT(t).allFinite());
    EXPECT_TRUE(velocity.evaluateDeBoorT(t).allFinite());
    EXPECT_TRUE(acceleration.evaluateDeBoorT(t).allFinite());
  }
}

TEST(ClearanceMode, NormalTrajectoryKeepsNormalBridgeFloor)
{
  const auto selection = race_ego_bridge::selectTrajectoryClearance(
    false, 0.36, true, 0.29, 0.25);
  EXPECT_TRUE(selection.accepted);
  EXPECT_DOUBLE_EQ(selection.required_clearance, 0.29);
}

TEST(ClearanceMode, MarkedConstrainedTrajectoryUsesConfiguredThreshold)
{
  const auto selection = race_ego_bridge::selectTrajectoryClearance(
    true, 0.25, true, 0.29, 0.25);
  EXPECT_TRUE(selection.accepted);
  EXPECT_DOUBLE_EQ(selection.required_clearance, 0.25);
}

TEST(ClearanceMode, UnmarkedLowClearanceIsRejected)
{
  const auto selection = race_ego_bridge::selectTrajectoryClearance(
    false, 0.25, true, 0.29, 0.25);
  EXPECT_FALSE(selection.accepted);
}

TEST(ClearanceMode, DisabledOrMismatchedConstrainedRequestIsRejected)
{
  EXPECT_FALSE(race_ego_bridge::selectTrajectoryClearance(
    true, 0.25, false, 0.29, 0.25).accepted);
  EXPECT_FALSE(race_ego_bridge::selectTrajectoryClearance(
    true, 0.24, true, 0.29, 0.25).accepted);
}

TEST(Trajectory, ContinuousReplanSwitch)
{
  EXPECT_TRUE(race_ego_bridge::transitionContinuous(
    {1.0, 2.0, 0.78}, {1.02, 2.01, 0.78}, 0.05));
  EXPECT_FALSE(race_ego_bridge::transitionContinuous(
    {1.0, 2.0, 0.78}, {1.30, 2.0, 0.78}, 0.05));
}

TEST(Trajectory, FullStateReplanSwitch)
{
  const race_ego_bridge::TrajectoryState previous{
    {1.0, 2.0, 0.78}, {0.20, 0.0, 0.0}, {0.05, 0.0, 0.0}};
  const race_ego_bridge::TrajectoryState continuous{
    {1.04, 2.0, 0.78}, {0.25, 0.0, 0.0}, {0.10, 0.0, 0.0}};
  const race_ego_bridge::TrajectoryState position_jump{
    {1.40, 2.0, 0.78}, {0.20, 0.0, 0.0}, {0.05, 0.0, 0.0}};
  EXPECT_TRUE(race_ego_bridge::transitionStateContinuous(
    previous, continuous, 0.05, 0.10, 0.20).continuous);
  const auto rejected = race_ego_bridge::transitionStateContinuous(
    previous, position_jump, 0.05, 0.10, 0.20);
  EXPECT_FALSE(rejected.continuous);
  EXPECT_NEAR(0.40, rejected.position_error, 1e-9);
}

TEST(Trajectory, SpatialSamplingNeverExceedsConfiguredSpacing)
{
  EXPECT_EQ(1U, race_ego_bridge::spatialSubdivisions(0.01, 0.04));
  EXPECT_EQ(2U, race_ego_bridge::spatialSubdivisions(0.08, 0.04));
  EXPECT_EQ(3U, race_ego_bridge::spatialSubdivisions(0.081, 0.04));
  EXPECT_EQ(0U, race_ego_bridge::spatialSubdivisions(0.10, 0.0));
}

TEST(Trajectory, ActiveRecheckSkipsAlreadyFlownHistory)
{
  const std::vector<double> sample_times{0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
  EXPECT_EQ(5U, race_ego_bridge::activeTrajectoryStartIndex(sample_times, 3.3, 0.5));
}

TEST(Trajectory, ActiveRecheckKeepsRecentTrackingHistory)
{
  const std::vector<double> sample_times{0.0, 0.1, 0.2, 0.3, 0.4};
  EXPECT_EQ(0U, race_ego_bridge::activeTrajectoryStartIndex(sample_times, 0.3, 0.2));
  EXPECT_EQ(4U, race_ego_bridge::activeTrajectoryStartIndex(sample_times, 9.0, 0.2));
}

TEST(Safety, EgoAndBridgeContinuousChecksAgreeOnSameRawSamples)
{
  pcl::PointCloud<pcl::PointXYZ> raw_points;
  raw_points.emplace_back(0.0F, 0.0F, 0.78F);
  raw_points.emplace_back(2.0F, 0.0F, 0.78F);
  plan_env::RawObstacleDistanceIndex ego_index;
  ego_index.rebuild(raw_points);
  pcl::KdTreeFLANN<pcl::PointXYZ> bridge_index;
  bridge_index.setInputCloud(raw_points.makeShared());
  constexpr double required_clearance = 0.509;
  const std::vector<Eigen::Vector3d> samples{{0.510, 0.0, 0.78}, {1.492, 0.0, 0.78}, {1.491, 0.0, 0.78}};
  std::size_t ego_first_failure = samples.size();
  std::size_t bridge_first_failure = samples.size();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    Eigen::Vector3d nearest;
    double ego_clearance = 0.0;
    ASSERT_TRUE(ego_index.nearest(samples[i], nearest, ego_clearance));
    pcl::PointXYZ query{static_cast<float>(samples[i].x()), static_cast<float>(samples[i].y()), static_cast<float>(samples[i].z())};
    std::vector<int> ids(1);
    std::vector<float> squared(1);
    ASSERT_EQ(1, bridge_index.nearestKSearch(query, 1, ids, squared));
    const double bridge_clearance = std::sqrt(static_cast<double>(squared.front()));
    if (ego_clearance < required_clearance && ego_first_failure == samples.size()) ego_first_failure = i;
    if (bridge_clearance < required_clearance && bridge_first_failure == samples.size()) bridge_first_failure = i;
    EXPECT_NEAR(ego_clearance, bridge_clearance, 1e-6);
    EXPECT_EQ(ego_clearance >= required_clearance, bridge_clearance >= required_clearance);
  }
  EXPECT_EQ(ego_first_failure, bridge_first_failure);
  EXPECT_EQ(1U, ego_first_failure);
}

// live_px4 projects the body-frame cloud with the PX4 pose. These freeze the
// geometry: a wrong sign or a swapped axis here would silently place obstacles
// on the wrong side of the vehicle, which the planner cannot detect.
TEST(Coordinates, BodyPointToMapAtOriginIsIdentity)
{
  const auto point = race_ego_bridge::bodyPointToMap(Vec3{1.5, 0.25, -0.1}, Vec3{}, 0.0);
  EXPECT_DOUBLE_EQ(1.5, point.x);
  EXPECT_DOUBLE_EQ(0.25, point.y);
  EXPECT_DOUBLE_EQ(-0.1, point.z);
}

TEST(Coordinates, BodyPointToMapTranslatesWithVehicle)
{
  const auto point =
    race_ego_bridge::bodyPointToMap(Vec3{2.0, 0.0, 0.0}, Vec3{1.0, -0.5, 0.78}, 0.0);
  EXPECT_DOUBLE_EQ(3.0, point.x);
  EXPECT_DOUBLE_EQ(-0.5, point.y);
  EXPECT_DOUBLE_EQ(0.78, point.z);
}

TEST(Coordinates, BodyPointToMapRotatesByYaw)
{
  // Facing +90 deg (map ENU): a point 2 m ahead of the body lands 2 m along +y.
  const auto point =
    race_ego_bridge::bodyPointToMap(Vec3{2.0, 0.0, 0.0}, Vec3{}, M_PI / 2.0);
  EXPECT_NEAR(0.0, point.x, 1e-9);
  EXPECT_NEAR(2.0, point.y, 1e-9);
  EXPECT_NEAR(0.0, point.z, 1e-9);
}

TEST(Coordinates, BodyPointToMapKeepsRangeFromVehicle)
{
  // Yaw must not change how far an obstacle is: that distance is what the
  // clearance contract is checked against.
  const Vec3 body{1.3, -0.4, 0.2};
  const Vec3 position{2.0, 3.0, 0.78};
  const double body_range = std::sqrt(body.x * body.x + body.y * body.y + body.z * body.z);
  for (const double yaw : {-2.5, -M_PI / 2.0, 0.0, 0.7, M_PI}) {
    const auto point = race_ego_bridge::bodyPointToMap(body, position, yaw);
    const double dx = point.x - position.x;
    const double dy = point.y - position.y;
    const double dz = point.z - position.z;
    EXPECT_NEAR(body_range, std::sqrt(dx * dx + dy * dy + dz * dz), 1e-9);
  }
}

TEST(Coordinates, BodyPointToMapMatchesPx4YawConversion)
{
  // PX4 reports heading in NED. NED north (yaw=0) is ENU/map +Y.
  const double yaw_map = race_ego_bridge::nedYawToMap(0.0);
  const auto point = race_ego_bridge::bodyPointToMap(Vec3{1.0, 0.0, 0.0}, Vec3{}, yaw_map);
  EXPECT_NEAR(0.0, point.x, 1e-9);
  EXPECT_NEAR(1.0, point.y, 1e-9);
}
