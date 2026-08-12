#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "race_super_planner_ros2/obstacle_aware_smoothing_policy.hpp"

namespace smoothing = race_super_planner_ros2::smoothing;

namespace
{
template<typename ClearanceFn>
auto segmentSafety(ClearanceFn clearance, const double required)
{
  return [clearance, required](
    const smoothing::Point3 & start, const smoothing::Point3 & finish)
         {
           constexpr double step = 0.01;
           const std::size_t count = std::max<std::size_t>(
             1, static_cast<std::size_t>(std::ceil(
               smoothing::distance2d(start, finish) / step)));
           for (std::size_t index = 0; index <= count; ++index) {
             if (clearance(
                 smoothing::interpolate(
                   start, finish, static_cast<double>(index) / count)) < required)
             {
               return false;
             }
           }
           return true;
         };
}

double maximumHeadingJump(const std::vector<smoothing::Point3> & path)
{
  double maximum = 0.0;
  if (path.size() < 3) {
    return maximum;
  }
  double previous = std::atan2(
    path[1].y - path[0].y, path[1].x - path[0].x);
  for (std::size_t index = 2; index < path.size(); ++index) {
    const double heading = std::atan2(
      path[index].y - path[index - 1].y,
      path[index].x - path[index - 1].x);
    const double delta = std::atan2(
      std::sin(heading - previous), std::cos(heading - previous));
    maximum = std::max(maximum, std::abs(delta));
    previous = heading;
  }
  return maximum;
}

std::vector<smoothing::Point3> oldChaikin(
  const std::vector<smoothing::Point3> & input)
{
  std::vector<smoothing::Point3> output;
  output.push_back(input.front());
  for (std::size_t index = 1; index < input.size(); ++index) {
    output.push_back(smoothing::interpolate(input[index - 1], input[index], 0.25));
    output.push_back(smoothing::interpolate(input[index - 1], input[index], 0.75));
  }
  output.push_back(input.back());
  return output;
}
}  // namespace

TEST(ObstacleAwareSmoothing, CornerCutCannotConsumeRawClearance)
{
  const auto corner_clearance = [](const smoothing::Point3 & point) {
      return std::hypot(point.x + 0.5, point.y - 0.5);
    };
  const std::vector<smoothing::Point3> raw{
    {-1.0, -1.0, 0.78}, {-1.0, 1.0, 0.78}, {1.0, 1.0, 0.78}};
  const auto safe = segmentSafety(corner_clearance, 0.466);
  const double raw_clearance =
    smoothing::minimumClearance(raw, corner_clearance, 0.01);
  const auto old = oldChaikin(raw);
  const double old_clearance =
    smoothing::minimumClearance(old, corner_clearance, 0.01);
  const auto result = smoothing::acceptedStepSmooth(
    raw, 2, corner_clearance, safe, 0.01, 1.0e-6);

  EXPECT_LT(old_clearance, raw_clearance);
  EXPECT_TRUE(result.assessment.accepted);
  EXPECT_GE(result.assessment.candidate_minimum + 1.0e-6, raw_clearance);
  EXPECT_TRUE(result.assessment.global_clearance_preserved);
  EXPECT_TRUE(result.assessment.local_clearance_preserved);
  std::cout << "[CORNER_CUT_REGRESSION] raw_clearance=" << raw_clearance
            << " old_smoother_clearance=" << old_clearance
            << " new_smoother_clearance=" << result.assessment.candidate_minimum
            << " required_clearance=0.466 new_path_safe="
            << (result.assessment.segments_safe ? "true" : "false")
            << " clearance_preserved="
            << (result.assessment.global_clearance_preserved ? "true" : "false")
            << std::endl;
}

TEST(ObstacleAwareSmoothing, NarrowCorridorKeepsWallClearance)
{
  const auto corridor_clearance = [](const smoothing::Point3 & point) {
      return std::min(std::abs(0.60 - point.y), std::abs(-0.60 - point.y));
    };
  const std::vector<smoothing::Point3> raw{
    {0.0, 0.0, 0.78}, {0.4, 0.01, 0.78}, {0.8, 0.0, 0.78},
    {1.2, -0.01, 0.78}, {1.6, 0.0, 0.78}};
  const auto safe = segmentSafety(corridor_clearance, 0.58);
  const auto result = smoothing::acceptedStepSmooth(
    raw, 3, corridor_clearance, safe, 0.01, 1.0e-6);
  const double raw_clearance =
    smoothing::minimumClearance(raw, corridor_clearance, 0.01);

  EXPECT_TRUE(result.assessment.accepted);
  EXPECT_GE(result.assessment.candidate_minimum + 1.0e-6, raw_clearance);
  EXPECT_GE(result.assessment.candidate_minimum, 0.58);
  std::cout << "[NARROW_CORRIDOR_REGRESSION] raw_clearance=" << raw_clearance
            << " new_clearance=" << result.assessment.candidate_minimum
            << " selected_raw_or_smooth=" << (result.changed ? "SMOOTHED" : "RAW")
            << " safe=" << (result.assessment.accepted ? "true" : "false")
            << std::endl;
}

TEST(ObstacleAwareSmoothing, OpenSpaceStillImprovesGeometry)
{
  const auto open_clearance = [](const smoothing::Point3 &) {
      return std::numeric_limits<double>::infinity();
    };
  const auto always_safe = [](const smoothing::Point3 &, const smoothing::Point3 &) {
      return true;
    };
  const std::vector<smoothing::Point3> raw{
    {0.0, 0.0, 0.78}, {0.5, 0.25, 0.78}, {1.0, -0.20, 0.78},
    {1.5, 0.20, 0.78}, {2.0, 0.0, 0.78}};
  const auto result = smoothing::acceptedStepSmooth(
    raw, 3, open_clearance, always_safe, 0.02, 1.0e-6);

  EXPECT_TRUE(result.changed);
  EXPECT_LT(smoothing::pathLength(result.path), smoothing::pathLength(raw));
  EXPECT_LT(maximumHeadingJump(result.path), maximumHeadingJump(raw));
  std::cout << "[OPEN_SPACE_SMOOTHING] raw_curvature="
            << maximumHeadingJump(raw)
            << " smoothed_curvature=" << maximumHeadingJump(result.path)
            << " raw_length=" << smoothing::pathLength(raw)
            << " smoothed_length=" << smoothing::pathLength(result.path)
            << " smoothing_effective=true" << std::endl;
}

TEST(ObstacleAwareSmoothing, StraightPathDoesNotOscillate)
{
  const auto open_clearance = [](const smoothing::Point3 &) {
      return std::numeric_limits<double>::infinity();
    };
  const auto always_safe = [](const smoothing::Point3 &, const smoothing::Point3 &) {
      return true;
    };
  const std::vector<smoothing::Point3> raw{
    {0.0, 0.0, 0.78}, {0.5, 0.0, 0.78}, {1.0, 0.0, 0.78},
    {1.5, 0.0, 0.78}, {2.0, 0.0, 0.78}};
  const auto result = smoothing::acceptedStepSmooth(
    raw, 3, open_clearance, always_safe, 0.02, 1.0e-6);

  EXPECT_FALSE(result.changed);
  ASSERT_EQ(result.path.size(), raw.size());
  for (std::size_t index = 0; index < raw.size(); ++index) {
    EXPECT_DOUBLE_EQ(result.path[index].x, raw[index].x);
    EXPECT_DOUBLE_EQ(result.path[index].y, raw[index].y);
  }
  std::cout << "[STRAIGHT_PATH_REGRESSION] max_raw_to_smoothed_deviation=0"
            << " oscillation=false" << std::endl;
}

TEST(ObstacleAwareSmoothing, UnsafeUpdateFallsBackToRaw)
{
  const auto clearance = [](const smoothing::Point3 & point) {
      return 1.0 - std::abs(point.y);
    };
  const auto only_original_segments = [](
    const smoothing::Point3 & start, const smoothing::Point3 & finish)
    {
      return std::abs(start.x - finish.x) <= 0.51 &&
             (std::abs(start.y) < 1.0e-12 || std::abs(finish.y) < 1.0e-12);
    };
  const std::vector<smoothing::Point3> raw{
    {0.0, 0.0, 0.78}, {0.5, 0.2, 0.78}, {1.0, 0.0, 0.78}};
  const auto result = smoothing::acceptedStepSmooth(
    raw, 2, clearance, only_original_segments, 0.01, 1.0e-6);

  EXPECT_FALSE(result.changed);
  EXPECT_EQ(result.path.size(), raw.size());
  EXPECT_DOUBLE_EQ(result.path[1].y, raw[1].y);
  std::cout << "[FALLBACK_RAW_REGRESSION] selected=RAW unsafe_update_rejected=true"
            << std::endl;
}
