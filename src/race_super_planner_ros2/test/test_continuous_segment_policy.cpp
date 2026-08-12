#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "race_super_planner_ros2/continuous_segment_policy.hpp"

namespace
{

double clearanceToObstacle(
  const race_super_planner_ros2::policy::SegmentPoint2D & point,
  const race_super_planner_ros2::policy::SegmentPoint2D & obstacle)
{
  return std::hypot(point.x - obstacle.x, point.y - obstacle.y);
}

}  // namespace

TEST(ContinuousSegmentPolicy, RejectsUnsafeInteriorWhenEndpointsAreSafe)
{
  using race_super_planner_ros2::policy::SegmentPoint2D;
  const SegmentPoint2D start{-0.530, 0.0};
  const SegmentPoint2D end{0.530, 0.0};
  const SegmentPoint2D obstacle{0.0, 0.0};
  constexpr double required_clearance = 0.509;
  constexpr double step = 0.02;

  EXPECT_GT(clearanceToObstacle(start, obstacle), required_clearance);
  EXPECT_GT(clearanceToObstacle(end, obstacle), required_clearance);
  const std::size_t samples =
    race_super_planner_ros2::policy::segmentSampleCount(start, end, step);
  bool safe = true;
  double min_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index <= samples; ++index) {
    const auto sample = race_super_planner_ros2::policy::segmentSample(start, end, index, samples);
    min_clearance = std::min(min_clearance, clearanceToObstacle(sample, obstacle));
    safe = safe && clearanceToObstacle(sample, obstacle) >= required_clearance;
  }
  EXPECT_FALSE(safe);
  EXPECT_LT(min_clearance, required_clearance);
}

TEST(ContinuousSegmentPolicy, UsesAtMostTwoCentimetreSamples)
{
  using race_super_planner_ros2::policy::SegmentPoint2D;
  const SegmentPoint2D start{0.0, 0.0};
  const SegmentPoint2D end{0.11, 0.0};
  const auto samples = race_super_planner_ros2::policy::segmentSampleCount(start, end, 0.02);
  EXPECT_EQ(samples, 6U);
  EXPECT_LE(0.11 / static_cast<double>(samples), 0.02);
}

TEST(ContinuousSegmentPolicy, KeepsSafeCellChainWhenShortcutCrossesObstacle)
{
  using race_super_planner_ros2::policy::SegmentPoint2D;
  const SegmentPoint2D start{-1.0, 0.0};
  const SegmentPoint2D waypoint{0.0, 0.70};
  const SegmentPoint2D end{1.0, 0.0};
  const SegmentPoint2D obstacle{0.0, 0.0};
  constexpr double required_clearance = 0.509;

  auto segment_safe = [&](const SegmentPoint2D & a, const SegmentPoint2D & b) {
      const auto count = race_super_planner_ros2::policy::segmentSampleCount(a, b, 0.02);
      for (std::size_t index = 0; index <= count; ++index) {
        if (clearanceToObstacle(
            race_super_planner_ros2::policy::segmentSample(a, b, index, count), obstacle) <
          required_clearance)
        {
          return false;
        }
      }
      return true;
    };

  EXPECT_TRUE(segment_safe(start, waypoint));
  EXPECT_TRUE(segment_safe(waypoint, end));
  EXPECT_FALSE(segment_safe(start, end));
}

TEST(ContinuousSegmentPolicy, FirstGateCentreLineHasSafeSamplesOnly)
{
  using race_super_planner_ros2::policy::SegmentPoint2D;
  const SegmentPoint2D start{-0.5, -0.015};
  const SegmentPoint2D end{0.5, -0.015};
  const SegmentPoint2D lower_wall{0.0, -0.5665};
  const SegmentPoint2D upper_wall{0.0, 0.5365};
  constexpr double required_clearance = 0.509;
  const auto count = race_super_planner_ros2::policy::segmentSampleCount(start, end, 0.02);
  for (std::size_t index = 0; index <= count; ++index) {
    const auto sample = race_super_planner_ros2::policy::segmentSample(start, end, index, count);
    const double clearance = std::min(
      clearanceToObstacle(sample, lower_wall), clearanceToObstacle(sample, upper_wall));
    EXPECT_GE(clearance, required_clearance) << "sample=" << index;
  }
}

TEST(ContinuousSegmentPolicy, HistoricalWallStrikeClearanceRemainsUnsafe)
{
  constexpr double required_clearance = 0.466;
  constexpr double historical_raw_clearance = 0.426;
  EXPECT_LT(historical_raw_clearance, required_clearance);
}
