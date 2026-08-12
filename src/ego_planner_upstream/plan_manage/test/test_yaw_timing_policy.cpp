#include <gtest/gtest.h>

#include <ego_planner/yaw_timing_policy.h>

#include <limits>

namespace
{

TEST(YawTimingPolicy, CalculatesOnlyAfterSimulationTimeAdvances)
{
  EXPECT_TRUE(ego_planner::YawTimingPolicy::canCalculateYawRate(0.01));
  EXPECT_TRUE(ego_planner::YawTimingPolicy::canCalculateYawRate(1.0e-5));
}

TEST(YawTimingPolicy, RejectsDuplicateOrInvalidSimulationTime)
{
  EXPECT_FALSE(ego_planner::YawTimingPolicy::canCalculateYawRate(0.0));
  EXPECT_FALSE(ego_planner::YawTimingPolicy::canCalculateYawRate(-0.01));
  EXPECT_FALSE(ego_planner::YawTimingPolicy::canCalculateYawRate(
      std::numeric_limits<double>::quiet_NaN()));
}

}  // namespace
