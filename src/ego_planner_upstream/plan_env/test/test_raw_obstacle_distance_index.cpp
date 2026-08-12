#include <chrono>

#include <gtest/gtest.h>

#include "plan_env/occupancy_safety_model.h"
#include "plan_env/raw_obstacle_distance_index.h"

namespace
{

TEST(RawObstacleDistanceIndex, UsesFloatingPointRawObstacleCoordinates)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.emplace_back(0.0F, -0.5515F, 0.78F);
  cloud.emplace_back(0.0F, 0.5515F, 0.78F);
  plan_env::RawObstacleDistanceIndex index;
  index.rebuild(cloud);

  Eigen::Vector3d nearest;
  double distance = 0.0;
  ASSERT_TRUE(index.nearest(Eigen::Vector3d(0.0, 0.0, 0.78), nearest, distance));
  EXPECT_NEAR(0.5515, distance, 1e-5);
  EXPECT_TRUE(plan_env::OccupancySafetyModel::isContinuouslySafe(distance, 0.509));
  EXPECT_FALSE(plan_env::OccupancySafetyModel::isContinuouslySafe(0.508, 0.509));
}

TEST(RawObstacleDistanceIndex, QueryCostIsBoundedForPlannerSampling)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int x = 0; x < 200; ++x)
    for (int y = 0; y < 100; ++y)
      cloud.emplace_back(0.01F * x, 0.01F * y, 0.78F);
  plan_env::RawObstacleDistanceIndex index;
  index.rebuild(cloud);

  const auto start = std::chrono::steady_clock::now();
  Eigen::Vector3d nearest;
  double distance = 0.0;
  for (int sample = 0; sample < 1000; ++sample)
    ASSERT_TRUE(index.nearest(Eigen::Vector3d(1.0, 0.5 + sample * 1e-6, 0.78), nearest, distance));
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  EXPECT_LT(elapsed_ms, 100.0);
}

}  // namespace
