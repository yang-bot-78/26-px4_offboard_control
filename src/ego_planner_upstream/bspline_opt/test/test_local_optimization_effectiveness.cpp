#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <atomic>
#include <iomanip>
#include <memory>
#include <limits>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "bspline_opt/bspline_optimizer.h"
#include "plan_env/grid_map.h"
#include "traj_utils/plan_container.hpp"

namespace
{

using ego_planner::BsplineOptimizer;
using ego_planner::ControlPoints;
using namespace std::chrono_literals;

class LocalOptimizationEffectivenessTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static rclcpp::Node::SharedPtr makeNode(
    double required_center_clearance,
    double optimization_dist0 = std::numeric_limits<double>::quiet_NaN(),
    double grid_map_clearance = std::numeric_limits<double>::quiet_NaN(),
    double planning_clearance = std::numeric_limits<double>::quiet_NaN())
  {
    if (std::isnan(optimization_dist0)) {
      optimization_dist0 = required_center_clearance;
    }
    if (std::isnan(grid_map_clearance)) {
      grid_map_clearance = required_center_clearance;
    }
    if (std::isnan(planning_clearance)) {
      planning_clearance = grid_map_clearance;
    }
    static std::atomic<unsigned int> instance{0};
    const std::string node_namespace =
      "/local_optimization_effectiveness_" + std::to_string(++instance);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("ego_diagnostics.enable", false),
      rclcpp::Parameter("grid_map/resolution", 0.15),
      rclcpp::Parameter("grid_map/map_size_x", 8.0),
      rclcpp::Parameter("grid_map/map_size_y", 8.0),
      rclcpp::Parameter("grid_map/map_size_z", 2.0),
      rclcpp::Parameter("grid_map/local_update_range_x", 4.0),
      rclcpp::Parameter("grid_map/local_update_range_y", 4.0),
      rclcpp::Parameter("grid_map/local_update_range_z", 1.0),
      rclcpp::Parameter("grid_map/obstacles_inflation", grid_map_clearance),
      rclcpp::Parameter("grid_map/planning_inflation", planning_clearance),
      rclcpp::Parameter("grid_map/fx", 387.0),
      rclcpp::Parameter("grid_map/fy", 387.0),
      rclcpp::Parameter("grid_map/cx", 321.0),
      rclcpp::Parameter("grid_map/cy", 243.0),
      rclcpp::Parameter("grid_map/use_depth_filter", false),
      rclcpp::Parameter("grid_map/depth_filter_tolerance", 0.15),
      rclcpp::Parameter("grid_map/depth_filter_maxdist", 5.0),
      rclcpp::Parameter("grid_map/depth_filter_mindist", 0.2),
      rclcpp::Parameter("grid_map/depth_filter_margin", 2),
      rclcpp::Parameter("grid_map/k_depth_scaling_factor", 1000.0),
      rclcpp::Parameter("grid_map/skip_pixel", 2),
      rclcpp::Parameter("grid_map/min_ray_length", 0.1),
      rclcpp::Parameter("grid_map/max_ray_length", 5.0),
      rclcpp::Parameter("grid_map/visualization_truncate_height", 2.0),
      rclcpp::Parameter("grid_map/virtual_ceil_height", -1.0),
      rclcpp::Parameter("grid_map/virtual_ceil_yp", 7.0),
      rclcpp::Parameter("grid_map/virtual_ceil_yn", -7.0),
      rclcpp::Parameter("grid_map/frame_id", "map"),
      rclcpp::Parameter("grid_map/ground_height", 0.0),
      rclcpp::Parameter("grid_map/odom_depth_timeout", 1.0),
      rclcpp::Parameter("optimization/lambda_smooth", 1.0),
      rclcpp::Parameter("optimization/lambda_collision", 1.2),
      rclcpp::Parameter("optimization/lambda_feasibility", 0.4),
      rclcpp::Parameter("optimization/lambda_fitness", 1.0),
      rclcpp::Parameter("required_center_clearance", required_center_clearance),
      rclcpp::Parameter("optimization/dist0", optimization_dist0),
      rclcpp::Parameter("optimization/swarm_clearance", 0.5),
      rclcpp::Parameter("optimization/max_vel", 0.6),
      rclcpp::Parameter("optimization/max_acc", 0.8),
      rclcpp::Parameter("optimization/order", 3)
    });
    return std::make_shared<rclcpp::Node>(
      "optimizer_clearance_contract",
      node_namespace, options);
  }

  static void declareGridMapClearanceForContractOnly(
    const rclcpp::Node::SharedPtr & node)
  {
    node->declare_parameter(
      "grid_map/obstacles_inflation",
      std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter(
      "grid_map/planning_inflation",
      std::numeric_limits<double>::quiet_NaN());
  }

  static void initializeMapWithFarObstacle(
    const rclcpp::Node::SharedPtr & map_node, const std::shared_ptr<GridMap> & map)
  {
    initializeMap(
      map_node, map, {Eigen::Vector3d(3.0, 3.0, 0.78)});
  }

  static void initializeMap(
    const rclcpp::Node::SharedPtr & map_node, const std::shared_ptr<GridMap> & map,
    const std::vector<Eigen::Vector3d> & obstacle_points)
  {
    auto source = std::make_shared<rclcpp::Node>(
      "map_source", map_node->get_namespace());
    auto odom_pub = source->create_publisher<nav_msgs::msg::Odometry>("grid_map/odom", 10);
    auto cloud_pub =
      source->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/cloud", 10);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(map_node);
    executor.add_node(source);
    const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
    while ((odom_pub->get_subscription_count() == 0 ||
      cloud_pub->get_subscription_count() == 0) &&
      std::chrono::steady_clock::now() < discovery_deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose.position.z = 0.78;
    odom_pub->publish(odom);
    for (int index = 0; index < 5; ++index) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }

    pcl::PointCloud<pcl::PointXYZ> points;
    for (const auto & point : obstacle_points) {
      points.push_back(pcl::PointXYZ(
          static_cast<float>(point.x()), static_cast<float>(point.y()),
          static_cast<float>(point.z())));
    }
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(points, cloud);
    cloud.header.frame_id = "map";
    cloud_pub->publish(cloud);
    for (int index = 0; index < 5; ++index) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_GE(map->getMapRevision(), 1U);
  }

  static ControlPoints straightChallenge(double clearance, int point_count = 10)
  {
    ControlPoints control_points;
    control_points.resize(point_count);
    control_points.clearance = clearance;
    for (int index = 0; index < control_points.size; ++index) {
      control_points.points.col(index) =
        Eigen::Vector3d(0.25 * static_cast<double>(index), 0.0, 0.78);
    }

    // A wall/obstacle lies 0.30 m to +Y. The straight seed violates the
    // 0.466 m contract, while the -Y half-plane is unobstructed.
    for (int index = 3; index <= 6; ++index) {
      control_points.base_point[index].push_back(
        Eigen::Vector3d(control_points.points(0, index), 0.30, 0.78));
      control_points.direction[index].push_back(Eigen::Vector3d(0.0, -1.0, 0.0));
    }
    return control_points;
  }

  static Eigen::MatrixXd runMappedChallenge(
    double required_center_clearance, bool & success, double & final_cost,
    int & iterations, int & termination, std::size_t & collision_segments,
    double & preopt_min_clearance, double & postopt_min_clearance)
  {
    // The production contract has one canonical input. The runtime aliases
    // below are derived by makeNode exactly as node_parameter_overlays() does;
    // this test never independently substitutes optimization/dist0.
    const auto node = makeNode(required_center_clearance);
    auto map = std::make_shared<GridMap>();
    map->initMap(node);

    std::vector<Eigen::Vector3d> obstacle_points;
    for (double y = -0.10; y <= 0.1001; y += 0.025) {
      for (double z = 0.68; z <= 0.8801; z += 0.05) {
        obstacle_points.emplace_back(1.10, y, z);
      }
    }
    initializeMap(node, map, obstacle_points);

    BsplineOptimizer optimizer;
    optimizer.setParam(node);
    optimizer.setEnvironment(map);
    optimizer.a_star_.reset(new AStar);
    optimizer.a_star_->initGridMap(map, Eigen::Vector3i(100, 100, 100));
    ego_planner::SwarmTrajData swarm;
    optimizer.setSwarmTrajs(&swarm);

    ControlPoints challenge = straightChallenge(required_center_clearance, 14);
    // The actual map, rather than the handcrafted base points above, must
    // generate the rebound directions for this end-to-end challenge.
    for (auto & points : challenge.base_point) {
      points.clear();
    }
    for (auto & directions : challenge.direction) {
      directions.clear();
    }
    const Eigen::Vector3d endpoint =
      (challenge.points.col(challenge.size - 3) +
       4.0 * challenge.points.col(challenge.size - 2) +
       challenge.points.col(challenge.size - 1)) / 6.0;
    optimizer.setLocalTargetPt(endpoint);

    Eigen::MatrixXd seed = challenge.points;
    preopt_min_clearance = minimumSplineClearance(seed, 0.5, map);
    const auto segments = optimizer.initControlPoints(seed, true);
    collision_segments = segments.size();
    Eigen::MatrixXd optimized;
    success = optimizer.BsplineOptimizeTrajRebound(optimized, 0.5);
    postopt_min_clearance = minimumSplineClearance(optimized, 0.5, map);
    final_cost = NAN;
    iterations = optimizer.getLastReboundIterationCount();
    termination = optimizer.getLastReboundTerminationCode();
    return optimized;
  }

  static double minimumSplineClearance(
    const Eigen::MatrixXd & control_points, const double interval,
    const std::shared_ptr<GridMap> & map)
  {
    ego_planner::UniformBspline trajectory(control_points, 3, interval);
    double start = 0.0;
    double end = 0.0;
    trajectory.getTimeSpan(start, end);
    double minimum = std::numeric_limits<double>::infinity();
    for (double time = start; time <= end + 1.0e-9; time += 0.01)
    {
      Eigen::Vector3d nearest;
      double distance = std::numeric_limits<double>::infinity();
      map->queryContinuousOccupancy(
        trajectory.evaluateDeBoorT(std::min(time, end)), nearest, distance);
      minimum = std::min(minimum, distance);
    }
    return minimum;
  }
};

TEST_F(LocalOptimizationEffectivenessTest, InvalidClearanceContractsAreRejected)
{
  const std::vector<double> invalid_values = {
    -1.0, 0.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()};
  for (const double invalid : invalid_values)
  {
    const auto contract =
      BsplineOptimizer::diagnoseObstacleClearanceContract(invalid, invalid);
    EXPECT_FALSE(contract.valid);

    const auto node = makeNode(invalid);
    declareGridMapClearanceForContractOnly(node);
    BsplineOptimizer optimizer;
    EXPECT_THROW(optimizer.setParam(node), std::invalid_argument);
  }
}

TEST_F(LocalOptimizationEffectivenessTest, OptimizationClearanceBelowHardLimitIsRejected)
{
  const auto contract =
    BsplineOptimizer::diagnoseObstacleClearanceContract(0.466, 0.300);
  EXPECT_FALSE(contract.valid);
  EXPECT_EQ("OPTIMIZATION_CLEARANCE_BELOW_HARD_LIMIT", contract.reason);

  const auto node = makeNode(0.466, 0.300, 0.466);
  declareGridMapClearanceForContractOnly(node);
  BsplineOptimizer optimizer;
  EXPECT_THROW(optimizer.setParam(node), std::invalid_argument);
}

TEST_F(LocalOptimizationEffectivenessTest, OptimizationTargetMayExceedHardLimit)
{
  const auto contract =
    BsplineOptimizer::diagnoseObstacleClearanceContract(0.29, 0.40);
  EXPECT_TRUE(contract.valid);

  const auto node = makeNode(0.29, 0.40, 0.29, 0.36);
  declareGridMapClearanceForContractOnly(node);
  BsplineOptimizer optimizer;
  EXPECT_NO_THROW(optimizer.setParam(node));
}

TEST_F(LocalOptimizationEffectivenessTest, PlanningClearanceMustStayInsideContract)
{
  for (const double planning_clearance : {0.28, 0.41}) {
    const auto node = makeNode(0.29, 0.40, 0.29, planning_clearance);
    declareGridMapClearanceForContractOnly(node);
    BsplineOptimizer optimizer;
    EXPECT_THROW(optimizer.setParam(node), std::invalid_argument);
  }
}

TEST_F(LocalOptimizationEffectivenessTest, GridMapAliasDriftIsRejected)
{
  const auto node = makeNode(0.466, 0.466, 0.300);
  declareGridMapClearanceForContractOnly(node);
  BsplineOptimizer optimizer;
  EXPECT_THROW(optimizer.setParam(node), std::invalid_argument);
}

TEST_F(LocalOptimizationEffectivenessTest, ForcedAvoidanceUsesProductionClearanceContract)
{
  constexpr double required_center_clearance = 0.466;
  const auto signal = BsplineOptimizer::diagnoseReboundObstacleSignal(
    Eigen::Vector3d(1.0, 0.0, 0.78),
    Eigen::Vector3d(1.0, 0.30, 0.78),
    Eigen::Vector3d(0.0, -1.0, 0.0), required_center_clearance);
  ASSERT_GT(signal.gradient.norm(), 0.0);

  bool success = false;
  double final_cost = NAN;
  int iterations = -1;
  int termination = 0;
  std::size_t collision_segments = 0;
  double preopt_min_clearance = NAN;
  double postopt_min_clearance = NAN;
  const Eigen::MatrixXd optimized = runMappedChallenge(
    required_center_clearance, success, final_cost, iterations, termination,
    collision_segments, preopt_min_clearance, postopt_min_clearance);

  ASSERT_GT(collision_segments, 0U);
  ASSERT_EQ(14, optimized.cols());
  const double max_lateral_deviation =
    optimized.row(1).cwiseAbs().maxCoeff();
  EXPECT_TRUE(success);
  EXPECT_GT(max_lateral_deviation, 0.10);
  EXPECT_LT(preopt_min_clearance, required_center_clearance);
  EXPECT_GE(postopt_min_clearance, required_center_clearance);
  EXPECT_NE(lbfgs::LBFGS_ALREADY_MINIMIZED, termination);
  std::cout
    << std::fixed << std::setprecision(6)
    << "[FORCED_LOCAL_AVOIDANCE_AFTER_FIX] "
    << "required_center_clearance=" << required_center_clearance
    << " runtime_dist0=" << required_center_clearance
    << " obstacle_distance=0.300"
    << " initial_obstacle_gradient_norm=" << signal.gradient.norm()
    << " preopt_min_clearance=" << preopt_min_clearance
    << " postopt_min_clearance=" << postopt_min_clearance
    << " max_lateral_deviation=" << max_lateral_deviation
    << " optimizer_iterations=" << iterations
    << " termination=" << termination
    << " avoided_obstacle=" << (success ? "true" : "false") << std::endl;
}

}  // namespace
