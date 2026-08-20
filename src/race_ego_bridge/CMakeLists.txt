cmake_minimum_required(VERSION 3.8)
project(race_ego_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(race_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(Eigen3 REQUIRED)
find_package(plan_env REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(race_ego_odom_bridge src/ego_odom_bridge.cpp)
ament_target_dependencies(
  race_ego_odom_bridge rclcpp nav_msgs mavros_msgs tf2 tf2_geometry_msgs)

add_executable(race_ego_cloud_bridge src/ego_cloud_bridge.cpp)
ament_target_dependencies(
  race_ego_cloud_bridge rclcpp geometry_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros
  tf2_sensor_msgs)

add_executable(race_ego_goal_bridge src/ego_goal_bridge.cpp)
ament_target_dependencies(
  race_ego_goal_bridge rclcpp geometry_msgs mavros_msgs nav_msgs race_msgs std_msgs)

add_executable(race_ego_trajectory_bridge src/ego_trajectory_bridge.cpp)
ament_target_dependencies(
  race_ego_trajectory_bridge rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
  visualization_msgs mavros_msgs race_msgs quadrotor_msgs traj_utils bspline_opt pcl_conversions)
ament_target_dependencies(race_ego_trajectory_bridge plan_env)
target_link_libraries(race_ego_trajectory_bridge ${PCL_LIBRARIES})

install(TARGETS
  race_ego_odom_bridge race_ego_cloud_bridge race_ego_goal_bridge race_ego_trajectory_bridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_race_ego_safety test/test_race_ego_safety.cpp)
  target_include_directories(test_race_ego_safety PRIVATE include)
  ament_target_dependencies(test_race_ego_safety bspline_opt plan_env)
  target_include_directories(test_race_ego_safety PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_race_ego_safety Eigen3::Eigen ${PCL_LIBRARIES})
  ament_add_gtest(
    test_handover_transaction_policy test/test_handover_transaction_policy.cpp)
  target_include_directories(test_handover_transaction_policy PRIVATE include)
endif()

ament_package()
