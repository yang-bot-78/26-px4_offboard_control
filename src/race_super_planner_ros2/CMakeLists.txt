cmake_minimum_required(VERSION 3.8)
project(race_super_planner_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(race_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(Eigen3 REQUIRED)

add_executable(super_planner_ros2_node src/super_planner_ros2_node.cpp)
target_include_directories(super_planner_ros2_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(super_planner_ros2_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(super_planner_ros2_node
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  race_msgs
  sensor_msgs
  std_msgs
  traj_utils
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

install(TARGETS
  super_planner_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_path_fallback_policy test/test_path_fallback_policy.cpp)
  if(TARGET test_path_fallback_policy)
    target_include_directories(test_path_fallback_policy PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
  ament_add_gtest(test_global_grid_policy test/test_global_grid_policy.cpp)
  if(TARGET test_global_grid_policy)
    target_include_directories(test_global_grid_policy PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
  ament_add_gtest(test_narrow_corridor_policy test/test_narrow_corridor_policy.cpp)
  if(TARGET test_narrow_corridor_policy)
    target_include_directories(test_narrow_corridor_policy PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
  ament_add_gtest(test_continuous_segment_policy test/test_continuous_segment_policy.cpp)
  if(TARGET test_continuous_segment_policy)
    target_include_directories(test_continuous_segment_policy PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
  ament_add_gtest(
    test_obstacle_aware_smoothing_policy
    test/test_obstacle_aware_smoothing_policy.cpp)
  if(TARGET test_obstacle_aware_smoothing_policy)
    target_include_directories(test_obstacle_aware_smoothing_policy PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
  ament_add_gtest(test_takeoff_path_policy test/test_takeoff_path_policy.cpp)
  if(TARGET test_takeoff_path_policy)
    target_include_directories(test_takeoff_path_policy PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
  ament_add_gtest(
    test_local_goal_lifecycle_policy
    test/test_local_goal_lifecycle_policy.cpp)
  if(TARGET test_local_goal_lifecycle_policy)
    target_include_directories(test_local_goal_lifecycle_policy PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
endif()

ament_package()
