cmake_minimum_required(VERSION 3.8)
project(race_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(race_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(offboard_waypoint_node src/offboard_waypoint_node.cpp)
target_include_directories(offboard_waypoint_node PRIVATE include)
ament_target_dependencies(
  offboard_waypoint_node rclcpp mavros_msgs race_msgs geometry_msgs nav_msgs
  std_msgs std_srvs tf2 tf2_geometry_msgs)

install(TARGETS
  offboard_waypoint_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  mission/mission_sequencer_node.py
  mission/save_waypoint.py
  mission/export_mission_waypoints.py
  mission/waypoint_visualizer.py
  mission/global_waypoint_task_node.py
  mission/waypoint_state_machine_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  mission/waypoint_common.py
  mission/waypoint_behaviors.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_takeoff_handover_policy test/test_takeoff_handover_policy.cpp)
  target_include_directories(test_takeoff_handover_policy PRIVATE include)
  ament_add_gtest(test_global_planner_status_policy test/test_global_planner_status_policy.cpp)
  target_include_directories(test_global_planner_status_policy PRIVATE include)
  # Guards the ENU<->NED boundary and the per-axis type_mask generation.  These
  # are the highest-risk part of the MAVROS port: a swapped axis flies the
  # vehicle in the wrong direction and no integration test catches it earlier.
  ament_add_gtest(test_mavros_frame_utils test/test_mavros_frame_utils.cpp)
  target_include_directories(test_mavros_frame_utils PRIVATE include)
  # Pre-arm continuity gate and in-flight FAULT guard for the external-vision
  # chain.  Absent from the simulation node; required on the real aircraft.
  ament_add_gtest(test_ev_health_tracker test/test_ev_health_tracker.cpp)
  target_include_directories(test_ev_health_tracker PRIVATE include)
  # A PX4 EKF local-origin reset may replace a disarmed IDLE hold only after
  # stable confirmation.  The policy must remain impossible while armed.
  ament_add_gtest(
    test_local_origin_rebase_guard test/test_local_origin_rebase_guard.cpp)
  target_include_directories(test_local_origin_rebase_guard PRIVATE include)
  add_test(
    NAME test_mission_sequencer
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_sequencer.py)
  add_test(
    NAME test_mission_waypoint_export
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_waypoint_export.py)
  add_test(
    NAME test_waypoint_visualizer
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_waypoint_visualizer.py)
  add_test(
    NAME test_waypoint_state_machine
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_waypoint_state_machine.py)
  add_test(
    NAME test_waypoint_behaviors
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_waypoint_behaviors.py)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
