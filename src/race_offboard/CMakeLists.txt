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
  mission/list_waypoints.py
  mission/goto_waypoint.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  mission/waypoint_common.py
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
  # Separates "the planner may not plan" from "PX4 has no localization".  A
  # stale LiDAR posterior must hold the planner without interrupting EV, and the
  # resulting hold must be recoverable rather than a mission abort.
  ament_add_gtest(test_frlio_health_policy test/test_frlio_health_policy.cpp)
  target_include_directories(test_frlio_health_policy PRIVATE include)
  # A PX4 EKF local-origin reset may replace a disarmed IDLE hold only after
  # stable confirmation.  The policy must remain impossible while armed.
  ament_add_gtest(
    test_local_origin_rebase_guard test/test_local_origin_rebase_guard.cpp)
  target_include_directories(test_local_origin_rebase_guard PRIVATE include)
  # PX4 local-Z resets are edge-triggered by z_reset_counter. The tracker
  # prevents a persistent delta_z field from translating a hold repeatedly.
  ament_add_gtest(
    test_px4_local_z_reset_tracker test/test_px4_local_z_reset_tracker.cpp)
  target_include_directories(test_px4_local_z_reset_tracker PRIVATE include)
  ament_add_gtest(
    test_px4_local_pose_reset_matcher test/test_px4_local_pose_reset_matcher.cpp)
  target_include_directories(test_px4_local_pose_reset_matcher PRIVATE include)
  add_test(
    NAME test_mission_sequencer
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_sequencer.py)
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
