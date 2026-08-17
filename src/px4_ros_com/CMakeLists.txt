cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# ============================================================================
# EV 链唯一路径（2026-08-17）
#
#   MID-360 -> FAST-LIO -> /Odometry -> fastlio_odometry_guard
#     -> /Odometry/healthy -> fastlio_px4_vehicle_odometry
#     -> /fmu/in/vehicle_visual_odometry -> PX4 EKF2
#
# 原生 VehicleOdometry 是生产写入通路，因为它能携带 reset_counter。
# MAVROS ODOMETRY bridge 仅保留作显式诊断/兼容路径，生产 launch 默认关闭。
#
# 新增任何 PX4 EV 写者前，先读 坐标转换.md。
# ============================================================================

# bridges/fastlio_odometry_guard
add_executable(fastlio_odometry_guard src/bridges/fastlio_odometry_guard.cpp)
ament_target_dependencies(fastlio_odometry_guard geometry_msgs nav_msgs rclcpp)
install(TARGETS fastlio_odometry_guard DESTINATION lib/${PROJECT_NAME})

# bridges/fastlio_mavros_odometry_bridge —— 兼容/诊断路径（默认关闭）
add_executable(fastlio_mavros_odometry_bridge src/bridges/fastlio_mavros_odometry_bridge.cpp)
ament_target_dependencies(fastlio_mavros_odometry_bridge geometry_msgs nav_msgs rclcpp std_msgs)
install(TARGETS fastlio_mavros_odometry_bridge DESTINATION lib/${PROJECT_NAME})

# PX4-native EV path.  Unlike nav_msgs/Odometry through MAVROS, VehicleOdometry
# carries reset_counter all the way into EKF2.
add_executable(fastlio_px4_vehicle_odometry src/bridges/fastlio_px4_vehicle_odometry.cpp)
ament_target_dependencies(
	fastlio_px4_vehicle_odometry geometry_msgs nav_msgs px4_msgs rclcpp sensor_msgs std_msgs)
target_link_libraries(fastlio_px4_vehicle_odometry frame_transforms Eigen3::Eigen)
install(TARGETS fastlio_px4_vehicle_odometry DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs nav_msgs sensor_msgs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

if(BUILD_TESTING)
	find_package(Python3 REQUIRED COMPONENTS Interpreter)
	find_package(ament_cmake_gtest REQUIRED)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()

	# pytest 的插件自动加载会与系统 anyio 插件冲突，统一关闭。
	set(PYTEST_ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1;PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")

	ament_add_gtest(test_lever_arm_compensation test/test_lever_arm_compensation.cpp)
	target_include_directories(test_lever_arm_compensation PRIVATE include)

	foreach(pytest_name
			test_ev_health
			test_ev_odometry_acceptance
			test_mid360_lever_arm_config
			test_mavros_state_monitor)
		add_test(
			NAME ${pytest_name}
			COMMAND ${Python3_EXECUTABLE} -m pytest -q
				${CMAKE_CURRENT_SOURCE_DIR}/test/${pytest_name}.py)
		set_tests_properties(${pytest_name} PROPERTIES ENVIRONMENT "${PYTEST_ENV}")
	endforeach()
endif()

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/check_fastlio_vision_yaw.py
  scripts/fix_mavros_odometry_frames.py
  scripts/fix_mavros_identity.py
  scripts/fastlio_ev_health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
