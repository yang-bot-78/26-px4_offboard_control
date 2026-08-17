cmake_minimum_required(VERSION 3.8)
project(race_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)

# ============================================================
# fastlio_bridge_node (existing)
# ============================================================
add_executable(fastlio_bridge_node src/fastlio_bridge_node.cpp)
target_include_directories(fastlio_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fastlio_bridge_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# ============================================================
# map_io_node - PCD 地图保存/加载/发布
# ============================================================
add_executable(map_io_node src/map_io_node.cpp)
target_include_directories(map_io_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_io_node
  rclcpp
  sensor_msgs
  std_srvs
  PCL
  pcl_conversions
)
target_compile_definitions(map_io_node PRIVATE
  PCL_NO_PRECOMPILE
)

# ============================================================
# mapping_manager_node - 建图管理 (自动加载/保存)
# ============================================================
add_executable(mapping_manager_node src/mapping_manager_node.cpp)
target_include_directories(mapping_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mapping_manager_node
  rclcpp
  std_srvs
  std_msgs
)

# ============================================================
# frame_transform_node - 坐标系 TF 发布
# ============================================================
add_executable(frame_transform_node src/frame_transform_node.cpp)
target_include_directories(frame_transform_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frame_transform_node
  rclcpp
  geometry_msgs
  tf2_ros
)

# ============================================================
# race_mapping_demo_node - 调试用 demo 点云发布
# ============================================================
add_executable(race_mapping_demo_node src/race_mapping_demo_node.cpp)
target_include_directories(race_mapping_demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(race_mapping_demo_node
  rclcpp
  sensor_msgs
)

# ============================================================
# Install all executables
# ============================================================
install(TARGETS
  fastlio_bridge_node
  map_io_node
  mapping_manager_node
  frame_transform_node
  race_mapping_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
