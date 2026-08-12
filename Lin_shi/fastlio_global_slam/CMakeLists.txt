cmake_minimum_required(VERSION 3.8)
project(fastlio_global_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(GTSAM QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadMap.srv"
  "srv/Relocalize.srv"
  "srv/SaveMap.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(fastlio_global_backend src/fastlio_global_backend.cpp)
ament_target_dependencies(
  fastlio_global_backend
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  visualization_msgs
)
rosidl_target_interfaces(fastlio_global_backend ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(fastlio_global_backend PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(fastlio_global_backend ${PCL_LIBRARIES})

if(GTSAM_FOUND)
  target_compile_definitions(fastlio_global_backend PRIVATE FASTLIO_GLOBAL_SLAM_HAS_GTSAM=1)
  target_include_directories(fastlio_global_backend PRIVATE ${GTSAM_INCLUDE_DIR})
  target_link_libraries(fastlio_global_backend gtsam)
endif()

install(TARGETS fastlio_global_backend DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
