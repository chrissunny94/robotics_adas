cmake_minimum_required(VERSION 3.8)
project(calibration_fusion)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# LiDAR + Camera executable
add_executable(lidar_camera_node
  src/lidar_camera.cpp
  src/main.cpp
)

ament_target_dependencies(lidar_camera_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  message_filters
)

# LiDAR + LiDAR executable
add_executable(lidar_lidar_node
  src/lidar_lidar.cpp
  src/main.cpp
)

ament_target_dependencies(lidar_lidar_node
  rclcpp
  sensor_msgs
  message_filters
)

# Install executables
install(TARGETS
  lidar_camera_node
  lidar_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
