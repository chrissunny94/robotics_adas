cmake_minimum_required(VERSION 3.15)
project(CPP_Interview_Examples LANGUAGES CXX)

# Use modern C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- C++17 examples ---
add_executable(if_constexpr CPP_17/if_constexpr.cpp)
set_target_properties(if_constexpr PROPERTIES CXX_STANDARD 17)

add_executable(std_optional CPP_17/std_optional.cpp)
set_target_properties(std_optional PROPERTIES CXX_STANDARD 17)

add_executable(structured_bindings CPP_17/structured_bindings.cpp)
set_target_properties(structured_bindings PROPERTIES CXX_STANDARD 17)

# Parallel algorithms not supported in AppleClang/libc++
# add_executable(parallel_algorithms CPP_17/parallel_algorithms.cpp)
# set_target_properties(parallel_algorithms PROPERTIES CXX_STANDARD 17)


# --- C++20 examples ---
add_executable(concepts CPP_20/concepts.cpp)
set_target_properties(concepts PROPERTIES CXX_STANDARD 20)

add_executable(ranges CPP_20/ranges.cpp)
set_target_properties(ranges PROPERTIES CXX_STANDARD 20)

#add_executable(modules CPP_20/modules.cpp)
#set_target_properties(modules PROPERTIES CXX_STANDARD 20)

add_executable(coroutines CPP_20/coroutines.cpp)
set_target_properties(coroutines PROPERTIES CXX_STANDARD 20)


add_executable(tii CPP_20/coroutines.cpp)
set_target_properties(tii PROPERTIES CXX_STANDARD 20)

add_executable(dijkstras CPP_20/dijkstras.cpp)
set_target_properties(dijkstras PROPERTIES CXX_STANDARD 20)

# Require C++17 or C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_pub CPP_20/image_pub.cpp)
ament_target_dependencies(image_pub rclcpp sensor_msgs cv_bridge image_transport OpenCV)

install(TARGETS
  image_pub
  DESTINATION lib/${PROJECT_NAME}
)
