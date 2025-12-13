cmake_minimum_required(VERSION 3.8)
project(envsim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Setup Default Build Type as Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# Setup C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Setup Eigen flags
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=1048576)
add_definitions(-DEIGEN_DONT_PARALLELIZE -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(dodgelib REQUIRED)
find_package(dodgeros REQUIRED)
find_package(envsim_msgs REQUIRED)
find_package(dodgeros_msgs REQUIRED)
find_package(flightlib REQUIRED)
find_package(zmqpp_vendor REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${flightlib_INCLUDE_DIRS}
)

# Create executable
add_executable(visionsim_node src/visionsim_node.cpp)

ament_target_dependencies(visionsim_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  rosgraph_msgs
  dodgelib
  dodgeros
  envsim_msgs
  dodgeros_msgs
  flightlib
  zmqpp_vendor
)

target_link_libraries(visionsim_node
  ${OpenCV_LIBRARIES}
  ${flightlib_LIBRARIES}
  yaml-cpp
  zmq
  /opt/ros/humble/lib/libzmqpp.so
)

# Install targets
install(TARGETS visionsim_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and parameter files
install(DIRECTORY launch parameters resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
