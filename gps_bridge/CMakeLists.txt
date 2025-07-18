cmake_minimum_required(VERSION 3.8)
project(gps_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Add executable
add_executable(gps_bridge_node src/gps_bridge_node.cpp)

# Dependencies
ament_target_dependencies(gps_bridge_node 
  rclcpp
  sensor_msgs
  px4_msgs
)

# Install
install(TARGETS gps_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()