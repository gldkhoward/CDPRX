cmake_minimum_required(VERSION 3.5)
project(cable_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(cable_controller src/cable_controller.cpp)
ament_target_dependencies(cable_controller rclcpp std_msgs)

install(TARGETS
  cable_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()