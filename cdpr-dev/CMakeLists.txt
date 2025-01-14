cmake_minimum_required(VERSION 3.8)
project(cdpr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Tensions.msg"
  DEPENDENCIES geometry_msgs
)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# CDPR Gazebo plugin
add_library(cdpr_plugin SHARED
  src/cdpr_plugin.cpp
)
target_include_directories(cdpr_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cdpr_plugin
  rclcpp
  gazebo_ros
  gazebo_msgs
  sensor_msgs
  geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(cdpr_plugin
  ${cpp_typesupport_target}
)

# Parameter executable
add_executable(param src/param.cpp)
target_include_directories(param PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(param
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install
install(
  TARGETS cdpr_plugin param
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(cdpr_plugin)
ament_export_dependencies(
  rclcpp
  gazebo_ros
  sensor_msgs
  geometry_msgs
)

ament_package()