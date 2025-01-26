cmake_minimum_required(VERSION 3.8)
project(cable_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlatformForces.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_package()

# Original more detailed configuration:
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
#
# find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED) 
#
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/RobotState.msg"
#   "msg/CableTension.msg"
#   "msg/EndEffectorPose.msg"
#   "msg/SystemConfig.msg"
#   "srv/UpdateConfig.srv"
#   "srv/ControlMode.srv"
#   "srv/CalibrationService.srv"
#   DEPENDENCIES std_msgs geometry_msgs
# )