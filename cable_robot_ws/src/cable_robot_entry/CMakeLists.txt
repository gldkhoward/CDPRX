#cable_robot_ws/src/cable_robot_entry/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(cable_robot_entry)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()