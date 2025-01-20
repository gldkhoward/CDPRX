cmake_minimum_required(VERSION 3.8)
project(cable_robot_core)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Create the core library
add_library(${PROJECT_NAME}
  src/core/types.cpp
  #src/kinematics/forward_kinematics.cpp
  src/kinematics/inverse_kinematics.cpp
  #src/kinematics/jacobian.cpp
  #src/workspace/wrench_closure.cpp
  #src/workspace/wrench_feasible.cpp
  #src/workspace/workspace_analyzer.cpp
  #src/controllers/tension_controller.cpp
  #src/controllers/trajectory_controller.cpp
  src/utils/utils.cpp
  #src/utils/configuration.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install includes
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Kinematics tests
  ament_add_gtest(test_forward_kinematics
    test/kinematics/test_forward_kinematics.cpp
  )
  target_link_libraries(test_forward_kinematics ${PROJECT_NAME})
  
  ament_add_gtest(test_inverse_kinematics
    test/kinematics/test_inverse_kinematics.cpp
  )
  target_link_libraries(test_inverse_kinematics ${PROJECT_NAME})
  
  # Workspace tests
  ament_add_gtest(test_workspace
    test/workspace/test_workspace.cpp
  )
  target_link_libraries(test_workspace ${PROJECT_NAME})
  
  # Controller tests
  ament_add_gtest(test_controllers
    test/controllers/test_controllers.cpp
  )
  target_link_libraries(test_controllers ${PROJECT_NAME})
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  visualization_msgs
  yaml-cpp
  Eigen3
  tf2_ros
)

ament_package()