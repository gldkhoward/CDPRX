cmake_minimum_required(VERSION 3.8)
    project(cable_robot_gazebo)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(gazebo_msgs REQUIRED)
    install(DIRECTORY launch worlds
        DESTINATION share/${PROJECT_NAME}
    )
    
   # Add the force_applier executable
    add_executable(force_applier src/force_applier.cpp)

    # Link dependencies
    ament_target_dependencies(force_applier
    rclcpp
    geometry_msgs
    gazebo_msgs
    )

    # Install the executable
    install(TARGETS
    force_applier
    DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()
    