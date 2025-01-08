#!/usr/bin/env python3

import os
import shutil
from pathlib import Path

def create_file(path, content=''):
    """Create a file with given content."""
    with open(path, 'w') as f:
        f.write(content)

def create_project_structure():
    # Project root directory
    root_dir = Path('cable_robot_ws')
    
    # Create root directory
    root_dir.mkdir(exist_ok=True)
    
    # Create main directories
    dirs = [
        'src/cable_robot_firmware/src',
        'src/cable_robot_firmware/include',
        'src/cable_robot_firmware/lib',
        'src/cable_robot_firmware/test',
        'src/cable_robot/launch',
        'src/cable_robot/config',
        'src/cable_robot/src',
        'src/cable_robot/include/cable_robot',
        'src/cable_robot/scripts',
        'src/cable_robot/msg',
        'src/cable_robot/srv',
        'src/cable_robot_description/urdf',
        'src/cable_robot_description/meshes/visual',
        'src/cable_robot_description/meshes/collision',
        'src/cable_robot_description/gazebo',
        'cad/assemblies',
        'cad/parts/frame',
        'cad/parts/pulleys',
        'cad/parts/end_effector',
        'cad/exports/step',
        'cad/exports/stl',
        'docs/mechanical',
        'docs/electrical',
        'docs/software/api',
        'tests/unit',
        'tests/integration',
        'scripts'
    ]
    
    for dir_path in dirs:
        (root_dir / dir_path).mkdir(parents=True, exist_ok=True)
    
    # Create demo files
    
    # Firmware files
    create_file(root_dir / 'src/cable_robot_firmware/platformio.ini', '''
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
    '''.strip())
    
    create_file(root_dir / 'src/cable_robot_firmware/src/main.cpp', '''
#include <Arduino.h>
#include "motor.h"
#include "sensors.h"

void setup() {
    Serial.begin(115200);
    // Initialize motors and sensors
}

void loop() {
    // Main control loop
}
    '''.strip())
    
    # ROS2 package files
    create_file(root_dir / 'src/cable_robot/package.xml', '''
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cable_robot</name>
  <version>0.0.1</version>
  <description>Cable-driven parallel robot control package</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
    '''.strip())
    
    create_file(root_dir / 'src/cable_robot/CMakeLists.txt', '''
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
    '''.strip())
    
    # Launch file
    create_file(root_dir / 'src/cable_robot/launch/robot.launch.py', '''
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cable_robot',
            executable='cable_controller',
            name='cable_controller',
            output='screen'
        ),
    ])
    '''.strip())
    
    # URDF file
    create_file(root_dir / 'src/cable_robot_description/urdf/cable_robot.urdf.xacro', '''
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="cable_robot">
    <!-- Robot description goes here -->
    <link name="base_link">
        <!-- Base link properties -->
    </link>
</robot>
    '''.strip())
    
    # Structure documentation
    create_file(root_dir / 'STRUCTURE.md', '''
# Cable-Driven Robot Project Structure

## Overview
This repository contains all necessary files for developing and operating a cable-driven parallel robot, including mechanical design, firmware, and ROS2 control software.

## Directory Structure

### /src
Contains all source code for the project.

#### /src/cable_robot_firmware
ESP32 firmware for low-level control:
- `src/`: Main firmware source files
- `include/`: Header files
- `lib/`: Custom libraries
- `test/`: Firmware unit tests
- `platformio.ini`: PlatformIO configuration

#### /src/cable_robot
Main ROS2 package for robot control:
- `launch/`: ROS2 launch files
- `config/`: Configuration files
- `src/`: C++ source code
- `include/`: Header files
- `scripts/`: Python scripts
- `msg/`: Custom message definitions
- `srv/`: Custom service definitions

#### /src/cable_robot_description
Robot description package:
- `urdf/`: URDF model files
- `meshes/`: 3D mesh files
- `gazebo/`: Gazebo simulation files

### /cad
SolidWorks CAD files:
- `assemblies/`: Main assembly files
- `parts/`: Individual component files
- `exports/`: Exported STEP and STL files

### /docs
Project documentation:
- `mechanical/`: Assembly guides, BOM
- `electrical/`: Wiring diagrams
- `software/`: API documentation

### /tests
Test files:
- `unit/`: Unit tests
- `integration/`: Integration tests

### /scripts
Utility scripts for building and managing the project

## Getting Started
1. Clone this repository
2. Install ROS2 and required dependencies
3. Build the workspace: `colcon build`
4. Source the workspace: `source install/setup.bash`

## Development Workflow
1. Mechanical changes should be made in SolidWorks and exported to appropriate formats
2. Update URDF files when mechanical design changes
3. Develop and test firmware using PlatformIO
4. Develop ROS2 nodes for high-level control
5. Test in simulation before deploying to hardware
    '''.strip())
    
    # Create utility scripts
    create_file(root_dir / 'scripts/setup_workspace.sh', '''
#!/bin/bash
cd ..
colcon build
source install/setup.bash
    '''.strip())
    
    os.chmod(root_dir / 'scripts/setup_workspace.sh', 0o755)
    
    print(f"Project structure created in {root_dir}")

if __name__ == "__main__":
    create_project_structure()