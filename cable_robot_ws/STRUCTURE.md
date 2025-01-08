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