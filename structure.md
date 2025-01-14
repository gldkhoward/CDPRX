# Cable-Driven Parallel Robot (CDPR) System

## Project Overview
This project implements a modular Cable-Driven Parallel Robot (CDPR) system using ROS2. The system is designed to support both simulation and physical hardware operation, with a focus on reconfigurability and ease of use.

## Directory Structure

```plaintext
cable_robot_ws/
├── src/                           # Source packages directory
│   ├── cable_robot_entry/        # System entry points and launch files
│   ├── cable_robot_interfaces/   # Message and service definitions
│   ├── cable_robot_core/        # Core CDPR functionality
│   ├── cable_robot_description/ # Robot description files
│   ├── cable_robot_gazebo/     # Simulation environment
│   ├── cable_robot_hardware/   # Hardware interface
│   └── cable_robot_gui/       # User interface
├── config/                   # Global configuration files
├── docs/                    # Project documentation
└── scripts/                # Utility scripts