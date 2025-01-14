#!/usr/bin/env python3

import os
import sys
import shutil
from pathlib import Path
from typing import List, Dict, Union

class WorkspaceSetup:
    def __init__(self, base_path: str = "cable_robot_ws"):
        """Initialize workspace setup with base path."""
        self.base_path = Path(base_path)
        self.src_path = self.base_path / "src"
        
    def create_directory(self, path: Union[str, Path]) -> None:
        """Create directory if it doesn't exist."""
        dir_path = Path(path)
        if not dir_path.exists():
            dir_path.mkdir(parents=True)
            print(f"Created directory: {dir_path}")

    def create_file_with_content(self, path: Union[str, Path], content: str) -> None:
        """Create a file with the specified content."""
        file_path = Path(path)
        self.create_directory(file_path.parent)
        with open(file_path, 'w') as f:
            f.write(content)
        print(f"Created file: {file_path}")

    def create_file_with_comment(self, path: Union[str, Path], description: str) -> None:
        """Create a file with just a path comment and description."""
        file_path = Path(path)
        content = f"# {file_path}\n# Description: {description}\n"
        self.create_file_with_content(file_path, content)

    def create_package_xml(self, package_path: Path, package_name: str, dependencies: List[str]) -> None:
        """Create a package.xml file."""
        deps = "\n".join(f"  <depend>{dep}</depend>" for dep in dependencies)
        
        content = f'''<?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
    <name>{package_name}</name>
    <version>0.0.1</version>
    <description>TODO: Package description</description>
    <maintainer email="todo@todo.com">TODO: Maintainer name</maintainer>
    <license>TODO: License declaration</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    {deps}

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
    </package>
    '''
        self.create_file_with_content(package_path / "package.xml", content)

    def create_cmakelists(self, package_path: Path, package_name: str, is_interface_package: bool = False) -> None:
        """Create a CMakeLists.txt file."""
        interface_content = '''
    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    endif()

    rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RobotState.msg"
    "msg/CableTension.msg"
    "msg/EndEffectorPose.msg"
    "msg/SystemConfig.msg"
    "srv/UpdateConfig.srv"
    "srv/ControlMode.srv"
    "srv/CalibrationService.srv"
    DEPENDENCIES std_msgs geometry_msgs
    )''' if is_interface_package else ''

        # Generate the package dependencies part
        pkg_deps = "\n".join(f"find_package({dep}) REQUIRED" for dep in ["rosidl_default_generators"]) if is_interface_package else ""

        content = f'''cmake_minimum_required(VERSION 3.8)
    project({package_name})

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    {pkg_deps}
    {interface_content}

    ament_package()
    '''
        self.create_file_with_content(package_path / "CMakeLists.txt", content)

    def create_package(self, package_name: str, dependencies: List[str] = None) -> None:
        """Create a basic ROS2 package structure."""
        if dependencies is None:
            dependencies = []
        
        package_path = self.src_path / package_name
        self.create_directory(package_path)
        
        # Create overview.md
        overview_content = f"""# {package_name}

## Overview
TODO: Add package overview and description

## Purpose
TODO: Describe the main purpose of this package

## Components
TODO: List main components and their functions

## Dependencies
{', '.join(dependencies) if dependencies else 'No direct dependencies'}
"""
        self.create_file_with_content(package_path / "overview.md", overview_content)
        
        # Create package.xml and CMakeLists.txt
        self.create_package_xml(package_path, package_name, dependencies)
        self.create_cmakelists(package_path, package_name, package_name == "cable_robot_interfaces")
        
        print(f"Created package: {package_name}")

    def create_entry_package(self):
        """Create the cable_robot_entry package with launch files."""
        package_name = "cable_robot_entry"
        dependencies = ["launch", "launch_ros", "cable_robot_description", "cable_robot_core"]
        
        self.create_package(package_name, dependencies)
        launch_path = self.src_path / package_name / "launch"

        # Create launch files
        launch_files = {
            "robot.launch.py": "Main launch file for real hardware setup",
            "simulation.launch.py": "Launch file for Gazebo simulation",
            "demo.launch.py": "Launch file for demonstration configurations",
            "test.launch.py": "Launch file for testing configurations"
        }

        for launch_file, description in launch_files.items():
            self.create_file_with_comment(
                launch_path / launch_file,
                description
            )

    def create_interfaces_package(self):
        """Create the cable_robot_interfaces package with message and service definitions."""
        package_name = "cable_robot_interfaces"
        dependencies = ["rosidl_default_generators", "std_msgs", "geometry_msgs"]
        
        self.create_package(package_name, dependencies)
        
        # Create message definitions
        msg_path = self.src_path / package_name / "msg"
        msg_files = {
            "RobotState.msg": "Message definition for overall robot state",
            "CableTension.msg": "Message definition for cable tension values",
            "EndEffectorPose.msg": "Message definition for end effector pose",
            "SystemConfig.msg": "Message definition for system configuration"
        }

        for msg_file, description in msg_files.items():
            self.create_file_with_comment(
                msg_path / msg_file,
                description
            )

        # Create service definitions
        srv_path = self.src_path / package_name / "srv"
        srv_files = {
            "UpdateConfig.srv": "Service definition for updating system configuration",
            "ControlMode.srv": "Service definition for changing control modes",
            "CalibrationService.srv": "Service definition for system calibration"
        }

        for srv_file, description in srv_files.items():
            self.create_file_with_comment(
                srv_path / srv_file,
                description
            )

    def create_core_package(self):
        """Create the cable_robot_core package with core functionality."""
        package_name = "cable_robot_core"
        dependencies = [
            "rclcpp",
            "cable_robot_interfaces",
            "geometry_msgs",
            "tf2",
            "tf2_ros"
        ]
        
        self.create_package(package_name, dependencies)
        
        # Create include directory structure
        include_base = self.src_path / package_name / "include" / "cable_robot_core"
        
        # Kinematics headers
        kinematics_files = {
            "forward_kinematics.hpp": "Header for forward kinematics calculations",
            "inverse_kinematics.hpp": "Header for inverse kinematics calculations"
        }
        for file_name, desc in kinematics_files.items():
            self.create_file_with_comment(
                include_base / "kinematics" / file_name,
                desc
            )
        
        # Controllers headers
        controller_files = {
            "tension_controller.hpp": "Header for cable tension control system",
            "trajectory_controller.hpp": "Header for end-effector trajectory control"
        }
        for file_name, desc in controller_files.items():
            self.create_file_with_comment(
                include_base / "controllers" / file_name,
                desc
            )
        
        # Utils headers
        utils_files = {
            "math_utils.hpp": "Header for mathematical utility functions",
            "configuration.hpp": "Header for system configuration management"
        }
        for file_name, desc in utils_files.items():
            self.create_file_with_comment(
                include_base / "utils" / file_name,
                desc
            )
        
        # Create src directory structure
        src_base = self.src_path / package_name / "src"
        
        # Node implementations
        node_files = {
            "kinematics_node.cpp": "ROS2 node for kinematics calculations",
            "controller_node.cpp": "ROS2 node for robot control"
        }
        for file_name, desc in node_files.items():
            self.create_file_with_comment(
                src_base / "nodes" / file_name,
                desc
            )
        
        # Library implementations
        lib_files = {
            "kinematics.cpp": "Implementation of kinematics calculations",
            "controllers.cpp": "Implementation of control systems"
        }
        for file_name, desc in lib_files.items():
            self.create_file_with_comment(
                src_base / "lib" / file_name,
                desc
            )

    def create_description_package(self):
        """Create the cable_robot_description package with URDF and configuration."""
        package_name = "cable_robot_description"
        dependencies = [
            "xacro",
            "urdf",
            "robot_state_publisher"
        ]
        
        self.create_package(package_name, dependencies)
        
        # Create URDF directory structure
        urdf_base = self.src_path / package_name / "urdf"
        
        # Main URDF file
        self.create_file_with_comment(
            urdf_base / "cable_robot.urdf.xacro",
            "Main robot URDF description file"
        )
        
        # Component XACROs
        component_files = {
            "frame.xacro": "XACRO file for robot frame description",
            "cable.xacro": "XACRO file for cable system description",
            "end_effector.xacro": "XACRO file for end effector description",
            "pulley.xacro": "XACRO file for pulley system description"
        }
        for file_name, desc in component_files.items():
            self.create_file_with_comment(
                urdf_base / "components" / file_name,
                desc
            )
        
        # Create directories for meshes
        mesh_dirs = [
            self.src_path / package_name / "meshes" / "visual",
            self.src_path / package_name / "meshes" / "collision"
        ]
        for dir_path in mesh_dirs:
            self.create_directory(dir_path)
        
        # Create configuration files
        config_files = {
            "default.yaml": "Default robot configuration parameters",
            "custom.yaml": "Template for custom robot configurations"
        }
        config_base = self.src_path / package_name / "config" / "robot_configurations"
        for file_name, desc in config_files.items():
            self.create_file_with_comment(
                config_base / file_name,
                desc
            )

    def create_gazebo_package(self):
        """Create the cable_robot_gazebo package for simulation."""
        package_name = "cable_robot_gazebo"
        dependencies = [
            "gazebo_ros",
            "gazebo_ros_pkgs",
            "cable_robot_description",
            "cable_robot_interfaces"
        ]
        
        self.create_package(package_name, dependencies)
        
        # Create world file
        worlds_path = self.src_path / package_name / "worlds"
        self.create_file_with_comment(
            worlds_path / "cable_robot.world",
            "Gazebo world file for cable robot simulation environment"
        )
        
        # Create plugin directory and files
        plugin_path = self.src_path / package_name / "plugins" / "cable_plugin"
        self.create_directory(plugin_path)
        
        # Create launch files
        launch_path = self.src_path / package_name / "launch"
        self.create_file_with_comment(
            launch_path / "gazebo_simulation.launch.py",
            "Launch file for Gazebo simulation with cable robot"
        )

    def create_hardware_package(self):
        """Create the cable_robot_hardware package for hardware interface."""
        package_name = "cable_robot_hardware"
        dependencies = [
            "rclcpp",
            "hardware_interface",
            "controller_manager",
            "cable_robot_interfaces",
            "pluginlib"
        ]
        
        self.create_package(package_name, dependencies)
        
        # Create include directory structure
        include_base = self.src_path / package_name / "include" / "cable_robot_hardware"
        
        # Hardware interface headers
        hw_files = {
            "hardware_interface.hpp": "Header for ROS2 hardware interface implementation",
            "motor_controller.hpp": "Header for motor controller communication and control"
        }
        for file_name, desc in hw_files.items():
            self.create_file_with_comment(
                include_base / file_name,
                desc
            )
        
        # Create src directory structure
        src_base = self.src_path / package_name / "src"
        src_files = {
            "hardware_interface_node.cpp": "ROS2 node for hardware interface implementation",
            "motor_controller.cpp": "Implementation of motor controller functionality"
        }
        for file_name, desc in src_files.items():
            self.create_file_with_comment(
                src_base / file_name,
                desc
            )

    def setup_global_config(self):
        """Set up global configuration directories and files."""
        config_base = self.base_path / "config"
        
        # Robot configurations
        robot_config_files = {
            "default_setup.yaml": "Default robot setup parameters",
            "testing_setup.yaml": "Configuration for testing scenarios"
        }
        for file_name, desc in robot_config_files.items():
            self.create_file_with_comment(
                config_base / "robot_configs" / file_name,
                desc
            )
        
        # Controller configurations
        self.create_file_with_comment(
            config_base / "controller_configs" / "pid_gains.yaml",
            "PID controller gains and parameters"
        )
        
        # Simulation configurations
        self.create_file_with_comment(
            config_base / "simulation_configs" / "gazebo_params.yaml",
            "Gazebo simulation parameters and settings"
        )

    def create_gui_package(self):
        """Create the cable_robot_gui package for user interface."""
        package_name = "cable_robot_gui"
        dependencies = [
            "rclcpp",
            "cable_robot_interfaces",
            "qt5",
            "rviz2"
        ]
        
        self.create_package(package_name, dependencies)
        
        # Create src directory structure
        src_base = self.src_path / package_name / "src"
        
        # Main GUI node
        self.create_file_with_comment(
            src_base / "gui_node.cpp",
            "Main ROS2 node for GUI application"
        )
        
        # Create widgets directory
        self.create_directory(src_base / "widgets")
        
        # Create resource directories
        resource_base = self.src_path / package_name / "resources"
        for resource_dir in ["images", "styles"]:
            self.create_directory(resource_base / resource_dir)
        
        # Create config file
        self.create_file_with_comment(
            self.src_path / package_name / "config" / "gui_config.yaml",
            "Configuration file for GUI settings and parameters"
        )

    def setup_documentation(self):
        """Set up documentation directory structure."""
        docs_base = self.base_path / "docs"
        
        # Architecture documentation
        arch_files = {
            "system_overview.md": "High-level overview of the system architecture",
            "node_structure.md": "Detailed description of ROS2 node structure"
        }
        for file_name, desc in arch_files.items():
            self.create_file_with_comment(
                docs_base / "architecture" / file_name,
                desc
            )
        
        # Development documentation
        dev_files = {
            "setup_guide.md": "Guide for setting up the development environment",
            "contribution_guide.md": "Guidelines for contributing to the project"
        }
        for file_name, desc in dev_files.items():
            self.create_file_with_comment(
                docs_base / "development" / file_name,
                desc
            )
        
        # API documentation
        self.create_file_with_comment(
            docs_base / "api" / "interfaces.md",
            "Documentation of ROS2 interfaces and APIs"
        )

    def setup_utility_scripts(self):
        """Set up utility scripts."""
        scripts_base = self.base_path / "scripts"
        
        script_files = {
            "setup_workspace.sh": "Script to set up the development workspace",
            "build_all.sh": "Script to build all packages",
            "run_tests.sh": "Script to run all tests"
        }
        
        for script_name, desc in script_files.items():
            script_content = f"""#!/bin/bash
# {desc}
# TODO: Add script implementation
"""
            self.create_file_with_content(scripts_base / script_name, script_content)
            # Make scripts executable
            script_path = scripts_base / script_name
            script_path.chmod(script_path.stat().st_mode | 0o111)

    def setup_workspace(self):
        """Set up the complete workspace structure."""
        # Create base directories
        self.create_directory(self.base_path)
        self.create_directory(self.src_path)
        
        # Create packages
        self.create_entry_package()
        self.create_interfaces_package()
        self.create_core_package()
        self.create_description_package()
        self.create_gazebo_package()
        self.create_hardware_package()
        self.create_gui_package()
        
        # Set up additional directories
        self.setup_global_config()
        self.setup_documentation()
        self.setup_utility_scripts()
        
        print("Complete workspace structure created successfully")

def main():
    setup = WorkspaceSetup()
    setup.setup_workspace()

if __name__ == "__main__":
    main()