#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory

def get_workspace_root():
    """Get the workspace root directory from the current package path"""
    pkg_dir = get_package_share_directory('cable_robot_description')
    # Navigate up from install/pkg_name/share to workspace root
    workspace_root = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..'))
    # Check if we're in the install directory and go up one more level if needed
    if os.path.basename(workspace_root) == 'install':
        workspace_root = os.path.dirname(workspace_root)
    return workspace_root

class URDFGenerator(Node):
    def __init__(self):
        super().__init__('urdf_generator')
        
        # Set up workspace paths
        ws_root = get_workspace_root()
        self.base_output_dir = os.path.join(ws_root, 'generated_urdfs')
        self.config_dir = os.path.join(self.base_output_dir, 'config')
        
        # Create directories if they don't exist
        os.makedirs(self.base_output_dir, exist_ok=True)
        os.makedirs(self.config_dir, exist_ok=True)
        
        self.config_path = os.path.join(self.config_dir, 'robot_config.json')
        self.urdf_path = os.path.join(self.base_output_dir, 'robot.urdf')
        
        # Create a timer to periodically check for config changes
        self.create_timer(1.0, self.check_config)
        self.last_modified = 0
        
        self.get_logger().info('URDF Generator initialized')
        self.get_logger().info(f'Watching for changes in: {self.config_path}')
        self.get_logger().info(f'URDF will be generated at: {self.urdf_path}')

    def check_config(self):
        """Check if config file has been modified and generate URDF if needed."""
        try:
            if os.path.exists(self.config_path):
                current_modified = os.path.getmtime(self.config_path)
                if current_modified > self.last_modified:
                    self.last_modified = current_modified
                    self.generate_urdf()
        except Exception as e:
            self.get_logger().error(f'Error checking config: {str(e)}')

    def generate_urdf(self):
        """Generate URDF file from config."""
        try:
            # Load configuration
            with open(self.config_path, "r") as f:
                config = json.load(f)

            platform = config["platform"]
            dimensions = platform["dimensions"]
            mass = platform["mass"]
            attachment_points = platform["attachment_points"]

            # Calculate inertia values (simplified box model)
            # For a box: Ixx = m(y² + z²)/12, Iyy = m(x² + z²)/12, Izz = m(x² + y²)/12
            x, y, z = dimensions
            ixx = mass * (y**2 + z**2) / 12
            iyy = mass * (x**2 + z**2) / 12
            izz = mass * (x**2 + y**2) / 12

            # URDF template
            urdf_template = f"""<?xml version="1.0"?>
<robot name="cable_robot_platform">
    <!-- Base Link: World -->
    <link name="world"/>

    <!-- Platform Link -->
    <link name="platform">
        <!-- Visual Properties -->
        <visual>
            <geometry>
                <box size="{dimensions[0]} {dimensions[1]} {dimensions[2]}"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
        <!-- Collision Properties -->
        <collision>
            <geometry>
                <box size="{dimensions[0]} {dimensions[1]} {dimensions[2]}"/>
            </geometry>
        </collision>
        <!-- Inertial Properties -->
        <inertial>
            <mass value="{mass}"/>
            <inertia 
                ixx="{ixx}" ixy="0.0" ixz="0.0" 
                iyy="{iyy}" iyz="0.0" 
                izz="{izz}"/>
        </inertial>
    </link>

    <!-- Floating Joint to Spawn in Gazebo -->
    <joint name="floating_joint" type="floating">
        <parent link="world"/>
        <child link="platform"/>
    </joint>

    <!-- Attachment Points -->"""

            # Add attachment points
            for i, point in enumerate(attachment_points, start=1):
                urdf_template += f"""
    <!-- Attachment Point {i} -->
    <link name="attachment_point_{i}">
        <visual>
            <geometry>
                <sphere radius="0.02"/>
            </geometry>
            <material name="red">
                <color rgba="1 0 0 1"/>
            </material>
        </visual>
        <inertial>
            <mass value="0.01"/>
            <inertia 
                ixx="0.0001" ixy="0.0" ixz="0.0" 
                iyy="0.0001" iyz="0.0" 
                izz="0.0001"/>
        </inertial>
    </link>
    <joint name="attachment_joint_{i}" type="fixed">
        <parent link="platform"/>
        <child link="attachment_point_{i}"/>
        <origin xyz="{point[0]} {point[1]} {point[2]}" rpy="0 0 0"/>
    </joint>"""

            urdf_template += "\n</robot>"

            # Save URDF to file
            with open(self.urdf_path, "w") as f:
                f.write(urdf_template)

            self.get_logger().info(f'Generated URDF file: {self.urdf_path}')

        except FileNotFoundError:
            self.get_logger().warn('Config file not found. Waiting for it to be created...')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in config file: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error generating URDF: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        generator = URDFGenerator()
        rclpy.spin(generator)
    except Exception as e:
        print(f"Error running URDF generator: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()