#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('cdprx')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'components', 'test_robot.urdf.xacro')
    
    # Get URDF via xacro
    robot_description_content = Command(['xacro ', urdf_file])
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Launch RViz2 as a process with explicit library path
    rviz_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH '
            'LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 '
            '/opt/ros/humble/lib/rviz2/rviz2'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_process
    ])