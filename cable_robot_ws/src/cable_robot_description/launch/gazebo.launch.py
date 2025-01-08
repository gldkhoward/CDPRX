#File path: cable_robot_ws/src/cable_robot_description/launch/gazebo.launch.py

#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    pkg_name = 'cable_robot_description'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Gazebo
    gazebo_pkg_dir = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'pause': 'false',
        }.items()
    )

    # URDF
    xacro_file = os.path.join(pkg_dir, 'urdf', 'cable_robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 30.0,
        }]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'cable_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])