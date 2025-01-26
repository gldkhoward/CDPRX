#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to the SDF files
    desc_pkg_dir = get_package_share_directory('cable_robot_description')
    pkg_dir = get_package_share_directory('cable_robot_gazebo')
    sdf_folder = os.path.join( desc_pkg_dir, 'urdf')

    # Declare launch arguments
    declare_world_sdf_file = DeclareLaunchArgument(
        'world_sdf_file',
        default_value='empty.sdf',
        description='Path to the SDF world file'
    )

    # Set environment variables for Gazebo Fortress
    set_gz_sim_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([desc_pkg_dir, 'models'])
    )
    set_gz_sim_plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_PLUGIN_PATH',
        PathJoinSubstitution([desc_pkg_dir, 'plugins'])
    )

    # Include the Gazebo launch file from ros_gz_sim
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': PathJoinSubstitution([pkg_dir, 'worlds', 'empty.world']),  # Replace with your world file
            'on_exit_shutdown': 'True'
        }.items()
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the Gazebo launch action
    ld.add_action(declare_world_sdf_file)
    ld.add_action(set_gz_sim_resource_path)
    ld.add_action(set_gz_sim_plugin_path)
    ld.add_action(gz_sim_launch)

    # Spawn each pillar separately
    for i in range(1, 5):
        sdf_path = os.path.join(sdf_folder, f"pillar_{i}.sdf")
        spawn_pillar = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', sdf_path,
                '-name', f'pillar_{i}',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0'
            ],
            output='screen'
        )
        ld.add_action(spawn_pillar)

    # Spawn the platform
    platform_sdf_path = os.path.join(sdf_folder, f"platform.sdf")
    spawn_platform = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', platform_sdf_path,
            '-name', 'cable_robot_platform',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    ld.add_action(spawn_platform)

    return ld