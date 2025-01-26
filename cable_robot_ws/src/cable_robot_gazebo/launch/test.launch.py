from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the SDF files
    pkg_dir = get_package_share_directory('cable_robot_description')
    sdf_folder = os.path.join(pkg_dir, 'urdf')

    # Gazebo launch file
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    # Include the Gazebo launch file with the necessary plugins
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'use_sim_time': 'true'  # Ensure ROS 2 uses Gazebo's simulation time
        }.items()
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the Gazebo launch action
    ld.add_action(gazebo_launch)

    # Spawn each pillar separately
    for i in range(1, 5):
        sdf_path = os.path.join(sdf_folder, f"pillar_{i}.sdf")
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', f'pillar_{i}',  # Entity name
                '-file', sdf_path          # Path to the SDF file
            ],
            output='screen'
        )
        ld.add_action(spawn_entity_node)

    # Spawn the platform
    platform_sdf_path = os.path.join(sdf_folder, f"platform.sdf")
    spawn_platform_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cable_robot_platform',  # Entity name
            '-file', platform_sdf_path,          # Path to the SDF file
        ],
        output='screen'
    )
    ld.add_action(spawn_platform_node)

    return ld