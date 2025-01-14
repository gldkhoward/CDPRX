# cable_robot_ws/src/cable_robot_entry/launch/simulation.launch.py
# Description: Launch file for Gazebo simulation

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    entry_pkg_dir = get_package_share_directory('cable_robot_entry')
    description_pkg_dir = get_package_share_directory('cable_robot_description')
    gazebo_pkg_dir = get_package_share_directory('cable_robot_gazebo')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Include Gazebo simulation launch
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_dir, 'launch', 'gazebo_simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Return launch description
    return LaunchDescription([
        declare_use_sim_time,
        gazebo_simulation
    ])