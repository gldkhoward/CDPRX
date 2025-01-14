#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    gazebo_pkg_dir = get_package_share_directory('cable_robot_gazebo')
    description_pkg_dir = get_package_share_directory('cable_robot_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    
    # Set Gazebo model path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(description_pkg_dir, 'urdf'), os.path.join(gazebo_pkg_dir, 'models')]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf',
        }.items()
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/simple_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/simple_arm/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # Spawn the robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(description_pkg_dir, 'urdf', 'caroca.sdf'),
            '-name', 'simple_arm',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        bridge,
        spawn,
    ])