#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get paths to config files
    pkg_share = FindPackageShare('CDPRX')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'testPillar.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'viz.rviz'])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                FindExecutable(name='xacro'), ' ', urdf_path
            ])
        }]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])