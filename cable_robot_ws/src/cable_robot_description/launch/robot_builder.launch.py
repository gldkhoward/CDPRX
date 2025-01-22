from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('cable_robot_description')
    
    return LaunchDescription([
        Node(
            package='cable_robot_description',
            namespace='robot_builer',
            executable='gui_node',
            name='robot_builder_gui',
            output='screen'
        ),
        Node(
            package='cable_robot_description',
            namespace='robot_builer',
            executable='urdf_generator',
            name='urdf_generator',
            output='screen'
        )
    ])