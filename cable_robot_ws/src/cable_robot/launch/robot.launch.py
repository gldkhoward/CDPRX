from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cable_robot',
            executable='cable_controller',
            name='cable_controller',
            output='screen'
        ),
    ])