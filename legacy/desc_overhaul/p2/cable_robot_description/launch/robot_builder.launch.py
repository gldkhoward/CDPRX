from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the robot builder GUI
        Node(
            package='cable_robot_description',
            executable='robot_builder_gui.py',
            output='screen',
            name='robot_builder_gui'
        ),
        # Start the URDF generator (optional, can be triggered manually)
        Node(
            package='cable_robot_description',
            executable='generate_urdf.py',
            output='screen',
            name='generate_urdf'
        )
    ])