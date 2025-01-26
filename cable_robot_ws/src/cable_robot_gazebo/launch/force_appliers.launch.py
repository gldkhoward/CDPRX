from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the nodes'
        ),
        DeclareLaunchArgument(
            'num_attachment_points',
            default_value='4',
            description='Number of cable attachment points'
        ),
        
        Node(
            package='cable_robot_gazebo',
            executable='force_applier',
            name='force_applier',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'num_attachment_points': LaunchConfiguration('num_attachment_points')
            }],
            arguments=['--ros-args', '--log-level', 'debug'],
            output='screen'
        )
    ])