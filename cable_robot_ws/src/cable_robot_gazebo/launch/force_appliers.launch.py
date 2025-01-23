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
        
        *[Node(
            package='cable_robot_gazebo',
            executable='force_applier',
            name=f'force_applier_{i}',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'id': i}],
            output='screen'
        ) for i in range(1, 5)]
    ])