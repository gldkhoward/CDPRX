from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the Xacro file
    pkg_dir = get_package_share_directory('cable_robot_description')
    xacro_path = os.path.join(pkg_dir, 'urdf', 'test_robot.urdf.xacro')

    # Generate URDF from Xacro
    robot_description = Command(['xacro ', xacro_path])

    # Gazebo launch file
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    # Node to publish the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Node to spawn the robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'test_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-topic', '/robot_description'  # Use the topic instead of a file
        ],
        output='screen'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)

    return ld