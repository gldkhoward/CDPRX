from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def get_workspace_root():
    """Get the workspace root directory from the current package path"""
    pkg_dir = get_package_share_directory('cable_robot_description')
    workspace_root = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..'))
    if os.path.basename(workspace_root) == 'install':
        workspace_root = os.path.dirname(workspace_root)
    return workspace_root

def generate_launch_description():
    # Get the workspace root and set up the path to the generated URDF
    workspace_root = get_workspace_root()
    generated_urdf_path = os.path.join(workspace_root, 'generated_urdfs', 'robot.urdf')

    # Verify URDF exists
    if not os.path.exists(generated_urdf_path):
        raise FileNotFoundError(f"URDF file not found at {generated_urdf_path}. Please generate it first using the robot builder.")

    # Load the URDF content
    with open(generated_urdf_path, 'r') as file:
        robot_description = file.read()

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
            '-entity', 'cable_robot_platform',
            '-file', generated_urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add a log message to show the URDF path
    ld.add_action(LogInfo(msg=['Using URDF file: ', generated_urdf_path]))

    # Add the actions to the launch description
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)

    return ld