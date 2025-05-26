import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Declare arguments for the launch file
    pkg_name = 'simulation' # Your package name
    robot_name_in_gazebo = 'yahboomcar' # The name of your robot in Gazebo (from your URDF)

    # Get paths
    pkg_share_dir = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'yahboomcar.urdf') # Ensure this path is correct

    # For a plain URDF, read the file content
    with open(urdf_path, 'r') as file:
        robot_description_content = file.read()

    gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')

    # Arguments for Gazebo
    gazebo_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_ros_package_dir, 'worlds', 'empty.world'), # Or your custom world file path
        description='Gazebo world file'
    )

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            gazebo_ros_package_dir, 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # Publish the robot description to the ROS parameter server
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Spawn the robot into Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name_in_gazebo,
            '-x', '0.0', # Initial X position
            '-y', '0.0', # Initial Y position
            '-z', '0.1'  # Initial Z position (ensure it's above ground)
        ],
        output='screen'
    )

    # Your custom node from entry_points (assuming it's a Python node now)
    odom_to_vel_node = Node(
        package='simulation',
        executable='odom_to_vel_exe', # This must match the name in setup.py entry_points
        name='sim_odom_to_vel', # A unique name for the node instance
        output='screen'
    )

    return LaunchDescription([
        gazebo_world_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        odom_to_vel_node # Include your custom node
    ])
