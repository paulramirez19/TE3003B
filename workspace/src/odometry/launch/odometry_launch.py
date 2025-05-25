from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('odometry'),
        'config',
        'odometry.yaml'
    )
    return LaunchDescription([
        Node(
            package='odometry',
            executable='odometry',
            name='odometry',
            parameters=[config]
        )
    ])