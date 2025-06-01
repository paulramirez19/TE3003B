from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('filter_tuning'),
        'config',
        'input_control.yaml'
    )
    return LaunchDescription([
        Node(
            package='filter_tuning',
            executable='filter_tuning',
            name='filter_tuning'
        ),
        Node(
            package='filter_tuning',
            executable='input_control',
            name='input_control',
            parameters=[config]
        )
    ])