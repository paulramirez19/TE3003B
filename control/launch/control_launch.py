from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'control.yaml')
    return LaunchDescription([
        Node(
            package='control',
            namespace='',
            executable='control',
            name='controller',
            parameters=[config]
        ),
    ])
