from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sensor_fusion'),
        'config',
        'sensor_fusion.yaml'
    )
    return LaunchDescription([
        Node(
            package='sensor_fusion',
            executable='sensor_fusion',
            name='sensor_fusion',
            parameters=[config]
        )
    ])
