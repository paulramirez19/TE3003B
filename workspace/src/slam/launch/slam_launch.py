import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('slam')


    return LaunchDescription([
        Node(
	    package='slam',
	    executable='pose_publisher_exe', # This must match the name in setup.py entry_points
            name='pose_publisher', # A unique name for the node instance
	    output='screen'
        )
    ])
