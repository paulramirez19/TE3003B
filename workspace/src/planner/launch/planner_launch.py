import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Dynamically determine the path to your map YAML file
    map_yaml_path = PathJoinSubstitution([
        FindPackageShare('planner'),
        'maps',
        'warehouse_map.yaml'
    ])

    map_yaml_path_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value=map_yaml_path,
        description='Full path to the warehouse_map.yaml file'
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_path_arg, 

        # Node 1: Static Map Publisher Node
        Node(
            package='planner',
            executable='static_map_publisher_node', # New executable
            name='static_map_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'map_yaml_path': map_yaml_path} # Pass map path to the map publisher
            ],
            emulate_tty=True,
        ),

        # Node 2: Custom A* Planner Node
        Node(
            package='planner',
            executable='custom_astar_planner_node',
            name='custom_astar_planner',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'planning_frequency': 2.0},
                {'goal_tolerance_xy': 0.3},
                {'obstacle_threshold': 80},
                # No map_yaml_path parameter needed here, as it subscribes to /map
            ],
            emulate_tty=True,
        ),

        # Node 3: Test Robot Pose/Odometry Publisher
        Node(
            package='test_publishers',
            executable='test_robot_pose_odom_publisher',
            name='test_robot_pose_odom_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'initial_x': 0.0},
                {'initial_y': 0.0},
                {'initial_theta': 0.0},
                {'linear_velocity': 0.1}
            ],
            emulate_tty=True,
        )
    ])
