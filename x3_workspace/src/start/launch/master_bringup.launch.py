import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction # Removed DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # --- No Launch Arguments Needed if not using use_sim_time ---
    # Removed use_sim_time_arg

    # --- Get Package Share Directories ---
    # Ensure these package names match your actual package names in your workspace
    slam_gmapping_pkg_dir = get_package_share_directory('slam_gmapping')
    slam_pkg_dir = get_package_share_directory('slam')
    odometry_pkg_dir = get_package_share_directory('odometry')
    control_pkg_dir = get_package_share_directory('control')
    sensor_fusion_pkg_dir = get_package_share_directory('sensor_fusion')
    planner_pkg_dir = get_package_share_directory('planner')

    ld = LaunchDescription()

    # --- Define a Base Delay for Staging ---
    # Adjust these delay times based on how long each component typically takes to initialize.
    # Start with a small value (e.g., 2-5 seconds) and increase if necessary.
    base_delay = 3.0 # seconds
    current_delay = 0.0

    # --- Launch Files in Sequential Stages ---

    # Stage 1: Launch Odometry
    # Provide the robot's self-localization feedback
    ld.add_action(
        TimerAction(
            period=current_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(odometry_pkg_dir, 'launch', 'odometry_launch.py')
                    ),
                    launch_arguments={
                        # No use_sim_time argument needed here
                    }.items()
                )
            ]
        )
    )
    current_delay += base_delay # Increment delay for the next stage

    # Stage 2: Launch SLAM Gmapping (or other SLAM method)
    # Requires odometry and sensor data (e.g., laser scans) to build a map and localize
    ld.add_action(
        TimerAction(
            period=current_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_gmapping_pkg_dir, 'launch', 'slam_gmapping.launch.py')
                    ),
                    launch_arguments={
                        # No use_sim_time argument needed here
                    }.items()
                )
            ]
        )
    )
    current_delay += base_delay
    
    # Stage 3: Launch SLAM (If this is a different SLAM method or a higher-level SLAM)
    ld.add_action(
        TimerAction(
            period=current_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_pkg_dir, 'launch', 'slam_launch.py')
                    ),
                    launch_arguments={
                        # No use_sim_time argument needed here
                    }.items()
                )
            ]
        )
    )
    current_delay += base_delay
    
    # Stage 4: Launch Control
    # Takes commands (e.g., from planner or teleop) and sends motor commands to the robot.
    # It typically relies on the robot's current pose from odometry or fused localization.
    ld.add_action(
        TimerAction(
            period=current_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(control_pkg_dir, 'launch', 'control_launch.py')
                    ),
                    launch_arguments={
                        # No use_sim_time argument needed here
                    }.items()
                )
            ]
        )
    )

    # Stage 5: Launch Sensor Fusion
    # Often combines different sensor inputs (e.g., IMU, odometry, GPS) to provide a more robust pose estimate.
    # This usually depends on raw sensor data and/or odometry.
    ld.add_action(
        TimerAction(
            period=current_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(sensor_fusion_pkg_dir, 'launch', 'sensor_fusion_launch.py')
                    ),
                    launch_arguments={
                        # No use_sim_time argument needed here
                    }.items()
                )
            ]
        )
    )
    current_delay += base_delay

    '''
    # Stage 6: Launch Planner
    # Your custom planner node, which depends on map, current pose, and a goal.
    ld.add_action(
        TimerAction(
            period=current_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(planner_pkg_dir, 'launch', 'planner_launcher.py')
                    ),
                    launch_arguments={
                        # No use_sim_time argument needed here
                    }.items()
                )
            ]
        )
    )
    current_delay += base_delay
    '''

    

    return ld
