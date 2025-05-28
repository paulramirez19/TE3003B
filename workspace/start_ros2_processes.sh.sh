#!/bin/bash

# This script launches multiple ROS 2 launch files simultaneously.


PID_FILE="/tmp/ros2_launch_pids.txt" # File to store PIDs
rm -f "$PID_FILE" # Clean up any old PID file at start

echo "Starting multiple ROS 2 launch files..."

source install/setup.bash

ros2 launch slam_gmapping slam_gmapping.launch.py &
echo $! >> "$PID_FILE" # Save PID to file

ros2 launch odometry odometry_launch.py &
echo $! >> "$PID_FILE" # Save PID to file

ros2 launch sensor_fusion sensor_fusion_launch.py &
echo $! >> "$PID_FILE" # Save PID to file

ros2 launch control control_launch.py &
echo $! >> "$PID_FILE" # Save PID to file

ros2 launch slam slam_launch.py &
echo $! >> "$PID_FILE" # Save PID to file

echo "All specified launch files have been initiated. PIDs saved to $PID_FILE"
echo "To stop them, run: ./stop_ros2_processes.sh"
