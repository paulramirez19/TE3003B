echo "Starting multiple ROS 2 launch files..."
source install/setup.bash
ros2 launch s8_py_slam spawn_robot.launch.py &
PID_6=$!
ros2 launch s8_py_slam display_robot.launch.py &
PID_7=$!
