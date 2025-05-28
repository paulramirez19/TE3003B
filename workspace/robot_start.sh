echo "Starting multiple ROS 2 launch files..."
source install/setup.bash
ros2 launch slam_gmapping slam_gmapping.launch.py &
PID_1=$!
ros2 launch odometry odometry_launch.py &
PID_2=$!
ros2 launch sensor_fusion sensor_fusion_launch.py &
PID_3=$!
ros2 launch control control_launch.py&
PID_4=$!
ros2 launch slam slam_launch.py&
PID_5=$!

