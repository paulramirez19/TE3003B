#!/bin/bash

echo "Attempting to stop all ROS 2 processes..."

# Send SIGINT (graceful termination request)
echo "Sending SIGINT to all 'ros2' processes."
pkill -SIGINT ros2
sleep 5 # Give them time to shut down

# Check if any 'ros2' processes are still running
if pgrep ros2 > /dev/null; then
    echo "Some 'ros2' processes are still running after SIGINT. Escalating to SIGTERM."
    pkill -SIGTERM ros2 # Send SIGTERM (stronger termination request)
    sleep 5 # Give them more time
    
    # Final check
    if pgrep ros2 > /dev/null; then
        echo "Warning: Some 'ros2' processes still alive after SIGTERM. Sending SIGKILL."
        pkill -SIGKILL ros2 # Force kill
        echo "Forced termination of remaining 'ros2' processes."
    else
        echo "All 'ros2' processes terminated after SIGTERM."
    fi
else
    echo "All 'ros2' processes terminated gracefully."
fi
