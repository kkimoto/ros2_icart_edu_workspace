
source install/setup.bash

ros2 lifecycle set /teleop_lifecycle_wrapper configure
sleep 2
ros2 lifecycle set /teleop_lifecycle_wrapper activate

echo "Launching slam_sync.launch.py in the background..."
ros2 launch edu_robot slam_sync.launch.py > /tmp/slam_launch.log 2>&1 &
SLAM_PID=$!

echo "SLAM Launch PID: $SLAM_PID"
echo "SLAM is running. Build your map now."

# --- Wait for User Input to Terminate ---

echo "Press [Enter] to terminate SLAM/Teleop nodes..."
read -r # Waits for any keyboard input (e.g., Return/Enter key)

# Deactivate Teleop Node (Lifecycle Node)
echo "Deactivating /teleop_lifecycle_wrapper..."
ros2 lifecycle set /teleop_lifecycle_wrapper deactivate
sleep 2
ros2 lifecycle set /teleop_lifecycle_wrapper cleanup

# 3. Terminate SLAM Node using its PID
echo "Terminating /slam_toolbox (PID: $SLAM_PID)..."
# Send SIGINT (graceful termination signal, equivalent to Ctrl+C)
kill -INT "$SLAM_PID"

# Wait a moment for cleanup
sleep 2

echo "SLAM and Teleop processes stopped."
