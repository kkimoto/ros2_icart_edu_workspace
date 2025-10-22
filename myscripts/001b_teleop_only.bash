
source install/setup.bash

ros2 lifecycle set /teleop_lifecycle_wrapper configure
sleep 2
ros2 lifecycle set /teleop_lifecycle_wrapper activate

# --- Wait for User Input to Terminate ---

echo "Press [Enter] to cleanup Teleop nodes..."
read -r # Waits for any keyboard input (e.g., Return/Enter key)

# Deactivate Teleop Node (Lifecycle Node)
echo "Deactivating /teleop_lifecycle_wrapper..."
ros2 lifecycle set /teleop_lifecycle_wrapper deactivate
sleep 2
ros2 lifecycle set /teleop_lifecycle_wrapper cleanup

# Wait a moment for cleanup
sleep 2

echo "Teleop processes cleanupped."
