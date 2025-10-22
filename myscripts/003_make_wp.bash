
ros2 launch edu_robot make_wp.launch.py  > /tmp/make_wp_launch.log 2>&1 &
MAKE_WP_PID=$!
echo "MAKE_WayPoint PID: $MAKE_WP_PID"
sleep 3

ros2 lifecycle set /map_server configure
sleep 1
ros2 lifecycle set /map_server activate
sleep 1

ros2 lifecycle set /amcl configure
sleep 1
ros2 lifecycle set /amcl activate
sleep 3

ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{
 header: { frame_id: 'map' },
 pose: {
   pose: {
     position: { x: 0.0, y: 0.0, z: 0.0 },
     orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
   },
   covariance: [0.0,0,0,0,0,0,0,0.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
 } }" --once

echo "Press [Enter] to cleanup AMCL and MAP Server nodes..."
read -r # Waits for any keyboard input (e.g., Return/Enter key)

echo "Deactivating /amcl"
ros2 lifecycle set /amcl deactivate
sleep 2
ros2 lifecycle set /amcl cleanup

echo "Deactivating /map_server"
ros2 lifecycle set /map_server deactivate
sleep 2
ros2 lifecycle set /map_server cleanup

# Wait a moment for cleanup
sleep 2

echo "AMCL and MAP Server processes cleanupped."

kill -INT "$MAKE_WP_PID"

echo "AMCL and MAP Server processes killed (terminated)."
