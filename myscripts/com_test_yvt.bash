echo "Launch YVT"
ros2 launch edu_robot urg3d_node2.launch.py &
sleep 3
ros2 lifecycle set /yvt_3d configure
sleep 1
ros2 lifecycle set /yvt_3d activate
