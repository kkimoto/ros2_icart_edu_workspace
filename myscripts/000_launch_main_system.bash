
source install/setup.bash

# TF

echo "Launch TF and SCAN-MERGER and JOY"
ros2 launch edu_robot static_transforms.launch.py &
sleep 1

#
# locomotion system (life cycle unconfigured)
#

echo "Launch YP-SPUR"
ros2 launch edu_robot yp_spur.launch.py &
sleep 3
echo "Configure YP-SPUR"
ros2 lifecycle set /yp_spur_lifecycle_wrapper configure
sleep 1
echo "ACTIVATE YP-SPUR"
ros2 lifecycle set /yp_spur_lifecycle_wrapper activate

echo "Launch ICART"
ros2 launch edu_robot icart.launch.py &
sleep 5
echo "CONFIGURE ICART"
ros2 lifecycle set /icart_lifecycle_wrapper configure
echo "ACTIVATE ICART"
ros2 lifecycle set /icart_lifecycle_wrapper activate
sleep 1

#
# sensor system  (life cycle unconfigured)
#

echo "Launch URG"
ros2 launch edu_robot urg_node2.launch.py &
sleep 3
echo "CONFIGURE URG1"
ros2 lifecycle set /urg_node2_1st configure
sleep 1
echo "ACTIVATE URG1"
ros2 lifecycle set /urg_node2_1st activate
sleep 1
echo "CONFIGURE URG2"
ros2 lifecycle set /urg_node2_2nd configure
sleep 1
echo "ACTIVATE URG2"
ros2 lifecycle set /urg_node2_2nd activate
sleep 3

echo "Launch YVT"
ros2 launch edu_robot urg3d_node2.launch.py &
sleep 3
ros2 lifecycle set /yvt_3d configure
sleep 1
ros2 lifecycle set /yvt_3d activate
sleep 3
ros2 launch edu_robot yvt_to_scan.launch.py &

# scan merger
ros2 launch edu_robot laser_scan_merger.launch.py &
sleep 1

echo "Launch TELEOP"
ros2 launch edu_robot teleop.launch.py &

sleep 2
echo "main_system DONE"
