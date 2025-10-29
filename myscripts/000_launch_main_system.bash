
source install/setup.bash

#
# locomotion system (life cycle unconfigured)
#

echo "Launch YP-SPUR"
ros2 launch edu_robot yp_spur.launch.py &
sleep 5
echo "Configure YP-SPUR"
ros2 lifecycle set /yp_spur_lifecycle_wrapper configure
sleep 1
echo "ACTIVATE YP-SPUR"
ros2 lifecycle set /yp_spur_lifecycle_wrapper activate

echo "Launch ICART"
ros2 launch edu_robot icart.launch.py &
sleep 3
echo "CONFIGURE ICART"
ros2 lifecycle set /icart_lifecycle_wrapper configure
echo "ACTIVATE ICART"
ros2 lifecycle set /icart_lifecycle_wrapper activate
sleep 3

#
# sensor system  (life cycle unconfigured)
#

echo "Launch URG"
ros2 launch edu_robot urg_node2.launch.py &
sleep 7
echo "CONFIGURE URG1"
ros2 lifecycle set /urg_node2_1st configure
echo "ACTIVATE URG1"
ros2 lifecycle set /urg_node2_1st activate
echo "CONFIGURE URG2"
ros2 lifecycle set /urg_node2_2nd configure
echo "ACTIVATE URG2"
ros2 lifecycle set /urg_node2_2nd activate
sleep 3

# utility

echo "Launch TF and SCAN-MERGER and JOY"
ros2 launch edu_robot static_transforms.launch.py &
sleep 3
ros2 launch edu_robot laser_scan_merger.launch.py &
sleep 1

echo "Launch TELEOP"
ros2 launch edu_robot teleop.launch.py &

echo "main_system DONE"
