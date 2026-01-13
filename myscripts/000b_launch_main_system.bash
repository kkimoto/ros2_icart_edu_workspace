
source install/setup.bash

# robot_bringup_1.launch.py launches nodes below
# ---
# static_transforms.launch.py 
# yp_spur_b.launch.py
# urg_node2_b.launch.py
# urg3d_node2_b.launch.py
# yvt_to_scan_b.launch.py
# laser_scan_merger_b.launch.py
# teleop_b.launch.py
# icart_b.launch.py

ros2 launch edu_robot robot_bringup_1.launch.py

