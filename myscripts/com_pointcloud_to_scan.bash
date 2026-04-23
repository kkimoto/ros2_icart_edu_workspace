
#ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --params-file /home/kimoto/ros2_workspace/edu_ws/src/edu_robot/config/params_default.yaml -r cloud_in:=/hokuyo_cloud2   -r scan:=/yvt_scan 

#ros2 run edu_robot pointcloud_to_laserscan_node --ros-args --params-file /home/kimoto/ros2_workspace/edu_ws/src/edu_robot/config/params_default.yaml -r cloud_in:=/hokuyo_cloud2   -r scan:=/yvt_scan 

ros2 launch edu_robot test_pcl2scan.launch.py

