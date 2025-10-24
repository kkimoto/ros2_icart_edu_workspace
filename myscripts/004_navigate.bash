source install/setup.bash

#ros2 launch edu_robot bringup_sequenced.launch.py 

ros2 launch edu_robot bringup_sequenced.launch.py map:=/home/kimoto/ros2_workspace/edu_ws/map/map.yaml params_file:=/home/kimoto/ros2_workspace/edu_ws/src/edu_robot/config/params_default.yaml
