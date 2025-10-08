# prerequisite
# sudo apt install ros-humble-topic-tools

# record remapped
#(ros2 bag record /scan_1st /scan_2nd /icart_mini/odom /tf  --output minimal_bag2   --storage mcap)&
(ros2 bag record /scan_1st /scan_2nd /icart_mini/odom /tf /merged_scan  --output minimal_bag3   --storage mcap)&

sleep 3

# relay (remap)
(ros2 run topic_tools relay /urg_node1/scan /scan_1st)&

# relay (remap)
(ros2 run topic_tools relay /urg_node2/scan /scan_2nd)&

# relay (remap)
(ros2 run topic_tools relay /scan /merged_scan)&

# play
ros2 bag play ../bagdata/arno1/241124_atc/2024-11-24-11-07-02   --topics /urg_node1/scan /urg_node2/scan /icart_mini/odom /tf /scan

