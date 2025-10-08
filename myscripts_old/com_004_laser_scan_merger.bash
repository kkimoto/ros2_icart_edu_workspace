source install/setup.bash
ros2 launch arno_robot sensors_2d_base.launch.py \
  base_frame:=base_link \
  front_frame:=laser_frame_1 rear_frame:=laser_frame_2 \
  merged_scan:=/scan
