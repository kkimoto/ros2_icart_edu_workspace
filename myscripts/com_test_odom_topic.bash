ros2 topic echo /tf --filter "m.transforms[0].header.frame_id == 'odom'"|egrep sec
