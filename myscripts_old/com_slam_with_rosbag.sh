
source install/setup.bash

ros2 launch arno_robot static_transforms.launch.py &
sleep 2
ros2 launch arno_robot slam_sync_standalone.launch.py

