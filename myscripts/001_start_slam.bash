
source install/setup.bash

ros2 lifecycle set /teleop_lifecycle_wrapper configure
sleep 3
ros2 lifecycle set /teleop_lifecycle_wrapper activate

ros2 launch edu_robot slam_sync.launch.py &

