#!/usr/bin/env bash
set -euo pipefail

echo "[reset] stopping ros2 daemon..."
ros2 daemon stop || true

echo "[reset] killing common ROS 2 processes..."
pkill -f component_container || true
pkill -f rviz2 || true
pkill -f ros2 || true
pkill -f python3.*launch || true
pkill -f urg_node2_node || true
pkill -f static_transform_publisher || true

echo "[reset] clearing latest logs..."
rm -rf ~/.ros/log/latest || true

echo "[reset] clearing shared memory segments..."
shopt -s nullglob
for p in /dev/shm/*fastdds* /dev/shm/*FastDDS* /dev/shm/*dds* /dev/shm/*iceoryx* /dev/shm/*ros2*; do
  rm -rf "$p" || true
done
shopt -u nullglob

# added
killall -w -r ros2 || true
sleep 3;
#

killall joy_node
killall icart_mini_driver

echo "[reset] starting ros2 daemon..."
ros2 daemon start || true

echo "[reset] done. Now re-source your overlays in order:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/ros2_workspaces/edu_ws/install/setup.bash"
