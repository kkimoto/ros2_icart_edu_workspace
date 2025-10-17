#!/usr/bin/env bash
# Simple helper for controlling the urg_node2_lifecycle_wrapper node
# Usage:
#   ./lifecycle_urg_node2.bash <state>
# Example:
#   ./lifecycle_urg_node2.bash configure
#   ./lifecycle_urg_node2.bash activate
#   ./lifecycle_urg_node2.bash deactivate
#   ./lifecycle_urg_node2.bash cleanup
#   ./lifecycle_urg_node2.bash shutdown

NODE="/urg_node2_1st"
NODE2="/urg_node2_2nd"

if [ $# -ne 1 ]; then
  echo "Usage: $0 <configure|activate|deactivate|cleanup|shutdown>"
  exit 1
fi

CMD="$1"

# Verify ROS2 is sourced
if ! command -v ros2 >/dev/null 2>&1; then
  echo "Error: ros2 command not found. Please source your ROS2 environment."
  exit 1
fi

echo "Sending lifecycle transition '$CMD' to $NODE ..."
ros2 lifecycle set "$NODE" "$CMD"

echo "Sending lifecycle transition '$CMD' to $NODE2 ..."
ros2 lifecycle set "$NODE2" "$CMD"
