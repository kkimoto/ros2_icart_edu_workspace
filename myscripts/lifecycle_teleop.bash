#!/usr/bin/env bash
# Simple helper for controlling the teleop_lifecycle_wrapper node
# Usage:
#   ./lifecycle_teleop.bash <state>
# Example:
#   ./lifecycle_teleop.bash configure
#   ./lifecycle_teleop.bash activate
#   ./lifecycle_teleop.bash deactivate
#   ./lifecycle_teleop.bash cleanup
#   ./lifecycle_teleop.bash shutdown

NODE="/teleop_lifecycle_wrapper"

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
