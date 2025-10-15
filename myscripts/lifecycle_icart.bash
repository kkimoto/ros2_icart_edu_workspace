#!/usr/bin/env bash
# Simple helper for controlling the icart_lifecycle_wrapper node
# Usage:
#   ./lifecycle_icart.bash <state>
# Example:
#   ./lifecycle_icart.bash configure
#   ./lifecycle_icart.bash activate
#   ./lifecycle_icart.bash deactivate
#   ./lifecycle_icart.bash cleanup
#   ./lifecycle_icart.bash shutdown

NODE="/icart_lifecycle_wrapper"

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
