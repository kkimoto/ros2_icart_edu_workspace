source install/setup.bash

#ros2 launch edu_robot bringup_sequenced.launch.py 

ros2 launch edu_robot bringup_sequenced.launch.py map:=/home/kimoto/ros2_workspace/edu_ws/map/map.yaml params_file:=/home/kimoto/ros2_workspace/edu_ws/src/edu_robot/config/params_default.yaml > /tmp/nav2_launch.log 2>&1 &

####

# Wait a few seconds for all child nodes to start
sleep 3

NAV2_LAUNCH_PID=$!
echo "Launch PID: $NAV2_LAUNCH_PID"

echo "Press [Enter] to terminate Nav2 container and manager..."
read -r

# --- 1. Nav2 container を kill ---
CONTAINER_PID=$(ros2 node list | grep nav2_container | xargs -I{} ps -ef | grep {} | grep -v grep | awk '{print $2}')
if [ -n "$CONTAINER_PID" ]; then
    echo "Killing Nav2 container PID: $CONTAINER_PID"
    kill -INT $CONTAINER_PID
fi

# --- 2. lifecycle_manager_navigation を shutdown ---
if ros2 node list | grep -q lifecycle_manager_navigation; then
    echo "Shutting down lifecycle_manager_navigation"
    ros2 lifecycle set /lifecycle_manager_navigation shutdown
fi

# --- 3. launch_ros_* ノードを kill ---
for launch_node in $(ros2 node list | grep launch_ros_); do
    PID=$(ps -ef | grep "$launch_node" | grep -v grep | awk '{print $2}')
    if [ -n "$PID" ]; then
        echo "Killing launch node: $launch_node PID: $PID"
        kill -INT $PID
    fi
done

# --- 4. launch プロセス自体も終了 ---
echo "Killing launch PID: $NAV2_LAUNCH_PID"
kill -INT "$NAV2_LAUNCH_PID"

echo "Nav2 container and manager terminated. Robot base nodes untouched."
