
# 1. Create the target directory (if it doesn't exist)
mkdir -p map

# 2. Run the map saver, specifying the full path and filename prefix
ros2 run nav2_map_server map_saver_cli -f map/mymap
