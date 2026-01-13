import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    global ser
    # パス設定
    edu_robot_pkg_dir = get_package_share_directory('edu_robot')
    config_file_path = os.path.join(edu_robot_pkg_dir, 'config', 'params_default.yaml')

    # 1. パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        full_config = yaml.safe_load(file)
        if 'scan_merger' not in full_config or 'ros__parameters' not in full_config['scan_merger']:
            raise KeyError("params_default.yaml に 'scan_merger: ros__parameters' ブロックがありません。")
            
        config_params_merger = full_config['scan_merger']['ros__parameters']
        
    # 2. ComposableNodeContainer を起動
    # remappings を削除し、パラメータファイルの scan_topics 設定に従わせます
    merger_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="scan_merger",
        namespace="",
        composable_node_descriptions=[
            ComposableNode(
                package="laser_scan_merger",
                plugin="util::LaserScanMerger",
                name="laser_scan_merger_node",
                parameters=[config_params_merger], 
                # remappings は削除
            )
        ],
        output="screen"
    )


    return LaunchDescription([
        merger_container,
    ])
