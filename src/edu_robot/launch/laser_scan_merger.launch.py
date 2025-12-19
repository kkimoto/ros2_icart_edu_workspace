import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer # <-- コンテナアクションを使用
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # パス設定
    edu_robot_pkg_dir = get_package_share_directory('edu_robot')
    config_file_path = os.path.join(edu_robot_pkg_dir, 'config', 'params_default.yaml')

    # 1. パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        full_config = yaml.safe_load(file)
        if 'scan_merger' not in full_config or 'ros__parameters' not in full_config['scan_merger']:
            raise KeyError("params_default.yaml に 'scan_merger: ros__parameters' ブロックがありません。")
            
        config_params_merger = full_config['scan_merger']['ros__parameters']
        
    # Launch Argumentの定義 (トピック名をLaunchファイル側で制御するため)
    scan_topic_name_1st_arg = DeclareLaunchArgument('scan_topic_name_1st', default_value='scan_1st')
    scan_topic_name_2nd_arg = DeclareLaunchArgument('scan_topic_name_2nd', default_value='scan_2nd')
    scan_topic_name_3rd_arg = DeclareLaunchArgument('scan_topic_name_3rd', default_value='scan_3rd')

    
    # 2. ComposableNodeContainer を起動し、その中に Merging コンポーネントをロードする
    merger_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="scan_merger", # 独自のノード名に変更
        namespace="",
        composable_node_descriptions=[
            ComposableNode(
                package="laser_scan_merger",
                plugin="util::LaserScanMerger", # オリジナル Launch からプラグイン名を抽出
                name="laser_scan_merger_node",
                
                # 統合されたYAMLファイルから読み込んだパラメータを使用
                parameters=[config_params_merger], 
                
                # リマップ設定: Lidarの出力トピック名とMergerの入力トピック名を合わせる
                remappings=[
                    ('scan_in_1', LaunchConfiguration('scan_topic_name_1st')),
                    ('scan_in_2', LaunchConfiguration('scan_topic_name_2nd')),
                    ('scan_in_3', LaunchConfiguration('scan_topic_name_3rd')),
                    ('scan_out', 'scan_merged'),
                ],
            )
        ],
        output="screen"
    )

    return LaunchDescription([
        scan_topic_name_1st_arg,
        scan_topic_name_2nd_arg,
        scan_topic_name_3rd_arg,
        merger_container,
    ])
