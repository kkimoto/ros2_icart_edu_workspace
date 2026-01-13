#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # パスの設定
    edu_robot_dir = get_package_share_directory('edu_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # デフォルトのマップファイルとパラメータファイルのパス
    # ※ マップのパスは環境に合わせて修正してください（今回はbashスクリプトのパスを参考にしています）
    default_map_path = '/home/kimoto/ros2_workspace/edu_ws/map/map.yaml'
    default_params_file = os.path.join(edu_robot_dir, 'config', 'params_default.yaml')

    # Launch 引数の定義
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )

    # Nav2 の一括起動 (bringup_launch.py)
    # これ1つで Localization (AMCL/MapServer) と Navigation (Planner/Controller) が両方立ち上がります
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true',  # 自動で Active にする
            'use_composition': 'False', #地図が見えなかったのでfalseにしてみた。'True', # コンポーネント合成を使って軽量化（推奨）
            'slam': 'False'       # SLAMではなく、既存地図モードで起動
        }.items()
    )

    return LaunchDescription([
        map_yaml_arg,
        params_file_arg,
        nav2_bringup_launch
    ])
