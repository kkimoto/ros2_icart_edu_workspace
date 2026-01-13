#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_edu_robot = get_package_share_directory('edu_robot')

    # 1. TF (Static Transforms) 【ここに追加】
    # これがないとセンサーデータの位置関係が不明になります
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edu_robot, 'launch', 'static_transforms.launch.py')
        )
    )

    # 2. yp_spur の読み込み
    yp_spur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edu_robot, 'launch', 'yp_spur_b.launch.py')
        )
    )

    # 3. 2D LiDAR (urg_node2) の読み込み
    urg_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edu_robot, 'launch', 'urg_node2_b.launch.py')
        )
    )

    # 4. 【追加】 3D LiDAR (urg3d_node2) の読み込み
    # 独立して動くので、ここで読み込むだけでOK
    urg3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edu_robot, 'launch', 'urg3d_node2_b.launch.py')
        )
    )

    # 5. 【追加】 3D -> 2D 変換 (yvt_to_scan)
    # 3D LiDARのデータを使って変換しますが、普通のノードなので並列起動でOKです
    yvt_to_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edu_robot, 'launch', 'yvt_to_scan_b.launch.py')
        )
    )

    # 6. Scan Merger (引数不要)
    scan_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edu_robot, 'launch', 'laser_scan_merger_b.launch.py')
        )
    )

    # 7. 【追加】 Teleop (Joystick)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edu_robot, 'launch', 'teleop_b.launch.py')
        )
    )

    # 8. icart の定義
    icart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_edu_robot, 'launch', 'icart_b.launch.py')
        )
    )

    # 9. タイマーアクション (yp_spur -> icart の依存解決用)
    delayed_icart_start = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Master: 5 seconds passed. Launching icart..."),
            icart_launch
        ]
    )

    return LaunchDescription([
        tf_launch,
        yp_spur_launch,
        urg_node_launch,
        urg3d_launch,
        yvt_to_scan_launch,
        scan_merger_launch,
        teleop_launch,
        delayed_icart_start
    ])
