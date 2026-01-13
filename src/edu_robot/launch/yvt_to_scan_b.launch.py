from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            # 入力トピック(cloud_in) と 出力トピック(scan) の設定
            # urg3d_node2 が出すトピック名に合わせてください（元ファイルの設定を維持しています）
            remappings=[
                ('cloud_in', '/hokuyo_cloud2'),
                ('scan', '/yvt_2dscan')
            ],
            parameters=[{
                # ターゲットフレーム（必要に応じてロボットのTF構成に合わせて変更してください）
                # コメントアウトされていたので、デフォルトかTFツリー依存になります
                # 'target_frame': 'base_link', 
                
                'transform_tolerance': 1.0,
                'min_height': -2.00,
                'max_height': 0.13,
                'angle_min': -1.4,
                'angle_max': 1.4,
                'angle_increment': 0.0087,
                'scan_time': 0.3333,
                'range_min': 0.05,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            output='screen'
        )
    ])
