import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# 2台LiDAR接続時のurg_node2複数起動（自動Activate版）

def generate_launch_description():
    # edu_robot パッケージの共有ディレクトリを取得
    edu_robot_pkg_dir = get_package_share_directory('edu_robot')

    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        edu_robot_pkg_dir,
        'config',
        'params_default.yaml'
    )

    # パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        full_config = yaml.safe_load(file)
        config_params_1st = full_config['lidar1']['ros__parameters']
        config_params_2nd = full_config['lidar2']['ros__parameters']

    # 引数定義 (auto_start は削除しました。常に自動起動します)
    node_name_1st_arg = DeclareLaunchArgument('node_name_1st', default_value='urg_node2_1st')
    node_name_2nd_arg = DeclareLaunchArgument('node_name_2nd', default_value='urg_node2_2nd')
    scan_topic_name_1st_arg = DeclareLaunchArgument('scan_topic_name_1st', default_value='scan_1st')
    scan_topic_name_2nd_arg = DeclareLaunchArgument('scan_topic_name_2nd', default_value='scan_2nd')

    
    # ---------------------------------------------------------
    # Lifecycle Nodes
    # ---------------------------------------------------------
    lifecycle_node_1st = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name_1st'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name_1st'))],
        parameters=[config_params_1st],
        namespace='',
        output='screen',
    )

    lifecycle_node_2nd = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name_2nd'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name_2nd'))],
        parameters=[config_params_2nd],
        namespace='',
        output='screen',
    )

    # ---------------------------------------------------------
    # Event Handlers (強制的に実行するように修正)
    # ---------------------------------------------------------

    # [1台目] 起動 -> Configure
    urg_node2_node_1st_configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node_1st,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_1st),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )

    # [1台目] Configure完了(inactive) -> Activate
    urg_node2_node_1st_activate = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node_1st,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_1st),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        )
    )

    # [2台目] 起動 -> Configure
    urg_node2_node_2nd_configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node_2nd,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_2nd),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )

    # [2台目] Configure完了(inactive) -> Activate
    urg_node2_node_2nd_activate = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node_2nd,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_2nd),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        )
    )

    return LaunchDescription([
        node_name_1st_arg,
        node_name_2nd_arg,
        scan_topic_name_1st_arg,
        scan_topic_name_2nd_arg,

        # Nodes
        lifecycle_node_1st,
        lifecycle_node_2nd,

        # Event Handlers (無条件実行)
        urg_node2_node_1st_configure,
        urg_node2_node_1st_activate,
        urg_node2_node_2nd_configure,
        urg_node2_node_2nd_activate,
    ])
