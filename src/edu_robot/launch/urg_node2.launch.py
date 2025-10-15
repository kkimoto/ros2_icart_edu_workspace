import os
import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# 2台LiDAR接続時のurg_node2複数起動（edu_robotパッケージ用）

def generate_launch_description():
    # edu_robot パッケージの共有ディレクトリを取得
    edu_robot_pkg_dir = get_package_share_directory('edu_robot')

    # パラメータファイルのパス設定
    # 統合パッケージのconfigディレクトリを参照
    config_file_path = os.path.join(
        edu_robot_pkg_dir,
        'config',
        'params_default.yaml'
    )

    # パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        # YAMLファイルをロードし、urg_node2のノードパラメータを抽出
        full_config = yaml.safe_load(file)
        
        # 'urg_node2' ノード名でパラメータが分離されていることを期待
        # 例: full_config['lidar1']['ros__parameters']
        
        # 1台目のパラメータを抽出 (YAML内で 'lidar1' キーを使用していると仮定)
        config_params_1st = full_config['lidar1']['ros__parameters']
        
        # 2台目のパラメータを抽出 (YAML内で 'lidar2' キーを使用していると仮定)
        config_params_2nd = full_config['lidar2']['ros__parameters']


    # Launch Argumentsの定義
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='false',
                                         description='Automatically configure and activate the nodes.')
    # auto_start を 'false' に設定することで、ノードが起動時に 'unconfigured' 状態から始まるようにします。

    node_name_1st_arg = DeclareLaunchArgument('node_name_1st', default_value='urg_node2_1st')
    node_name_2nd_arg = DeclareLaunchArgument('node_name_2nd', default_value='urg_node2_2nd')
    scan_topic_name_1st_arg = DeclareLaunchArgument('scan_topic_name_1st', default_value='scan_1st')
    scan_topic_name_2nd_arg = DeclareLaunchArgument('scan_topic_name_2nd', default_value='scan_2nd')

    
    # urg_node2をライフサイクルノードとして起動（1台目）
    # lifecycle_nodeを定義するだけで、ノードはデフォルトで Unconfigured 状態で起動します。
    lifecycle_node_1st = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name_1st'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name_1st'))],
        parameters=[config_params_1st],
        namespace='',
        output='screen',
        # ノードが Unconfigured 状態で起動するために auto_start=False は指定しません (デフォルト動作)。
    )

    # urg_node2をライフサイクルノードとして起動（2台目）
    lifecycle_node_2nd = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name_2nd'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name_2nd'))],
        parameters=[config_params_2nd],
        namespace='',
        output='screen',
    )


    # --- auto_start=true の場合のイベントハンドラ ---

    # 1台目: Unconfigured状態からInactive状態への遷移 (configure)
    urg_node2_node_1st_configure_event_handler = RegisterEventHandler(
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
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # 1台目: Inactive状態からActive状態への遷移 (activate)
    # configureが完了 (inactive状態に遷移) したら activate を実行
    urg_node2_node_1st_activate_event_handler = RegisterEventHandler(
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
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # 2台目: Unconfigured状態からInactive状態への遷移 (configure)
    # 1台目のアクティベートが完了してから実行したい場合はイベントハンドラを変更しますが、ここでは同時に起動します。
    urg_node2_node_2nd_configure_event_handler = RegisterEventHandler(
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
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # 2台目: Inactive状態からActive状態への遷移 (activate)
    urg_node2_node_2nd_activate_event_handler = RegisterEventHandler(
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
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    return LaunchDescription([
        # Launch Arguments
        auto_start_arg,
        node_name_1st_arg,
        node_name_2nd_arg,
        scan_topic_name_1st_arg,
        scan_topic_name_2nd_arg,

        # Lifecycle Nodes (Start in 'unconfigured' state)
        lifecycle_node_1st,
        lifecycle_node_2nd,

        # Event Handlers (Only execute if auto_start=true)
        urg_node2_node_1st_configure_event_handler,
        urg_node2_node_1st_activate_event_handler,
        urg_node2_node_2nd_configure_event_handler,
        urg_node2_node_2nd_activate_event_handler,
    ])
