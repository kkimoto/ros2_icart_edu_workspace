#!/usr/bin/env python3

import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import launch.events

def generate_launch_description():
    yp_spur_node = LifecycleNode(
        package='edu_robot',
        executable='yp_spur_lifecycle_wrapper',
        name='yp_spur_lifecycle_wrapper',
        namespace='',
        output='screen',
        emulate_tty=True,
    )

    # 1. Configure への遷移リクエスト
    # ノードが完全に立ち上がるのを少しだけ待ってから(0.5秒)、コマンドを投げます
    configure_cmd = TimerAction(
        period=0.5,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(yp_spur_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ]
    )

    # 2. Configure完了(inactive)したら -> Activate
    # これはイベントハンドラで受け取れるはずです
    activate_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=yp_spur_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(yp_spur_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        yp_spur_node,
        configure_cmd,         # イベント待ちではなく、コマンド実行に変更
        activate_on_configure,
    ])
