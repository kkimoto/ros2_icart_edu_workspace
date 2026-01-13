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
    # 1. ノード定義
    icart_node = LifecycleNode(
        package='edu_robot',
        executable='icart_lifecycle_wrapper',
        name='icart_lifecycle_wrapper',
        namespace='',
        output='screen',
        emulate_tty=True,
    )

    # 2. 起動直後に Configure コマンド発行 (0.5秒遅延)
    configure_cmd = TimerAction(
        period=0.5,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(icart_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ]
    )

    # 3. Configure完了(inactive)したら -> Activate
    activate_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=icart_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(icart_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        icart_node,
        configure_cmd,
        activate_on_configure,
    ])
