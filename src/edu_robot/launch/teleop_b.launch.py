#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import launch.events

def generate_launch_description():
    # --- Arguments ---
    joy_config = LaunchConfiguration('joy_config', default='ps3')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')
    publish_stamped_twist = LaunchConfiguration('publish_stamped_twist', default='false')
    joy_vel = LaunchConfiguration('joy_vel', default='cmd_vel')
    
    pkg_dir = get_package_share_directory('edu_robot')
    config_file_path = os.path.join(pkg_dir, 'config', 'params_default.yaml')

    # --- 1. Joy Node (Standard Node) ---
    joy_node = Node(
        package='joy', 
        executable='joy_node', 
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )

    # --- 2. Teleop Wrapper (Lifecycle Node) ---
    teleop_node = LifecycleNode(
        package='edu_robot',
        executable='teleop_lifecycle_wrapper',
        name='teleop_lifecycle_wrapper',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'config_file_path': config_file_path},
            {'publish_stamped_twist': publish_stamped_twist}
        ],
        remappings=[
            ('/cmd_vel', joy_vel)
        ]
    )

    # --- 3. Automatic Lifecycle Transition ---
    
    # [起動 -> Configure] (0.5秒後にコマンド発行)
    configure_cmd = TimerAction(
        period=0.5,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(teleop_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ]
    )

    # [Configure完了 -> Activate]
    activate_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=teleop_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(teleop_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        # Args
        DeclareLaunchArgument('joy_config', default_value='ps3'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        DeclareLaunchArgument('publish_stamped_twist', default_value='false'),

        # Nodes
        joy_node,
        teleop_node,

        # Lifecycle Management
        configure_cmd,
        activate_on_configure
    ])
