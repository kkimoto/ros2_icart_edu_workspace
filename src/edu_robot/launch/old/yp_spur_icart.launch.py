#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Watchdog node
    watchdog = Node(
        package='edu_robot',
        executable='ypspur_icartmini_watchdog',
        name='yp_spur_lifecycle_watchdog',
        output='screen',
        emulate_tty=True,
    )

    # Lifecycle wrapper node
    icart_wrapper = Node(
        package='edu_robot',
        executable='icart_lifecycle_wrapper',
        name='icart_lifecycle_wrapper',
        output='screen',
        emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(watchdog)
    ld.add_action(icart_wrapper)

    return ld
