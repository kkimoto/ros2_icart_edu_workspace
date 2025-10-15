#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    icart_node = Node(
        package='edu_robot',
        executable='icart_lifecycle_wrapper',
        name='icart_lifecycle_wrapper',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([icart_node])
