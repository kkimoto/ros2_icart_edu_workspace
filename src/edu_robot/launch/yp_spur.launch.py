#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    yp_spur_node = Node(
        package='edu_robot',
        executable='yp_spur_lifecycle_wrapper',
        name='yp_spur_lifecycle_wrapper',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([yp_spur_node])
