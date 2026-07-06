#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    edu_robot_dir = get_package_share_directory('edu_robot')
    keepout_params_file = os.path.join(edu_robot_dir, 'config', 'params_default.yaml')

    keepout_mask_yaml = '/home/kimoto/ros2_workspace/edu_ws/map/keepout_mask.yaml'

    filter_mask_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        namespace='',
        output='screen',
        parameters=[keepout_params_file]
#        parameters=[{
#            'use_sim_time': False,
#            'frame_id': 'map',
#            'topic_name': 'keepout_filter_mask',
#            'yaml_filename': keepout_mask_yaml,
#        }]
    )

    costmap_filter_info_server = LifecycleNode(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_costmap_filter_info_server',
        namespace='',
        output='screen',
        parameters=[keepout_params_file]
#        parameters=[{
#            'use_sim_time': False,
#            'filter_info_topic': 'costmap_filter_info',
#            'type': 0,
#            'mask_topic': 'keepout_filter_mask',
#            'base': 0.0,
#            'multiplier': 1.0,
#        }]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='keepout_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
            ],
        }]
    )

    return LaunchDescription([
        filter_mask_server,
        costmap_filter_info_server,
        lifecycle_manager,
    ])
