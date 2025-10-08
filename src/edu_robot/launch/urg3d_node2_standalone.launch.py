# Copyright 2022 HOKUYO AUTOMATIC CO., LTD.
# (Relocated & adapted for edu_robot by you)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Load parameter file from edu_robot instead of urg3d_node2
    config_file_path = os.path.join(
        get_package_share_directory('edu_robot'),
        'config',
        'params_default.yaml'   # keep filename; adjust if you rename
    )

    with open(config_file_path, 'r') as f:
        # Preserve original schema: pick the node's ros__parameters subtree
        # params.yaml:
        #   urg3d_node2:
        #     ros__parameters:
        #       ...
        config_params = yaml.safe_load(f)['urg3d_node2']['ros__parameters']

    lifecycle_node = LifecycleNode(
        package='urg3d_node2',
        executable='urg3d_node2_node',  # confirm with: ros2 pkg executables urg3d_node2
        name=LaunchConfiguration('node_name'),
        remappings=[('hokuyo_cloud2', LaunchConfiguration('scan_topic_name'))],
        parameters=[config_params],
        namespace='',
        output='screen',
    )

    # Configure on start if auto_start
    urg3d_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Activate after configuring if auto_start
    urg3d_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Parameters:
    #   auto_start      : auto-transition to Active at launch (default true)
    #   node_name       : node name (default "urg3d_node2")
    #   scan_topic_name : topic name (default "hokuyo_cloud2")
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('node_name', default_value='urg3d_node2'),
        DeclareLaunchArgument('scan_topic_name', default_value='hokuyo_cloud2'),
        lifecycle_node,
        urg3d_node2_node_configure_event_handler,
        urg3d_node2_node_activate_event_handler,
    ])
