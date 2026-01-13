import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory

# 修正済みインポート
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition # ← ここを修正
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import launch.events

def generate_launch_description():

    pkg_share = get_package_share_directory('edu_robot')
    params_file = os.path.join(pkg_share, 'config', 'params_default.yaml')

    # Load YAML
    with open(params_file, 'r') as f:
        config_yaml = yaml.safe_load(f)

    ### urg3d_node2 (YVT)
    yvt_3d_params = config_yaml.get('yvt_3d', {}).get('ros__parameters', {})

    # Lifecycle node
    yvt_node = LifecycleNode(
        package='urg3d_node2',
        executable='urg3d_node2_node',
        name='yvt_3d',
        namespace='',
        output='screen',
        parameters=[yvt_3d_params],
        emulate_tty=True
    )

    # 1. 起動 -> Configure
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=yvt_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(yvt_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        )
    )

    # 2. Configure完了(inactive) -> Activate
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=yvt_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(yvt_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    ld = LaunchDescription()
    ld.add_action(yvt_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
