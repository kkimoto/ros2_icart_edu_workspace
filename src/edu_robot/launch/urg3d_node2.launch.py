import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('edu_robot')
    params_file = os.path.join(pkg_share, 'config', 'params_default.yaml')

    # Load YAML
    with open(params_file, 'r') as f:
        config_yaml = yaml.safe_load(f)

    ### urg3d_node2 (YVT)

    # Extract parameters under the actual key
    yvt_3d_params = config_yaml.get('yvt_3d', {}).get('ros__parameters', {})

    # Lifecycle node
    yvt_node = LifecycleNode(
        package='urg3d_node2',
        executable='urg3d_node2_node',
        name='yvt_3d',  # ROS2 node name
        namespace='',
        output='screen',
        parameters=[yvt_3d_params],
        emulate_tty=True
    )

    ld = LaunchDescription()
    ld.add_action(yvt_node)

    return ld
