# launch/nav2_map_amcl_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('edu_robot')  # change
    map_param_file = os.path.join(pkg_share, 'config', 'params_default.yaml')
    amcl_param_file = os.path.join(pkg_share, 'config', 'params_default.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[map_param_file]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_param_file]
    )

    return LaunchDescription([map_server, amcl])

