import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_edu = get_package_share_directory('edu_robot')
    slam_params_file = os.path.join(pkg_edu, 'config', 'params_default.yaml')

    start_sync_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          #no-life-cycle#{'use_lifecycle_node': True} 
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(start_sync_slam_toolbox_node)

    return ld
