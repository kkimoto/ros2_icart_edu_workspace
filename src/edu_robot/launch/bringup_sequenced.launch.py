from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('edu_robot')

    # Default values
    map_file_default = os.path.join(pkg_share, 'map', 'map.yaml')
    params_file_default = os.path.join(pkg_share, 'config', 'params_default.yaml')

    # Arguments
    map_arg = DeclareLaunchArgument('map', default_value=map_file_default, description='Full path to map yaml file')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=params_file_default, description='Full path to the Nav2 parameters file')

    # Launch Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true' # A common and good default for autostart
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        params_file_arg,
        nav2_bringup_launch
    ])
