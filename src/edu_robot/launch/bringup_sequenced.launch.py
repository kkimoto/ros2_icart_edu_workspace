# bringup_sequenced_checked.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os
import sys

def _check_files(context, *args, **kwargs):
    """OpaqueFunction called at launch-time to verify map & params exist."""
    map_path = LaunchConfiguration('map').perform(context)
    params_path = LaunchConfiguration('params_file').perform(context)

    missing = []
    # expand user and relative paths
    map_path = os.path.expanduser(map_path)
    params_path = os.path.expanduser(params_path)

    if not os.path.isabs(map_path):
        map_path = os.path.abspath(map_path)
    if not os.path.isabs(params_path):
        params_path = os.path.abspath(params_path)

    if not os.path.exists(map_path):
        missing.append(f"map file not found: {map_path}")
    if not os.path.exists(params_path):
        missing.append(f"params file not found: {params_path}")

    if missing:
        for m in missing:
            print("ERROR:", m, file=sys.stderr)
        # Raise to abort the launch and show a clear traceback in the log
        raise FileNotFoundError("\n".join(missing))

    # Optional: print resolved paths for user visibility
    print(f"Using map: {map_path}")
    print(f"Using params file: {params_path}")
    return []

def generate_launch_description():
    # Try to use installed package share for sensible defaults; fall back otherwise.
    try:
        pkg_share = get_package_share_directory('edu_robot')
        default_map = os.path.join(pkg_share, 'map', 'map.yaml')
        default_params = os.path.join(pkg_share, 'config', 'params_default.yaml')
    except PackageNotFoundError:
        # Edit these workspace fallbacks if your workspace location differs
        workspace_root = os.path.expanduser('/home/kimoto/ros2_workspace/edu_ws')
        default_map = os.path.join(workspace_root, 'map', 'map.yaml')
        default_params = os.path.join(workspace_root, 'src', 'edu_robot', 'config', 'params_default.yaml')

    # nav2 bringup package (installed)
    try:
        pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    except PackageNotFoundError:
        print("ERROR: nav2_bringup package not found in ament index. Make sure Nav2 is installed.", file=sys.stderr)
        raise

    # Declare launch arguments
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to Nav2 params yaml file'
    )
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (gazebo) clock'
    )

    # Include the standard nav2 bringup, forwarding map and params_file
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true'
        }.items()
    )

    # OpaqueFunction will run _check_files and abort if missing
    check_files_action = OpaqueFunction(function=_check_files)

    ld = LaunchDescription()

    # add args, check, then bringup
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_sim_time)
    ld.add_action(check_files_action)
    ld.add_action(nav2_bringup)

    return ld
