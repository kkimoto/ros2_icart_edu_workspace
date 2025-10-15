#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Define Launch Arguments ---
    joy_config = LaunchConfiguration('joy_config', default='ps3')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0') # Using device path is safer than ID
    publish_stamped_twist = LaunchConfiguration('publish_stamped_twist', default='false')
    joy_vel = LaunchConfiguration('joy_vel', default='cmd_vel')
    
    # Resolve the path to the main configuration file for the teleop node
    pkg_dir = get_package_share_directory('edu_robot')
    config_file_path = os.path.join(pkg_dir, 'config', 'params_default.yaml')

    return LaunchDescription([
        # Declare arguments for user override
        DeclareLaunchArgument('joy_config', default_value='ps3', description='Joystick configuration profile name.'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0', description='Joystick device path.'),
        DeclareLaunchArgument('joy_vel', default_value='cmd_vel', description='Topic name for velocity commands.'),
        DeclareLaunchArgument('publish_stamped_twist', default_value='false', description='Whether to publish geometry_msgs/TwistStamped.'),

        # --- 2. Launch the Joy Node (Publishes /joy) ---
        # This node is essential for reading the joystick input and generating the /joy topic.
        Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # --- 3. Launch the Teleop Lifecycle Wrapper ---
        # This wrapper launches the teleop_node (which subscribes to /joy) when activated.
        Node(
            package='edu_robot',
            executable='teleop_lifecycle_wrapper',
            name='teleop_lifecycle_wrapper',
            output='screen',
            emulate_tty=True,
            # Pass the configuration file path to the wrapper node's parameter
            parameters=[
                {'config_file_path': config_file_path},
                # Pass necessary runtime parameters to the teleop_node subprocess
                {'publish_stamped_twist': publish_stamped_twist}
            ],
            # Remap /cmd_vel if the user changes the launch argument
            remappings=[
                ('/cmd_vel', joy_vel)
            ]
        ),
    ])
