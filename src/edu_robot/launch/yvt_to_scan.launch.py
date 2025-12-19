# launch/test_pcl2scan.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scanner_ns = LaunchConfiguration('scanner')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner',
            default_value='scanner',
            description='Namespace for sample topics'
        ),

        # A dummy pointcloud publisher included in the package for testing
        Node(
            package='pointcloud_to_laserscan',
            executable='dummy_pointcloud_publisher',
            name='cloud_publisher',
            remappings=[
                ('cloud', [scanner_ns, '/cloud'])
            ],
            parameters=[{
                'cloud_frame_id': 'hokuyo3d',
                'cloud_extent': 2.0,
                'cloud_size': 500
            }]
        ),

        # static transform: map -> cloud
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '0', '0', '0',   # x y z
                '0', '0', '0', '1',  # qx qy qz qw
                'map', 'cloud'   # parent child
            ]
        ),

        # the converter node
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', ['/hokuyo_cloud2']),
                ('scan', ['/yvt_2dscan'])
            ],
            parameters=[{
                # Important: tune these for your sensor!
                #'target_frame': 'hokuyo3d',
                'transform_tolerance': 1.0,  # 0.01,
                'min_height':  -2.00,        # against raw data
                'max_height':  0.13,         # against raw data
                'angle_min': -1.4, # 80deg # -1.5708,        # -pi/2
                'angle_max':  1.4, # 80deg #  1.5708,        #  pi/2
                'angle_increment': 0.0087,   # ~0.5 deg
                'scan_time': 0.3333,
                'range_min': 0.05,           # <- set to small (0.05) not 0.45
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        )
    ])
