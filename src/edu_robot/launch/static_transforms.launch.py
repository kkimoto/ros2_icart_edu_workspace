from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # base_link to laser_frame_1 (ROS2 laser1 frame by kimoto)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser_frame_1",
            arguments=[
                "--x", "0.082", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "laser_frame_1"
            ]
        ),

        # base_link to laser_frame_2 (ROS2 laser2 frame by kimoto)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser_frame_2",
            arguments=[
                "--x", "-0.082", "--y", "0.00", "--z", "0.1",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "3.14159",
                "--frame-id", "base_link", "--child-frame-id", "laser_frame_2"
            ]
        ),

        # base_link to laser1 (ROS1 laser1 frame in old rosbag)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser1",
            arguments=[
                "--x", "0.082", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "laser1"
            ]
        ),

        # base_link to laser2 (ROS1 laser2 frame in old rosbag)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser2",
            arguments=[
                "--x", "-0.082", "--y", "0.00", "--z", "0.1",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "3.14159",
                "--frame-id", "base_link", "--child-frame-id", "laser2"
            ]
        ),

        # base_link to base_scan
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_base_scan",
            arguments=[
                "--x", "-0.0", "--y", "0.0", "--z", "0.1",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "base_scan"
            ]
        ),

        # base_footprint to base_link
        # footprint is usually a frame for merged-scan topic and odometer on the ground.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_footprint_to_base_link",
            arguments=[
                "--x", "-0.082", "--y", "0.00", "--z", "0.1",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_footprint", "--child-frame-id", "base_link"
            ]
        ),

    ])
