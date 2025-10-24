# static_transforms_checked.launch.py
"""
Static transform publishers for edu_robot.

Discussion / rationale (short):
- base_footprint: 2D footprint frame lying on the ground (Z = 0).
- base_link: robot body frame attached to the robot, usually above the ground.
  A non-zero Z offset between base_footprint and base_link (e.g. 0.10 m)
  is physically correct and important for 3D consistency (point clouds, IMU, 3D LiDAR).
- For 2D localization/navigation (AMCL, costmaps, planners) a small z offset
  does not break behavior because they operate in the XY plane and ignore Z.
  Keeping the physical offset is future-proof and prevents ambiguous 3D transforms.
- Use respawn=True and output="screen" while debugging so static publishers
  are restarted automatically and their logs are visible if they die.

Edit the x,y,z,roll,pitch,yaw values below to the real sensor offsets on your robot.
If you have a merged scan header frame (base_scan), ensure it is transformable
to the base frame (base_link or base_footprint) via the TF tree.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    # base_footprint -> base_link : keep z non-zero for physical realism (e.g. 0.10 m)
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_footprint_to_base_link",
            output="screen",
            respawn=True,
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.10",    # 10 cm above ground
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_footprint",
                "--child-frame-id", "base_link"
            ],
        )
    )

    # base_link -> laser_frame_1 (adjust offsets to actual mount)
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser_frame_1",
            output="screen",
            respawn=True,
            arguments=[
                "--x", "0.082", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "laser_frame_1"
            ],
        )
    )

    # base_link -> laser_frame_2 (example: sensor mounted mirrored on other side)
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser_frame_2",
            output="screen",
            respawn=True,
            arguments=[
                "--x", "-0.082", "--y", "0.0", "--z", "0.10",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "3.141592653589793",
                "--frame-id", "base_link",
                "--child-frame-id", "laser_frame_2"
            ],
        )
    )

    # base_link -> base_scan (the merged scan header uses this frame)
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_base_scan",
            output="screen",
            respawn=True,
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.10",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "base_scan"
            ],
        )
    )

    return LaunchDescription(nodes)
