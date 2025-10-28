import math
from typing import List, Tuple

# Define a type alias for clarity, representing the pose as:
# ([x, y, z], [qx, qy, qz, qw])
PoseType = Tuple[List[float], List[float]]

def convert_pose2xya(pose: PoseType) -> Tuple[float, float, float]:
    """
    Converts a ROS2-style 2D pose (position [x, y, z] and orientation [qx, qy, qz, qw])
    into a simple (x, y, angle_degrees) tuple.

    This function relies on the 2D restriction: z=0, qx=0, qy=0, meaning the rotation
    is purely around the Z-axis (Yaw).

    Args:
        pose: A tuple containing two lists: (position_list, quaternion_list).
              Example: ([-0.082, 0.0, 0.0], [0.0, 0.0, qz, qw])

    Returns:
        A tuple (x, y, angle_degrees).
    """
    # 1. Extract position (x, y)
    position = pose[0]
    x = position[0]
    y = position[1]

    # 2. Extract quaternion components (qz, qw)
    quaternion = pose[1]
    qz = quaternion[2]
    qw = quaternion[3]

    # 3. Calculate Yaw (angle in radians)
    # For a pure Z-axis rotation (qx=0, qy=0), the yaw angle (theta) is calculated
    # using the formula: theta = 2 * atan2(qz, qw)
    angle_radians = 2 * math.atan2(qz, qw)

    # 4. Convert radians to degrees
    angle_degrees = math.degrees(angle_radians)

    # Normalize the angle to be within the standard yaw range (-180 to 180 degrees)
    # This step is mostly for cosmetic display and consistency.
    angle_degrees = (angle_degrees + 180) % 360 - 180

    return (x, y, angle_degrees)

def convert_xya2pose(x: float, y: float, angle_degrees: float) -> PoseType:
    """
    Converts a 2D pose (x, y, angle_degrees) into a ROS2-style pose format,
    assuming a 2D restriction (z=0, and rotation only around the Z-axis).

    Args:
        x: X-coordinate.
        y: Y-coordinate.
        angle_degrees: Yaw angle in degrees.

    Returns:
        A tuple containing two lists: (position_list, quaternion_list).
        Example: ([x, y, 0.0], [0.0, 0.0, qz, qw])
    """
    # 1. Convert degrees to radians
    angle_radians = math.radians(angle_degrees)

    # 2. Calculate quaternion components for Z-axis rotation (Yaw)
    # The quaternion components for a rotation theta around the Z-axis are:
    # qx=0, qy=0, qz=sin(theta/2), qw=cos(theta/2)
    half_angle = angle_radians / 2.0
    qw = math.cos(half_angle)
    qz = math.sin(half_angle)
    qx = 0.0  # Required for 2D restriction
    qy = 0.0  # Required for 2D restriction

    # 3. Construct the output pose lists
    # Position: [x, y, z] where z=0.0
    position_list = [x, y, 0.0]

    # Orientation: [qx, qy, qz, qw]
    quaternion_list = [qx, qy, qz, qw]

    return (position_list, quaternion_list)

# --- Demonstration ---
if __name__ == '__main__':
    print("--- ROS2 2D Pose Conversion Demonstration ---")
    
    # 1. Example: Near zero rotation (similar to your input)
    # Note: The original qz value is very small due to floating point precision for 0 degrees.
    original_pose_1 = ([-0.082, 0.0, 0.0], [0.0, 0.0, 1.1894092295279684e-20, 1.0])
    xya_1 = convert_pose2xya(original_pose_1)
    converted_pose_1 = convert_xya2pose(*xya_1)

    print(f"\n[Example 1: Near Zero Angle]")
    print(f"Original Pose: {original_pose_1}")
    print(f"--> X, Y, Angle: ({xya_1[0]:.3f}, {xya_1[1]:.3f}, {xya_1[2]:.5f}°)")
    print(f"--> Reconverted Pose: {converted_pose_1}")

    # 2. Example: 90 degree rotation
    angle_2 = 90.0
    x_2, y_2 = 1.5, -2.5
    pose_2 = convert_xya2pose(x_2, y_2, angle_2)
    xya_2 = convert_pose2xya(pose_2)

    print(f"\n[Example 2: 90 Degrees]")
    print(f"Original XYA: ({x_2}, {y_2}, {angle_2}°)")
    print(f"--> Converted Pose: {pose_2}")
    print(f"--> Converted back to XYA: ({xya_2[0]:.3f}, {xya_2[1]:.3f}, {xya_2[2]:.3f}°)")

    # 3. Example: -135 degree rotation
    angle_3 = -135.0
    x_3, y_3 = 10.0, 5.0
    pose_3 = convert_xya2pose(x_3, y_3, angle_3)
    xya_3 = convert_pose2xya(pose_3)

    print(f"\n[Example 3: -135 Degrees]")
    print(f"Original XYA: ({x_3}, {y_3}, {angle_3}°)")
    print(f"--> Converted Pose: {pose_3}")
    print(f"--> Converted back to XYA: ({xya_3[0]:.3f}, {xya_3[1]:.3f}, {xya_3[2]:.3f}°)")

    # 4. Example: 180 degree rotation
    angle_4 = 180.0
    x_4, y_4 = 1.0, 0.0
    pose_4 = convert_xya2pose(x_4, y_4, angle_4)
    xya_4 = convert_pose2xya(pose_4)

    print(f"\n[Example 4: 180 Degrees]")
    print(f"Original XYA: ({x_4}, {y_4}, {angle_4}°)")
    print(f"--> Converted Pose: {pose_4}")
    print(f"--> Converted back to XYA: ({xya_4[0]:.3f}, {xya_4[1]:.3f}, {xya_4[2]:.3f}°)")
