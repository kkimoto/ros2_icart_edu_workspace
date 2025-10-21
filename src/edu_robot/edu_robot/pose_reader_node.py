#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

class PoseReader(Node):
    def __init__(self):
        super().__init__('pose_reader')
        
        # Create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to periodically read pose (every 1 second)
        self.timer = self.create_timer(1.0, self.read_pose)
        
        self.get_logger().info('Pose Reader Node Started')
    
    def quaternion_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def read_pose(self):
        try:
            # Look up transform from map to base_link (or base_footprint)
            transform = self.tf_buffer.lookup_transform(
                'map',              # target frame
                'base_footprint',   # source frame (change to 'base_link' if needed)
                rclpy.time.Time()   # get latest available
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Extract orientation (quaternion)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Convert to yaw angle (in radians)
            yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            yaw_degrees = math.degrees(yaw)
            
            self.get_logger().info(
                f'Robot Pose in Map: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m, '
                f'yaw={yaw:.3f}rad ({yaw_degrees:.1f} deg) quaternion: {qx} {qy} {qz} {qw}'
            )
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
