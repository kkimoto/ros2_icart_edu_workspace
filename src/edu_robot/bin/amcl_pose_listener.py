#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclPoseReader(Node):
    def __init__(self):
        super().__init__('amcl_pose_reader')
        # Subscribe to /amcl_pose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract orientation (quaternion)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Print or use the values as needed
        self.get_logger().info(
            f'AMCL Pose -> x: {x:.3f}, y: {y:.3f}, orientation (qx,qy,qz,qw): [{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
