import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math
import time
import sys

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        # /initialpose トピックへのパブリッシャーを作成
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def publish_initial_pose(self, x, y, theta):
        msg = PoseWithCovarianceStamped()

        # ヘッダーの設定
        msg.header.frame_id = 'map'  # 通常は map フレーム
        msg.header.stamp = self.get_clock().now().to_msg()

        # 座標の設定
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # 向き（Yaw角）をクォータニオンに変換
        q = self.euler_to_quaternion(0, 0, theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # 共分散行列の設定 (6x6 = 36要素)
        # 対角成分に少し値を入れて「ある程度の不確かさがある」ことをAMCLに伝えます
        # 分散が0だとAMCLがパーティクルを拡散できなくなることがあります
        covariance = [0.0] * 36
        covariance[0] = 0.25  # X軸の分散
        covariance[7] = 0.25  # Y軸の分散
        covariance[35] = 0.06 # Yaw軸の分散(約15度^2)
        
        msg.pose.covariance = covariance

        self.get_logger().info(f'Publishing Initial Pose: x={x}, y={y}, theta={theta}')
        self.publisher_.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        オイラー角からクォータニオン(x, y, z, w)への変換
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):

    if len(sys.argv)<=3:
      print( f'{sys.argv[0]} x y angle' )
      exit()

    x = float( sys.argv[1] )
    y = float( sys.argv[2] )
    ang = float( sys.argv[3] )
    theta = ang * (math.pi/180.0)

    rclpy.init(args=args)
    node = InitialPosePublisher()

    # パブリッシャーとの接続が確立するのを少し待つ（重要）
    # すぐに送るとsubscriber(amcl)が受け取れないことがあるため
    time.sleep(1.0)

    # ここに初期位置を指定 (例: x=2.0, y=1.0, theta=1.57rad)
    node.publish_initial_pose(x, y, theta)
    #node.publish_initial_pose(x=0.0, y=0.0, theta=0.0)

    # 送信したら終了
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
