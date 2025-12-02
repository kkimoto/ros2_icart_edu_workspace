import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

import os
import json
import math


class SequentialNavigator(Node):
    """
    Manages waypoints and action client state.
    The node itself does NOT block; main() runs the loop.
    """
    IDLE = 0
    SENDING_GOAL = 1
    WAITING_GOAL_RESPONSE = 2
    WAITING_RESULT = 3
    DONE = 4

    def __init__(self, waypoints):
        super().__init__('file_sequential_navigator')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.waypoints = waypoints
        self.current_index = 0

        # State machine
        self.state = self.IDLE
        self.send_goal_future = None
        self.result_future = None

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

    def start_next_goal_if_any(self):
        """
        If there are more waypoints, start sending the next goal.
        Called from the main loop when state is IDLE.
        """
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('✅ All waypoints processed!')
            self.state = self.DONE
            return

        wp = self.waypoints[self.current_index]
        position_data = wp[0]
        orientation_data = wp[1]

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()

        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'

        goal_pose.pose.position.x = position_data[0]
        goal_pose.pose.position.y = position_data[1]
        goal_pose.pose.position.z = position_data[2]

        goal_pose.pose.orientation.x = orientation_data[0]
        goal_pose.pose.orientation.y = orientation_data[1]
        goal_pose.pose.orientation.z = orientation_data[2]
        goal_pose.pose.orientation.w = orientation_data[3]

        goal_msg.pose = goal_pose

        self.get_logger().info(
            f'🚀 Sending goal {self.current_index + 1}/{len(self.waypoints)} '
            f'to ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})...'
        )

        # Ensure server is ready (blocking here is usually okay once at start)
        self._action_client.wait_for_server()

        # Send the goal asynchronously; we will poll its future in the main loop
        self.send_goal_future = self._action_client.send_goal_async(goal_msg)
        self.state = self.WAITING_GOAL_RESPONSE

    def process(self):
        """
        Called repeatedly from main loop after spin_once().
        Implements a simple state machine.
        """
        if self.state == self.IDLE:
            # Start next waypoint if available
            self.start_next_goal_if_any()

        elif self.state == self.WAITING_GOAL_RESPONSE:
            if self.send_goal_future is None:
                # Shouldn't happen, but be safe
                self.state = self.IDLE
                return

            if self.send_goal_future.done():
                goal_handle = self.send_goal_future.result()
                if not goal_handle.accepted:
                    self.get_logger().error(
                        f'Goal {self.current_index + 1} rejected by action server.'
                    )
                    # Move on to next waypoint
                    self.current_index += 1
                    self.state = self.IDLE
                    self.send_goal_future = None
                    return

                self.get_logger().info('Goal accepted. Waiting for result...')
                self.result_future = goal_handle.get_result_async()
                self.send_goal_future = None
                self.state = self.WAITING_RESULT

        elif self.state == self.WAITING_RESULT:
            if self.result_future is None:
                # Shouldn't happen, but be safe
                self.state = self.IDLE
                return

            if self.result_future.done():
                result_response = self.result_future.result()
                status = result_response.status  # int
                # result = result_response.result  # not used here, but available

                if status == 4:  # SUCCEEDED
                    self.get_logger().info(
                        f'Goal {self.current_index + 1} succeeded.'
                    )
                else:
                    self.get_logger().warn(
                        f'Goal {self.current_index + 1} failed with status: {status}.'
                    )

                # Move to next waypoint
                self.current_index += 1
                self.result_future = None
                self.state = self.IDLE

        elif self.state == self.DONE:
            # Nothing more to do; main() can decide to break the loop.
            pass


def load_waypoints_from_file(file_path):
    waypoints = []
    if not os.path.exists(file_path):
        print(f"ERROR: Waypoint file not found at: {file_path}")
        return waypoints

    print(f"Loading waypoints from: {file_path} x,y,a style...")

    waypoints = []

    fin = open( file_path, 'r' )
    for line in fin.readlines():
      if line[0]=='#':
        continue
      data = line.split()
      if len(data)<3:
        continue
      waypoints.append( [ float(data[0]), float(data[1]), float(data[2]) ] )

    return waypoints


def main(args=None):
    #WAYPOINT_FILE = 'wpdata/wp.json'
    #WAYPOINT_FILE = 'wpdata/wp_xya.json'  # x, y, angle_degree
    WAYPOINT_FILE = 'wpdata/wp_xya.txt'  # x, y, angle_degree

    xya_list = load_waypoints_from_file(WAYPOINT_FILE)
    waypoints_list = []

    for xya in xya_list:
        x = xya[0]
        y = xya[1]
        ang = xya[2]
        th = math.radians(ang)
        waypoints_list.append(
            [[x, y, 0.0],
             [0.0, 0.0, math.sin(th / 2.0), math.cos(th / 2.0)]]
        )
        print(f"waipoint: {x:.3f} {y:.3f} {ang:.3f}")

    if not waypoints_list:
        print("No waypoints loaded. Exiting.")
        return

    rclpy.init(args=args)
    navigator = SequentialNavigator(waypoints_list)

    try:
        # Explicit top-level loop
        loop_rate_sec = 0.1  # main loop period
        while rclpy.ok():
            # Process a bit of ROS I/O
            rclpy.spin_once(navigator, timeout_sec=loop_rate_sec)

            # Your own logic / state machine
            navigator.process()

            # You can add any other checks or interactions here
            if navigator.state == SequentialNavigator.DONE:
                break

    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
