import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import necessary messages
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

# File handling imports
import os
import math

class SequentialNavigator(Node):
    """
    ROS 2 Action Client to send NavigateToPose goals sequentially.
    """
    def __init__(self, waypoints):
        super().__init__('file_sequential_navigator')
        
        # Action Client for the /navigate_to_pose action (handled by bt_navigator)
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )
        
        self.waypoints = waypoints
        self.current_goal_index = 0
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')
        self.prev_t0 = time.time()

    def send_next_goal(self):
        """
        Sends the next waypoint in the list as a NavigateToPose goal.
        """
        if self.current_goal_index >= len(self.waypoints):
            self.get_logger().info('✅ All waypoints processed! Navigation task complete.')
            # Stop the ROS 2 spinning
            raise SystemExit 
        
        self.get_logger().info(f'Waiting for /navigate_to_pose action server...')
        self._action_client.wait_for_server()

        # --- 1. Construct the Goal Message ---
        # The waypoint structure is: [[x, y, z], [qx, qy, qz, qw]]
        waypoint_data = self.waypoints[self.current_goal_index]
        position_data = waypoint_data[0]
        orientation_data = waypoint_data[1]
        
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        
        # Set the Header (REQUIRED: needs frame_id)
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        # **IMPORTANT**: Use the frame ID of your map
        goal_pose.header.frame_id = 'map' 
        
        # Set Position
        goal_pose.pose.position.x = position_data[0]
        goal_pose.pose.position.y = position_data[1]
        goal_pose.pose.position.z = position_data[2]

        # Set Orientation (Quaternion)
        goal_pose.pose.orientation.x = orientation_data[0]
        goal_pose.pose.orientation.y = orientation_data[1]
        goal_pose.pose.orientation.z = orientation_data[2]
        goal_pose.pose.orientation.w = orientation_data[3]
        
        goal_msg.pose = goal_pose

        self.get_logger().info(f'🚀 Sending goal {self.current_goal_index + 1}/{len(self.waypoints)} '
                               f'to ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})...')
        print('goal', goal_pose)

        # --- 2. Send the Goal and Get a Future ---
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # --- 3. Add a Callback for when the goal is accepted/rejected ---
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Called when the action server responds whether the goal was accepted.
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server!')
            self.current_goal_index += 1
            # Try to send the next goal immediately
            self.send_next_goal() 
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        
        # --- 4. Add a Callback for the final result ---
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Called when the goal completes (success or failure).
        """
        result = future.result().result
        status = future.result().status
        
        # Check the status (SUCCEEDED = 4)
        if status == 4:
            self.get_logger().info(f'Goal {self.current_goal_index + 1} succeeded! Moving to next waypoint.')
        else:
            self.get_logger().warn(f'Goal {self.current_goal_index + 1} failed with status: {status}. Attempting next goal.')

        # Move to the next waypoint regardless of success/failure
        self.current_goal_index += 1
        
        # Send the next goal
        print("waiting a while")
        time.sleep(3)
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        t0 = time.time()
        if (t0-self.prev_t0)<0.5:
          prev_t0 = t0
          return
        self.prev_t0 = t0

        """
        Called periodically by the action server with navigation updates.
        """
        feedback = feedback_msg.feedback
        # Optional: Log or display the distance remaining
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')


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
    # --- Define the name of your waypoint file ---
    # Ensure this file is in the same directory as the script, or provide a full path.
    #WAYPOINT_FILE = 'wpdata/wp.json'
    WAYPOINT_FILE = 'wpdata/wp_xya.txt' # x,y,angle_degree
    # ---------------------------------------------
    
    xya_list = load_waypoints_from_file(WAYPOINT_FILE)
    waypoints_list = list()
    for xya in xya_list:
        x = xya[0]
        y = xya[1]
        ang = xya[2]
        th  = math.radians(ang)
        waypoints_list.append( [ [x,y,0.0],[0.0,0.0,math.sin(th/2.0), math.cos(th/2.0)] ] )
        print(f"waipoint: {x:.3} {y:.3} {ang:.3}")

    #print(waypoints_list)
    #for wp in waypoints_list:
    #    print(wp)
    #exit()

    if not waypoints_list:
        print("No waypoints loaded. Exiting.")
        return

    rclpy.init(args=args)
    
    try:
        navigator = SequentialNavigator(waypoints_list)
        # Start the navigation by sending the first goal
        navigator.send_next_goal() 
        
        # Keep the node alive and processing callbacks
        rclpy.spin(navigator)

    except SystemExit:
        # This exception is raised when all waypoints are done
        pass
    except Exception as e:
        # Catch any other ROS 2 exceptions
        if 'navigator' in locals():
            navigator.get_logger().error(f'An error occurred: {e}')
        
    finally:
        # Shutdown the node and ROS client library
        if 'navigator' in locals():
            navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
