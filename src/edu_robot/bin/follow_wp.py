import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import os

# --- SequentialNavigator Class (Same as before, handles the ROS 2 logic) ---
class SequentialNavigator(Node):
    """
    ROS 2 Action Client to send NavigateToPose goals sequentially.
    """
    def __init__(self, waypoints):
        super().__init__('file_sequential_navigator')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.waypoints = waypoints
        self.current_goal_index = 0
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

    def send_next_goal(self):
        if self.current_goal_index >= len(self.waypoints):
            self.get_logger().info('✅ All waypoints processed! Navigation task complete.')
            raise SystemExit 
        
        self.get_logger().info(f'Waiting for /navigate_to_pose action server...')
        self._action_client.wait_for_server()

        # --- Construct the Goal Message ---
        position_data, orientation_data = self.waypoints[self.current_goal_index]
        
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        
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

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server!')
            self.current_goal_index += 1
            self.send_next_goal() 
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Goal {self.current_goal_index + 1} succeeded!')
        else:
            self.get_logger().warn(f'Goal {self.current_goal_index + 1} failed with status: {status}')

        self.current_goal_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        pass # Feedback logging is suppressed for brevity in this example
# --------------------------------------------------------------------------


def load_waypoints_from_file(file_path):
    """
    Reads waypoints from a text file and converts them to the required list format.
    File format expected: x, y, z, qx, qy, qz, qw (one waypoint per line)
    """
    waypoints = []
    if not os.path.exists(file_path):
        print(f"ERROR: Waypoint file not found at: {file_path}")
        return waypoints

    print(f"Loading waypoints from: {file_path}")
    with open(file_path, 'r') as f:
        for line in f:
            # Skip empty lines or comments
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            try:
                # Split the line by comma and convert all parts to float
                parts = [float(p.strip()) for p in line.split(',')]
                
                if len(parts) == 7:
                    # Format: [[x, y, z], [qx, qy, qz, qw]]
                    position = parts[0:3]
                    orientation = parts[3:7]
                    waypoints.append([position, orientation])
                else:
                    print(f"WARNING: Skipping line with wrong number of values (expected 7): {line}")

            except ValueError:
                print(f"WARNING: Skipping line with non-numeric values: {line}")

    print(f"Successfully loaded {len(waypoints)} valid waypoints.")
    return waypoints


def main(args=None):
    # --- Define the name of your waypoint file ---
    WAYPOINT_FILE = 'wpdata/wp.txt'
    # ---------------------------------------------
    
    waypoints_list = load_waypoints_from_file(WAYPOINT_FILE)

    if not waypoints_list:
        print("No waypoints loaded. Exiting.")
        return

    rclpy.init(args=args)
    
    try:
        navigator = SequentialNavigator(waypoints_list)
        navigator.send_next_goal() 
        rclpy.spin(navigator)

    except SystemExit:
        pass
    except Exception as e:
        if 'navigator' in locals():
            navigator.get_logger().error(f'An error occurred: {e}')
        else:
             print(f'An error occurred before node initialization: {e}')
        
    finally:
        if 'navigator' in locals():
            navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
