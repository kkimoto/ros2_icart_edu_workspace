import time
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

# Import necessary messages
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped # for /amcl_pose
from tf2_ros import Buffer, TransformListener           # for /tf

# File handling imports
import os
import math

# quaternion map
import quaternionmap as qm

class SequentialNavigator(Node):
    """
    ROS 2 Action Client to send NavigateToPose goals sequentially.
    """
    def __init__(self):
        super().__init__('file_sequential_navigator')
        
        self.pose = None

        # Action Client for the /navigate_to_pose action (handled by bt_navigator)
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )
        
        self.waypoints = list()
        self.current_goal_index = 0


        # for reading /amcl_pose
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # for TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_pose_from_tf(self, target_frame='map', source_frame='base_footprint', timeout_sec=5.0):
        # Make sure the listener has time to fill the buffer (and that spin is running)
        try:
            # Wait until transform exists or timeout
            can = self.tf_buffer.can_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=timeout_sec)
            )
            if not can:
                self.get_logger().warning(
                    f'No transform available from "{source_frame}" to "{target_frame}" after {timeout_sec}s'
                )
                return None

            # lookup_transform expects (target_frame, source_frame, time)
            t = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warning(f'TF lookup failed: {e}')
            return None

        pose = PoseStamped()
        pose.header = t.header
        pose.header.frame_id = target_frame
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation

        return pose

    def input_wp(self):

        waypoint = []
        tmp_pose = self.get_pose_from_tf('map', 'base_footprint')
        self.pose = [ [tmp_pose.pose.position.x,tmp_pose.pose.position.y,tmp_pose.pose.position.z],
                      [tmp_pose.pose.orientation.x,tmp_pose.pose.orientation.y,
                       tmp_pose.pose.orientation.z,tmp_pose.pose.orientation.w]]
        print('current-pose', self.pose)

        while True:
            command = input('$ ')
            if len(command)>0:
              com = command.split()
            else:
              print('type "quit" to end.')
              continue

            if com[0]=='help':
                print('commands:')
                print('  help  : show this help.')
                print('  quit  : to quit.')
                print('  fs x y angle  : set waypoint at x,y,angle in FS coordinate.')
                print('  gl x y angle  : set waypoint at x,y,angle in GL coordinate.')
                continue

            elif com[0]=='get':
                print(self.pose, 'angle', math.degrees( qm.qzw2rad( self.pose[1][2], self.pose[1][3]) ) )
                # do not break

            elif com[0]=='quit':
                self.current_goal_index = -1  # Minus value means to exit.
                break

            elif com[0]=='fs':
                x = 0.0
                y = 0.0
                a = 0.0

                if len(com)>=2:
                    x = float(com[1])
                if len(com)>=3:
                    y = float(com[2])
                if len(com)>=4:
                    a = float(com[3])
                th  = math.radians(a)
                print('xyath', x,y,a,th)
                wp_fs = [ [x,y,0.0],[0.0,0.0,math.sin(th/2.0), math.cos(th/2.0)] ]
                waypoint = qm.get_fs_pose_in_map( wp_fs, self.pose )
                print('wp', waypoint)
                break

            elif com[0]=='gl':
                x = 0.0
                y = 0.0
                a = 0.0

                if len(com)>=2:
                    x = float(com[1])
                if len(com)>=3:
                    y = float(com[2])
                if len(com)>=4:
                    a = float(com[3])
                th  = math.radians(a)
                print('xyath', x,y,a,th)
                wp_fs = [ [x,y,0.0],[0.0,0.0,math.sin(th/2.0), math.cos(th/2.0)] ]
                print('wp', wp_fs)
                waypoint = wp_fs
                break

            else:
                print('command error.')
                continue

        return waypoint

    def send_next_goal(self):
        print('start send_next_goal()')

        """
        Sends the next waypoint in the list as a NavigateToPose goal.
        """
        waypoint_data = self.input_wp()
        print('waypoint_data', waypoint_data)

        if self.current_goal_index < 0:
            self.get_logger().info('✅ All waypoints processed! Navigation task complete.')
            # Stop the ROS 2 spinning
            print('end send_next_goal() with False')
            return False  # to exit
        
        self.get_logger().info(f'Waiting for /navigate_to_pose action server...')
        self._action_client.wait_for_server()

        # --- 1. Construct the Goal Message ---
        # The waypoint structure is: [[x, y, z], [qx, qy, qz, qw]]
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

        print('end send_next_goal() with True')
        return True  # to continue

    def goal_response_callback(self, future):
        """
        Called when the action server responds whether the goal was accepted.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server!')
            self.current_goal_index += 1
            print('BANG not accepted')
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
        
        #donot## Send the next goal
        #donot#self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        """
        Called periodically by the action server with navigation updates.
        """
        feedback = feedback_msg.feedback
        # Optional: Log or display the distance remaining
        # self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')


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
        self.pose = [ [x,y,0],[qx,qy,qz,qw] ]

def load_waypoints_from_file(file_path):
    waypoints = []
    if not os.path.exists(file_path):
        print(f"ERROR: Waypoint file not found at: {file_path}")
        return waypoints

    print(f"Loading waypoints from: {file_path} x,y,a style...")

    waypoints = []

    fin = open( file_path, 'r' )
    for line in fin.readlines():
      data = line.split()
      waypoints.append( [ float(data[0]), float(data[1]), float(data[2]) ] )
    return waypoints

def main(args=None):
    rclpy.init(args=args)
    navigator = None
    spin_thread = None
    try:
        navigator = SequentialNavigator()

        # Start the spinner thread
        spin_thread = threading.Thread(
            target=lambda: rclpy.spin(navigator),
            daemon=True
        )
        spin_thread.start()

        # Run the blocking goal/input loop in main thread.
        # Make sure send_next_goal returns instead of calling sys.exit()
        while navigator.send_next_goal():
            print('looping send_next_goal()')
            pass

        # If send_next_goal returns (e.g., user typed 'quit'), proceed to clean shutdown.
        navigator.get_logger().info('send_next_goal finished — shutting down')

    except Exception as e:
        # log error on navigator if available
        if navigator is not None:
            navigator.get_logger().error(f'Exception in main: {e}')
        else:
            print(f'Exception in main before navigator created: {e}')

    finally:
        # Defensive: destroy node, shutdown rclpy, and join spin thread
        try:
            if navigator is not None:
                try:
                    navigator.get_logger().info('Destroying node...')
                except Exception:
                    pass
                try:
                    navigator.destroy_node()
                except Exception:
                    pass

            # This will cause rclpy.spin() to return.
            try:
                rclpy.shutdown()
            except Exception:
                pass

            if spin_thread is not None and spin_thread.is_alive():
                spin_thread.join(timeout=2.0)

        except Exception as e:
            print(f'Error during final cleanup: {e}')

if __name__ == '__main__':
    main()
