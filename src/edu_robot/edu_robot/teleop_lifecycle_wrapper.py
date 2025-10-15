#!/usr/bin/env python3
import subprocess
import os
import signal
import time

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from ament_index_python.packages import get_package_share_directory

class TeleopLifecycleWrapper(LifecycleNode):
    """
    A lifecycle wrapper node that launches the teleop_twist_joy node as a subprocess
    when activated and gracefully stops it when deactivated.
    """
    def __init__(self):
        super().__init__('teleop_lifecycle_wrapper')
        self.process = None

        # Parameter to control the core teleop command
        self.declare_parameter('target_pkg', 'teleop_twist_joy')
        self.declare_parameter('target_exec', 'teleop_node')

        # Parameter for the configuration file path
        # This parameter will be set via the launch file to point to params_default.yaml
        self.declare_parameter('config_file_path', '')
        
        self.declare_parameter('start_delay_sec', 0.5)

    def _start_driver_proc(self):
        target_pkg = self.get_parameter('target_pkg').get_parameter_value().string_value
        target_exec = self.get_parameter('target_exec').get_parameter_value().string_value
        config_file_path = self.get_parameter('config_file_path').get_parameter_value().string_value
        
        # Base command: ['ros2', 'run', 'teleop_twist_joy', 'teleop_node']
        cmd = ['ros2', 'run', target_pkg, target_exec]

        # Append parameter file argument if a path is provided
        if config_file_path:
            self.get_logger().info(f'Loading parameters for wrapped node from: {config_file_path}')
            cmd.extend(['--ros-args', '--params-file', config_file_path])

        self.get_logger().info(f'Starting teleop driver: {cmd}')
        
        # Ensure env is inherited so ROS_DOMAIN_ID, etc, are preserved
        # Setsid is critical for process group management (to stop children processes)
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
        
        # Wait for the subprocess to initialize
        time.sleep(self.get_parameter('start_delay_sec').get_parameter_value().double_value)

    def _stop_driver_proc(self):
        if not self.process:
            return
        
        self.get_logger().info('Stopping teleop driver process...')
        
        try:
            # Send SIGINT to whole process group for graceful exit (first attempt)
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        except Exception:
            pass # Process might have already died
        
        # wait short time then force kill
        try:
            self.process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning('Teleop driver did not exit, sending SIGTERM')
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception:
                pass
            try:
                self.process.wait(timeout=2.0)
            except Exception:
                self.get_logger().warning('Force killing teleop driver')
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.process = None

    # Lifecycle Callbacks (standard implementation)
    def on_configure(self, state):
        self.get_logger().info('on_configure')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('on_activate -> start teleop driver')
        try:
            self._start_driver_proc()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Failed to start teleop driver: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state):
        self.get_logger().info('on_deactivate -> stop teleop driver')
        try:
            self._stop_driver_proc()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Failed to stop teleop driver: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state):
        self.get_logger().info('on_cleanup')
        # ensure stopped
        self._stop_driver_proc()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('on_shutdown')
        self._stop_driver_proc()
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    lg = TeleopLifecycleWrapper()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lg)
    try:
        executor.spin()
    finally:
        lg.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
