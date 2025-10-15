#!/usr/bin/env python3
import subprocess
import os
import signal
import time

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class IcartLifecycleWrapper(LifecycleNode):
    def __init__(self):
        super().__init__('icart_lifecycle_wrapper')
        self.process = None
        # parameter to control the command; override via ros2 param or launch
        self.declare_parameter('driver_cmd', ['ros2', 'run', 'icart_mini_driver', 'icart_mini_driver'])
        self.declare_parameter('start_delay_sec', 0.5)

    def _start_driver_proc(self):
        cmd = self.get_parameter('driver_cmd').get_parameter_value().string_array_value
        if not cmd:
            # fallback to string form
            cmd = ['ros2', 'run', 'icart_mini_driver', 'icart_mini_driver']
        self.get_logger().info(f'Starting driver: {cmd}')
        # Ensure env is inherited so ROS_DOMAIN_ID, etc, are preserved
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
        time.sleep(self.get_parameter('start_delay_sec').get_parameter_value().double_value)

    def _stop_driver_proc(self):
        if not self.process:
            return
        self.get_logger().info('Stopping driver process...')
        try:
            # send SIGINT to whole process group for graceful exit
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        except Exception:
            pass
        # wait short time then force kill
        try:
            self.process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning('Driver did not exit, sending SIGTERM')
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception:
                pass
            try:
                self.process.wait(timeout=2.0)
            except Exception:
                self.get_logger().warning('Force killing driver')
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.process = None

    # lifecycle callbacks
    def on_configure(self, state):
        self.get_logger().info('on_configure')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('on_activate -> start driver')
        try:
            self._start_driver_proc()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Failed to start driver: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state):
        self.get_logger().info('on_deactivate -> stop driver')
        try:
            self._stop_driver_proc()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Failed to stop driver: {e}')
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
    lg = IcartLifecycleWrapper()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lg)
    try:
        executor.spin()
    finally:
        lg.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
