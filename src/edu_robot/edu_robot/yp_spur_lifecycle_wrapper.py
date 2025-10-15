#!/usr/bin/env python3
import subprocess
import os
import signal
import time

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn


class YpSpurLifecycleWrapper(LifecycleNode):
    def __init__(self):
        super().__init__('yp_spur_lifecycle_wrapper')
        self.process = None

        # --- CORRECTION APPLIED HERE ---
        # The actual command line needed is:
        # ypspur-coordinator -p  /home/kimoto/ros2_workspace/lib/yp-spur-params/rsj-seminar2016.param -d /dev/ttyACM0
        
        # We declare the parameter with the required command as the default value.
        self.declare_parameter(
            'driver_cmd', 
            ['ypspur-coordinator', 
             '-p', 
             '/home/kimoto/ros2_workspace/lib/yp-spur-params/rsj-seminar2016.param', 
             '-d', 
             '/dev/ttyACM0']
        )
        # --- END OF CORRECTION ---
        
        self.declare_parameter('start_delay_sec', 0.5)

    def _start_driver_proc(self):
        cmd = self.get_parameter('driver_cmd').get_parameter_value().string_array_value
        if not cmd:
            # Fallback to a safe default if the parameter is empty, though unlikely now.
            cmd = ['ypspur-coordinator'] 
        self.get_logger().info(f'Starting yp-spur: {cmd}')
        # Note: subprocess.Popen expects the command as a list of strings
        self.process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        time.sleep(self.get_parameter('start_delay_sec').get_parameter_value().double_value)

    def _stop_driver_proc(self):
        if not self.process:
            return
        self.get_logger().info('Stopping yp-spur process...')
        try:
            # os.killpg is used to kill the entire process group created by preexec_fn=os.setsid
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        except Exception:
            pass
        try:
            self.process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning('yp-spur did not exit, sending SIGTERM')
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception:
                pass
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warning('Force killing yp-spur')
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.process = None

    # --- Lifecycle callbacks ---
    def on_configure(self, state):
        self.get_logger().info('on_configure')
        # No resources (like publishers or timers) are created here, 
        # but this is where ROS2 parameters are typically finalized.
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('on_activate -> start yp-spur')
        try:
            self._start_driver_proc()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Failed to start yp-spur: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state):
        self.get_logger().info('on_deactivate -> stop yp-spur')
        try:
            self._stop_driver_proc()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Failed to stop yp-spur: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state):
        self.get_logger().info('on_cleanup')
        self._stop_driver_proc()
        # Clean up any ROS 2 entities declared in on_configure if applicable
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('on_shutdown')
        self._stop_driver_proc()
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = YpSpurLifecycleWrapper()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
