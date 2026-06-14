#!/usr/bin/env python3
"""Run the physical rotation test on the robot after an explicit ROS request."""

import os
import signal
import subprocess
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


def normalized_namespace(value):
    return value.strip().strip('/')


class RotationTestServer(Node):
    def __init__(self):
        super().__init__('rotation_test_server')
        default_namespace = normalized_namespace(
            os.environ.get('ROS_NAMESPACE', '')
        )
        self.declare_parameter('namespace', default_namespace)
        self.declare_parameter('angular_speed', 0.15)
        self.declare_parameter('slow_angular_speed', 0.10)
        self.declare_parameter('sensor_timeout', 15.0)
        self.running_lock = threading.Lock()
        self.process_lock = threading.Lock()
        self.test_process = None
        self.callback_group = ReentrantCallbackGroup()
        self.service = self.create_service(
            SetBool,
            'start_rotation_test',
            self.start_test,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            'rotation test server ready; an armed SetBool(data=true) request '
            'is required to move the robot'
        )

    def start_test(self, request, response):
        if not request.data:
            return self.stop_test(response)
        if not self.running_lock.acquire(blocking=False):
            response.success = False
            response.message = 'rotation test is already running'
            return response

        try:
            return self.run_test(response)
        finally:
            with self.process_lock:
                self.test_process = None
            self.running_lock.release()

    def stop_test(self, response):
        with self.process_lock:
            process = self.test_process
        if process is None or process.poll() is not None:
            response.success = True
            response.message = 'no rotation test is running'
            return response

        self.get_logger().warning('remote stop requested for rotation test')
        os.killpg(process.pid, signal.SIGINT)
        response.success = True
        response.message = 'stop signal sent to rotation test'
        return response

    def run_test(self, response):
        try:
            namespace = normalized_namespace(
                self.get_parameter('namespace').value
            )
            command = [
                'ros2',
                'run',
                'bringup_mobile',
                'record_rotation_test.py',
                '--armed',
                '--disable-recording',
                '--angular-speed',
                str(self.get_parameter('angular_speed').value),
                '--slow-angular-speed',
                str(self.get_parameter('slow_angular_speed').value),
                '--sensor-timeout',
                str(self.get_parameter('sensor_timeout').value),
            ]
            if namespace:
                command.extend(['--namespace', namespace])

            self.get_logger().warning(
                'armed rotation test requested; starting test locally on Jetson'
            )
            process = subprocess.Popen(
                command,
                start_new_session=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            with self.process_lock:
                self.test_process = process
            stdout, stderr = process.communicate()
            output = '\n'.join(
                part.strip()
                for part in (stdout, stderr)
                if part.strip()
            )
            response.success = process.returncode == 0
            status = 'completed' if response.success else 'failed'
            response.message = (
                f'rotation test {status} with code {process.returncode}'
            )
            if output:
                response.message += f'\n{output[-3000:]}'
            self.get_logger().info(response.message)
            return response
        except Exception as error:
            response.success = False
            response.message = f'failed to start rotation test: {error}'
            self.get_logger().error(response.message)
            return response


def main():
    rclpy.init()
    node = RotationTestServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
