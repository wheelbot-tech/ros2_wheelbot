#!/usr/bin/env python3
"""Record MCAP locally while a Jetson service executes the rotation test."""

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import SetBool


def normalized_namespace(value):
    return value.strip().strip('/')


def namespaced_name(namespace, name):
    if name.startswith('/'):
        return name
    if namespace:
        return f'/{namespace}/{name}'
    return f'/{name}'


class RotationTestClient(Node):
    def __init__(self, args):
        super().__init__('start_rotation_test')
        self.args = args
        service_name = namespaced_name(
            args.namespace,
            'start_rotation_test',
        )
        self.client = self.create_client(SetBool, service_name)

    def call(self):
        if not self.client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(
                f'Jetson rotation service {self.client.srv_name} is unavailable'
            )
        request = SetBool.Request()
        request.data = True
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.args.test_timeout,
        )
        if not future.done():
            raise RuntimeError(
                f'Jetson rotation test exceeded {self.args.test_timeout:.1f}s'
            )
        response = future.result()
        if response is None:
            raise RuntimeError('rotation service returned no response')
        return response

    def stop_remote_test(self):
        if not self.client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = False
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Record rotation topics on this computer while Jetson executes '
            'the physical rotation test.'
        )
    )
    parser.add_argument(
        '--namespace',
        default=os.environ.get('ROS_NAMESPACE', 'robot_1'),
    )
    parser.add_argument('--output-dir', default='DEBUG')
    parser.add_argument('--service-timeout', type=float, default=10.0)
    parser.add_argument('--test-timeout', type=float, default=180.0)
    parser.add_argument('--record-start-time', type=float, default=1.0)
    parser.add_argument(
        '--armed',
        action='store_true',
        help='required confirmation that the physical robot may move',
    )
    args = parser.parse_args(remove_ros_args()[1:])
    args.namespace = normalized_namespace(args.namespace)
    return args


def recording_command(args, output_path):
    topics = [
        namespaced_name(args.namespace, 'odom'),
        namespaced_name(args.namespace, 'swerve_controller/odom'),
        namespaced_name(args.namespace, 'imu/data'),
        namespaced_name(args.namespace, 'key_vel'),
        namespaced_name(args.namespace, 'swerve_controller/cmd_vel'),
        namespaced_name(args.namespace, 'joint_states'),
        namespaced_name(args.namespace, 'tf'),
        namespaced_name(args.namespace, 'tf_static'),
        '/diagnostics',
        '/rosout',
    ]
    return [
        'ros2',
        'bag',
        'record',
        '--storage',
        'mcap',
        '--output',
        str(output_path),
        *topics,
    ]


def stop_recording(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)


def main():
    args = parse_args()
    if not args.armed:
        raise SystemExit('refusing to move robot without --armed')

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = output_dir / f'remote_rotation_validation_{timestamp}'
    recorder = subprocess.Popen(
        recording_command(args, bag_path),
        start_new_session=True,
    )

    rclpy.init()
    node = RotationTestClient(args)
    request_completed = False
    try:
        node.get_logger().info(f'recording MCAP locally at {bag_path}')
        try:
            recorder.wait(timeout=args.record_start_time)
        except subprocess.TimeoutExpired:
            pass
        else:
            raise RuntimeError(
                f'ros2 bag record exited early with code {recorder.returncode}'
            )

        response = node.call()
        request_completed = True
        if response.success:
            node.get_logger().info(response.message)
        else:
            raise RuntimeError(response.message)
    finally:
        if not request_completed:
            node.stop_remote_test()
        stop_recording(recorder)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
