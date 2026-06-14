#!/usr/bin/env python3
"""Validate WheelBot rotation signs and repeated 360-degree yaw closure."""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, JointState


def quaternion_yaw(orientation):
    """Return yaw in radians from a geometry_msgs Quaternion."""
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def degrees(angle_rad):
    return math.degrees(angle_rad)


def normalized_namespace(value):
    return value.strip().strip('/')


def namespaced_name(namespace, name):
    if name.startswith('/'):
        return name
    if namespace:
        return f'/{namespace}/{name}'
    return f'/{name}'


def namespaced_frame(namespace, frame):
    if frame.startswith('/'):
        return frame.lstrip('/')
    if namespace and not frame.startswith(f'{namespace}/'):
        return f'{namespace}/{frame}'
    return frame


class UnwrappedYaw:
    """Track continuous yaw across the -pi/pi boundary."""

    def __init__(self):
        self.raw = None
        self.value = None

    def update(self, raw_yaw):
        if self.raw is None:
            self.raw = raw_yaw
            self.value = raw_yaw
            return

        delta = math.atan2(
            math.sin(raw_yaw - self.raw),
            math.cos(raw_yaw - self.raw),
        )
        self.raw = raw_yaw
        self.value += delta


class RotationRecorder(Node):
    """Run measured CW and CCW rotation trials and record their results."""

    def __init__(self, args):
        super().__init__('record_rotation_test')
        self.args = args
        self.publisher = self.create_publisher(TwistStamped, args.cmd_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_callback, 20)
        self.create_subscription(
            Odometry,
            args.raw_odom_topic,
            self.raw_odom_callback,
            20,
        )
        self.create_subscription(Imu, args.imu_topic, self.imu_callback, 50)
        self.create_subscription(
            JointState,
            args.joint_states_topic,
            self.joint_states_callback,
            50,
        )

        self.odom_yaw = UnwrappedYaw()
        self.raw_odom_yaw = UnwrappedYaw()
        self.imu_yaw = UnwrappedYaw()
        self.odom_received_at = None
        self.raw_odom_received_at = None
        self.imu_received_at = None
        self.odom_x = None
        self.odom_y = None
        self.bag_process = None
        self.record_joint_samples = False
        self.joint_samples = []
        self.joint_recording_resume_at = 0.0
        self.latest_joint_sample = {}
        self.latest_joint_sample_at = None
        self.data_lock = threading.Lock()

    def odom_callback(self, msg):
        with self.data_lock:
            self.odom_yaw.update(quaternion_yaw(msg.pose.pose.orientation))
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            self.odom_received_at = time.monotonic()

    def raw_odom_callback(self, msg):
        with self.data_lock:
            self.raw_odom_yaw.update(quaternion_yaw(msg.pose.pose.orientation))
            self.raw_odom_received_at = time.monotonic()

    def imu_callback(self, msg):
        if msg.orientation_covariance[0] < 0.0:
            return
        with self.data_lock:
            self.imu_yaw.update(quaternion_yaw(msg.orientation))
            self.imu_received_at = time.monotonic()

    def joint_states_callback(self, msg):
        received_at = time.monotonic()
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        sample = {}
        for module in ('FR', 'RL'):
            steering_name = f'{module}_steering_joint'
            right_name = f'{module}_drive_right_joint'
            left_name = f'{module}_drive_left_joint'
            if not all(
                name in positions
                for name in (steering_name, right_name, left_name)
            ):
                continue
            if not all(name in velocities for name in (right_name, left_name)):
                continue
            sample[module] = {
                'received_at': received_at,
                'steering_rad': positions[steering_name],
                'right_velocity_rad_s': velocities[right_name],
                'left_velocity_rad_s': velocities[left_name],
            }
        if sample:
            with self.data_lock:
                self.latest_joint_sample = sample
                self.latest_joint_sample_at = received_at
                if (
                    self.record_joint_samples
                    and received_at >= self.joint_recording_resume_at
                ):
                    self.joint_samples.append(sample)

    def measurement_snapshot(self):
        with self.data_lock:
            return {
                'odom_yaw': self.odom_yaw.value,
                'raw_odom_yaw': self.raw_odom_yaw.value,
                'imu_yaw': self.imu_yaw.value,
                'odom_received_at': self.odom_received_at,
                'raw_odom_received_at': self.raw_odom_received_at,
                'imu_received_at': self.imu_received_at,
                'odom_x': self.odom_x,
                'odom_y': self.odom_y,
            }

    def steering_watchdog_error(self):
        with self.data_lock:
            latest_at = self.latest_joint_sample_at
            latest_sample = dict(self.latest_joint_sample)
        if latest_at is None:
            return None
        if time.monotonic() - latest_at > self.args.max_sensor_age:
            return None

        worst = None
        for module, sample in latest_sample.items():
            error = math.atan2(
                math.sin(sample['steering_rad'] - self.args.steering_target),
                math.cos(sample['steering_rad'] - self.args.steering_target),
            )
            if worst is None or abs(error) > abs(worst[1]):
                worst = (module, error)
        return worst

    def summarize_module_steering(self, module):
        with self.data_lock:
            joint_samples = list(self.joint_samples)
        samples = [
            sample[module]
            for sample in joint_samples
            if (
                module in sample
                and sample[module]['received_at'] >= self.steering_analysis_start_at
            )
        ]
        if not samples:
            return {'sample_count': 0, 'pass': False}

        steering = [sample['steering_rad'] for sample in samples]
        target = self.args.steering_target
        errors = [
            math.atan2(math.sin(value - target), math.cos(value - target))
            for value in steering
        ]
        mean = sum(steering) / len(steering)
        variance = sum((value - mean) ** 2 for value in steering) / len(steering)
        outside = [
            abs(error) > self.args.steering_alignment_tolerance
            for error in errors
        ]
        threshold_crossings = sum(
            outside[index] != outside[index - 1]
            for index in range(1, len(outside))
        )
        traction = [
            0.5 * (
                sample['right_velocity_rad_s']
                + sample['left_velocity_rad_s']
            )
            for sample in samples
        ]
        drive_difference = [
            abs(
                sample['right_velocity_rad_s']
                - sample['left_velocity_rad_s']
            )
            for sample in samples
        ]
        stddev = math.sqrt(variance)
        max_abs_error = max(abs(error) for error in errors)
        return {
            'sample_count': len(samples),
            'steering_target_deg': degrees(target),
            'steering_mean_deg': degrees(mean),
            'steering_stddev_deg': degrees(stddev),
            'steering_min_deg': degrees(min(steering)),
            'steering_max_deg': degrees(max(steering)),
            'max_abs_steering_error_deg': degrees(max_abs_error),
            'outside_alignment_tolerance_percent': (
                100.0 * sum(outside) / len(outside)
            ),
            'alignment_threshold_crossings': threshold_crossings,
            'traction_zero_percent': (
                100.0 * sum(abs(value) <= 0.05 for value in traction)
                / len(traction)
            ),
            'mean_abs_drive_difference_rad_s': (
                sum(drive_difference) / len(drive_difference)
            ),
            'pass': (
                stddev <= self.args.max_steering_stddev
                and max_abs_error <= self.args.max_steering_error
            ),
        }

    def publish_twist(self, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.twist.angular.z = angular_z
        self.publisher.publish(msg)

    def spin_sleep(self, duration_s):
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(max(0.0, min(0.05, deadline - time.monotonic())))

    def hold(self, angular_z, duration_s):
        deadline = time.monotonic() + duration_s
        period_s = 1.0 / self.args.rate
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_twist(angular_z)
            time.sleep(period_s)

    def stop_robot(self, duration_s=None):
        duration_s = self.args.stop_time if duration_s is None else duration_s
        self.hold(0.0, duration_s)
        self.publish_twist(0.0)

    def wait_for_controller(self):
        deadline = time.monotonic() + self.args.controller_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return
            time.sleep(0.1)
        self.get_logger().warning(
            f'no local ROS graph subscriber found on {self.publisher.topic_name}; '
            'continuing because rmw_zenoh may hide remote subscribers'
        )

    def wait_for_measurements(self):
        deadline = time.monotonic() + self.args.sensor_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            snapshot = self.measurement_snapshot()
            if (
                snapshot['odom_yaw'] is not None
                and snapshot['raw_odom_yaw'] is not None
                and snapshot['imu_yaw'] is not None
            ):
                break
            time.sleep(0.05)
        snapshot = self.measurement_snapshot()
        if snapshot['odom_yaw'] is None:
            raise RuntimeError(
                f'no Odometry received on {self.args.odom_topic}; '
                'the test requires fused base-frame yaw'
            )

        if snapshot['raw_odom_yaw'] is None:
            raise RuntimeError(
                f'no raw Odometry received on {self.args.raw_odom_topic}'
            )
        if snapshot['imu_yaw'] is None:
            raise RuntimeError(
                f'no valid IMU orientation received on {self.args.imu_topic}; '
                'IMU orientation is required to control physical rotation'
            )

    def ensure_fresh_measurement(self, label):
        received_at_key = {
            'fused odometry': 'odom_received_at',
            'raw wheel odometry': 'raw_odom_received_at',
            'IMU orientation': 'imu_received_at',
        }[label]
        received_at = self.measurement_snapshot()[received_at_key]
        if received_at is None:
            raise RuntimeError(f'{label} has not been received')
        age = time.monotonic() - received_at
        if age <= self.args.max_sensor_age:
            return

        self.get_logger().warning(
            f'{label} is stale by {age:.3f} seconds; waiting for fresh samples'
        )
        deadline = time.monotonic() + self.args.sensor_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            current_received_at = self.measurement_snapshot()[received_at_key]
            if current_received_at is not None:
                age = time.monotonic() - current_received_at
            if age <= self.args.max_sensor_age:
                return
            time.sleep(0.05)

        raise RuntimeError(f'{label} is stale by {age:.3f} seconds')

    def ensure_fresh_measurements(self):
        self.ensure_fresh_measurement('IMU orientation')
        self.ensure_fresh_measurement('fused odometry')
        self.ensure_fresh_measurement('raw wheel odometry')

    def pause_until_measurements_fresh(self, label):
        snapshot = self.measurement_snapshot()
        now = time.monotonic()
        measurements = (
            ('IMU orientation', snapshot['imu_received_at']),
            ('fused odometry', snapshot['odom_received_at']),
            ('raw wheel odometry', snapshot['raw_odom_received_at']),
        )
        stale = [
            (measurement, math.inf if received_at is None else now - received_at)
            for measurement, received_at in measurements
            if received_at is None or now - received_at > self.args.max_sensor_age
        ]
        if not stale:
            return 0.0

        stale_summary = ', '.join(
            f'{measurement}={age:.3f}s' if math.isfinite(age)
            else f'{measurement}=missing'
            for measurement, age in stale
        )
        self.get_logger().warning(
            f'{label}: feedback stale ({stale_summary}); '
            'holding zero command until communication recovers'
        )

        outage_started_at = time.monotonic()
        deadline = outage_started_at + self.args.sensor_timeout
        period_s = 1.0 / self.args.rate
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_twist(0.0)
            snapshot = self.measurement_snapshot()
            now = time.monotonic()
            received_times = (
                snapshot['imu_received_at'],
                snapshot['odom_received_at'],
                snapshot['raw_odom_received_at'],
            )
            if all(
                received_at is not None
                and now - received_at <= self.args.max_sensor_age
                for received_at in received_times
            ):
                outage_duration = now - outage_started_at
                self.get_logger().info(
                    f'{label}: communication recovered after '
                    f'{outage_duration:.3f} seconds; resuming rotation'
                )
                return outage_duration
            time.sleep(period_s)

        self.publish_twist(0.0)
        outage_duration = time.monotonic() - outage_started_at
        raise RuntimeError(
            f'{label}: communication did not recover within '
            f'{outage_duration:.3f} seconds; robot remains stopped'
        )

    def start_recording(self, output_path):
        command = [
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '--output', str(output_path),
        ]
        if self.args.record_all_topics:
            command.append('--all-topics')
        else:
            command.extend([
                self.args.odom_topic,
                self.args.raw_odom_topic,
                self.args.imu_topic,
                self.args.cmd_topic,
                self.args.controller_cmd_topic,
                self.args.joint_states_topic,
                self.args.tf_topic,
                self.args.tf_static_topic,
                '/diagnostics',
                '/rosout',
            ])
        self.get_logger().info(f'starting MCAP recording: {output_path}')
        self.bag_process = subprocess.Popen(command, start_new_session=True)

        deadline = time.monotonic() + self.args.record_start_timeout
        while time.monotonic() < deadline:
            return_code = self.bag_process.poll()
            if return_code is not None:
                self.bag_process = None
                raise RuntimeError(f'ros2 bag record exited early with code {return_code}')
            if output_path.exists():
                self.spin_sleep(self.args.record_settle_time)
                return
            time.sleep(0.1)

        self.stop_recording()
        raise RuntimeError(f'ros2 bag record did not create {output_path}')

    def stop_recording(self):
        if self.bag_process is None:
            return

        process = self.bag_process
        self.bag_process = None
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=self.args.record_stop_timeout)
            except subprocess.TimeoutExpired:
                self.get_logger().warning('ros2 bag record did not stop; terminating it')
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.0)

        if process.returncode not in (0, 130, -signal.SIGINT):
            raise RuntimeError(f'ros2 bag record exited with code {process.returncode}')

    def rotate_once(self, direction, label):
        self.ensure_fresh_measurements()
        start_snapshot = self.measurement_snapshot()
        start_odom_yaw = start_snapshot['odom_yaw']
        start_raw_odom_yaw = start_snapshot['raw_odom_yaw']
        start_imu_yaw = start_snapshot['imu_yaw']
        start_time = time.monotonic()
        target = 2.0 * math.pi
        sign_checked = False
        steering_error_since = None
        best_progress = 0.0
        last_progress_at = start_time
        communication_outages = []
        steering_watchdog_enable_at = (
            start_time + self.args.steering_watchdog_grace
        )
        with self.data_lock:
            self.joint_samples = []
            self.steering_analysis_start_at = (
                start_time + self.args.steering_watchdog_grace
            )
            self.joint_recording_resume_at = self.steering_analysis_start_at
            self.record_joint_samples = True

        try:
            self.get_logger().info(
                f'{label}: target={direction * 360:+.0f} deg, '
                f'fast={direction * self.args.angular_speed:+.3f} rad/s'
            )

            while rclpy.ok():
                outage_duration = self.pause_until_measurements_fresh(label)
                if outage_duration > 0.0:
                    communication_outages.append(outage_duration)
                    start_time += outage_duration
                    last_progress_at = time.monotonic()
                    steering_error_since = None
                    steering_watchdog_enable_at = (
                        time.monotonic() + self.args.steering_watchdog_grace
                    )
                    with self.data_lock:
                        self.joint_recording_resume_at = (
                            steering_watchdog_enable_at
                        )
                elapsed = time.monotonic() - start_time
                if elapsed > self.args.turn_timeout:
                    raise RuntimeError(
                        f'{label}: exceeded '
                        f'{self.args.turn_timeout:.1f} s timeout'
                    )

                imu_delta = (
                    self.measurement_snapshot()['imu_yaw'] - start_imu_yaw
                )
                progress = direction * imu_delta
                if progress >= best_progress + self.args.stall_progress:
                    best_progress = progress
                    last_progress_at = time.monotonic()
                if progress >= self.args.sign_check_angle:
                    sign_checked = True
                elif imu_delta * direction < -self.args.sign_check_angle:
                    raise RuntimeError(
                        f'{label}: wrong IMU yaw sign; '
                        f'command sign={direction:+.0f}, '
                        f'IMU delta={degrees(imu_delta):+.2f} deg'
                    )

                if time.monotonic() >= steering_watchdog_enable_at:
                    worst = self.steering_watchdog_error()
                    if (
                        worst is not None
                        and abs(worst[1]) > self.args.live_steering_error
                    ):
                        if steering_error_since is None:
                            steering_error_since = time.monotonic()
                        elif (
                            time.monotonic() - steering_error_since
                            >= self.args.steering_error_duration
                        ):
                            self.stop_robot(0.5)
                            raise RuntimeError(
                                f'{label}: steering watchdog stopped robot; '
                                f'{worst[0]} error='
                                f'{degrees(worst[1]):+.2f} deg'
                            )
                    else:
                        steering_error_since = None

                    if (
                        time.monotonic() - last_progress_at
                        >= self.args.rotation_stall_timeout
                    ):
                        self.stop_robot(0.5)
                        raise RuntimeError(
                            f'{label}: rotation stall watchdog stopped robot; '
                            f'IMU progress remained near '
                            f'{degrees(best_progress):.2f} deg'
                        )

                remaining = target - progress
                if remaining <= self.args.stop_tolerance:
                    break

                speed = self.args.angular_speed
                if remaining < self.args.slowdown_angle:
                    speed = self.args.slow_angular_speed
                self.publish_twist(direction * speed)
                time.sleep(1.0 / self.args.rate)
        finally:
            with self.data_lock:
                self.record_joint_samples = False

        self.stop_robot(self.args.settle_time)
        self.ensure_fresh_measurements()

        end_snapshot = self.measurement_snapshot()
        imu_delta = end_snapshot['imu_yaw'] - start_imu_yaw
        odom_delta = end_snapshot['odom_yaw'] - start_odom_yaw
        raw_odom_delta = end_snapshot['raw_odom_yaw'] - start_raw_odom_yaw
        directional_imu = direction * imu_delta
        directional_raw_odom = direction * raw_odom_delta
        imu_error = directional_imu - target
        raw_odom_imu_error = directional_raw_odom - directional_imu
        wheel_error_percent = (
            100.0 * raw_odom_imu_error / directional_imu
            if abs(directional_imu) > 1e-6
            else math.nan
        )
        sign_pass = sign_checked and directional_imu > 0.0
        closure_pass = abs(imu_error) <= self.args.max_turn_error
        wheel_consistency_pass = (
            abs(raw_odom_imu_error) <= self.args.max_odom_imu_disagreement
        )
        module_steering = {
            module: self.summarize_module_steering(module)
            for module in ('FR', 'RL')
        }
        steering_stability_pass = all(
            summary['pass'] for summary in module_steering.values()
        )
        result = {
            'label': label,
            'direction': 'ccw' if direction > 0.0 else 'cw',
            'command_sign': int(direction),
            'duration_s': time.monotonic() - start_time,
            'imu_delta_deg': degrees(imu_delta),
            'odom_delta_deg': degrees(odom_delta),
            'raw_wheel_odom_delta_deg': degrees(raw_odom_delta),
            'imu_turn_error_deg': degrees(imu_error),
            'raw_odom_imu_disagreement_deg': degrees(raw_odom_imu_error),
            'wheel_odom_error_percent': wheel_error_percent,
            'communication_outage_count': len(communication_outages),
            'communication_outage_total_s': sum(communication_outages),
            'communication_outage_durations_s': communication_outages,
            'sign_pass': sign_pass,
            'closure_pass': closure_pass,
            'wheel_consistency_pass': wheel_consistency_pass,
            'module_steering': module_steering,
            'steering_stability_pass': steering_stability_pass,
        }
        self.get_logger().info(
            f'{label}: IMU={result["imu_delta_deg"]:+.3f} deg, '
            f'wheel odom={result["raw_wheel_odom_delta_deg"]:+.3f} deg, '
            f'IMU error={result["imu_turn_error_deg"]:+.3f} deg, '
            f'wheel disagreement={result["raw_odom_imu_disagreement_deg"]:+.3f} deg '
            f'({wheel_error_percent:+.1f}%), '
            f'sign={"PASS" if sign_pass else "FAIL"}, '
            f'closure={"PASS" if closure_pass else "FAIL"}, '
            f'wheels={"PASS" if wheel_consistency_pass else "FAIL"}, '
            f'steering={"PASS" if steering_stability_pass else "FAIL"}'
        )
        for module, summary in module_steering.items():
            if summary['sample_count'] == 0:
                self.get_logger().warning(f'{label}: no {module} joint samples')
                continue
            self.get_logger().info(
                f'{label} {module}: '
                f'mean={summary["steering_mean_deg"]:+.2f} deg, '
                f'stddev={summary["steering_stddev_deg"]:.2f} deg, '
                f'range={summary["steering_min_deg"]:+.2f}..'
                f'{summary["steering_max_deg"]:+.2f} deg, '
                f'max error={summary["max_abs_steering_error_deg"]:.2f} deg, '
                f'crossings={summary["alignment_threshold_crossings"]}'
            )
        return result

    def run(self):
        if not self.args.armed:
            self.get_logger().warning(
                'dry-run only; pass --armed after lifting hazards and clearing the robot area'
            )
            self.get_logger().info(
                f'plan: {self.args.turns} CW 360-degree turn(s), then '
                f'{self.args.turns} CCW turn(s), controlled by {self.args.imu_topic}'
            )
            return

        self.wait_for_controller()
        self.wait_for_measurements()
        output_dir = Path(self.args.output_dir).expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = output_dir / f'rotation_validation_{timestamp}'
        report_path = output_dir / f'rotation_validation_{timestamp}.json'

        self.stop_robot(self.args.initial_stop_time)
        self.ensure_fresh_measurements()
        suite_start = self.measurement_snapshot()
        suite_start_imu_yaw = suite_start['imu_yaw']
        suite_start_odom_yaw = suite_start['odom_yaw']
        suite_start_raw_odom_yaw = suite_start['raw_odom_yaw']
        suite_start_x = suite_start['odom_x']
        suite_start_y = suite_start['odom_y']
        results = []
        failure = None
        interrupted = False

        if not self.args.disable_recording:
            self.start_recording(bag_path)
        try:
            for index in range(self.args.turns):
                results.append(self.rotate_once(-1.0, f'CW turn {index + 1}'))
                self.stop_robot(self.args.between_turns_time)
            for index in range(self.args.turns):
                results.append(self.rotate_once(1.0, f'CCW turn {index + 1}'))
                self.stop_robot(self.args.between_turns_time)
        except Exception as error:
            failure = str(error)
        except KeyboardInterrupt:
            interrupted = True
            failure = 'test interrupted manually'
        finally:
            self.stop_robot(self.args.final_stop_time)
            self.stop_recording()

        suite_end = self.measurement_snapshot()
        suite_imu_yaw_error = suite_end['imu_yaw'] - suite_start_imu_yaw
        suite_odom_yaw_error = suite_end['odom_yaw'] - suite_start_odom_yaw
        suite_raw_odom_yaw_error = (
            suite_end['raw_odom_yaw'] - suite_start_raw_odom_yaw
        )
        position_error = math.hypot(
            suite_end['odom_x'] - suite_start_x,
            suite_end['odom_y'] - suite_start_y,
        )
        report = {
            'timestamp': timestamp,
            'odom_topic': self.args.odom_topic,
            'raw_odom_topic': self.args.raw_odom_topic,
            'imu_topic': self.args.imu_topic,
            'turns_per_direction': self.args.turns,
            'angular_speed_rad_s': self.args.angular_speed,
            'slow_angular_speed_rad_s': self.args.slow_angular_speed,
            'max_turn_error_deg': degrees(self.args.max_turn_error),
            'max_odom_imu_disagreement_deg': degrees(
                self.args.max_odom_imu_disagreement
            ),
            'steering_target_deg': degrees(self.args.steering_target),
            'steering_alignment_tolerance_deg': degrees(
                self.args.steering_alignment_tolerance
            ),
            'max_steering_stddev_deg': degrees(
                self.args.max_steering_stddev
            ),
            'max_steering_error_deg': degrees(
                self.args.max_steering_error
            ),
            'live_steering_error_deg': degrees(
                self.args.live_steering_error
            ),
            'steering_watchdog_grace_s': self.args.steering_watchdog_grace,
            'steering_error_duration_s': self.args.steering_error_duration,
            'rotation_stall_timeout_s': self.args.rotation_stall_timeout,
            'stall_progress_deg': degrees(self.args.stall_progress),
            'trials': results,
            'return_to_start_imu_yaw_error_deg': degrees(suite_imu_yaw_error),
            'return_to_start_fused_odom_yaw_error_deg': degrees(
                suite_odom_yaw_error
            ),
            'return_to_start_raw_odom_yaw_error_deg': degrees(
                suite_raw_odom_yaw_error
            ),
            'return_to_start_position_error_m': position_error,
            'return_to_start_pass': (
                abs(suite_imu_yaw_error) <= self.args.max_return_error
            ),
            'all_sign_checks_pass': all(item['sign_pass'] for item in results),
            'all_turn_closures_pass': all(item['closure_pass'] for item in results),
            'all_wheel_consistency_checks_pass': all(
                item['wheel_consistency_pass'] for item in results
            ),
            'all_steering_stability_checks_pass': all(
                item['steering_stability_pass'] for item in results
            ),
            'completed_trials': len(results),
            'expected_trials': 2 * self.args.turns,
            'failure': failure,
            'bag_path': None if self.args.disable_recording else str(bag_path),
        }
        report_path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')

        self.get_logger().info(
            f'return-to-start IMU yaw error='
            f'{degrees(suite_imu_yaw_error):+.3f} deg, '
            f'wheel odom yaw error={degrees(suite_raw_odom_yaw_error):+.3f} deg, '
            f'position drift={position_error:.3f} m, '
            f'result={"PASS" if report["return_to_start_pass"] else "FAIL"}'
        )
        self.get_logger().info(f'report saved to {report_path}')
        if interrupted:
            raise KeyboardInterrupt
        if failure is not None:
            raise RuntimeError(failure)


def positive_float(value):
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError('value must be greater than zero')
    return parsed


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError('value must be greater than zero')
    return parsed


def angle_degrees(value):
    return math.radians(positive_float(value))


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Validate ROS base-frame rotation signs, repeated 360-degree turns, '
            'and CW/CCW return-to-start closure.'
        )
    )
    parser.add_argument(
        '--cmd-topic',
        default='key_vel',
        help='twist_mux input topic; relative names use --namespace',
    )
    parser.add_argument(
        '--odom-topic',
        default='odom',
        help='fused base-frame nav_msgs/Odometry topic; relative names use --namespace',
    )
    parser.add_argument('--raw-odom-topic', default='swerve_controller/odom')
    parser.add_argument('--imu-topic', default='imu/data')
    parser.add_argument('--controller-cmd-topic', default='swerve_controller/cmd_vel')
    parser.add_argument('--joint-states-topic', default='joint_states')
    parser.add_argument('--tf-topic', default='tf')
    parser.add_argument('--tf-static-topic', default='tf_static')
    parser.add_argument('--frame-id', default='base_footprint')
    parser.add_argument(
        '--namespace',
        default=os.environ.get('ROS_NAMESPACE', ''),
        help='topic/frame namespace; defaults to ROS_NAMESPACE',
    )
    parser.add_argument('--output-dir', default='DEBUG')
    parser.add_argument('--turns', type=positive_int, default=1)
    parser.add_argument('--angular-speed', type=positive_float, default=0.15)
    parser.add_argument('--slow-angular-speed', type=positive_float, default=0.10)
    parser.add_argument(
        '--slowdown-angle-deg',
        type=angle_degrees,
        default=math.radians(25.0),
    )
    parser.add_argument(
        '--stop-tolerance-deg',
        type=angle_degrees,
        default=math.radians(2.0),
    )
    parser.add_argument(
        '--sign-check-angle-deg',
        type=angle_degrees,
        default=math.radians(8.0),
    )
    parser.add_argument(
        '--max-turn-error-deg',
        type=angle_degrees,
        default=math.radians(8.0),
    )
    parser.add_argument(
        '--max-return-error-deg',
        type=angle_degrees,
        default=math.radians(10.0),
    )
    parser.add_argument(
        '--max-odom-imu-disagreement-deg',
        type=angle_degrees,
        default=math.radians(10.0),
        help='maximum allowed wheel-odometry versus IMU yaw difference per turn',
    )
    parser.add_argument(
        '--steering-target-deg',
        type=angle_degrees,
        default=math.radians(45.0),
        help='expected FR/RL steering angle during pure rotation',
    )
    parser.add_argument(
        '--steering-alignment-tolerance-deg',
        type=angle_degrees,
        default=0.10,
        help='hardware steering alignment tolerance used for crossing counts',
    )
    parser.add_argument(
        '--max-steering-stddev-deg',
        type=angle_degrees,
        default=math.radians(4.0),
        help='maximum steering standard deviation allowed per module and turn',
    )
    parser.add_argument(
        '--max-steering-error-deg',
        type=angle_degrees,
        default=math.radians(12.0),
        help='maximum instantaneous steering error allowed per module and turn',
    )
    parser.add_argument(
        '--live-steering-error-deg',
        type=angle_degrees,
        default=math.radians(15.0),
        help='stop immediately when steering exceeds this error for too long',
    )
    parser.add_argument(
        '--steering-watchdog-grace',
        type=positive_float,
        default=3.0,
        help='initial steering alignment time before enabling the watchdog',
    )
    parser.add_argument(
        '--steering-error-duration',
        type=positive_float,
        default=0.75,
        help='continuous excessive steering-error duration before stopping',
    )
    parser.add_argument(
        '--rotation-stall-timeout',
        type=positive_float,
        default=5.0,
        help='stop when IMU yaw makes no meaningful progress for this duration',
    )
    parser.add_argument(
        '--stall-progress-deg',
        type=angle_degrees,
        default=math.radians(2.0),
        help='IMU yaw increase considered meaningful watchdog progress',
    )
    parser.add_argument('--turn-timeout', type=positive_float, default=60.0)
    parser.add_argument('--initial-stop-time', type=positive_float, default=3.0)
    parser.add_argument('--settle-time', type=positive_float, default=2.0)
    parser.add_argument('--between-turns-time', type=positive_float, default=2.0)
    parser.add_argument('--final-stop-time', type=positive_float, default=3.0)
    parser.add_argument('--stop-time', type=positive_float, default=1.0)
    parser.add_argument('--rate', type=positive_float, default=20.0)
    parser.add_argument('--controller-timeout', type=positive_float, default=5.0)
    parser.add_argument('--sensor-timeout', type=positive_float, default=8.0)
    parser.add_argument('--max-sensor-age', type=positive_float, default=1.0)
    parser.add_argument('--record-start-timeout', type=positive_float, default=5.0)
    parser.add_argument('--record-settle-time', type=positive_float, default=1.0)
    parser.add_argument('--record-stop-timeout', type=positive_float, default=10.0)
    parser.add_argument(
        '--record-all-topics',
        action='store_true',
        help='record every discovered topic instead of the reduced validation set',
    )
    parser.add_argument(
        '--disable-recording',
        action='store_true',
        help='run the test without starting a local rosbag recorder',
    )
    parser.add_argument(
        '--armed',
        action='store_true',
        help='record bags and publish non-zero rotation commands',
    )
    args = parser.parse_args(remove_ros_args()[1:])
    args.namespace = normalized_namespace(args.namespace)
    args.cmd_topic = namespaced_name(args.namespace, args.cmd_topic)
    args.odom_topic = namespaced_name(args.namespace, args.odom_topic)
    args.raw_odom_topic = namespaced_name(args.namespace, args.raw_odom_topic)
    args.imu_topic = namespaced_name(args.namespace, args.imu_topic)
    args.controller_cmd_topic = namespaced_name(
        args.namespace,
        args.controller_cmd_topic,
    )
    args.joint_states_topic = namespaced_name(
        args.namespace,
        args.joint_states_topic,
    )
    args.tf_topic = namespaced_name(args.namespace, args.tf_topic)
    args.tf_static_topic = namespaced_name(args.namespace, args.tf_static_topic)
    args.frame_id = namespaced_frame(args.namespace, args.frame_id)
    args.slowdown_angle = args.slowdown_angle_deg
    args.stop_tolerance = args.stop_tolerance_deg
    args.sign_check_angle = args.sign_check_angle_deg
    args.max_turn_error = args.max_turn_error_deg
    args.max_return_error = args.max_return_error_deg
    args.max_odom_imu_disagreement = args.max_odom_imu_disagreement_deg
    args.steering_target = args.steering_target_deg
    args.steering_alignment_tolerance = (
        args.steering_alignment_tolerance_deg
    )
    args.max_steering_stddev = args.max_steering_stddev_deg
    args.max_steering_error = args.max_steering_error_deg
    args.live_steering_error = args.live_steering_error_deg
    args.stall_progress = args.stall_progress_deg
    if args.slow_angular_speed > args.angular_speed:
        parser.error('--slow-angular-speed must not exceed --angular-speed')
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = RotationRecorder(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warning('interrupted; stopping robot and recording')
    except Exception as error:
        node.get_logger().error(str(error))
        raise
    finally:
        if node.args.armed:
            node.stop_robot(0.5)
        node.stop_recording()
        executor.shutdown()
        executor_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
