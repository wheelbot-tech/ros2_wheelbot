#!/usr/bin/env python3
"""Open-loop WheelBot rectangle and diagonal motion script."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class RectangleDiagonalsPath(Node):
    def __init__(self, args):
        super().__init__('rectangle_diagonals_path')
        self.args = args
        self.publisher = self.create_publisher(TwistStamped, args.cmd_topic, 10)

    def publish_twist(self, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self.publisher.publish(msg)

    def hold(self, vx, vy, wz, duration_s):
        if not self.args.armed:
            self.get_logger().info(
                f'dry-run: vx={vx:.3f} m/s vy={vy:.3f} m/s wz={wz:.3f} rad/s '
                f'for {duration_s:.2f} s'
            )
            return

        end_time = time.monotonic() + duration_s
        period_s = 1.0 / self.args.rate
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_twist(vx, vy, wz)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_s)

    def stop(self, duration_s=None):
        duration_s = self.args.stop_time if duration_s is None else duration_s
        self.get_logger().info(f'stop for {duration_s:.2f} s')
        self.hold(0.0, 0.0, 0.0, duration_s)
        if self.args.armed:
            self.publish_twist(0.0, 0.0, 0.0)

    def move_vector(self, dx, dy, label):
        distance = math.hypot(dx, dy)
        if distance <= 0.0:
            return

        duration_s = distance / self.args.linear_speed
        vx = dx / duration_s
        vy = dy / duration_s
        self.get_logger().info(
            f'{label}: dx={dx:.2f} m dy={dy:.2f} m, '
            f'distance={distance:.2f} m, duration={duration_s:.2f} s'
        )
        self.hold(vx, vy, 0.0, duration_s)
        self.stop()

    def rotate_360(self, label):
        direction = 1.0 if self.args.rotation_direction >= 0.0 else -1.0
        if self.args.rotation_settle_time > 0.0:
            settle_wz = direction * self.args.rotation_settle_speed
            self.get_logger().info(
                f'{label}: align steering for spin, wz={settle_wz:.3f} rad/s '
                f'for {self.args.rotation_settle_time:.2f} s'
            )
            self.hold(0.0, 0.0, settle_wz, self.args.rotation_settle_time)
            self.stop(self.args.rotation_settle_stop_time)

        duration_s = (2.0 * math.pi) / self.args.angular_speed
        wz = direction * self.args.angular_speed
        self.get_logger().info(f'{label}: rotate 360 deg for {duration_s:.2f} s')
        self.hold(0.0, 0.0, wz, duration_s)
        self.stop()

    def run(self):
        if not self.args.armed:
            self.get_logger().warn('dry-run only; pass --armed to publish non-zero velocity commands')

        length = self.args.length
        width = self.args.width

        self.stop(self.args.initial_stop_time)

        self.move_vector(length, 0.0, 'rectangle side 1')
        self.move_vector(0.0, width, 'rectangle side 2')
        self.move_vector(-length, 0.0, 'rectangle side 3')
        self.move_vector(0.0, -width, 'rectangle side 4')

        self.move_vector(length / 2.0, width / 2.0, 'diagonal 1 to center')
        self.rotate_360('center stop on diagonal 1')
        self.move_vector(length / 2.0, width / 2.0, 'diagonal 1 from center')

        self.move_vector(-length, 0.0, 'transfer to second diagonal corner')

        self.move_vector(length / 2.0, -width / 2.0, 'diagonal 2 to center')
        self.rotate_360('center stop on diagonal 2')
        self.move_vector(length / 2.0, -width / 2.0, 'diagonal 2 from center')

        self.stop(self.args.final_stop_time)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Drive an omnidirectional WheelBot on a 2.6 m x 1.7 m rectangle, then both diagonals.'
    )
    parser.add_argument('--cmd-topic', default='/swerve_controller/cmd_vel')
    parser.add_argument('--frame-id', default='base_link')
    parser.add_argument('--length', type=float, default=2.6)
    parser.add_argument('--width', type=float, default=1.7)
    parser.add_argument('--linear-speed', type=float, default=0.20)
    parser.add_argument('--angular-speed', type=float, default=0.50)
    parser.add_argument('--rotation-settle-speed', type=float, default=0.15)
    parser.add_argument('--rotation-settle-time', type=float, default=4.0)
    parser.add_argument('--rotation-settle-stop-time', type=float, default=0.5)
    parser.add_argument('--rotation-direction', type=float, default=1.0)
    parser.add_argument('--stop-time', type=float, default=1.0)
    parser.add_argument('--initial-stop-time', type=float, default=2.0)
    parser.add_argument('--final-stop-time', type=float, default=2.0)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument(
        '--armed',
        action='store_true',
        help='publish non-zero velocity commands; without this flag the script only prints the plan',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RectangleDiagonalsPath(args)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted; sending stop command')
        node.stop(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
