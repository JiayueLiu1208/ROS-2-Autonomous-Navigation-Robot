#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from asclinic_pkg.msg import LeftRightFloat32
from nav_msgs.msg import Odometry


class LiuGoStraightDistance(Node):
    def __init__(self):
        super().__init__('liu_go_straight_distance')

        # Basic parameters
        self.declare_parameter('target_distance_m', 0.5)
        self.declare_parameter('duty_cycle_percent', 20.0)
        self.declare_parameter('direction', 1)  # +1 forward, -1 backward
        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('command_publish_period', 0.05)
        self.declare_parameter('verbose', True)

        # Static trim
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 0.96)

        # P heading control
        self.declare_parameter('yaw_kp', 8.0)
        self.declare_parameter('max_correction', 4.0)
        self.declare_parameter('yaw_deadband', 0.02)

        # Startup yaw averaging
        self.declare_parameter('startup_yaw_sample_count', 10)

        self.target_distance_m = float(self.get_parameter('target_distance_m').value)
        self.duty_cycle_percent = float(self.get_parameter('duty_cycle_percent').value)
        self.direction = int(self.get_parameter('direction').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.command_publish_period = float(self.get_parameter('command_publish_period').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.left_trim = float(self.get_parameter('left_trim').value)
        self.right_trim = float(self.get_parameter('right_trim').value)

        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_correction = float(self.get_parameter('max_correction').value)
        self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)
        self.startup_yaw_sample_count = int(self.get_parameter('startup_yaw_sample_count').value)

        if self.direction not in [1, -1]:
            self.get_logger().warn('direction must be +1 or -1, defaulting to +1')
            self.direction = 1

        if self.duty_cycle_percent < 0.0:
            self.duty_cycle_percent = abs(self.duty_cycle_percent)

        if self.startup_yaw_sample_count < 1:
            self.startup_yaw_sample_count = 1

        # State
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.odom_received = False
        self.finished = False
        self.command_seq_num = 1

        self.yaw_samples = []
        self.ready_to_move = False

        # ROS interfaces
        self.cmd_pub = self.create_publisher(
            LeftRightFloat32,
            self.cmd_topic,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.cmd_timer = self.create_timer(
            self.command_publish_period,
            self.command_timer_callback
        )

        self.get_logger().info('========================================')
        self.get_logger().info('[GO STRAIGHT] Node started')
        self.get_logger().info(f'[GO STRAIGHT] target_distance_m     = {self.target_distance_m:.3f}')
        self.get_logger().info(f'[GO STRAIGHT] duty_cycle_percent   = {self.duty_cycle_percent:.3f}')
        self.get_logger().info(f'[GO STRAIGHT] direction            = {self.direction}')
        self.get_logger().info(f'[GO STRAIGHT] left_trim            = {self.left_trim:.3f}')
        self.get_logger().info(f'[GO STRAIGHT] right_trim           = {self.right_trim:.3f}')
        self.get_logger().info(f'[GO STRAIGHT] yaw_kp               = {self.yaw_kp:.3f}')
        self.get_logger().info(f'[GO STRAIGHT] max_correction       = {self.max_correction:.3f}')
        self.get_logger().info(f'[GO STRAIGHT] yaw_deadband         = {self.yaw_deadband:.4f}')
        self.get_logger().info(f'[GO STRAIGHT] startup_yaw_samples  = {self.startup_yaw_sample_count}')
        self.get_logger().info(f'[GO STRAIGHT] odom_topic           = {self.odom_topic}')
        self.get_logger().info(f'[GO STRAIGHT] cmd_topic            = {self.cmd_topic}')
        self.get_logger().info('[GO STRAIGHT] Waiting for odometry...')
        self.get_logger().info('========================================')

    def quaternion_to_yaw(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def clamp(self, value, vmin, vmax):
        return max(vmin, min(vmax, value))

    def average_angles(self, angles):
        sin_sum = 0.0
        cos_sum = 0.0
        for a in angles:
            sin_sum += math.sin(a)
            cos_sum += math.cos(a)
        return math.atan2(sin_sum, cos_sum)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('[GO STRAIGHT] First odometry received.')

        if self.finished:
            return

        if not self.ready_to_move:
            self.yaw_samples.append(self.current_yaw)

            if self.verbose:
                self.get_logger().info(
                    f'[GO STRAIGHT] Collecting startup yaw: '
                    f'{len(self.yaw_samples)}/{self.startup_yaw_sample_count}, '
                    f'current_yaw={self.current_yaw:.4f}'
                )

            if len(self.yaw_samples) >= self.startup_yaw_sample_count:
                self.start_yaw = self.average_angles(self.yaw_samples)
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.ready_to_move = True

                self.get_logger().info(
                    f'[GO STRAIGHT] Start pose recorded: '
                    f'x={self.start_x:.4f}, y={self.start_y:.4f}, '
                    f'avg_yaw={self.start_yaw:.4f}'
                )
            return

        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        distance_travelled = math.sqrt(dx * dx + dy * dy)
        yaw_error = self.wrap_angle(self.current_yaw - self.start_yaw)

        if self.verbose:
            self.get_logger().info(
                f'[GO STRAIGHT] x={self.current_x:.4f}, y={self.current_y:.4f}, '
                f'yaw={self.current_yaw:.4f}, yaw_error={yaw_error:.4f}, '
                f'travelled={distance_travelled:.4f} m'
            )

        if distance_travelled >= self.target_distance_m:
            self.finish_motion(distance_travelled)

    def command_timer_callback(self):
        if self.finished:
            return

        if not self.odom_received:
            self.publish_motor_command(0.0, 0.0)
            return

        if not self.ready_to_move:
            self.publish_motor_command(0.0, 0.0)
            return

        base_cmd = self.direction * self.duty_cycle_percent

        left_base = base_cmd * self.left_trim
        right_base = base_cmd * self.right_trim

        yaw_error = self.wrap_angle(self.current_yaw - self.start_yaw)

        if abs(yaw_error) < self.yaw_deadband:
            yaw_error = 0.0

        correction = self.yaw_kp * yaw_error
        correction = self.clamp(correction, -self.max_correction, self.max_correction)

        # Forward: if yaw_error > 0, reduce left and increase right
        # Backward: reverse correction sign
        if self.direction == 1:
            left_cmd = left_base - correction
            right_cmd = right_base + correction
        else:
            left_cmd = left_base + correction
            right_cmd = right_base - correction

        if self.verbose:
            self.get_logger().info(
                f'[GO STRAIGHT CMD] left_base={left_base:.3f}, right_base={right_base:.3f}, '
                f'yaw_error={yaw_error:.4f}, correction={correction:.3f}, '
                f'left_cmd={left_cmd:.3f}, right_cmd={right_cmd:.3f}'
            )

        self.publish_motor_command(left_cmd, right_cmd)

    def publish_motor_command(self, left_cmd, right_cmd):
        msg = LeftRightFloat32()
        msg.left = float(left_cmd)
        msg.right = float(right_cmd)
        msg.seq_num = self.command_seq_num
        self.cmd_pub.publish(msg)
        self.command_seq_num += 1

    def finish_motion(self, distance_travelled):
        if self.finished:
            return

        self.finished = True
        self.publish_motor_command(0.0, 0.0)

        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        yaw_error = self.wrap_angle(self.current_yaw - self.start_yaw)

        self.get_logger().info('========================================')
        self.get_logger().info('[GO STRAIGHT] Target reached')
        self.get_logger().info(f'[GO STRAIGHT] distance_travelled = {distance_travelled:.4f} m')
        self.get_logger().info(f'[GO STRAIGHT] final_x = {self.current_x:.4f}')
        self.get_logger().info(f'[GO STRAIGHT] final_y = {self.current_y:.4f}')
        self.get_logger().info(f'[GO STRAIGHT] final_yaw = {self.current_yaw:.4f} rad')
        self.get_logger().info(f'[GO STRAIGHT] yaw_error = {yaw_error:.4f} rad')
        self.get_logger().info(f'[GO STRAIGHT] dx = {dx:.4f}, dy = {dy:.4f}')
        self.get_logger().info('========================================')

        rclpy.shutdown()

    def destroy_node(self):
        try:
            self.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiuGoStraightDistance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[GO STRAIGHT] Keyboard interrupt, stopping.')
    finally:
        try:
            node.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()