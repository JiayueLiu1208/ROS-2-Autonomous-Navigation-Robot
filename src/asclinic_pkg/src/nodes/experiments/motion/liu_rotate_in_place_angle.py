#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from asclinic_pkg.msg import LeftRightFloat32
from nav_msgs.msg import Odometry


class LiuRotateInPlaceAngle(Node):
    def __init__(self):
        super().__init__('liu_rotate_in_place_angle')

        # Parameters
        self.declare_parameter('target_angle_deg', 360.0)
        self.declare_parameter('duty_cycle_percent', 8.0)
        self.declare_parameter('rotation_direction', 1)   # +1 one way, -1 the other way
        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('command_publish_period', 0.05)
        self.declare_parameter('verbose', True)

        self.target_angle_deg = float(self.get_parameter('target_angle_deg').value)
        self.duty_cycle_percent = float(self.get_parameter('duty_cycle_percent').value)
        self.rotation_direction = int(self.get_parameter('rotation_direction').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.command_publish_period = float(self.get_parameter('command_publish_period').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        if self.rotation_direction not in [1, -1]:
            self.get_logger().warn('rotation_direction must be +1 or -1, defaulting to +1')
            self.rotation_direction = 1

        if self.duty_cycle_percent < 0.0:
            self.duty_cycle_percent = abs(self.duty_cycle_percent)

        self.target_angle_rad = abs(self.target_angle_deg) * math.pi / 180.0

        # State
        self.odom_received = False
        self.finished = False

        self.prev_yaw = None
        self.accumulated_yaw = 0.0

        self.current_yaw = 0.0
        self.command_seq_num = 1

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
        self.get_logger().info('[ROTATE] Node started')
        self.get_logger().info(f'[ROTATE] target_angle_deg      = {self.target_angle_deg:.2f}')
        self.get_logger().info(f'[ROTATE] duty_cycle_percent    = {self.duty_cycle_percent:.2f}')
        self.get_logger().info(f'[ROTATE] rotation_direction    = {self.rotation_direction}')
        self.get_logger().info(f'[ROTATE] odom_topic           = {self.odom_topic}')
        self.get_logger().info(f'[ROTATE] cmd_topic            = {self.cmd_topic}')
        self.get_logger().info('[ROTATE] Waiting for odometry...')
        self.get_logger().info('========================================')

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def angle_diff(self, current, previous):
        return self.wrap_angle(current - previous)

    def odom_callback(self, msg: Odometry):
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if not self.odom_received:
            self.odom_received = True
            self.prev_yaw = self.current_yaw
            self.accumulated_yaw = 0.0
            self.get_logger().info(
                f'[ROTATE] Initial yaw recorded: {self.current_yaw:.4f} rad'
            )
            return

        if self.finished:
            return

        delta_yaw = self.angle_diff(self.current_yaw, self.prev_yaw)
        self.accumulated_yaw += delta_yaw
        self.prev_yaw = self.current_yaw

        travelled_angle = abs(self.accumulated_yaw)

        if self.verbose:
            self.get_logger().info(
                f'[ROTATE] yaw={self.current_yaw:.4f} rad, '
                f'accumulated={self.accumulated_yaw:.4f} rad '
                f'({self.accumulated_yaw * 180.0 / math.pi:.2f} deg)'
            )

        if travelled_angle >= self.target_angle_rad:
            self.finish_motion()

    def command_timer_callback(self):
        if self.finished:
            return

        if not self.odom_received:
            self.publish_motor_command(0.0, 0.0)
            return

        # In-place rotation:
        # left and right wheels must spin in opposite directions
        left_cmd = self.rotation_direction * self.duty_cycle_percent
        right_cmd = -self.rotation_direction * self.duty_cycle_percent

        self.publish_motor_command(left_cmd, right_cmd)

    def publish_motor_command(self, left_cmd, right_cmd):
        msg = LeftRightFloat32()
        msg.left = float(left_cmd)
        msg.right = float(right_cmd)
        msg.seq_num = self.command_seq_num
        self.cmd_pub.publish(msg)
        self.command_seq_num += 1

    def finish_motion(self):
        if self.finished:
            return

        self.finished = True
        self.publish_motor_command(0.0, 0.0)

        final_angle_deg = self.accumulated_yaw * 180.0 / math.pi
        target_signed_deg = self.rotation_direction * abs(self.target_angle_deg)
        error_deg = final_angle_deg - target_signed_deg

        self.get_logger().info('========================================')
        self.get_logger().info('[ROTATE] Target reached')
        self.get_logger().info(f'[ROTATE] target_angle = {target_signed_deg:.2f} deg')
        self.get_logger().info(f'[ROTATE] actual_angle = {final_angle_deg:.2f} deg')
        self.get_logger().info(f'[ROTATE] error       = {error_deg:.2f} deg')
        self.get_logger().info(f'[ROTATE] final_yaw   = {self.current_yaw:.4f} rad')
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
    node = LiuRotateInPlaceAngle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[ROTATE] Keyboard interrupt, stopping.')
    finally:
        try:
            node.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()