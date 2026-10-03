#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32


class PittRotateBothWheelsByAngle(Node):
    def __init__(self):
        super().__init__('pitt_rotate_both_wheels_by_angle')

        # ==============================
        # Parameters
        # ==============================
        self.declare_parameter('target_angle_deg', 360.0)
        self.declare_parameter('duty_cycle_percent', 10.0)
        self.declare_parameter('direction', 1)                  # +1 or -1
        self.declare_parameter('counts_per_revolution', 4480)   # from RoboClaw node
        self.declare_parameter('command_publish_period', 0.05)  # seconds
        self.declare_parameter('verbose', True)

        self.target_angle_deg = float(self.get_parameter('target_angle_deg').value)
        self.duty_cycle_percent = float(self.get_parameter('duty_cycle_percent').value)
        self.direction = int(self.get_parameter('direction').value)
        self.counts_per_revolution = int(self.get_parameter('counts_per_revolution').value)
        self.command_publish_period = float(self.get_parameter('command_publish_period').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        if self.direction not in [1, -1]:
            self.get_logger().warn('Parameter "direction" must be +1 or -1. Defaulting to +1.')
            self.direction = 1

        if self.duty_cycle_percent < 0.0:
            self.get_logger().warn('Parameter "duty_cycle_percent" must be non-negative. Taking absolute value.')
            self.duty_cycle_percent = abs(self.duty_cycle_percent)

        # Convert target angle to target encoder counts
        self.target_counts = abs(self.target_angle_deg) / 360.0 * self.counts_per_revolution

        # ==============================
        # Internal state
        # ==============================
        self.left_accum_counts = 0
        self.right_accum_counts = 0

        self.left_done = False
        self.right_done = False
        self.test_started = False
        self.test_finished = False
        self.received_first_encoder_msg = False

        self.command_seq_num = 1

        # ==============================
        # ROS interfaces
        # ==============================
        self.motor_cmd_pub = self.create_publisher(
            LeftRightFloat32,
            'set_motor_duty_cycle',
            10
        )

        self.encoder_sub = self.create_subscription(
            LeftRightInt32,
            'encoder_counts',
            self.encoder_callback,
            10
        )

        self.command_timer = self.create_timer(
            self.command_publish_period,
            self.command_timer_callback
        )

        # ==============================
        # Log startup information
        # ==============================
        self.get_logger().info('============================================')
        self.get_logger().info('[WHEEL TEST] Node started')
        self.get_logger().info(f'[WHEEL TEST] target_angle_deg        = {self.target_angle_deg:.2f}')
        self.get_logger().info(f'[WHEEL TEST] target_counts           = {self.target_counts:.2f}')
        self.get_logger().info(f'[WHEEL TEST] duty_cycle_percent      = {self.duty_cycle_percent:.2f}')
        self.get_logger().info(f'[WHEEL TEST] direction               = {self.direction}')
        self.get_logger().info(f'[WHEEL TEST] counts_per_revolution   = {self.counts_per_revolution}')
        self.get_logger().info(f'[WHEEL TEST] command_publish_period  = {self.command_publish_period:.3f} s')
        self.get_logger().info('[WHEEL TEST] Waiting for encoder data...')
        self.get_logger().info('============================================')

    # --------------------------------------------------
    # Encoder callback
    # --------------------------------------------------
    def encoder_callback(self, msg: LeftRightInt32):
        if self.test_finished:
            return

        # First encoder message received: use it as proof that driver node is alive
        if not self.received_first_encoder_msg:
            self.received_first_encoder_msg = True
            self.test_started = True
            self.get_logger().info('[WHEEL TEST] First encoder message received. Starting wheel motion.')

        # Accumulate delta counts
        self.left_accum_counts += int(msg.left)
        self.right_accum_counts += int(msg.right)

        # Progress in commanded direction
        left_progress = self.direction * self.left_accum_counts
        right_progress = self.direction * self.right_accum_counts

        # Stop each wheel individually once target is reached
        if (not self.left_done) and (left_progress >= self.target_counts):
            self.left_done = True
            self.get_logger().info(
                f'[WHEEL TEST] Left wheel reached target. '
                f'accumulated_counts = {self.left_accum_counts}'
            )

        if (not self.right_done) and (right_progress >= self.target_counts):
            self.right_done = True
            self.get_logger().info(
                f'[WHEEL TEST] Right wheel reached target. '
                f'accumulated_counts = {self.right_accum_counts}'
            )

        # Optional progress print
        if self.verbose and (not self.test_finished):
            self.get_logger().info(
                f'[WHEEL TEST] Progress | '
                f'left_counts = {self.left_accum_counts}, '
                f'right_counts = {self.right_accum_counts}'
            )

        # If both wheels finished, stop and report
        if self.left_done and self.right_done:
            self.finish_test()

    # --------------------------------------------------
    # Timer callback to keep publishing motor commands
    # --------------------------------------------------
    def command_timer_callback(self):
        if self.test_finished:
            return

        # Do not move until encoder stream is confirmed alive
        if not self.received_first_encoder_msg:
            self.publish_motor_command(0.0, 0.0)
            return

        left_cmd = 0.0 if self.left_done else self.direction * self.duty_cycle_percent
        right_cmd = 0.0 if self.right_done else self.direction * self.duty_cycle_percent

        self.publish_motor_command(left_cmd, right_cmd)

    # --------------------------------------------------
    # Publish motor duty cycle command
    # --------------------------------------------------
    def publish_motor_command(self, left_cmd: float, right_cmd: float):
        msg = LeftRightFloat32()
        msg.left = float(left_cmd)
        msg.right = float(right_cmd)
        msg.seq_num = self.command_seq_num
        self.motor_cmd_pub.publish(msg)
        self.command_seq_num += 1

    # --------------------------------------------------
    # Stop test and print final results
    # --------------------------------------------------
    def finish_test(self):
        if self.test_finished:
            return

        self.test_finished = True

        # Stop both motors
        self.publish_motor_command(0.0, 0.0)

        # Convert counts to actual angle
        left_actual_angle_deg = (self.left_accum_counts / self.counts_per_revolution) * 360.0
        right_actual_angle_deg = (self.right_accum_counts / self.counts_per_revolution) * 360.0

        commanded_signed_angle = self.direction * abs(self.target_angle_deg)

        left_error_deg = left_actual_angle_deg - commanded_signed_angle
        right_error_deg = right_actual_angle_deg - commanded_signed_angle

        self.get_logger().info('============================================')
        self.get_logger().info('[WHEEL TEST] Test finished')
        self.get_logger().info(
            f'[WHEEL TEST] Left wheel  | '
            f'cmd = {commanded_signed_angle:.2f} deg, '
            f'act = {left_actual_angle_deg:.2f} deg, '
            f'err = {left_error_deg:.2f} deg'
        )
        self.get_logger().info(
            f'[WHEEL TEST] Right wheel | '
            f'cmd = {commanded_signed_angle:.2f} deg, '
            f'act = {right_actual_angle_deg:.2f} deg, '
            f'err = {right_error_deg:.2f} deg'
        )
        self.get_logger().info('============================================')

        # Shutdown after reporting
        rclpy.shutdown()

    # --------------------------------------------------
    # Safe destroy
    # --------------------------------------------------
    def destroy_node(self):
        try:
            self.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PittRotateBothWheelsByAngle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[WHEEL TEST] KeyboardInterrupt received. Stopping motors.')
    finally:
        try:
            node.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()