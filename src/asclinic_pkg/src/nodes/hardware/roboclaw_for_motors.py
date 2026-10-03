#!/usr/bin/env python3

# Copyright (C) 2026, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#
# See the root of the repository for license details.
#
# ----------------------------------------------------------------------------
#     _    ____   ____ _ _       _          ____            _
#    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________
#   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \
#  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
# /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
#                                                 |___/
#
# DESCRIPTION:
# Node for controlling the wheel motors and reading wheel encoder data
# Driver: RoboClaw connected via USB serial
# Motors: Pololu 70:1 Metal Gearmotor 37Dx70L mm 12V with 64 CPR Encoder
#
# IMPORTANT:
# This version separates:
#   1) motor command direction multipliers
#   2) encoder sign direction multipliers
#
# Topics:
#   Subscribes : <namespace>/set_motor_duty_cycle        (asclinic_pkg/LeftRightFloat32)
#   Publishes  : <namespace>/current_motor_duty_cycle    (asclinic_pkg/LeftRightFloat32)
#   Publishes  : <namespace>/encoder_counts   (asclinic_pkg/LeftRightInt32)
#
# Test motors with:
#   ros2 topic pub --once /<namespace>/set_motor_duty_cycle \
#       asclinic_pkg/msg/LeftRightFloat32 "{left: 20.0, right: 20.0, seq_num: 1}"
# ----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from basicmicro import Basicmicro
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32

# Scale factor: percent [-100, 100] -> RoboClaw units [-32767, 32767]
PERCENT_TO_ROBOCLAW = 327.67


class ROBOCLAW_FOR_MOTORS(Node):

    def __init__(self):
        super().__init__('roboclaw_for_motors')

        # Getting Namespace
        ns_for_group = self.get_namespace()

        # ------------------------------------------------------------------ #
        # DECLARE AND LOAD PARAMETERS
        # ------------------------------------------------------------------ #

        # Verbosity level:
        #   0 : Info not displayed; warnings and errors still shown
        #   1 : Startup info displayed
        #   2 : Info about messages received displayed
        self.declare_parameter('motor_driver_verbosity', 1)
        self.m_verbosity = self.get_parameter('motor_driver_verbosity').value

        # USB serial port the RoboClaw is connected to
        self.declare_parameter('roboclaw_usb_port', '/dev/ttyACM0')
        self.m_usb_port = self.get_parameter('roboclaw_usb_port').value

        # Baud rate for the RoboClaw serial connection
        self.declare_parameter('roboclaw_baud_rate', 38400)
        self.m_baud_rate = self.get_parameter('roboclaw_baud_rate').value

        # RoboClaw packet-serial address (default 0x80 = 128)
        self.declare_parameter('roboclaw_address', 128)
        self.m_address = self.get_parameter('roboclaw_address').value

        # Maximum duty cycle that can be commanded [0.0, 100.0] %
        self.declare_parameter('motor_driver_max_duty_cycle_limit_in_percent', 100.0)
        self.m_max_duty_cycle = self.get_parameter(
            'motor_driver_max_duty_cycle_limit_in_percent').value

        # Clamp to valid range
        if self.m_max_duty_cycle < 0.0:
            self.m_max_duty_cycle = 0.0
            self.get_logger().warn(
                '[ROBOCLAW FOR MOTORS] Max duty cycle clamped to 0 %.')
        if self.m_max_duty_cycle > 100.0:
            self.m_max_duty_cycle = 100.0
            self.get_logger().warn(
                '[ROBOCLAW FOR MOTORS] Max duty cycle clamped to 100 %.')

        # ------------------------------------------------------------------ #
        # MOTOR COMMAND DIRECTION MULTIPLIERS
        # These affect the actual motor commands sent to RoboClaw
        # ------------------------------------------------------------------ #
        self.declare_parameter('motor_driver_left_side_multiplier', 1.0)
        self.m_left_mult = float(
            self.get_parameter('motor_driver_left_side_multiplier').value)

        self.declare_parameter('motor_driver_right_side_multiplier', 1.0)
        self.m_right_mult = float(
            self.get_parameter('motor_driver_right_side_multiplier').value)

        # ------------------------------------------------------------------ #
        # ENCODER SIGN MULTIPLIERS
        # These affect only the published encoder delta signs
        # ------------------------------------------------------------------ #
        self.declare_parameter('encoder_left_side_multiplier', 1.0)
        self.m_encoder_left_mult = float(
            self.get_parameter('encoder_left_side_multiplier').value)

        self.declare_parameter('encoder_right_side_multiplier', 1.0)
        self.m_encoder_right_mult = float(
            self.get_parameter('encoder_right_side_multiplier').value)

        # Timer period [s] for publishing encoder counts
        self.declare_parameter('delta_t_for_publishing_encoder_counts', 0.1)
        self.m_encoder_delta_t = self.get_parameter(
            'delta_t_for_publishing_encoder_counts').value

        # ------------------------------------------------------------------ #
        # PRINT STARTUP INFO
        # ------------------------------------------------------------------ #
        if self.m_verbosity >= 1:
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] USB port            : {self.m_usb_port}')
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Baud rate           : {self.m_baud_rate}')
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Address             : 0x{self.m_address:02X} ({self.m_address})')
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Max duty cycle      : {self.m_max_duty_cycle} %')
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Motor left mult     : {self.m_left_mult}')
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Motor right mult    : {self.m_right_mult}')
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Encoder left mult   : {self.m_encoder_left_mult}')
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Encoder right mult  : {self.m_encoder_right_mult}')

        # ------------------------------------------------------------------ #
        # OPEN THE ROBOCLAW USB CONNECTION
        # ------------------------------------------------------------------ #
        self.m_roboclaw = Basicmicro(self.m_usb_port, self.m_baud_rate)
        open_success = self.m_roboclaw.Open()

        if not open_success:
            self.get_logger().warn(
                f'[ROBOCLAW FOR MOTORS] FAILED to open RoboClaw on {self.m_usb_port}. '
                'Motor commands will be ignored until reconnected.')
            self.m_connected = False
        else:
            self.m_connected = True
            if self.m_verbosity >= 1:
                self.get_logger().info(
                    f'[ROBOCLAW FOR MOTORS] Successfully opened RoboClaw on {self.m_usb_port}')

        # ------------------------------------------------------------------ #
        # SEQUENCE NUMBERS AND ENCODER TRACKING STATE
        # ------------------------------------------------------------------ #
        self.m_seq_num = 1

        # Encoder: store previous absolute counts to compute per-interval delta.
        # Set to None so the first timer callback captures a baseline without publishing.
        self.m_prev_enc_left = None
        self.m_prev_enc_right = None
        self.m_enc_seq_num = 1

        # ------------------------------------------------------------------ #
        # MOTOR SUBSCRIBER AND PUBLISHER
        # ------------------------------------------------------------------ #
        self.m_set_duty_sub = self.create_subscription(
            LeftRightFloat32,
            'set_motor_duty_cycle',
            self.drive_motors_callback,
            1
        )

        self.m_current_duty_pub = self.create_publisher(
            LeftRightFloat32,
            'current_motor_duty_cycle',
            10
        )

        # ------------------------------------------------------------------ #
        # ENCODER COUNTS PUBLISHER AND TIMER
        # ------------------------------------------------------------------ #
        self.m_encoder_counts_pub = self.create_publisher(
            LeftRightInt32,
            'encoder_counts',
            10
        )

        self.m_encoder_timer = self.create_timer(
            self.m_encoder_delta_t,
            self.encoder_publish_callback
        )

        if self.m_verbosity >= 1:
            self.get_logger().info('[ROBOCLAW FOR MOTORS] Node initialisation complete')
            self.get_logger().info(
                '[ROBOCLAW FOR MOTORS] publish motor duty cycle requests from command line with: '
                f'ros2 topic pub --once {ns_for_group}/set_motor_duty_cycle '
                'asclinic_pkg/msg/LeftRightFloat32 "{left: 10.0, right: 10.0, seq_num: 1}"')
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Encoder counts publishing at '
                f'{1.0/self.m_encoder_delta_t:.1f} Hz on: {ns_for_group}/encoder_counts')

    # ---------------------------------------------------------------------- #
    # MOTOR SUBSCRIBER CALLBACK
    # ---------------------------------------------------------------------- #
    def drive_motors_callback(self, msg):
        if self.m_verbosity >= 2:
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Message received with left = {msg.left}, right = {msg.right}')

        # Apply motor direction multipliers
        duty_left = msg.left * self.m_left_mult
        duty_right = msg.right * self.m_right_mult

        # Clamp to [-max, +max]
        duty_left = max(-self.m_max_duty_cycle, min(self.m_max_duty_cycle, duty_left))
        duty_right = max(-self.m_max_duty_cycle, min(self.m_max_duty_cycle, duty_right))

        if not self.m_connected:
            self.get_logger().warn(
                '[ROBOCLAW FOR MOTORS] Not connected - motor command ignored.')
            return

        # Convert percent to RoboClaw duty units and send both motors atomically
        rc_left = int(duty_left * PERCENT_TO_ROBOCLAW)
        rc_right = int(duty_right * PERCENT_TO_ROBOCLAW)
        try:
            result = self.m_roboclaw.DutyM1M2(self.m_address, rc_left, rc_right)
        except Exception as e:
            self.get_logger().warn(
                f'[ROBOCLAW FOR MOTORS] FAILED - DutyM1M2 command raised: {e}. '
                'Check RoboClaw power, packet serial address, baud rate, and USB cable.'
            )
            return

        if not result:
            self.get_logger().warn(
                '[ROBOCLAW FOR MOTORS] FAILED - DutyM1M2 command not acknowledged by RoboClaw.')

        # Publish the duty cycle that was actually commanded
        out_msg = LeftRightFloat32()
        out_msg.left = duty_left
        out_msg.right = duty_right
        out_msg.seq_num = self.m_seq_num
        self.m_current_duty_pub.publish(out_msg)
        self.m_seq_num += 1

    # ---------------------------------------------------------------------- #
    # ENCODER PUBLISHER CALLBACK
    # ---------------------------------------------------------------------- #
    def encoder_publish_callback(self):
        if not self.m_connected:
            return

        # Read encoder values for both motors simultaneously
        try:
            result = self.m_roboclaw.GetEncoders(self.m_address)
        except Exception as e:
            self.get_logger().warn(
                f'[ROBOCLAW FOR MOTORS] FAILED to read encoder counts from RoboClaw: {e}')
            return

        if not result[0]:
            self.get_logger().warn(
                '[ROBOCLAW FOR MOTORS] FAILED to read encoder counts from RoboClaw.')
            return

        # GetEncoders returns unsigned 32-bit values, reinterpret as signed int32
        enc_left = result[1] if result[1] < 0x80000000 else result[1] - 0x100000000
        enc_right = result[2] if result[2] < 0x80000000 else result[2] - 0x100000000

        # First call: capture baseline and skip publishing
        if self.m_prev_enc_left is None:
            self.m_prev_enc_left = enc_left
            self.m_prev_enc_right = enc_right
            return

        # Delta counts since last timer tick
        delta_left = enc_left - self.m_prev_enc_left
        delta_right = enc_right - self.m_prev_enc_right

        # Apply encoder sign multipliers
        delta_left = int(delta_left * self.m_encoder_left_mult)
        delta_right = int(delta_right * self.m_encoder_right_mult)

        _INT32_MAX = 2147483647
        _INT32_MIN = -2147483648
        if not (_INT32_MIN <= delta_left <= _INT32_MAX and
                _INT32_MIN <= delta_right <= _INT32_MAX):
            self.get_logger().warn(
                f'[ROBOCLAW FOR MOTORS] Encoder delta out of int32 range '
                f'(left={delta_left}, right={delta_right}). '
                'Skipping this tick; counts will accumulate into the next publish.')
            return

        # Update baseline
        self.m_prev_enc_left = enc_left
        self.m_prev_enc_right = enc_right

        msg = LeftRightInt32()
        msg.left = delta_left
        msg.right = delta_right
        msg.seq_num = self.m_enc_seq_num
        self.m_encoder_counts_pub.publish(msg)
        self.m_enc_seq_num += 1

        if self.m_verbosity >= 2:
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Encoder counts published '
                f'(seq={msg.seq_num}, left={msg.left}, right={msg.right})')

    # ---------------------------------------------------------------------- #
    # DESTROY NODE
    # ---------------------------------------------------------------------- #
    def destroy_node(self):
        if self.m_connected:
            try:
                self.m_roboclaw.DutyM1M2(self.m_address, 0, 0)
            except Exception:
                pass
            try:
                self.m_roboclaw.close()
            except Exception:
                pass
            self.get_logger().info(
                f'[ROBOCLAW FOR MOTORS] Successfully closed RoboClaw on {self.m_usb_port}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ROBOCLAW_FOR_MOTORS()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
