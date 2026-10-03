#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from asclinic_pkg.msg import LeftRightFloat32
from nav_msgs.msg import Odometry


class LiuRectangleTrajectory(Node):
    def __init__(self):
        super().__init__('liu_rectangle_trajectory')

        # ==============================
        # Parameters
        # ==============================
        self.declare_parameter('long_side_m', 6.0)
        self.declare_parameter('short_side_m', 3.0)
        self.declare_parameter('turn_angle_deg', 90.0)

        self.declare_parameter('drive_duty_cycle_percent', 6.0)
        self.declare_parameter('turn_duty_cycle_percent', 4.0)

        self.declare_parameter('drive_direction', 1)   # +1 forward, -1 backward
        self.declare_parameter('turn_direction', 1)    # +1 one way, -1 the other way

        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('command_publish_period', 0.05)
        self.declare_parameter('verbose', True)

        self.long_side_m = float(self.get_parameter('long_side_m').value)
        self.short_side_m = float(self.get_parameter('short_side_m').value)
        self.turn_angle_deg = float(self.get_parameter('turn_angle_deg').value)

        self.drive_duty_cycle_percent = float(self.get_parameter('drive_duty_cycle_percent').value)
        self.turn_duty_cycle_percent = float(self.get_parameter('turn_duty_cycle_percent').value)

        self.drive_direction = int(self.get_parameter('drive_direction').value)
        self.turn_direction = int(self.get_parameter('turn_direction').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.command_publish_period = float(self.get_parameter('command_publish_period').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        if self.drive_direction not in [1, -1]:
            self.get_logger().warn('drive_direction must be +1 or -1, defaulting to +1')
            self.drive_direction = 1

        if self.turn_direction not in [1, -1]:
            self.get_logger().warn('turn_direction must be +1 or -1, defaulting to +1')
            self.turn_direction = 1

        if self.drive_duty_cycle_percent < 0.0:
            self.drive_duty_cycle_percent = abs(self.drive_duty_cycle_percent)

        if self.turn_duty_cycle_percent < 0.0:
            self.turn_duty_cycle_percent = abs(self.turn_duty_cycle_percent)

        self.turn_angle_rad = abs(self.turn_angle_deg) * math.pi / 180.0

        # Rectangle side sequence: 6 -> 3 -> 6 -> 3
        self.side_lengths = [
            abs(self.long_side_m),
            abs(self.short_side_m),
            abs(self.long_side_m),
            abs(self.short_side_m),
        ]

        # ==============================
        # State
        # ==============================
        self.odom_received = False
        self.finished = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None

        # phase = 'drive' or 'turn'
        # side_index = 0,1,2,3
        self.phase = 'drive'
        self.side_index = 0

        # Segment references
        self.segment_start_x = None
        self.segment_start_y = None

        self.turn_prev_yaw = None
        self.turn_accumulated_yaw = 0.0

        self.command_seq_num = 1

        # ==============================
        # ROS interfaces
        # ==============================
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
        self.get_logger().info('[RECTANGLE] Node started')
        self.get_logger().info(f'[RECTANGLE] long_side_m              = {self.long_side_m:.3f}')
        self.get_logger().info(f'[RECTANGLE] short_side_m             = {self.short_side_m:.3f}')
        self.get_logger().info(f'[RECTANGLE] turn_angle_deg           = {self.turn_angle_deg:.3f}')
        self.get_logger().info(f'[RECTANGLE] drive_duty_cycle_percent = {self.drive_duty_cycle_percent:.3f}')
        self.get_logger().info(f'[RECTANGLE] turn_duty_cycle_percent  = {self.turn_duty_cycle_percent:.3f}')
        self.get_logger().info(f'[RECTANGLE] drive_direction          = {self.drive_direction}')
        self.get_logger().info(f'[RECTANGLE] turn_direction           = {self.turn_direction}')
        self.get_logger().info(f'[RECTANGLE] odom_topic               = {self.odom_topic}')
        self.get_logger().info(f'[RECTANGLE] cmd_topic                = {self.cmd_topic}')
        self.get_logger().info('[RECTANGLE] Waiting for odometry...')
        self.get_logger().info('========================================')

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def angle_diff(self, current, previous):
        return self.wrap_angle(current - previous)

    def get_current_target_distance(self):
        return self.side_lengths[self.side_index]

    def publish_motor_command(self, left_cmd, right_cmd):
        msg = LeftRightFloat32()
        msg.left = float(left_cmd)
        msg.right = float(right_cmd)
        msg.seq_num = self.command_seq_num
        self.cmd_pub.publish(msg)
        self.command_seq_num += 1

    def start_drive_segment(self):
        self.phase = 'drive'
        self.segment_start_x = self.current_x
        self.segment_start_y = self.current_y
        target_distance = self.get_current_target_distance()

        self.get_logger().info(
            f'[RECTANGLE] Start driving side {self.side_index + 1}/4 '
            f'(target={target_distance:.3f} m) '
            f'from x={self.segment_start_x:.4f}, y={self.segment_start_y:.4f}'
        )

    def start_turn_segment(self):
        self.phase = 'turn'
        self.turn_prev_yaw = self.current_yaw
        self.turn_accumulated_yaw = 0.0

        self.get_logger().info(
            f'[RECTANGLE] Start turning corner {self.side_index + 1}/4 '
            f'from yaw={self.current_yaw:.4f} rad'
        )

    def finish_experiment(self):
        self.finished = True
        self.publish_motor_command(0.0, 0.0)

        dx = self.current_x - self.initial_x
        dy = self.current_y - self.initial_y
        dyaw = self.wrap_angle(self.current_yaw - self.initial_yaw)

        self.get_logger().info('========================================')
        self.get_logger().info('[RECTANGLE] Rectangle trajectory completed')
        self.get_logger().info(f'[RECTANGLE] final_x      = {self.current_x:.4f}')
        self.get_logger().info(f'[RECTANGLE] final_y      = {self.current_y:.4f}')
        self.get_logger().info(f'[RECTANGLE] final_yaw    = {self.current_yaw:.4f} rad')
        self.get_logger().info(f'[RECTANGLE] closure_dx   = {dx:.4f} m')
        self.get_logger().info(f'[RECTANGLE] closure_dy   = {dy:.4f} m')
        self.get_logger().info(f'[RECTANGLE] closure_dyaw = {dyaw:.4f} rad')
        self.get_logger().info('========================================')

        rclpy.shutdown()

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if not self.odom_received:
            self.odom_received = True

            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.initial_yaw = self.current_yaw

            self.start_drive_segment()
            return

        if self.finished:
            return

        if self.phase == 'drive':
            dx = self.current_x - self.segment_start_x
            dy = self.current_y - self.segment_start_y
            distance_travelled = math.sqrt(dx * dx + dy * dy)
            target_distance = self.get_current_target_distance()

            if self.verbose:
                self.get_logger().info(
                    f'[RECTANGLE][DRIVE] side={self.side_index + 1}/4, '
                    f'target={target_distance:.4f} m, '
                    f'distance={distance_travelled:.4f} m, '
                    f'x={self.current_x:.4f}, y={self.current_y:.4f}, yaw={self.current_yaw:.4f}'
                )

            if distance_travelled >= target_distance:
                self.publish_motor_command(0.0, 0.0)
                self.start_turn_segment()

        elif self.phase == 'turn':
            delta_yaw = self.angle_diff(self.current_yaw, self.turn_prev_yaw)
            self.turn_accumulated_yaw += delta_yaw
            self.turn_prev_yaw = self.current_yaw

            if self.verbose:
                self.get_logger().info(
                    f'[RECTANGLE][TURN] corner={self.side_index + 1}/4, '
                    f'accumulated={self.turn_accumulated_yaw:.4f} rad '
                    f'({self.turn_accumulated_yaw * 180.0 / math.pi:.2f} deg), '
                    f'yaw={self.current_yaw:.4f}'
                )

            if abs(self.turn_accumulated_yaw) >= self.turn_angle_rad:
                self.publish_motor_command(0.0, 0.0)
                self.side_index += 1

                if self.side_index >= 4:
                    self.finish_experiment()
                else:
                    self.start_drive_segment()

    def command_timer_callback(self):
        if self.finished:
            return

        if not self.odom_received:
            self.publish_motor_command(0.0, 0.0)
            return

        if self.phase == 'drive':
            cmd = self.drive_direction * self.drive_duty_cycle_percent
            self.publish_motor_command(cmd, cmd)

        elif self.phase == 'turn':
            cmd = self.turn_duty_cycle_percent
            left_cmd = self.turn_direction * cmd
            right_cmd = -self.turn_direction * cmd
            self.publish_motor_command(left_cmd, right_cmd)

    def destroy_node(self):
        try:
            self.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiuRectangleTrajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[RECTANGLE] Keyboard interrupt, stopping.')
    finally:
        try:
            node.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()