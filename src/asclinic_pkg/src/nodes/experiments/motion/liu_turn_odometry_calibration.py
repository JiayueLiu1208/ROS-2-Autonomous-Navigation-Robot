#!/usr/bin/env python3

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from asclinic_pkg.msg import LeftRightFloat32


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LiuTurnOdometryCalibration(Node):
    """Run a single in-place turn and log wheel-odometry yaw error."""

    def __init__(self):
        super().__init__('liu_turn_odometry_calibration')

        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('command_publish_period', 0.05)
        self.declare_parameter('save_directory', '~/asclinic-ros2/ros2_ws/results/turn_odom_calibration')

        self.declare_parameter('target_angle_deg', 45.0)
        self.declare_parameter('rotation_direction', 1)
        self.declare_parameter('duty_cycle_percent', 8.0)
        self.declare_parameter('timeout_sec', 5.0)
        self.declare_parameter('pre_settle_duration_sec', 0.60)
        self.declare_parameter('post_settle_duration_sec', 0.60)
        self.declare_parameter('trajectory_sample_period_sec', 0.05)
        self.declare_parameter('save_plot', True)

        self.declare_parameter('use_actual_angle', False)
        self.declare_parameter('actual_angle_deg', 45.0)
        self.declare_parameter('verbose', True)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.command_publish_period = max(
            0.02,
            float(self.get_parameter('command_publish_period').value),
        )
        self.save_directory = os.path.expanduser(
            str(self.get_parameter('save_directory').value)
        )

        self.target_angle_deg = abs(float(self.get_parameter('target_angle_deg').value))
        self.target_angle_rad = math.radians(self.target_angle_deg)
        self.rotation_direction = 1 if int(self.get_parameter('rotation_direction').value) >= 0 else -1
        self.duty_cycle_percent = abs(float(self.get_parameter('duty_cycle_percent').value))
        self.timeout_sec = max(0.1, float(self.get_parameter('timeout_sec').value))
        self.pre_settle_duration_sec = max(
            0.0,
            float(self.get_parameter('pre_settle_duration_sec').value),
        )
        self.post_settle_duration_sec = max(
            0.0,
            float(self.get_parameter('post_settle_duration_sec').value),
        )
        self.trajectory_sample_period_sec = max(
            0.02,
            float(self.get_parameter('trajectory_sample_period_sec').value),
        )
        self.save_plot = self._to_bool(self.get_parameter('save_plot').value)
        self.use_actual_angle = self._to_bool(self.get_parameter('use_actual_angle').value)
        self.actual_angle_deg = float(self.get_parameter('actual_angle_deg').value)
        self.verbose = self._to_bool(self.get_parameter('verbose').value)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.phase = 'waiting_for_odom'
        self.phase_start_time = None
        self.trial_start_time = None
        self.turn_start_time = None
        self.finished = False

        self.start_recorded = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.prev_yaw = None
        self.accumulated_yaw = 0.0

        self.current_left_cmd = 0.0
        self.current_right_cmd = 0.0
        self.command_seq_num = 1
        self.last_sample_time = None
        self.samples = []

        self.trace_csv_file = None
        self.trace_writer = None
        self.trace_csv_path = ''
        self.summary_csv_path = ''
        self.plot_png_path = ''
        self._open_csv_files()

        self.cmd_pub = self.create_publisher(LeftRightFloat32, self.cmd_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.cmd_timer = self.create_timer(
            self.command_publish_period,
            self.command_timer_callback,
        )

        self.get_logger().info('==================================================')
        self.get_logger().info('[TURN ODOM CAL] Node started')
        self.get_logger().info(f'[TURN ODOM CAL] target_angle_deg      = {self.target_angle_deg:.2f}')
        self.get_logger().info(f'[TURN ODOM CAL] rotation_direction    = {self.rotation_direction}')
        self.get_logger().info(f'[TURN ODOM CAL] duty_cycle_percent    = {self.duty_cycle_percent:.2f}')
        self.get_logger().info(f'[TURN ODOM CAL] timeout_sec           = {self.timeout_sec:.2f}')
        self.get_logger().info(f'[TURN ODOM CAL] odom_topic            = {self.odom_topic}')
        self.get_logger().info(f'[TURN ODOM CAL] cmd_topic             = {self.cmd_topic}')
        self.get_logger().info(f'[TURN ODOM CAL] trace_csv             = {self.trace_csv_path}')
        self.get_logger().info('[TURN ODOM CAL] Waiting for odometry...')
        self.get_logger().info('==================================================')

    def _to_bool(self, value):
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _open_csv_files(self):
        os.makedirs(self.save_directory, exist_ok=True)
        run_tag = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.trace_csv_path = os.path.join(
            self.save_directory,
            f'turn_odom_trace_{run_tag}.csv',
        )
        self.summary_csv_path = os.path.join(
            self.save_directory,
            f'turn_odom_summary_{run_tag}.csv',
        )
        self.plot_png_path = os.path.join(
            self.save_directory,
            f'turn_odom_trace_{run_tag}.png',
        )
        self.trace_csv_file = open(self.trace_csv_path, 'w', newline='', encoding='utf-8-sig')
        self.trace_writer = csv.writer(self.trace_csv_file)
        self.trace_writer.writerow([
            'time_sec',
            'phase',
            'left_cmd',
            'right_cmd',
            'x_odom',
            'y_odom',
            'yaw_odom_rad',
            'dx_body_m',
            'dy_body_m',
            'accumulated_yaw_rad',
            'accumulated_yaw_deg',
        ])
        self.trace_csv_file.flush()

    def odom_callback(self, msg):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('[TURN ODOM CAL] First odometry received.')

        if self.phase == 'turning' and self.prev_yaw is not None and not self.finished:
            delta_yaw = wrap_angle(self.current_yaw - self.prev_yaw)
            self.accumulated_yaw += delta_yaw
            self.prev_yaw = self.current_yaw

        self._maybe_record_sample()
        self._write_trace_row()

    def command_timer_callback(self):
        if self.finished:
            return

        if not self.odom_received:
            self.publish_motor_command(0.0, 0.0)
            return

        now = time.monotonic()
        if self.trial_start_time is None:
            self.trial_start_time = now
            self._start_phase('pre_settle', now)

        if self.phase == 'pre_settle':
            self.publish_motor_command(0.0, 0.0)
            if now - self.phase_start_time >= self.pre_settle_duration_sec:
                self._start_turn(now)
            return

        if self.phase == 'turning':
            if self._turn_complete(now):
                self._start_phase('post_settle', now)
                self.publish_motor_command(0.0, 0.0)
                return
            left_cmd = self.rotation_direction * self.duty_cycle_percent
            right_cmd = -self.rotation_direction * self.duty_cycle_percent
            self.publish_motor_command(left_cmd, right_cmd)
            return

        if self.phase == 'post_settle':
            self.publish_motor_command(0.0, 0.0)
            if now - self.phase_start_time >= self.post_settle_duration_sec:
                self.finish_motion()
            return

    def _start_phase(self, phase, now):
        self.phase = phase
        self.phase_start_time = now
        if self.verbose:
            self.get_logger().info(f'[TURN ODOM CAL] Phase: {phase}')

    def _start_turn(self, now):
        self._record_start_pose()
        self.turn_start_time = now
        self._start_phase('turning', now)

    def _record_start_pose(self):
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw
        self.prev_yaw = self.current_yaw
        self.accumulated_yaw = 0.0
        self.start_recorded = True
        self.get_logger().info(
            '[TURN ODOM CAL] Start pose recorded: '
            f'x={self.start_x:.4f}, y={self.start_y:.4f}, yaw={self.start_yaw:.4f} rad'
        )

    def _turn_complete(self, now):
        travelled = abs(self.accumulated_yaw)
        if travelled >= self.target_angle_rad:
            return True
        if self.turn_start_time is not None and now - self.turn_start_time >= self.timeout_sec:
            self.get_logger().warn('[TURN ODOM CAL] Turn timeout reached before target angle.')
            return True
        return False

    def _relative_body_displacement(self):
        if not self.start_recorded:
            return 0.0, 0.0
        dx_world = self.current_x - self.start_x
        dy_world = self.current_y - self.start_y
        c = math.cos(self.start_yaw)
        s = math.sin(self.start_yaw)
        dx_body = c * dx_world + s * dy_world
        dy_body = -s * dx_world + c * dy_world
        return dx_body, dy_body

    def _elapsed_time(self):
        if self.trial_start_time is None:
            return 0.0
        return time.monotonic() - self.trial_start_time

    def _maybe_record_sample(self):
        if not self.start_recorded:
            return
        now = time.monotonic()
        if (
            self.last_sample_time is not None
            and now - self.last_sample_time < self.trajectory_sample_period_sec
        ):
            return
        self.last_sample_time = now
        dx_body, dy_body = self._relative_body_displacement()
        self.samples.append({
            'time_sec': self._elapsed_time(),
            'yaw_deg': math.degrees(self.current_yaw),
            'accum_yaw_deg': math.degrees(self.accumulated_yaw),
            'dx_body': dx_body,
            'dy_body': dy_body,
            'phase': self.phase,
        })

    def _write_trace_row(self):
        if self.trace_writer is None:
            return
        dx_body, dy_body = self._relative_body_displacement()
        self.trace_writer.writerow([
            f'{self._elapsed_time():.6f}',
            self.phase,
            f'{self.current_left_cmd:.4f}',
            f'{self.current_right_cmd:.4f}',
            f'{self.current_x:.6f}',
            f'{self.current_y:.6f}',
            f'{self.current_yaw:.6f}',
            f'{dx_body:.6f}',
            f'{dy_body:.6f}',
            f'{self.accumulated_yaw:.6f}',
            f'{math.degrees(self.accumulated_yaw):.6f}',
        ])
        self.trace_csv_file.flush()

    def publish_motor_command(self, left_cmd, right_cmd):
        self.current_left_cmd = float(left_cmd)
        self.current_right_cmd = float(right_cmd)
        msg = LeftRightFloat32()
        msg.left = self.current_left_cmd
        msg.right = self.current_right_cmd
        msg.seq_num = self.command_seq_num
        self.cmd_pub.publish(msg)
        self.command_seq_num += 1

    def finish_motion(self):
        if self.finished:
            return
        self.finished = True
        self.publish_motor_command(0.0, 0.0)
        self._maybe_record_sample()

        dx_body, dy_body = self._relative_body_displacement()
        odom_angle_deg = math.degrees(self.accumulated_yaw)
        target_signed_deg = self.rotation_direction * self.target_angle_deg
        error_deg = odom_angle_deg - target_signed_deg

        yaw_scale_text = 'not computed'
        if self.use_actual_angle:
            if abs(odom_angle_deg) > 1.0e-6:
                yaw_scale_text = f'{self.actual_angle_deg / odom_angle_deg:.6f}'
            else:
                yaw_scale_text = 'invalid: odom angle is too small'

        self._write_summary_csv(
            dx_body=dx_body,
            dy_body=dy_body,
            odom_angle_deg=odom_angle_deg,
            target_signed_deg=target_signed_deg,
            error_deg=error_deg,
            yaw_scale_text=yaw_scale_text,
        )
        if self.save_plot:
            self._save_plot()

        self.get_logger().info('==================================================')
        self.get_logger().info('[TURN ODOM CAL] Trial complete')
        self.get_logger().info(f'[TURN ODOM CAL] target_angle_deg     = {target_signed_deg:.3f}')
        self.get_logger().info(f'[TURN ODOM CAL] odom_angle_deg       = {odom_angle_deg:.3f}')
        self.get_logger().info(f'[TURN ODOM CAL] odom_error_deg       = {error_deg:.3f}')
        self.get_logger().info(f'[TURN ODOM CAL] odom_dx_body_m       = {dx_body:.4f}')
        self.get_logger().info(f'[TURN ODOM CAL] odom_dy_body_m       = {dy_body:.4f}')
        if abs(odom_angle_deg) > 1.0e-6:
            self.get_logger().info(
                '[TURN ODOM CAL] yaw scale formula   = '
                f'actual_angle_deg / {odom_angle_deg:.3f}'
            )
        if self.use_actual_angle:
            self.get_logger().info(f'[TURN ODOM CAL] actual_angle_deg    = {self.actual_angle_deg:.3f}')
            self.get_logger().info(f'[TURN ODOM CAL] turn_yaw_scale      = {yaw_scale_text}')
        self.get_logger().info(f'[TURN ODOM CAL] trace_csv            = {self.trace_csv_path}')
        self.get_logger().info(f'[TURN ODOM CAL] summary_csv          = {self.summary_csv_path}')
        if self.save_plot:
            self.get_logger().info(f'[TURN ODOM CAL] plot_png             = {self.plot_png_path}')
        self.get_logger().info('==================================================')

        rclpy.shutdown()

    def _write_summary_csv(
        self,
        dx_body,
        dy_body,
        odom_angle_deg,
        target_signed_deg,
        error_deg,
        yaw_scale_text,
    ):
        with open(self.summary_csv_path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['field', 'value'])
            writer.writerow(['target_angle_deg', self.target_angle_deg])
            writer.writerow(['target_signed_deg', target_signed_deg])
            writer.writerow(['rotation_direction', self.rotation_direction])
            writer.writerow(['duty_cycle_percent', self.duty_cycle_percent])
            writer.writerow(['timeout_sec', self.timeout_sec])
            writer.writerow(['pre_settle_duration_sec', self.pre_settle_duration_sec])
            writer.writerow(['post_settle_duration_sec', self.post_settle_duration_sec])
            writer.writerow(['odom_angle_deg', odom_angle_deg])
            writer.writerow(['odom_error_deg', error_deg])
            writer.writerow(['odom_dx_body_m', dx_body])
            writer.writerow(['odom_dy_body_m', dy_body])
            writer.writerow(['use_actual_angle', self.use_actual_angle])
            writer.writerow(['actual_angle_deg', self.actual_angle_deg])
            writer.writerow(['turn_yaw_scale', yaw_scale_text])
            writer.writerow(['trace_csv_path', self.trace_csv_path])
            writer.writerow(['plot_png_path', self.plot_png_path])

    def _save_plot(self):
        if len(self.samples) < 2:
            return
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception as e:
            self.get_logger().warn(
                f'[TURN ODOM CAL] Could not save plot because matplotlib is unavailable: {e}'
            )
            return

        times = [sample['time_sec'] for sample in self.samples]
        accum_yaws = [sample['accum_yaw_deg'] for sample in self.samples]
        xs = [sample['dx_body'] for sample in self.samples]
        ys = [sample['dy_body'] for sample in self.samples]
        target_signed_deg = self.rotation_direction * self.target_angle_deg

        fig, axes = plt.subplots(1, 2, figsize=(11.0, 5.0))
        axes[0].plot(times, accum_yaws, linewidth=1.4, label='odom accumulated yaw')
        axes[0].axhline(target_signed_deg, linestyle='--', linewidth=1.0, label='target')
        axes[0].grid(True, linestyle='--', alpha=0.35)
        axes[0].set_xlabel('time_sec')
        axes[0].set_ylabel('yaw_deg')
        axes[0].legend(loc='best')

        axes[1].plot(xs, ys, '-o', markersize=2.5, linewidth=1.2, label='odom drift')
        axes[1].scatter([xs[0]], [ys[0]], s=55, marker='o', label='start')
        axes[1].scatter([xs[-1]], [ys[-1]], s=70, marker='x', label='end')
        axes[1].set_aspect('equal', adjustable='box')
        axes[1].grid(True, linestyle='--', alpha=0.35)
        axes[1].set_xlabel('dx_body_m')
        axes[1].set_ylabel('dy_body_m')
        axes[1].legend(loc='best')

        fig.suptitle('turn odometry calibration')
        fig.tight_layout()
        fig.savefig(self.plot_png_path, dpi=160)
        plt.close(fig)

    def destroy_node(self):
        try:
            self.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        try:
            if self.trace_csv_file is not None and not self.trace_csv_file.closed:
                self.trace_csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiuTurnOdometryCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[TURN ODOM CAL] Keyboard interrupt, stopping.')
    finally:
        try:
            node.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()
