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


class LiuAvoidanceOdometryCalibration(Node):
    """Run repeatable stop-go avoidance motion and log wheel odometry.

    Default pattern:
      stop briefly, then trace a 0.5 m radius semicircle:
        short in-place turn, stop, short straight, stop

    This targets the odometry error mode seen during obstacle avoidance, where
    many small turn/go actions are worse than one long motion.
    """

    def __init__(self):
        super().__init__('liu_avoidance_odometry_calibration')

        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('command_publish_period', 0.05)
        self.declare_parameter('save_directory', '~/asclinic-ros2/ros2_ws/results/avoidance_odom_calibration')

        self.declare_parameter('pattern', 'semicircle')  # short_turns, bypass, or semicircle
        self.declare_parameter('repeats', 12)
        self.declare_parameter('turn_direction', 1)  # same convention as liu_rotate_in_place_angle.py
        self.declare_parameter('forward_direction', 1)
        self.declare_parameter('turn_duty_percent', 8.0)
        self.declare_parameter('straight_duty_percent', 12.0)
        self.declare_parameter('turn_duration_sec', 0.35)
        self.declare_parameter('turn_timeout_sec', 3.0)
        self.declare_parameter('straight_duration_sec', 0.25)
        self.declare_parameter('use_straight_distance', True)
        self.declare_parameter('straight_distance_m', 0.12)
        self.declare_parameter('straight_timeout_sec', 3.0)
        self.declare_parameter('semicircle_radius_m', 0.50)
        self.declare_parameter('semicircle_total_yaw_deg', 180.0)
        self.declare_parameter('semicircle_use_chord_distance', True)
        self.declare_parameter('settle_duration_sec', 0.20)
        self.declare_parameter('pre_settle_duration_sec', 0.60)
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 0.92)
        self.declare_parameter('trajectory_sample_period_sec', 0.20)
        self.declare_parameter('trajectory_print_max_points', 24)
        self.declare_parameter('save_plot', True)

        # Optional measured result. After a run, measure final heading relative
        # to the start heading and rerun with use_actual_delta_yaw:=true to get
        # an avoidance yaw scale recommendation.
        self.declare_parameter('use_actual_delta_yaw', False)
        self.declare_parameter('actual_delta_yaw_deg', 0.0)
        self.declare_parameter('use_actual_displacement', False)
        self.declare_parameter('actual_dx_body_m', 0.0)
        self.declare_parameter('actual_dy_body_m', 0.0)
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

        self.pattern = str(self.get_parameter('pattern').value).strip().lower()
        self.repeats = max(1, int(self.get_parameter('repeats').value))
        self.turn_direction = 1 if int(self.get_parameter('turn_direction').value) >= 0 else -1
        self.forward_direction = 1 if int(self.get_parameter('forward_direction').value) >= 0 else -1
        self.turn_duty_percent = abs(float(self.get_parameter('turn_duty_percent').value))
        self.straight_duty_percent = abs(float(self.get_parameter('straight_duty_percent').value))
        self.turn_duration_sec = max(0.0, float(self.get_parameter('turn_duration_sec').value))
        self.turn_timeout_sec = max(0.1, float(self.get_parameter('turn_timeout_sec').value))
        self.straight_duration_sec = max(0.0, float(self.get_parameter('straight_duration_sec').value))
        self.use_straight_distance = self._to_bool(
            self.get_parameter('use_straight_distance').value
        )
        self.straight_distance_m = max(
            0.0,
            float(self.get_parameter('straight_distance_m').value),
        )
        self.straight_timeout_sec = max(
            0.1,
            float(self.get_parameter('straight_timeout_sec').value),
        )
        self.semicircle_radius_m = max(
            0.01,
            float(self.get_parameter('semicircle_radius_m').value),
        )
        self.semicircle_total_yaw_deg = abs(
            float(self.get_parameter('semicircle_total_yaw_deg').value)
        )
        self.semicircle_use_chord_distance = self._to_bool(
            self.get_parameter('semicircle_use_chord_distance').value
        )
        self.settle_duration_sec = max(0.0, float(self.get_parameter('settle_duration_sec').value))
        self.pre_settle_duration_sec = max(
            0.0,
            float(self.get_parameter('pre_settle_duration_sec').value),
        )
        self.left_trim = float(self.get_parameter('left_trim').value)
        self.right_trim = float(self.get_parameter('right_trim').value)
        self.trajectory_sample_period_sec = max(
            0.02,
            float(self.get_parameter('trajectory_sample_period_sec').value),
        )
        self.trajectory_print_max_points = max(
            2,
            int(self.get_parameter('trajectory_print_max_points').value),
        )
        self.save_plot = self._to_bool(self.get_parameter('save_plot').value)

        self.use_actual_delta_yaw = self._to_bool(
            self.get_parameter('use_actual_delta_yaw').value
        )
        self.actual_delta_yaw_deg = float(
            self.get_parameter('actual_delta_yaw_deg').value
        )
        self.use_actual_displacement = self._to_bool(
            self.get_parameter('use_actual_displacement').value
        )
        self.actual_dx_body_m = float(self.get_parameter('actual_dx_body_m').value)
        self.actual_dy_body_m = float(self.get_parameter('actual_dy_body_m').value)
        self.verbose = self._to_bool(self.get_parameter('verbose').value)

        self.phases = self._build_phases()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.finished = False

        self.start_recorded = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.prev_yaw_for_accum = None
        self.accumulated_yaw = 0.0

        self.phase_index = 0
        self.phase_start_time = None
        self.phase_start_x = 0.0
        self.phase_start_y = 0.0
        self.phase_start_yaw = 0.0
        self.phase_start_accumulated_yaw = 0.0
        self.trial_start_time = None
        self.last_trajectory_sample_time = None
        self.trajectory_samples = []
        self.current_left_cmd = 0.0
        self.current_right_cmd = 0.0
        self.command_seq_num = 1

        self.trace_csv_file = None
        self.trace_writer = None
        self.summary_csv_path = ''
        self.trace_csv_path = ''
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
        self.get_logger().info('[AVOID ODOM CAL] Node started')
        self.get_logger().info(f'[AVOID ODOM CAL] pattern                 = {self.pattern}')
        self.get_logger().info(f'[AVOID ODOM CAL] repeats                 = {self.repeats}')
        self.get_logger().info(f'[AVOID ODOM CAL] turn_direction          = {self.turn_direction}')
        self.get_logger().info(f'[AVOID ODOM CAL] turn_duty_percent       = {self.turn_duty_percent:.2f}')
        self.get_logger().info(f'[AVOID ODOM CAL] straight_duty_percent   = {self.straight_duty_percent:.2f}')
        self.get_logger().info(f'[AVOID ODOM CAL] turn_duration_sec       = {self.turn_duration_sec:.2f}')
        self.get_logger().info(f'[AVOID ODOM CAL] turn_timeout_sec        = {self.turn_timeout_sec:.2f}')
        self.get_logger().info(f'[AVOID ODOM CAL] use_straight_distance   = {self.use_straight_distance}')
        self.get_logger().info(f'[AVOID ODOM CAL] straight_distance_m     = {self.straight_distance_m:.3f}')
        self.get_logger().info(f'[AVOID ODOM CAL] straight_duration_sec   = {self.straight_duration_sec:.2f}')
        self.get_logger().info(f'[AVOID ODOM CAL] straight_timeout_sec    = {self.straight_timeout_sec:.2f}')
        if self.pattern == 'semicircle':
            self.get_logger().info(f'[AVOID ODOM CAL] semicircle_radius_m    = {self.semicircle_radius_m:.3f}')
            self.get_logger().info(f'[AVOID ODOM CAL] semicircle_total_yaw   = {self.semicircle_total_yaw_deg:.2f} deg')
        self.get_logger().info(f'[AVOID ODOM CAL] settle_duration_sec     = {self.settle_duration_sec:.2f}')
        self.get_logger().info(f'[AVOID ODOM CAL] odom_topic              = {self.odom_topic}')
        self.get_logger().info(f'[AVOID ODOM CAL] cmd_topic               = {self.cmd_topic}')
        self.get_logger().info(f'[AVOID ODOM CAL] trace_csv               = {self.trace_csv_path}')
        self.get_logger().info(f'[AVOID ODOM CAL] save_plot               = {self.save_plot}')
        self.get_logger().info('[AVOID ODOM CAL] Waiting for odometry...')
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
            f'avoidance_odom_trace_{run_tag}.csv',
        )
        self.summary_csv_path = os.path.join(
            self.save_directory,
            f'avoidance_odom_summary_{run_tag}.csv',
        )
        self.plot_png_path = os.path.join(
            self.save_directory,
            f'avoidance_odom_trace_{run_tag}.png',
        )
        self.trace_csv_file = open(self.trace_csv_path, 'w', newline='', encoding='utf-8-sig')
        self.trace_writer = csv.writer(self.trace_csv_file)
        self.trace_writer.writerow([
            'time_sec',
            'phase_index',
            'phase_label',
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

    def _append_phase(
        self,
        label,
        duration,
        left_cmd,
        right_cmd,
        target_distance_m=0.0,
        target_yaw_rad=0.0,
    ):
        if duration <= 0.0:
            return
        self.phases.append({
            'label': label,
            'duration': float(duration),
            'left': float(left_cmd),
            'right': float(right_cmd),
            'target_distance_m': max(0.0, float(target_distance_m)),
            'target_yaw_rad': float(target_yaw_rad),
        })

    def _build_phases(self):
        self.phases = []
        self._append_phase('pre_settle', self.pre_settle_duration_sec, 0.0, 0.0)

        turn_left = self.turn_direction * self.turn_duty_percent
        turn_right = -self.turn_direction * self.turn_duty_percent
        forward = self.forward_direction * self.straight_duty_percent
        straight_left = forward * self.left_trim
        straight_right = forward * self.right_trim
        straight_phase_duration = (
            self.straight_timeout_sec
            if self.use_straight_distance and self.straight_distance_m > 0.0
            else self.straight_duration_sec
        )
        straight_phase_distance = (
            self.straight_distance_m
            if self.use_straight_distance
            else 0.0
        )

        if self.pattern == 'semicircle':
            segment_yaw_rad = (
                self.turn_direction
                * math.radians(self.semicircle_total_yaw_deg)
                / float(self.repeats)
            )
            half_segment_yaw_rad = 0.5 * segment_yaw_rad
            segment_distance_m = self._semicircle_segment_distance(abs(segment_yaw_rad))

            self._append_phase(
                'semicircle_initial_half_turn',
                self.turn_timeout_sec,
                turn_left,
                turn_right,
                target_yaw_rad=half_segment_yaw_rad,
            )
            self._append_phase('semicircle_settle_after_initial_turn', self.settle_duration_sec, 0.0, 0.0)

            for repeat_index in range(self.repeats):
                prefix = f'semicircle_{repeat_index + 1:02d}'
                self._append_phase(
                    f'{prefix}_straight',
                    self.straight_timeout_sec,
                    straight_left,
                    straight_right,
                    target_distance_m=segment_distance_m,
                )
                self._append_phase(f'{prefix}_settle_after_straight', self.settle_duration_sec, 0.0, 0.0)
                if repeat_index < self.repeats - 1:
                    self._append_phase(
                        f'{prefix}_turn_to_next_chord',
                        self.turn_timeout_sec,
                        turn_left,
                        turn_right,
                        target_yaw_rad=segment_yaw_rad,
                    )
                    self._append_phase(f'{prefix}_settle_after_turn', self.settle_duration_sec, 0.0, 0.0)

            self._append_phase(
                'semicircle_final_half_turn',
                self.turn_timeout_sec,
                turn_left,
                turn_right,
                target_yaw_rad=half_segment_yaw_rad,
            )
            self._append_phase('semicircle_settle_after_final_turn', self.settle_duration_sec, 0.0, 0.0)
        elif self.pattern == 'bypass':
            self._append_phase('turn_away', self.turn_duration_sec, turn_left, turn_right)
            self._append_phase('settle_after_turn_away', self.settle_duration_sec, 0.0, 0.0)
            self._append_phase(
                'straight_offset',
                straight_phase_duration,
                straight_left,
                straight_right,
                target_distance_m=straight_phase_distance,
            )
            self._append_phase('settle_after_straight_offset', self.settle_duration_sec, 0.0, 0.0)
            self._append_phase('turn_back', self.turn_duration_sec, -turn_left, -turn_right)
            self._append_phase('settle_after_turn_back', self.settle_duration_sec, 0.0, 0.0)
            self._append_phase(
                'straight_parallel',
                straight_phase_duration,
                straight_left,
                straight_right,
                target_distance_m=straight_phase_distance,
            )
        else:
            if self.pattern != 'short_turns':
                self.get_logger().warn(
                    f'[AVOID ODOM CAL] Unknown pattern "{self.pattern}", using short_turns.'
                )
                self.pattern = 'short_turns'
            for repeat_index in range(self.repeats):
                prefix = f'repeat_{repeat_index + 1:02d}'
                self._append_phase(f'{prefix}_short_turn', self.turn_duration_sec, turn_left, turn_right)
                self._append_phase(f'{prefix}_settle_after_turn', self.settle_duration_sec, 0.0, 0.0)
                self._append_phase(
                    f'{prefix}_short_straight',
                    straight_phase_duration,
                    straight_left,
                    straight_right,
                    target_distance_m=straight_phase_distance,
                )
                self._append_phase(f'{prefix}_settle_after_straight', self.settle_duration_sec, 0.0, 0.0)

        self._append_phase('final_stop', 0.50, 0.0, 0.0)
        return self.phases

    def _semicircle_segment_distance(self, segment_yaw_rad):
        if self.semicircle_use_chord_distance:
            return 2.0 * self.semicircle_radius_m * math.sin(segment_yaw_rad / 2.0)
        return self.semicircle_radius_m * segment_yaw_rad

    def _expected_semicircle_endpoint(self):
        if self.pattern != 'semicircle':
            return None
        segment_yaw_rad = (
            self.turn_direction
            * math.radians(self.semicircle_total_yaw_deg)
            / float(self.repeats)
        )
        half_segment_yaw_rad = 0.5 * segment_yaw_rad
        segment_distance_m = self._semicircle_segment_distance(abs(segment_yaw_rad))
        x_body = 0.0
        y_body = 0.0
        yaw = half_segment_yaw_rad
        for repeat_index in range(self.repeats):
            x_body += segment_distance_m * math.cos(yaw)
            y_body += segment_distance_m * math.sin(yaw)
            if repeat_index < self.repeats - 1:
                yaw += segment_yaw_rad
        yaw += half_segment_yaw_rad
        return x_body, y_body, math.degrees(yaw), segment_distance_m

    def _current_phase_label(self):
        if self.phase_index >= len(self.phases):
            return 'finished'
        return self.phases[self.phase_index]['label']

    def _record_start_pose(self):
        if self.start_recorded:
            return
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw
        self.prev_yaw_for_accum = self.current_yaw
        self.accumulated_yaw = 0.0
        self.start_recorded = True
        self.get_logger().info(
            '[AVOID ODOM CAL] Start pose recorded: '
            f'x={self.start_x:.4f}, y={self.start_y:.4f}, yaw={self.start_yaw:.4f} rad'
        )

    def odom_callback(self, msg):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('[AVOID ODOM CAL] First odometry received.')

        if self.start_recorded and self.prev_yaw_for_accum is not None and not self.finished:
            delta_yaw = wrap_angle(self.current_yaw - self.prev_yaw_for_accum)
            self.accumulated_yaw += delta_yaw
            self.prev_yaw_for_accum = self.current_yaw

        self._maybe_record_trajectory_sample()
        self._write_trace_row()

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

    def _write_trace_row(self):
        if self.trace_writer is None:
            return
        if self.trial_start_time is None:
            elapsed = 0.0
        else:
            elapsed = time.monotonic() - self.trial_start_time
        dx_body, dy_body = self._relative_body_displacement()
        self.trace_writer.writerow([
            f'{elapsed:.6f}',
            self.phase_index,
            self._current_phase_label(),
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

    def _maybe_record_trajectory_sample(self, force=False):
        if not self.start_recorded:
            return
        now = time.monotonic()
        if (
            not force
            and self.last_trajectory_sample_time is not None
            and now - self.last_trajectory_sample_time < self.trajectory_sample_period_sec
        ):
            return
        self.last_trajectory_sample_time = now
        if self.trial_start_time is None:
            elapsed = 0.0
        else:
            elapsed = now - self.trial_start_time
        dx_body, dy_body = self._relative_body_displacement()
        self.trajectory_samples.append({
            'time_sec': elapsed,
            'x': self.current_x,
            'y': self.current_y,
            'yaw': self.current_yaw,
            'dx_body': dx_body,
            'dy_body': dy_body,
            'accum_yaw_deg': math.degrees(self.accumulated_yaw),
            'phase': self._current_phase_label(),
        })

    def command_timer_callback(self):
        if self.finished:
            return

        if not self.odom_received:
            self.publish_motor_command(0.0, 0.0)
            return

        now = time.monotonic()
        if self.trial_start_time is None:
            self.trial_start_time = now

        if self.phase_index >= len(self.phases):
            self.finish_motion()
            return

        if self.phase_start_time is None:
            self._start_phase(now)

        phase = self.phases[self.phase_index]
        if self._phase_complete(phase, now):
            self.phase_index += 1
            if self.phase_index >= len(self.phases):
                self.finish_motion()
                return
            self._start_phase(now)
            phase = self.phases[self.phase_index]

        self.publish_motor_command(phase['left'], phase['right'])

    def _start_phase(self, now):
        self.phase_start_time = now
        phase = self.phases[self.phase_index]
        if not self.start_recorded and phase['label'] != 'pre_settle':
            self._record_start_pose()
        self.phase_start_x = self.current_x
        self.phase_start_y = self.current_y
        self.phase_start_yaw = self.current_yaw
        self.phase_start_accumulated_yaw = self.accumulated_yaw
        if self.verbose:
            target_text = ''
            if phase.get('target_distance_m', 0.0) > 0.0:
                target_text = ', target_dist=%.3fm' % phase['target_distance_m']
            if abs(phase.get('target_yaw_rad', 0.0)) > 0.0:
                target_text += ', target_yaw=%.2fdeg' % math.degrees(phase['target_yaw_rad'])
            self.get_logger().info(
                '[AVOID ODOM CAL] Phase %d/%d: %s for %.2fs%s, cmd=(%.2f, %.2f)'
                % (
                    self.phase_index + 1,
                    len(self.phases),
                    phase['label'],
                    phase['duration'],
                    target_text,
                    phase['left'],
                    phase['right'],
                )
            )

    def _phase_complete(self, phase, now):
        elapsed = now - self.phase_start_time
        target_distance_m = float(phase.get('target_distance_m', 0.0))
        if target_distance_m > 0.0:
            if self._phase_distance_travelled() >= target_distance_m:
                return True
        target_yaw_rad = float(phase.get('target_yaw_rad', 0.0))
        if abs(target_yaw_rad) > 0.0:
            if self._phase_yaw_travelled(target_yaw_rad) >= abs(target_yaw_rad):
                return True
        return elapsed >= float(phase['duration'])

    def _phase_distance_travelled(self):
        dx_world = self.current_x - self.phase_start_x
        dy_world = self.current_y - self.phase_start_y
        c = math.cos(self.phase_start_yaw)
        s = math.sin(self.phase_start_yaw)
        dx_body = c * dx_world + s * dy_world
        dy_body = -s * dx_world + c * dy_world
        if self.forward_direction >= 0:
            return max(0.0, dx_body)
        return max(0.0, -dx_body)

    def _phase_yaw_travelled(self, target_yaw_rad):
        yaw_delta = self.accumulated_yaw - self.phase_start_accumulated_yaw
        if target_yaw_rad >= 0.0:
            return max(0.0, yaw_delta)
        return max(0.0, -yaw_delta)

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
        self._maybe_record_trajectory_sample(force=True)

        dx_body, dy_body = self._relative_body_displacement()
        odom_delta_yaw_deg = math.degrees(self.accumulated_yaw)
        odom_displacement_m = math.hypot(dx_body, dy_body)

        yaw_scale_text = 'not computed'
        if self.use_actual_delta_yaw:
            if abs(odom_delta_yaw_deg) > 1.0e-6:
                yaw_scale = self.actual_delta_yaw_deg / odom_delta_yaw_deg
                yaw_scale_text = f'{yaw_scale:.6f}'
            else:
                yaw_scale_text = 'invalid: odom yaw is too small'

        displacement_scale_text = 'not computed'
        if self.use_actual_displacement:
            actual_disp = math.hypot(self.actual_dx_body_m, self.actual_dy_body_m)
            if odom_displacement_m > 1.0e-6:
                displacement_scale = actual_disp / odom_displacement_m
                displacement_scale_text = f'{displacement_scale:.6f}'
            else:
                displacement_scale_text = 'invalid: odom displacement is too small'

        self._write_summary_csv(
            dx_body=dx_body,
            dy_body=dy_body,
            odom_delta_yaw_deg=odom_delta_yaw_deg,
            odom_displacement_m=odom_displacement_m,
            yaw_scale_text=yaw_scale_text,
            displacement_scale_text=displacement_scale_text,
        )
        if self.save_plot:
            self._save_trajectory_plot()

        self.get_logger().info('==================================================')
        self.get_logger().info('[AVOID ODOM CAL] Trial complete')
        self.get_logger().info(f'[AVOID ODOM CAL] odom_dx_body_m          = {dx_body:.4f}')
        self.get_logger().info(f'[AVOID ODOM CAL] odom_dy_body_m          = {dy_body:.4f}')
        self.get_logger().info(f'[AVOID ODOM CAL] odom_displacement_m     = {odom_displacement_m:.4f}')
        self.get_logger().info(f'[AVOID ODOM CAL] odom_delta_yaw_deg      = {odom_delta_yaw_deg:.3f}')
        expected = self._expected_semicircle_endpoint()
        if expected is not None:
            expected_x, expected_y, expected_yaw_deg, segment_distance_m = expected
            self.get_logger().info(f'[AVOID ODOM CAL] semicircle_segment_m  = {segment_distance_m:.4f}')
            self.get_logger().info(
                '[AVOID ODOM CAL] semicircle_expected  = '
                f'dx={expected_x:.4f}, dy={expected_y:.4f}, yaw={expected_yaw_deg:.3f} deg'
            )
            self.get_logger().info(
                '[AVOID ODOM CAL] semicircle_odom_err  = '
                f'dx={dx_body - expected_x:.4f}, dy={dy_body - expected_y:.4f}, '
                f'yaw={odom_delta_yaw_deg - expected_yaw_deg:.3f} deg'
            )
        if abs(odom_delta_yaw_deg) > 1.0e-6:
            self.get_logger().info(
                '[AVOID ODOM CAL] yaw scale formula      = '
                f'actual_delta_yaw_deg / {odom_delta_yaw_deg:.3f}'
            )
        if self.use_actual_delta_yaw:
            self.get_logger().info(f'[AVOID ODOM CAL] actual_delta_yaw_deg    = {self.actual_delta_yaw_deg:.3f}')
            self.get_logger().info(f'[AVOID ODOM CAL] avoidance_yaw_scale    = {yaw_scale_text}')
        if self.use_actual_displacement:
            self.get_logger().info(f'[AVOID ODOM CAL] actual_dx_body_m        = {self.actual_dx_body_m:.4f}')
            self.get_logger().info(f'[AVOID ODOM CAL] actual_dy_body_m        = {self.actual_dy_body_m:.4f}')
            self.get_logger().info(f'[AVOID ODOM CAL] displacement_scale     = {displacement_scale_text}')
        self._print_trajectory_summary()
        self.get_logger().info(f'[AVOID ODOM CAL] trace_csv               = {self.trace_csv_path}')
        self.get_logger().info(f'[AVOID ODOM CAL] summary_csv             = {self.summary_csv_path}')
        if self.save_plot:
            self.get_logger().info(f'[AVOID ODOM CAL] plot_png                = {self.plot_png_path}')
        self.get_logger().info('==================================================')

        rclpy.shutdown()

    def _write_summary_csv(
        self,
        dx_body,
        dy_body,
        odom_delta_yaw_deg,
        odom_displacement_m,
        yaw_scale_text,
        displacement_scale_text,
    ):
        with open(self.summary_csv_path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['field', 'value'])
            writer.writerow(['pattern', self.pattern])
            writer.writerow(['repeats', self.repeats])
            writer.writerow(['turn_direction', self.turn_direction])
            writer.writerow(['turn_duty_percent', self.turn_duty_percent])
            writer.writerow(['straight_duty_percent', self.straight_duty_percent])
            writer.writerow(['turn_duration_sec', self.turn_duration_sec])
            writer.writerow(['turn_timeout_sec', self.turn_timeout_sec])
            writer.writerow(['straight_duration_sec', self.straight_duration_sec])
            writer.writerow(['use_straight_distance', self.use_straight_distance])
            writer.writerow(['straight_distance_m', self.straight_distance_m])
            writer.writerow(['straight_timeout_sec', self.straight_timeout_sec])
            writer.writerow(['semicircle_radius_m', self.semicircle_radius_m])
            writer.writerow(['semicircle_total_yaw_deg', self.semicircle_total_yaw_deg])
            writer.writerow(['semicircle_use_chord_distance', self.semicircle_use_chord_distance])
            writer.writerow(['settle_duration_sec', self.settle_duration_sec])
            writer.writerow(['trajectory_sample_period_sec', self.trajectory_sample_period_sec])
            writer.writerow(['trajectory_print_max_points', self.trajectory_print_max_points])
            writer.writerow(['save_plot', self.save_plot])
            writer.writerow(['plot_png_path', self.plot_png_path])
            writer.writerow(['odom_dx_body_m', dx_body])
            writer.writerow(['odom_dy_body_m', dy_body])
            writer.writerow(['odom_displacement_m', odom_displacement_m])
            writer.writerow(['odom_delta_yaw_deg', odom_delta_yaw_deg])
            writer.writerow(['use_actual_delta_yaw', self.use_actual_delta_yaw])
            writer.writerow(['actual_delta_yaw_deg', self.actual_delta_yaw_deg])
            writer.writerow(['avoidance_yaw_scale', yaw_scale_text])
            writer.writerow(['use_actual_displacement', self.use_actual_displacement])
            writer.writerow(['actual_dx_body_m', self.actual_dx_body_m])
            writer.writerow(['actual_dy_body_m', self.actual_dy_body_m])
            writer.writerow(['displacement_scale', displacement_scale_text])

    def _save_trajectory_plot(self):
        if len(self.trajectory_samples) < 2:
            return
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception as e:
            self.get_logger().warn(
                f'[AVOID ODOM CAL] Could not save plot because matplotlib is unavailable: {e}'
            )
            return

        xs = [sample['dx_body'] for sample in self.trajectory_samples]
        ys = [sample['dy_body'] for sample in self.trajectory_samples]

        fig, ax = plt.subplots(figsize=(7.0, 7.0))
        ax.plot(xs, ys, '-o', markersize=2.5, linewidth=1.2, label='odom trajectory')
        ax.scatter([xs[0]], [ys[0]], s=60, marker='o', label='start')
        ax.scatter([xs[-1]], [ys[-1]], s=70, marker='x', label='end')

        expected = self._expected_semicircle_endpoint()
        if expected is not None:
            expected_x, expected_y, _, _ = expected
            ax.scatter([expected_x], [expected_y], s=70, marker='+', label='expected end')
            self._plot_expected_semicircle(ax)

        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, linestyle='--', alpha=0.35)
        ax.set_xlabel('dx_body_m')
        ax.set_ylabel('dy_body_m')
        ax.set_title('avoidance odometry calibration')
        ax.legend(loc='best')
        fig.tight_layout()
        fig.savefig(self.plot_png_path, dpi=160)
        plt.close(fig)

    def _plot_expected_semicircle(self, ax):
        segment_yaw_rad = (
            self.turn_direction
            * math.radians(self.semicircle_total_yaw_deg)
            / float(self.repeats)
        )
        segment_distance_m = self._semicircle_segment_distance(abs(segment_yaw_rad))
        points_x = [0.0]
        points_y = [0.0]
        x_body = 0.0
        y_body = 0.0
        yaw = 0.5 * segment_yaw_rad
        for repeat_index in range(self.repeats):
            x_body += segment_distance_m * math.cos(yaw)
            y_body += segment_distance_m * math.sin(yaw)
            points_x.append(x_body)
            points_y.append(y_body)
            if repeat_index < self.repeats - 1:
                yaw += segment_yaw_rad
        ax.plot(points_x, points_y, '--', linewidth=1.0, label='expected semicircle')

    def _print_trajectory_summary(self):
        if not self.trajectory_samples:
            return

        sample_count = len(self.trajectory_samples)
        max_points = min(self.trajectory_print_max_points, sample_count)
        if sample_count <= max_points:
            samples = self.trajectory_samples
        else:
            indices = []
            for i in range(max_points):
                index = round(i * (sample_count - 1) / (max_points - 1))
                if index not in indices:
                    indices.append(index)
            samples = [self.trajectory_samples[index] for index in indices]

        self.get_logger().info(
            '[AVOID ODOM CAL] Estimated odom trajectory '
            f'({len(samples)}/{sample_count} samples):'
        )
        self.get_logger().info(
            '[AVOID ODOM CAL]   t_sec | x_odom | y_odom | yaw_deg | dx_body | dy_body | accum_yaw_deg | phase'
        )
        for sample in samples:
            self.get_logger().info(
                '[AVOID ODOM CAL]   '
                f'{sample["time_sec"]:6.2f} | '
                f'{sample["x"]:6.3f} | '
                f'{sample["y"]:6.3f} | '
                f'{math.degrees(sample["yaw"]):7.2f} | '
                f'{sample["dx_body"]:7.3f} | '
                f'{sample["dy_body"]:7.3f} | '
                f'{sample["accum_yaw_deg"]:13.2f} | '
                f'{sample["phase"]}'
            )

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
    node = LiuAvoidanceOdometryCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[AVOID ODOM CAL] Keyboard interrupt, stopping.')
    finally:
        try:
            node.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()
