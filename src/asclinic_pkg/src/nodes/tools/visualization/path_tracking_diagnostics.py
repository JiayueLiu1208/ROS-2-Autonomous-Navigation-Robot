#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from pathlib import Path

import matplotlib

_HEADLESS = not bool(os.environ.get('DISPLAY'))
if _HEADLESS:
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import rclpy
from asclinic_pkg.msg import LeftRightFloat32
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node


def quaternion_to_yaw(q) -> float:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1.0e-9:
        return 0.0
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class PathTrackingDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__('path_tracking_diagnostics')

        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('path_topic', 'reference_path')
        self.declare_parameter('motor_cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter(
            'save_directory',
            '~/asclinic-ros2/ros2_ws/results/straight_line_test',
        )
        self.declare_parameter('show_live', True)
        self.declare_parameter('plot_period_sec', 0.5)
        self.declare_parameter('max_points', 10000)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.motor_cmd_topic = str(self.get_parameter('motor_cmd_topic').value)
        save_root = Path(os.path.expanduser(str(self.get_parameter('save_directory').value)))
        self.show_live = as_bool(self.get_parameter('show_live').value) and not _HEADLESS
        self.plot_period_sec = max(0.2, float(self.get_parameter('plot_period_sec').value))
        self.max_points = max(100, int(self.get_parameter('max_points').value))

        run_tag = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = save_root / run_tag
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.trace_csv_path = self.run_dir / 'path_tracking_trace.csv'
        self.summary_path = self.run_dir / 'path_tracking_summary.txt'
        self.latest_png_path = self.run_dir / 'path_tracking_live.png'
        self.final_png_path = self.run_dir / 'path_tracking_result.png'

        self.trace_file = self.trace_csv_path.open('w', newline='', encoding='utf-8')
        self.trace_writer = csv.writer(self.trace_file)
        self.trace_writer.writerow([
            'time_sec',
            'x',
            'y',
            'yaw_rad',
            'cross_track_error_m',
            'left_duty_percent',
            'right_duty_percent',
        ])

        self.reference_path: list[tuple[float, float]] = []
        self.t_data: list[float] = []
        self.x_data: list[float] = []
        self.y_data: list[float] = []
        self.yaw_data: list[float] = []
        self.cross_track_data: list[float] = []
        self.left_duty_data: list[float] = []
        self.right_duty_data: list[float] = []

        self.start_time_sec: float | None = None
        self.latest_left_duty = 0.0
        self.latest_right_duty = 0.0
        self.received_odom = False

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(RosPath, self.path_topic, self.path_callback, 10)
        self.create_subscription(
            LeftRightFloat32,
            self.motor_cmd_topic,
            self.motor_callback,
            10,
        )
        self.create_timer(self.plot_period_sec, self.update_plot)

        self.fig = None
        self.axes = None
        if self.show_live:
            plt.ion()
            self.fig, self.axes = plt.subplots(2, 2, figsize=(11, 8))

        mode = 'live window + saved plots' if self.show_live else 'saved plots only'
        self.get_logger().info(
            '[DIAG] odom=%s path=%s motor=%s output=%s (%s)'
            % (self.odom_topic, self.path_topic, self.motor_cmd_topic, self.run_dir, mode)
        )

    def path_callback(self, msg: RosPath) -> None:
        points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        if points:
            self.reference_path = points

    def motor_callback(self, msg: LeftRightFloat32) -> None:
        self.latest_left_duty = float(msg.left)
        self.latest_right_duty = float(msg.right)

    def odom_callback(self, msg: Odometry) -> None:
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if self.start_time_sec is None:
            self.start_time_sec = stamp_sec
        t = stamp_sec - self.start_time_sec

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        cross_track_error = self.distance_to_reference_path(x, y)

        self.t_data.append(t)
        self.x_data.append(x)
        self.y_data.append(y)
        self.yaw_data.append(yaw)
        self.cross_track_data.append(cross_track_error)
        self.left_duty_data.append(self.latest_left_duty)
        self.right_duty_data.append(self.latest_right_duty)
        self.trim_data()

        self.trace_writer.writerow([
            f'{t:.4f}',
            f'{x:.6f}',
            f'{y:.6f}',
            f'{yaw:.6f}',
            f'{cross_track_error:.6f}' if math.isfinite(cross_track_error) else '',
            f'{self.latest_left_duty:.3f}',
            f'{self.latest_right_duty:.3f}',
        ])
        self.trace_file.flush()
        self.received_odom = True

    def trim_data(self) -> None:
        while len(self.t_data) > self.max_points:
            self.t_data.pop(0)
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.yaw_data.pop(0)
            self.cross_track_data.pop(0)
            self.left_duty_data.pop(0)
            self.right_duty_data.pop(0)

    def distance_to_reference_path(self, x: float, y: float) -> float:
        if len(self.reference_path) == 0:
            return float('nan')
        if len(self.reference_path) == 1:
            px, py = self.reference_path[0]
            return math.hypot(x - px, y - py)

        best = float('inf')
        for start, end in zip(self.reference_path, self.reference_path[1:]):
            sx, sy = start
            ex, ey = end
            dx = ex - sx
            dy = ey - sy
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq <= 1.0e-12:
                candidate = math.hypot(x - sx, y - sy)
            else:
                ratio = ((x - sx) * dx + (y - sy) * dy) / seg_len_sq
                ratio = max(0.0, min(1.0, ratio))
                proj_x = sx + ratio * dx
                proj_y = sy + ratio * dy
                candidate = math.hypot(x - proj_x, y - proj_y)
            best = min(best, candidate)
        return best

    def update_plot(self) -> None:
        if not self.received_odom:
            return
        self.draw_plot(self.latest_png_path, update_live=self.show_live)

    def draw_plot(self, output_path: Path, update_live: bool = False) -> None:
        if update_live and self.fig is not None and self.axes is not None:
            fig = self.fig
            axes = self.axes
            for axis in axes.flat:
                axis.clear()
        else:
            fig, axes = plt.subplots(2, 2, figsize=(11, 8))

        ax_path = axes[0, 0]
        if self.reference_path:
            ref_x = [p[0] for p in self.reference_path]
            ref_y = [p[1] for p in self.reference_path]
            ax_path.plot(ref_x, ref_y, 'k--', linewidth=1.4, label='reference')
            ax_path.plot(ref_x[0], ref_y[0], 'go', markersize=6, label='start')
            ax_path.plot(ref_x[-1], ref_y[-1], 'ro', markersize=6, label='goal')
        ax_path.plot(self.x_data, self.y_data, color='tab:blue', linewidth=1.8, label='odom')
        ax_path.set_title('Path Tracking')
        ax_path.set_xlabel('x (m)')
        ax_path.set_ylabel('y (m)')
        ax_path.axis('equal')
        ax_path.grid(True)
        ax_path.legend(loc='best')

        axes[0, 1].plot(self.t_data, self.x_data, label='x')
        axes[0, 1].plot(self.t_data, self.y_data, label='y')
        axes[0, 1].set_title('Position vs Time')
        axes[0, 1].set_xlabel('time (s)')
        axes[0, 1].set_ylabel('position (m)')
        axes[0, 1].grid(True)
        axes[0, 1].legend(loc='best')

        finite_errors = [
            value if math.isfinite(value) else float('nan')
            for value in self.cross_track_data
        ]
        axes[1, 0].plot(self.t_data, finite_errors, color='tab:red')
        axes[1, 0].set_title('Cross-track Error')
        axes[1, 0].set_xlabel('time (s)')
        axes[1, 0].set_ylabel('error (m)')
        axes[1, 0].grid(True)

        axes[1, 1].plot(self.t_data, self.left_duty_data, label='left')
        axes[1, 1].plot(self.t_data, self.right_duty_data, label='right')
        axes[1, 1].set_title('Motor Duty Command')
        axes[1, 1].set_xlabel('time (s)')
        axes[1, 1].set_ylabel('duty (%)')
        axes[1, 1].grid(True)
        axes[1, 1].legend(loc='best')

        fig.tight_layout()
        fig.savefig(output_path, dpi=150)

        if update_live:
            fig.canvas.draw_idle()
            plt.pause(0.001)
        else:
            plt.close(fig)

    def write_summary(self) -> None:
        if not self.received_odom:
            return
        finite_errors = [value for value in self.cross_track_data if math.isfinite(value)]
        rms_error = float('nan')
        max_error = float('nan')
        if finite_errors:
            rms_error = math.sqrt(sum(value * value for value in finite_errors) / len(finite_errors))
            max_error = max(finite_errors)

        with self.summary_path.open('w', encoding='utf-8') as summary_file:
            summary_file.write('Path tracking diagnostics summary\n')
            summary_file.write(f'odom_topic: {self.odom_topic}\n')
            summary_file.write(f'path_topic: {self.path_topic}\n')
            summary_file.write(f'motor_cmd_topic: {self.motor_cmd_topic}\n')
            summary_file.write(f'samples: {len(self.t_data)}\n')
            summary_file.write(f'final_x: {self.x_data[-1]:.6f}\n')
            summary_file.write(f'final_y: {self.y_data[-1]:.6f}\n')
            summary_file.write(f'final_yaw_rad: {self.yaw_data[-1]:.6f}\n')
            summary_file.write(f'rms_cross_track_error_m: {rms_error:.6f}\n')
            summary_file.write(f'max_cross_track_error_m: {max_error:.6f}\n')
            summary_file.write(f'trace_csv: {self.trace_csv_path}\n')
            summary_file.write(f'result_png: {self.final_png_path}\n')

    def destroy_node(self) -> None:
        try:
            if self.received_odom:
                self.draw_plot(self.final_png_path, update_live=False)
                self.write_summary()
                self.get_logger().info('[DIAG] result_png=%s' % self.final_png_path)
                self.get_logger().info('[DIAG] trace_csv=%s' % self.trace_csv_path)
                self.get_logger().info('[DIAG] summary=%s' % self.summary_path)
        except Exception as exc:
            self.get_logger().warn('[DIAG] Failed to save final diagnostics: %s' % exc)

        try:
            if not self.trace_file.closed:
                self.trace_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathTrackingDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
