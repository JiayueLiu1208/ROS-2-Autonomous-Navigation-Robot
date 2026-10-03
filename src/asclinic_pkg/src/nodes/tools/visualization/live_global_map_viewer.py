#!/usr/bin/env python3

import csv
import json
import math
import os
import signal
import socket
import sys
from datetime import datetime
from pathlib import Path

if 'MPLCONFIGDIR' not in os.environ:
    mpl_config_dir = Path(os.environ.get('TMPDIR', '/tmp')) / 'matplotlib'
    mpl_config_dir.mkdir(parents=True, exist_ok=True)
    os.environ['MPLCONFIGDIR'] = str(mpl_config_dir)

import matplotlib

def _display_looks_available() -> bool:
    display = os.environ.get('DISPLAY', '').strip()
    if not display:
        return False

    if display.startswith(':'):
        display_id = display[1:].split('.', 1)[0]
        return (Path('/tmp/.X11-unix') / f'X{display_id}').exists()

    if ':' not in display:
        return False
    host, display_suffix = display.rsplit(':', 1)
    display_id = display_suffix.split('.', 1)[0]
    try:
        port = 6000 + int(display_id)
    except ValueError:
        return False
    host = host or '127.0.0.1'
    if host == 'localhost':
        host = '127.0.0.1'
    try:
        with socket.create_connection((host, port), timeout=0.25):
            return True
    except OSError:
        return False


_HEADLESS = not _display_looks_available()
if _HEADLESS:
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Rectangle
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


def _add_path_planning_to_sys_path() -> None:
    here = Path(__file__).resolve()
    candidates = [
        here.parents[2] / 'path_planning',
        here.parent,
        Path.cwd() / 'src' / 'asclinic_pkg' / 'src' / 'nodes' / 'path_planning',
    ]
    for candidate in candidates:
        if (candidate / 'final_demo_layout.py').exists():
            sys.path.insert(0, str(candidate))
            return


_add_path_planning_to_sys_path()

from final_demo_layout import (  # noqa: E402
    COVERAGE_STOPS,
    HEIGHT_M,
    LECTERNS,
    MARKERS,
    PLANTS,
    STOP_POINTS,
    TABLES,
    WIDTH_M,
)
from preview_route import route_length  # noqa: E402


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


class LiveGlobalMapViewer(Node):
    def __init__(self) -> None:
        super().__init__('live_global_map_viewer')

        self.declare_parameter('odom_topic', '/asc/pitt_fused_odometry')
        self.declare_parameter('wheel_odom_topic', '/asc/wheel_odometry')
        self.declare_parameter('vision_odom_topic', '/asc/pitt_vision_odometry')
        self.declare_parameter('lidar_wall_odom_topic', '/asc/pitt_lidar_wall_odometry')
        self.declare_parameter(
            'lidar_scanmatch_odom_topic',
            '/asc/pitt_lidar_scanmatch_odometry',
        )
        self.declare_parameter('lidar_wall_active_timeout_sec', 2.5)
        self.declare_parameter('path_topic', '/asc/reference_path')
        self.declare_parameter('stop_poses_topic', '/asc/plant_stop_poses')
        self.declare_parameter('dynamic_obstacle_marker_topic', 'dynamic_obstacle_marker')
        self.declare_parameter('system_status_topic', '/asc/jetson_status')
        self.declare_parameter('explored_map_topic', '/asc/explored_map')
        self.declare_parameter('exploration_summary_topic', '/asc/exploration_summary')
        self.declare_parameter(
            'save_directory',
            '~/asclinic-ros2/ros2_ws/results/live_global_map',
        )
        self.declare_parameter('show_gui', True)
        self.declare_parameter('save_live_png', True)
        self.declare_parameter(
            'run_tag',
            '',
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('plot_period_sec', 0.25)
        self.declare_parameter('save_period_sec', 0.75)
        self.declare_parameter('trail_length', 2500)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.vision_odom_topic = str(self.get_parameter('vision_odom_topic').value)
        self.lidar_wall_odom_topic = str(
            self.get_parameter('lidar_wall_odom_topic').value
        )
        self.lidar_scanmatch_odom_topic = str(
            self.get_parameter('lidar_scanmatch_odom_topic').value
        )
        self.lidar_wall_active_timeout_sec = max(
            0.1,
            float(self.get_parameter('lidar_wall_active_timeout_sec').value),
        )
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.stop_poses_topic = str(self.get_parameter('stop_poses_topic').value)
        self.dynamic_obstacle_marker_topic = str(
            self.get_parameter('dynamic_obstacle_marker_topic').value
        )
        self.system_status_topic = str(self.get_parameter('system_status_topic').value)
        self.explored_map_topic = str(self.get_parameter('explored_map_topic').value)
        self.exploration_summary_topic = str(
            self.get_parameter('exploration_summary_topic').value
        )
        self.save_root = Path(
            os.path.expanduser(str(self.get_parameter('save_directory').value))
        )
        self.show_gui = as_bool(self.get_parameter('show_gui').value) and not _HEADLESS
        self.save_live_png = as_bool(self.get_parameter('save_live_png').value)
        configured_run_tag = str(self.get_parameter('run_tag').value).strip()
        if configured_run_tag.isdigit() and len(configured_run_tag) == 14:
            configured_run_tag = f'{configured_run_tag[:8]}_{configured_run_tag[8:]}'
        self.plot_period_sec = max(
            0.1,
            float(self.get_parameter('plot_period_sec').value),
        )
        self.save_period_sec = max(
            0.2,
            float(self.get_parameter('save_period_sec').value),
        )
        self.trail_length = max(10, int(self.get_parameter('trail_length').value))

        run_tag = configured_run_tag or datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = self.save_root / run_tag
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.live_png_path = self.run_dir / 'live_global_map.png'
        self.final_png_path = self.run_dir / 'final_global_map.png'
        self.trace_csv_path = self.run_dir / 'live_global_map_trace.csv'
        self.system_status_csv_path = self.run_dir / 'jetson_status_trace.csv'

        self.trace_file = self.trace_csv_path.open('w', newline='', encoding='utf-8')
        self.trace_writer = csv.writer(self.trace_file)
        self.trace_writer.writerow([
            'time_sec',
            'x',
            'y',
            'yaw_rad',
            'cross_track_error_m',
        ])
        self.trace_file.flush()

        self.system_status_file = self.system_status_csv_path.open(
            'w',
            newline='',
            encoding='utf-8',
        )
        self.system_status_writer = csv.writer(self.system_status_file)
        self.system_status_writer.writerow([
            'receive_time_sec',
            'stamp_sec',
            'hostname',
            'cpu_percent',
            'memory_percent',
            'memory_used_mb',
            'memory_total_mb',
            'temperature_c',
            'load_1min',
            'load_5min',
            'load_15min',
            'disk_percent',
        ])
        self.system_status_file.flush()

        self.reference_path: list[tuple[float, float]] = []
        self.walked_reference_path: list[tuple[float, float]] = []
        self.calculated_stop_points: list[tuple[float, float]] = []
        self.dynamic_obstacles: list[tuple[float, float]] = []
        self.history_x: list[float] = []
        self.history_y: list[float] = []
        self.history_t: list[float] = []
        self.wheel_history_x: list[float] = []
        self.wheel_history_y: list[float] = []
        self.wheel_history_t: list[float] = []
        self.vision_history_x: list[float] = []
        self.vision_history_y: list[float] = []
        self.vision_history_t: list[float] = []
        self.lidar_scanmatch_history_x: list[float] = []
        self.lidar_scanmatch_history_y: list[float] = []
        self.lidar_scanmatch_history_t: list[float] = []
        self.latest_pose: tuple[float, float, float] | None = None
        self.latest_wheel_pose: tuple[float, float, float] | None = None
        self.latest_vision_pose: tuple[float, float, float] | None = None
        self.latest_lidar_wall_pose: tuple[float, float, float] | None = None
        self.latest_lidar_wall_receive_sec: float | None = None
        self.lidar_wall_update_count = 0
        self.latest_lidar_scanmatch_pose: tuple[float, float, float] | None = None
        self.latest_lidar_scanmatch_receive_sec: float | None = None
        self.lidar_scanmatch_update_count = 0
        self.latest_system_status: dict[str, object] | None = None
        self.latest_system_status_receive_sec: float | None = None
        self.latest_explored_map: OccupancyGrid | None = None
        self.latest_exploration_summary: dict[str, object] | None = None
        self.latest_exploration_summary_receive_sec: float | None = None
        self.start_time_sec: float | None = None
        self.last_save_sec = 0.0
        self.received_odom = False
        self.received_wheel_odom = False
        self.received_vision_odom = False
        self.received_path = False

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(
            Odometry,
            self.wheel_odom_topic,
            self.wheel_odom_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.vision_odom_topic,
            self.vision_odom_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.lidar_wall_odom_topic,
            self.lidar_wall_odom_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.lidar_scanmatch_odom_topic,
            self.lidar_scanmatch_odom_callback,
            10,
        )
        self.create_subscription(RosPath, self.path_topic, self.path_callback, 10)
        self.create_subscription(
            PoseArray,
            self.stop_poses_topic,
            self.stop_poses_callback,
            10,
        )
        self.create_subscription(
            Marker,
            self.dynamic_obstacle_marker_topic,
            self.dynamic_obstacle_marker_callback,
            10,
        )
        self.create_subscription(
            String,
            self.system_status_topic,
            self.system_status_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.explored_map_topic,
            self.explored_map_callback,
            10,
        )
        self.create_subscription(
            String,
            self.exploration_summary_topic,
            self.exploration_summary_callback,
            10,
        )
        self.create_timer(self.plot_period_sec, self.update_plot)

        plt.ion()
        self.fig = plt.figure(figsize=(13.5, 8.6))
        grid = self.fig.add_gridspec(
            2,
            2,
            width_ratios=[4.9, 1.35],
            height_ratios=[1.0, 0.11],
        )
        self.ax_map = self.fig.add_subplot(grid[0, 0])
        self.ax_panel = self.fig.add_subplot(grid[0, 1])
        self.ax_legend = self.fig.add_subplot(grid[1, :])

        self.draw_plot(
            save_path=self.live_png_path if self.save_live_png else None,
            update_gui=False,
        )

        mode = 'live window + saved PNG' if self.show_gui else 'saved PNG only'
        self.get_logger().info(
            '[MAP_VIEW] fused=%s wheel=%s vision=%s lidar_wall=%s lidar_scanmatch=%s path=%s stops=%s lidar_obstacles=%s status=%s explore=%s summary=%s output=%s (%s)'
            % (
                self.odom_topic,
                self.wheel_odom_topic,
                self.vision_odom_topic,
                self.lidar_wall_odom_topic,
                self.lidar_scanmatch_odom_topic,
                self.path_topic,
                self.stop_poses_topic,
                self.dynamic_obstacle_marker_topic,
                self.system_status_topic,
                self.explored_map_topic,
                self.exploration_summary_topic,
                self.run_dir,
                mode,
            )
        )
        if self.show_gui:
            self.show_window()

    def path_callback(self, msg: RosPath) -> None:
        points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        if len(points) >= 2:
            self.reference_path = points
            self.received_path = True

    def stop_poses_callback(self, msg: PoseArray) -> None:
        self.calculated_stop_points = [
            (float(pose.position.x), float(pose.position.y))
            for pose in msg.poses
        ]

    def dynamic_obstacle_marker_callback(self, msg: Marker) -> None:
        if msg.action == Marker.DELETE:
            self.dynamic_obstacles = []
            return
        self.dynamic_obstacles = [
            (float(point.x), float(point.y))
            for point in msg.points
        ]

    def system_status_callback(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring malformed system status JSON: {exc}')
            return
        if not isinstance(status, dict):
            self.get_logger().warn('Ignoring system status message that is not a JSON object')
            return

        receive_sec = self.get_clock().now().nanoseconds * 1.0e-9
        self.latest_system_status = status
        self.latest_system_status_receive_sec = receive_sec
        self.system_status_writer.writerow([
            f'{receive_sec:.4f}',
            self.format_status_float(status.get('stamp_sec'), precision=4),
            str(status.get('hostname', '')),
            self.format_status_float(status.get('cpu_percent')),
            self.format_status_float(status.get('memory_percent')),
            self.format_status_float(status.get('memory_used_mb')),
            self.format_status_float(status.get('memory_total_mb')),
            self.format_status_float(status.get('temperature_c')),
            self.format_status_float(status.get('load_1min')),
            self.format_status_float(status.get('load_5min')),
            self.format_status_float(status.get('load_15min')),
            self.format_status_float(status.get('disk_percent')),
        ])
        self.system_status_file.flush()

    def explored_map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_explored_map = msg

    def exploration_summary_callback(self, msg: String) -> None:
        try:
            summary = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring malformed exploration summary JSON: {exc}')
            return
        if not isinstance(summary, dict):
            self.get_logger().warn('Ignoring exploration summary that is not a JSON object')
            return
        self.latest_exploration_summary = summary
        self.latest_exploration_summary_receive_sec = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )

    @staticmethod
    def status_float(value: object) -> float:
        if value is None:
            return float('nan')
        try:
            result = float(value)
        except (TypeError, ValueError):
            return float('nan')
        return result if math.isfinite(result) else float('nan')

    def format_status_float(self, value: object, precision: int = 2) -> str:
        result = self.status_float(value)
        return f'{result:.{precision}f}' if math.isfinite(result) else ''

    def status_line(self, label: str, value: object, unit: str = '', precision: int = 1) -> str:
        result = self.status_float(value)
        if not math.isfinite(result):
            return f'{label}: n/a'
        return f'{label}: {result:.{precision}f}{unit}'

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
        self.latest_pose = (x, y, yaw)

        self.history_t.append(t)
        self.history_x.append(x)
        self.history_y.append(y)
        self.trim_history()
        self.record_walked_reference_path(x, y)

        error = self.distance_to_path(x, y)
        self.trace_writer.writerow([
            f'{t:.4f}',
            f'{x:.6f}',
            f'{y:.6f}',
            f'{yaw:.6f}',
            f'{error:.6f}' if math.isfinite(error) else '',
        ])
        self.trace_file.flush()
        self.received_odom = True

    def wheel_odom_callback(self, msg: Odometry) -> None:
        x, y, yaw, t = self.pose_from_odometry(msg)
        self.latest_wheel_pose = (x, y, yaw)
        self.wheel_history_t.append(t)
        self.wheel_history_x.append(x)
        self.wheel_history_y.append(y)
        self.trim_pose_history(
            self.wheel_history_x,
            self.wheel_history_y,
            self.wheel_history_t,
        )
        self.received_wheel_odom = True

    def vision_odom_callback(self, msg: Odometry) -> None:
        x, y, yaw, t = self.pose_from_odometry(msg)
        self.latest_vision_pose = (x, y, yaw)
        self.vision_history_t.append(t)
        self.vision_history_x.append(x)
        self.vision_history_y.append(y)
        self.trim_pose_history(
            self.vision_history_x,
            self.vision_history_y,
            self.vision_history_t,
        )
        self.received_vision_odom = True

    def lidar_wall_odom_callback(self, msg: Odometry) -> None:
        x, y, yaw, _t = self.pose_from_odometry(msg)
        self.latest_lidar_wall_pose = (x, y, yaw)
        self.latest_lidar_wall_receive_sec = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )
        self.lidar_wall_update_count += 1

    def lidar_scanmatch_odom_callback(self, msg: Odometry) -> None:
        x, y, yaw, t = self.pose_from_odometry(msg)
        self.latest_lidar_scanmatch_pose = (x, y, yaw)
        self.lidar_scanmatch_history_t.append(t)
        self.lidar_scanmatch_history_x.append(x)
        self.lidar_scanmatch_history_y.append(y)
        self.trim_pose_history(
            self.lidar_scanmatch_history_x,
            self.lidar_scanmatch_history_y,
            self.lidar_scanmatch_history_t,
        )
        self.latest_lidar_scanmatch_receive_sec = (
            self.get_clock().now().nanoseconds * 1.0e-9
        )
        self.lidar_scanmatch_update_count += 1

    def pose_from_odometry(self, msg: Odometry) -> tuple[float, float, float, float]:
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if self.start_time_sec is None:
            self.start_time_sec = stamp_sec
        t = stamp_sec - self.start_time_sec

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        return x, y, yaw, t

    def trim_history(self) -> None:
        self.trim_pose_history(self.history_x, self.history_y, self.history_t)

    def trim_pose_history(
        self,
        history_x: list[float],
        history_y: list[float],
        history_t: list[float],
    ) -> None:
        while len(history_x) > self.trail_length:
            history_x.pop(0)
            history_y.pop(0)
            history_t.pop(0)

    def active_path(self) -> list[tuple[float, float]]:
        return self.reference_path

    def distance_to_path(self, x: float, y: float) -> float:
        path = self.active_path()
        distance, _proj_x, _proj_y = self.closest_point_on_path(path, x, y)
        return distance

    def closest_point_on_path(
        self,
        path: list[tuple[float, float]],
        x: float,
        y: float,
    ) -> tuple[float, float, float]:
        if len(path) < 2:
            return float('nan'), float('nan'), float('nan')

        best = float('inf')
        best_x = float('nan')
        best_y = float('nan')
        for start, end in zip(path, path[1:]):
            sx, sy = start
            ex, ey = end
            dx = ex - sx
            dy = ey - sy
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq <= 1.0e-12:
                proj_x = sx
                proj_y = sy
            else:
                ratio = ((x - sx) * dx + (y - sy) * dy) / seg_len_sq
                ratio = max(0.0, min(1.0, ratio))
                proj_x = sx + ratio * dx
                proj_y = sy + ratio * dy
            candidate = math.hypot(x - proj_x, y - proj_y)
            if candidate < best:
                best = candidate
                best_x = proj_x
                best_y = proj_y
        return best, best_x, best_y

    def record_walked_reference_path(self, x: float, y: float) -> None:
        if len(self.reference_path) < 2:
            return
        distance, proj_x, proj_y = self.closest_point_on_path(self.reference_path, x, y)
        if not math.isfinite(distance) or distance > 1.0:
            return
        if self.walked_reference_path:
            last_x, last_y = self.walked_reference_path[-1]
            if math.isfinite(last_x) and math.isfinite(last_y):
                gap = math.hypot(proj_x - last_x, proj_y - last_y)
                if gap < 0.04:
                    return
                if gap > 0.80:
                    self.walked_reference_path.append((float('nan'), float('nan')))
        self.walked_reference_path.append((proj_x, proj_y))
        while len(self.walked_reference_path) > self.trail_length:
            self.walked_reference_path.pop(0)

    def update_plot(self) -> None:
        if self.latest_pose is None and not self.show_gui and not self.save_live_png:
            return

        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        save_path = None
        if self.save_live_png and now_sec - self.last_save_sec >= self.save_period_sec:
            save_path = self.live_png_path
            self.last_save_sec = now_sec
        self.draw_plot(save_path=save_path)

    def draw_plot(
        self,
        save_path: Path | None = None,
        *,
        update_gui: bool = True,
    ) -> None:
        self.ax_map.clear()
        self.ax_panel.clear()
        self.ax_legend.clear()
        self.draw_map()
        self.draw_panel()
        self.draw_legend_below()

        self.fig.tight_layout()
        if save_path is not None:
            self.savefig_atomic(save_path)
        if update_gui and self.show_gui:
            self.update_gui_canvas()

    def show_window(self) -> None:
        try:
            plt.show(block=False)
            self.update_gui_canvas()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'Disabling live map GUI because the display backend failed: {exc}'
            )
            self.show_gui = False

    def update_gui_canvas(self) -> None:
        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'Disabling live map GUI because drawing failed: {exc}'
            )
            self.show_gui = False

    def savefig_atomic(self, save_path: Path) -> None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = save_path.with_name(f'.{save_path.stem}.tmp{save_path.suffix}')
        try:
            self.fig.savefig(tmp_path, dpi=130, format='png')
            os.replace(tmp_path, save_path)
        finally:
            if tmp_path.exists():
                tmp_path.unlink()

    def draw_map(self) -> None:
        ax = self.ax_map
        ax.set_title('Final Demo Live Global Map', fontsize=13, weight='bold')
        ax.set_xlim(-0.15, WIDTH_M + 0.15)
        ax.set_ylim(-0.15, HEIGHT_M + 0.20)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_xticks([float(i) for i in range(int(WIDTH_M) + 1)])
        ax.set_yticks([float(i) for i in range(int(math.floor(HEIGHT_M)) + 1)])
        ax.grid(which='major', color='#cbd5e1', linewidth=0.8, alpha=0.75)

        ax.add_patch(
            Rectangle(
                (0.0, 0.0),
                WIDTH_M,
                HEIGHT_M,
                facecolor='#ffffff',
                edgecolor='#111827',
                linewidth=1.6,
                zorder=0,
            )
        )

        for name, cx, cy, sx, sy in TABLES:
            self.draw_rect(cx, cy, sx, sy, '#9ca3af', '#4b5563', name)
        for name, cx, cy, sx, sy in LECTERNS:
            self.draw_rect(cx, cy, sx, sy, '#d8b56d', '#92400e', name)

        self.draw_markers()

        if len(self.walked_reference_path) >= 2:
            walked_x = [p[0] for p in self.walked_reference_path]
            walked_y = [p[1] for p in self.walked_reference_path]
            ax.plot(
                walked_x,
                walked_y,
                '--',
                color='#1d4ed8',
                linewidth=2.0,
                alpha=0.72,
                label='walked reference path',
                zorder=5.1,
            )

        if len(self.reference_path) >= 2:
            path_x = [p[0] for p in self.reference_path]
            path_y = [p[1] for p in self.reference_path]
            ax.plot(
                path_x,
                path_y,
                color='#2563eb',
                linewidth=3.0,
                label='dynamic planner reference path',
                zorder=5.2,
            )
            self.draw_path_arrows(self.reference_path)

        plant_lookup = {name: (x, y) for name, x, y in PLANTS}
        stop_labels = [plant_id for plant_id, _x, _y in STOP_POINTS]
        display_stop_points = (
            self.calculated_stop_points
            if self.calculated_stop_points
            else [(x, y) for _plant_id, x, y in STOP_POINTS]
        )
        for index, (stop_x, stop_y) in enumerate(display_stop_points):
            stop_id = stop_labels[index] if index < len(stop_labels) else f'S{index + 1}'
            if stop_id in plant_lookup:
                plant_x, plant_y = plant_lookup[stop_id]
                ax.plot(
                    [stop_x, plant_x],
                    [stop_y, plant_y],
                    linestyle=':',
                    color='#059669',
                    linewidth=1.5,
                    zorder=4,
                )
            ax.add_patch(
                Circle(
                    (stop_x, stop_y),
                    0.11,
                    facecolor='#f59e0b',
                    edgecolor='#92400e',
                    linewidth=1.5,
                    zorder=6,
                )
            )
            ax.text(stop_x + 0.08, stop_y + 0.08, f'{stop_id}_STOP', fontsize=8)

        for stop_id, stop_x, stop_y, _yaw_deg in COVERAGE_STOPS:
            ax.add_patch(
                Circle(
                    (stop_x, stop_y),
                    0.12,
                    facecolor='#0ea5e9',
                    edgecolor='#075985',
                    linewidth=1.5,
                    zorder=6,
                )
            )
            ax.text(stop_x + 0.08, stop_y + 0.08, f'{stop_id}_SEARCH', fontsize=8)

        for name, x, y in PLANTS:
            ax.add_patch(
                Circle(
                    (x, y),
                    0.17,
                    facecolor='#22c55e',
                    edgecolor='#166534',
                    linewidth=1.5,
                    zorder=7,
                )
            )
            ax.text(x, y, name, ha='center', va='center', fontsize=8, weight='bold')

        self.draw_exploration_overlay()
        self.draw_semantic_discoveries()

        if self.dynamic_obstacles:
            obstacle_x = [point[0] for point in self.dynamic_obstacles]
            obstacle_y = [point[1] for point in self.dynamic_obstacles]
            ax.scatter(
                obstacle_x,
                obstacle_y,
                s=28,
                marker='x',
                color='#d946ef',
                linewidths=1.6,
                label='LiDAR dynamic obstacle',
                zorder=9,
            )
            label_x, label_y = self.dynamic_obstacle_label_position()
            ax.text(
                label_x,
                label_y,
                f'LiDAR obstacles: {len(self.dynamic_obstacles)}',
                fontsize=8,
                color='#86198f',
                weight='bold',
                bbox={
                    'boxstyle': 'round,pad=0.25',
                    'facecolor': '#fae8ff',
                    'edgecolor': '#d946ef',
                    'alpha': 0.85,
                },
                zorder=12,
            )

        self.draw_pose_history(
            self.wheel_history_x,
            self.wheel_history_y,
            color='#f97316',
            label='wheel odometry trail',
            zorder=7.6,
            linestyle='-.',
            alpha=0.80,
        )
        self.draw_pose_history(
            self.vision_history_x,
            self.vision_history_y,
            color='#7c3aed',
            label='CV vision pose trail',
            zorder=7.8,
            linestyle=':',
            linewidth=1.8,
            alpha=0.82,
            marker='o',
        )
        self.draw_pose_history(
            self.lidar_scanmatch_history_x,
            self.lidar_scanmatch_history_y,
            color='#0891b2',
            label='LiDAR scan-map pose trail',
            zorder=7.9,
            linestyle='--',
            linewidth=1.9,
            alpha=0.86,
            marker='.',
        )
        self.draw_pose_history(
            self.history_x,
            self.history_y,
            color='#ef4444',
            label='fused localization trail',
            zorder=8.0,
            linewidth=1.8,
            alpha=0.72,
        )

        if self.latest_wheel_pose is not None:
            self.draw_pose_marker(
                self.latest_wheel_pose,
                color='#f97316',
                edge_color='#9a3412',
                label='wheel odometry pose',
                marker='s',
                zorder=9.4,
            )
        if self.latest_vision_pose is not None:
            self.draw_pose_marker(
                self.latest_vision_pose,
                color='#7c3aed',
                edge_color='#4c1d95',
                label='CV vision pose',
                marker='D',
                zorder=9.6,
            )
        if self.latest_lidar_scanmatch_pose is not None:
            self.draw_pose_marker(
                self.latest_lidar_scanmatch_pose,
                color='#0891b2',
                edge_color='#164e63',
                label='LiDAR scan-map pose',
                marker='P',
                zorder=9.8,
            )
        if self.latest_pose is not None:
            self.draw_pose_marker(
                self.latest_pose,
                color='#dc2626',
                edge_color='#7f1d1d',
                label='fused localization pose',
                marker='o',
                zorder=10.0,
            )

    def draw_legend_below(self) -> None:
        self.ax_legend.set_axis_off()
        handles, labels = self.ax_map.get_legend_handles_labels()
        unique_handles = []
        unique_labels = []
        seen = set()
        for handle, label in zip(handles, labels):
            if not label or label.startswith('_') or label in seen:
                continue
            seen.add(label)
            unique_handles.append(handle)
            unique_labels.append(label)
        if not unique_handles:
            return
        self.ax_legend.legend(
            unique_handles,
            unique_labels,
            loc='center',
            ncol=min(5, max(1, len(unique_handles))),
            fontsize=7.5,
            framealpha=0.95,
            borderaxespad=0.0,
        )

    def draw_exploration_overlay(self) -> None:
        if self.latest_explored_map is None:
            return
        msg = self.latest_explored_map
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            return
        grid = np.array(msg.data, dtype=np.int16).reshape((height, width))
        unknown = np.ma.masked_where(grid != -1, np.ones_like(grid, dtype=float))
        occupied = np.ma.masked_where(grid != 100, np.ones_like(grid, dtype=float))
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        extent = (
            origin_x,
            origin_x + width * float(msg.info.resolution),
            origin_y,
            origin_y + height * float(msg.info.resolution),
        )
        self.ax_map.imshow(
            unknown,
            origin='lower',
            extent=extent,
            cmap=ListedColormap(['#111827']),
            interpolation='nearest',
            alpha=0.34,
            zorder=7.2,
        )
        self.ax_map.imshow(
            occupied,
            origin='lower',
            extent=extent,
            cmap=ListedColormap(['#7f1d1d']),
            interpolation='nearest',
            alpha=0.30,
            zorder=7.25,
        )

    def draw_semantic_discoveries(self) -> None:
        summary = self.latest_exploration_summary
        if not isinstance(summary, dict):
            return

        confirmed = summary.get('confirmed_plants') or []
        if isinstance(confirmed, list):
            for plant in confirmed:
                if not isinstance(plant, dict):
                    continue
                try:
                    x = float(plant['x'])
                    y = float(plant['y'])
                except (KeyError, TypeError, ValueError):
                    continue
                plant_id = str(plant.get('id', 'plant'))
                self.ax_map.add_patch(
                    Circle(
                        (x, y),
                        0.27,
                        facecolor='none',
                        edgecolor='#16a34a',
                        linewidth=2.2,
                        zorder=11.5,
                    )
                )
                self.ax_map.text(
                    x + 0.18,
                    y + 0.22,
                    f'{plant_id} YOLO',
                    fontsize=7.5,
                    color='#14532d',
                    weight='bold',
                    zorder=12,
                )

        unknown_plants = summary.get('unknown_plants') or []
        if isinstance(unknown_plants, list):
            for plant in unknown_plants:
                if not isinstance(plant, dict):
                    continue
                try:
                    x = float(plant['x'])
                    y = float(plant['y'])
                    observations = int(plant.get('observations', 0))
                except (KeyError, TypeError, ValueError):
                    continue
                plant_id = str(plant.get('id', 'U?'))
                try:
                    captures = int(plant.get('captures', 0) or 0)
                except (TypeError, ValueError):
                    captures = 0
                captured = captures > 0 or str(plant.get('source', '')) == 'capture'
                if captured:
                    self.ax_map.add_patch(
                        Circle(
                            (x, y),
                            0.17,
                            facecolor='#fb923c',
                            edgecolor='#9a3412',
                            linewidth=1.5,
                            label='Captured unknown plant',
                            zorder=12,
                        )
                    )
                    self.ax_map.text(
                        x,
                        y,
                        plant_id,
                        ha='center',
                        va='center',
                        fontsize=7.5,
                        color='#7c2d12',
                        weight='bold',
                        zorder=12.2,
                    )
                else:
                    self.ax_map.scatter(
                        [x],
                        [y],
                        s=135,
                        marker='*',
                        color='#facc15',
                        edgecolor='#854d0e',
                        linewidth=1.2,
                        label='YOLO unknown plant candidate',
                        zorder=12,
                    )
                    self.ax_map.text(
                        x + 0.16,
                        y + 0.16,
                        f'{plant_id} n={observations}',
                        fontsize=7.5,
                        color='#854d0e',
                        weight='bold',
                        zorder=12.2,
                    )

        nearest = summary.get('nearest_frontier')
        if isinstance(nearest, dict):
            try:
                fx = float(nearest['x'])
                fy = float(nearest['y'])
            except (KeyError, TypeError, ValueError):
                return
            self.ax_map.scatter(
                [fx],
                [fy],
                s=58,
                marker='P',
                color='#0ea5e9',
                edgecolor='#075985',
                linewidth=1.0,
                label='nearest exploration frontier',
                zorder=11.7,
            )

    def dynamic_obstacle_label_position(self) -> tuple[float, float]:
        if not self.dynamic_obstacles:
            return 0.25, HEIGHT_M - 0.35
        avg_x = sum(point[0] for point in self.dynamic_obstacles) / len(self.dynamic_obstacles)
        avg_y = sum(point[1] for point in self.dynamic_obstacles) / len(self.dynamic_obstacles)
        label_x = max(0.25, min(WIDTH_M - 2.20, avg_x + 0.18))
        label_y = max(0.30, min(HEIGHT_M - 0.35, avg_y + 0.18))
        return label_x, label_y

    def draw_markers(self) -> None:
        marker_line_len = 0.36
        marker_arrow_len = 0.42
        for marker_id, x, y, _z, phi_deg in MARKERS:
            phi = math.radians(phi_deg)
            normal_x = math.cos(phi)
            normal_y = math.sin(phi)
            tangent_x = -normal_y
            tangent_y = normal_x
            x1 = x - tangent_x * marker_line_len / 2.0
            y1 = y - tangent_y * marker_line_len / 2.0
            x2 = x + tangent_x * marker_line_len / 2.0
            y2 = y + tangent_y * marker_line_len / 2.0
            self.ax_map.plot(
                [x1, x2],
                [y1, y2],
                color='#111827',
                linewidth=3.0,
                solid_capstyle='round',
                zorder=6,
            )
            self.ax_map.arrow(
                x,
                y,
                normal_x * marker_arrow_len,
                normal_y * marker_arrow_len,
                width=0.015,
                head_width=0.12,
                head_length=0.13,
                color='#1d4ed8',
                length_includes_head=True,
                zorder=7,
            )
            label_dx = 0.10 if x < WIDTH_M - 0.4 else -0.34
            label_dy = 0.18 if y < HEIGHT_M - 0.4 else -0.22
            self.ax_map.text(
                x + label_dx,
                y + label_dy,
                f'M{marker_id}',
                fontsize=7,
                color='#111827',
                zorder=8,
            )

    def draw_rect(
        self,
        cx: float,
        cy: float,
        width: float,
        height: float,
        face: str,
        edge: str,
        label: str,
    ) -> None:
        self.ax_map.add_patch(
            Rectangle(
                (cx - width / 2.0, cy - height / 2.0),
                width,
                height,
                facecolor=face,
                edgecolor=edge,
                linewidth=1.0,
                alpha=0.45,
                zorder=2,
            )
        )
        self.ax_map.text(cx, cy, label, ha='center', va='center', fontsize=8, zorder=3)

    def draw_path_arrows(self, path: list[tuple[float, float]]) -> None:
        if len(path) < 2:
            return
        stride = max(20, int(len(path) / 14))
        for idx in range(stride, len(path) - 1, stride):
            x1, y1 = path[idx]
            x2, y2 = path[idx + 1]
            dx = x2 - x1
            dy = y2 - y1
            norm = math.hypot(dx, dy)
            if norm <= 1.0e-6:
                continue
            length = 0.35
            self.ax_map.arrow(
                x1 - dx / norm * length / 2.0,
                y1 - dy / norm * length / 2.0,
                dx / norm * length,
                dy / norm * length,
                width=0.025,
                head_width=0.14,
                head_length=0.16,
                color='#2563eb',
                length_includes_head=True,
                zorder=5.4,
            )

    def draw_pose_history(
        self,
        history_x: list[float],
        history_y: list[float],
        color: str,
        label: str,
        zorder: float,
        linestyle: str = '-',
        linewidth: float = 1.5,
        alpha: float = 0.75,
        marker: str | None = None,
    ) -> None:
        if not history_x:
            return
        plot_kwargs = {
            'color': color,
            'linewidth': linewidth,
            'alpha': alpha,
            'linestyle': linestyle,
            'label': label,
            'zorder': zorder,
        }
        if marker is not None:
            plot_kwargs.update({
                'marker': marker,
                'markersize': 2.8,
                'markevery': max(1, len(history_x) // 45),
            })
        self.ax_map.plot(history_x, history_y, **plot_kwargs)

    def draw_pose_marker(
        self,
        pose: tuple[float, float, float],
        color: str,
        edge_color: str,
        label: str,
        marker: str,
        zorder: float,
    ) -> None:
        x, y, yaw = pose
        self.ax_map.scatter(
            [x],
            [y],
            s=105,
            color=color,
            edgecolor=edge_color,
            linewidth=1.4,
            marker=marker,
            label=label,
            zorder=zorder,
        )
        arrow_len = 0.38
        self.ax_map.arrow(
            x,
            y,
            arrow_len * math.cos(yaw),
            arrow_len * math.sin(yaw),
            width=0.026,
            head_width=0.14,
            head_length=0.16,
            length_includes_head=True,
            color=color,
            zorder=zorder + 0.1,
        )

    def lidar_wall_status_text(self) -> str:
        if self.latest_lidar_wall_receive_sec is None:
            return 'LiDAR wall: not used'
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        age_sec = max(0.0, now_sec - self.latest_lidar_wall_receive_sec)
        if age_sec <= self.lidar_wall_active_timeout_sec:
            return (
                f'LiDAR wall: USED ({age_sec:.1f}s ago, '
                f'{self.lidar_wall_update_count} updates)'
            )
        return (
            f'LiDAR wall: not used '
            f'(last {age_sec:.1f}s ago, {self.lidar_wall_update_count} updates)'
        )

    def lidar_scanmatch_status_text(self) -> str:
        if self.latest_lidar_scanmatch_receive_sec is None:
            return 'LiDAR scan-map: waiting'
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        age_sec = max(0.0, now_sec - self.latest_lidar_scanmatch_receive_sec)
        fresh = 'live' if age_sec <= 2.5 else 'stale'
        return (
            f'LiDAR scan-map: {fresh} '
            f'({age_sec:.1f}s, {self.lidar_scanmatch_update_count} updates)'
        )

    def draw_panel(self) -> None:
        ax = self.ax_panel
        ax.set_axis_off()
        ax.set_xlim(0.0, 1.0)
        ax.set_ylim(0.0, 1.0)

        active_path = self.active_path()
        route_len = (
            route_length([(f'p{i}', x, y) for i, (x, y) in enumerate(active_path)])
            if len(active_path) >= 2
            else float('nan')
        )
        route_len_text = f'{route_len:.2f} m' if math.isfinite(route_len) else 'n/a'
        lines = [
            'Live Status',
            '',
            f'path: {self.path_topic}',
            f'path source: {"ROS" if self.received_path else "waiting"}',
            f'route length: {route_len_text}',
            '',
            f'fused: {self.odom_topic}',
            f'wheel: {self.wheel_odom_topic}',
            f'CV: {self.vision_odom_topic}',
            f'LiDAR map: {self.lidar_scanmatch_odom_topic}',
            f'fused pts: {len(self.history_x)}',
            f'wheel pts: {len(self.wheel_history_x)}',
            f'CV pts: {len(self.vision_history_x)}',
            f'LiDAR map pts: {len(self.lidar_scanmatch_history_x)}',
            f'LiDAR obstacles: {len(self.dynamic_obstacles)}',
            self.lidar_wall_status_text(),
            self.lidar_scanmatch_status_text(),
            '',
        ]

        if self.latest_exploration_summary is None:
            lines.extend([
                'exploration: waiting',
                f'explore topic: {self.exploration_summary_topic}',
                '',
            ])
        else:
            summary = self.latest_exploration_summary
            coverage = self.status_float(summary.get('coverage_percent'))
            marker_count = len(summary.get('revealed_markers') or [])
            confirmed_count = len(summary.get('confirmed_plants') or [])
            unknown_count = len(summary.get('unknown_plants') or [])
            frontier_count = int(summary.get('frontier_count') or 0)
            if self.latest_exploration_summary_receive_sec is None:
                age_sec = 0.0
            else:
                now_sec = self.get_clock().now().nanoseconds * 1.0e-9
                age_sec = max(0.0, now_sec - self.latest_exploration_summary_receive_sec)
            coverage_text = f'{coverage:.1f}%' if math.isfinite(coverage) else 'n/a'
            lines.extend([
                f'exploration: {coverage_text} ({age_sec:.1f}s)',
                f'revealed markers: {marker_count}',
                f'YOLO plants: known {confirmed_count}, unknown {unknown_count}',
                f'frontiers: {frontier_count}',
                '',
            ])

        if self.latest_system_status is None:
            lines.extend([
                f'Jetson status: waiting',
                f'status topic: {self.system_status_topic}',
                '',
            ])
        else:
            status = self.latest_system_status
            hostname = str(status.get('hostname') or 'jetson')
            age_sec = 0.0
            if self.latest_system_status_receive_sec is not None:
                now_sec = self.get_clock().now().nanoseconds * 1.0e-9
                age_sec = max(0.0, now_sec - self.latest_system_status_receive_sec)
            fresh_text = 'stale' if age_sec > 3.0 else 'live'
            cpu = self.status_line('CPU', status.get('cpu_percent'), '%')
            mem = self.status_line('mem', status.get('memory_percent'), '%')
            temp = self.status_line('temp', status.get('temperature_c'), ' C')
            disk = self.status_line('disk', status.get('disk_percent'), '%')
            load_1min = self.status_float(status.get('load_1min'))
            load_text = f'load1: {load_1min:.2f}' if math.isfinite(load_1min) else 'load1: n/a'
            lines.extend([
                f'Jetson: {hostname} ({fresh_text}, {age_sec:.1f}s)',
                f'{cpu}   {mem}',
                f'{temp}   {load_text}',
                disk,
                '',
            ])

        if self.latest_pose is None:
            lines.extend(['waiting for odom...', '', f'output:', str(self.run_dir)])
        else:
            x, y, yaw = self.latest_pose
            error = self.distance_to_path(x, y)
            t = self.history_t[-1] if self.history_t else 0.0
            lines.extend([
                f'time: {t:.1f} s',
                f'x: {x:.3f} m',
                f'y: {y:.3f} m',
                f'yaw: {math.degrees(yaw):+.1f} deg',
                f'path error: {error:.3f} m' if math.isfinite(error) else 'path error: n/a',
                '',
                f'output:',
                str(self.run_dir),
            ])

        y = 0.96
        normal_step = min(0.050, 0.92 / max(1, len(lines)))
        blank_step = min(0.030, normal_step * 0.72)
        for idx, line in enumerate(lines):
            if idx == 0:
                ax.text(0.02, y, line, fontsize=12, weight='bold', va='top')
            elif line == str(self.run_dir):
                ax.text(0.02, y, line, fontsize=7.2, va='top', wrap=True)
            else:
                ax.text(0.02, y, line, fontsize=8.8, va='top')
            y -= normal_step if line else blank_step

    def destroy_node(self) -> bool:
        try:
            self.draw_plot(save_path=self.final_png_path)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not save final live map: {exc}')
        try:
            self.trace_file.close()
        except Exception:
            pass
        try:
            self.system_status_file.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveGlobalMapViewer()

    def request_shutdown(_signum, _frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
