#!/usr/bin/env python3

import json
import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

from asclinic_pkg.msg import LeftRightFloat32, MissionProgress, ServoPulseWidth


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def quaternion_to_yaw(q) -> float:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1.0e-9:
        return 0.0
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class FogInspectionMotorFilter(Node):
    """Low-priority motion hold for nearby unexplored free-space frontier."""

    UNKNOWN = -1
    FREE = 0

    def __init__(self) -> None:
        super().__init__('fog_inspection_motor_filter')

        self.declare_parameter('input_cmd_topic', 'set_motor_duty_cycle_raw')
        self.declare_parameter('output_cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('odom_topic', 'pitt_fused_odometry')
        self.declare_parameter('explored_map_topic', 'explored_map')
        self.declare_parameter('mission_progress_topic', 'mission_progress')
        self.declare_parameter(
            'plant_positive_save_gate_topic',
            'plant_detector/save_positive_enabled',
        )
        self.declare_parameter('camera_pan_topic', 'set_servo_pulse_width')
        self.declare_parameter('status_topic', 'fog_inspection/status')

        self.declare_parameter('enabled', True)
        self.declare_parameter('slow_scale', 0.30)
        self.declare_parameter('forward_command_threshold_duty', 2.0)
        self.declare_parameter('forward_turn_ratio_limit', 1.50)
        self.declare_parameter('attention_min_distance_m', 0.45)
        self.declare_parameter('attention_max_distance_m', 2.50)
        self.declare_parameter('attention_half_angle_deg', 105.0)
        self.declare_parameter('hold_trigger_distance_m', 1.20)
        self.declare_parameter('slow_before_hold_sec', 0.40)
        self.declare_parameter('max_slow_before_hold_sec', 1.50)
        self.declare_parameter('pre_hold_decel_enabled', True)
        self.declare_parameter('pre_hold_decel_scale', 0.70)
        self.declare_parameter('pre_hold_decel_duration_sec', 0.40)
        self.declare_parameter('hold_duration_sec', 1.80)
        self.declare_parameter('cooldown_sec', 1.00)
        self.declare_parameter('min_frontier_area_m2', 0.04)
        self.declare_parameter('blocked_cost_threshold', 80)
        self.declare_parameter('map_timeout_sec', 1.00)
        self.declare_parameter('odom_timeout_sec', 1.00)
        self.declare_parameter('mission_progress_timeout_sec', 1.00)
        self.declare_parameter('positive_save_timeout_sec', 1.00)
        self.declare_parameter('camera_command_period_sec', 0.10)
        self.declare_parameter('status_log_period_sec', 1.00)
        self.declare_parameter('publish_status', True)

        self.declare_parameter('camera_pan_channel', 14)
        self.declare_parameter('camera_pan_center_us', 1548)
        self.declare_parameter('camera_pan_min_us', 500)
        self.declare_parameter('camera_pan_max_us', 2500)
        self.declare_parameter('camera_pan_max_angle_deg', 180.0)
        self.declare_parameter('fog_camera_pan_max_angle_deg', 0.0)
        self.declare_parameter('camera_pan_left_positive_is_decreasing', False)

        self.input_cmd_topic = str(self.get_parameter('input_cmd_topic').value)
        self.output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.explored_map_topic = str(self.get_parameter('explored_map_topic').value)
        self.mission_progress_topic = str(
            self.get_parameter('mission_progress_topic').value
        )
        self.plant_positive_save_gate_topic = str(
            self.get_parameter('plant_positive_save_gate_topic').value
        ).strip()
        self.camera_pan_topic = str(self.get_parameter('camera_pan_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value).strip()

        self.enabled = as_bool(self.get_parameter('enabled').value)
        self.slow_scale = self._clamp(
            float(self.get_parameter('slow_scale').value),
            0.0,
            1.0,
        )
        self.forward_command_threshold_duty = max(
            0.0,
            float(self.get_parameter('forward_command_threshold_duty').value),
        )
        self.forward_turn_ratio_limit = max(
            0.0,
            float(self.get_parameter('forward_turn_ratio_limit').value),
        )
        self.attention_min_distance_m = max(
            0.0,
            float(self.get_parameter('attention_min_distance_m').value),
        )
        self.attention_max_distance_m = max(
            self.attention_min_distance_m,
            float(self.get_parameter('attention_max_distance_m').value),
        )
        self.attention_half_angle = math.radians(
            max(0.0, float(self.get_parameter('attention_half_angle_deg').value))
        )
        self.hold_trigger_distance_m = max(
            0.0,
            float(self.get_parameter('hold_trigger_distance_m').value),
        )
        self.slow_before_hold_sec = max(
            0.0,
            float(self.get_parameter('slow_before_hold_sec').value),
        )
        self.max_slow_before_hold_sec = max(
            self.slow_before_hold_sec,
            float(self.get_parameter('max_slow_before_hold_sec').value),
        )
        self.pre_hold_decel_enabled = as_bool(
            self.get_parameter('pre_hold_decel_enabled').value
        )
        self.pre_hold_decel_scale = self._clamp(
            float(self.get_parameter('pre_hold_decel_scale').value),
            0.0,
            1.0,
        )
        self.pre_hold_decel_duration_sec = max(
            0.0,
            float(self.get_parameter('pre_hold_decel_duration_sec').value),
        )
        self.hold_duration_sec = max(
            0.0,
            float(self.get_parameter('hold_duration_sec').value),
        )
        self.cooldown_sec = max(0.0, float(self.get_parameter('cooldown_sec').value))
        self.min_frontier_area_m2 = max(
            0.0,
            float(self.get_parameter('min_frontier_area_m2').value),
        )
        self.blocked_cost_threshold = int(
            self.get_parameter('blocked_cost_threshold').value
        )
        self.map_timeout_sec = max(0.0, float(self.get_parameter('map_timeout_sec').value))
        self.odom_timeout_sec = max(0.0, float(self.get_parameter('odom_timeout_sec').value))
        self.mission_progress_timeout_sec = max(
            0.0,
            float(self.get_parameter('mission_progress_timeout_sec').value),
        )
        self.positive_save_timeout_sec = max(
            0.0,
            float(self.get_parameter('positive_save_timeout_sec').value),
        )
        self.camera_command_period_sec = max(
            0.02,
            float(self.get_parameter('camera_command_period_sec').value),
        )
        self.status_log_period_sec = max(
            0.10,
            float(self.get_parameter('status_log_period_sec').value),
        )
        self.publish_status = as_bool(self.get_parameter('publish_status').value)

        self.camera_pan_channel = int(self.get_parameter('camera_pan_channel').value)
        self.camera_pan_center_us = int(self.get_parameter('camera_pan_center_us').value)
        self.camera_pan_min_us = int(self.get_parameter('camera_pan_min_us').value)
        self.camera_pan_max_us = int(self.get_parameter('camera_pan_max_us').value)
        if self.camera_pan_min_us > self.camera_pan_max_us:
            self.camera_pan_min_us, self.camera_pan_max_us = (
                self.camera_pan_max_us,
                self.camera_pan_min_us,
            )
        self.camera_pan_center_us = int(
            self._clamp(
                float(self.camera_pan_center_us),
                float(self.camera_pan_min_us),
                float(self.camera_pan_max_us),
            )
        )
        self.camera_pan_max_angle = math.radians(
            max(1.0, float(self.get_parameter('camera_pan_max_angle_deg').value))
        )
        fog_pan_limit_deg = float(self.get_parameter('fog_camera_pan_max_angle_deg').value)
        self.fog_camera_pan_max_angle = self.camera_pan_max_angle
        if fog_pan_limit_deg > 0.0:
            self.fog_camera_pan_max_angle = min(
                self.camera_pan_max_angle,
                math.radians(fog_pan_limit_deg),
            )
        self.camera_pan_left_positive_is_decreasing = as_bool(
            self.get_parameter('camera_pan_left_positive_is_decreasing').value
        )

        self.latest_grid: np.ndarray | None = None
        self.latest_info = None
        self.latest_map_time_sec: float | None = None
        self.latest_pose: tuple[float, float, float] | None = None
        self.latest_odom_time_sec: float | None = None
        self.mission_dwell_active = False
        self.latest_mission_time_sec: float | None = None
        self.positive_save_active = False
        self.latest_positive_save_time_sec: float | None = None

        self.state = 'IDLE'
        self.state_started_sec = 0.0
        self.pre_hold_until_sec = 0.0
        self.hold_until_sec = 0.0
        self.cooldown_until_sec = 0.0
        self.last_camera_command_sec = 0.0
        self.last_status_log_sec = 0.0
        self.last_status_publish_sec = 0.0
        self.last_target_summary = ''
        self.output_seq_num = 1

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10,
        )
        self.cmd_pub = self.create_publisher(
            LeftRightFloat32,
            self.output_cmd_topic,
            reliable_qos,
        )
        self.camera_pan_pub = self.create_publisher(
            ServoPulseWidth,
            self.camera_pan_topic,
            reliable_qos,
        )
        self.status_pub = None
        if self.publish_status and self.status_topic:
            self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(
            LeftRightFloat32,
            self.input_cmd_topic,
            self.cmd_callback,
            reliable_qos,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            reliable_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            self.explored_map_topic,
            self.map_callback,
            10,
        )
        self.create_subscription(
            MissionProgress,
            self.mission_progress_topic,
            self.mission_progress_callback,
            reliable_qos,
        )
        if self.plant_positive_save_gate_topic:
            self.create_subscription(
                Bool,
                self.plant_positive_save_gate_topic,
                self.positive_save_callback,
                reliable_qos,
            )

        self.get_logger().info(
            '[FOG INSPECTION] input=%s output=%s map=%s odom=%s camera=%s enabled=%s'
            % (
                self.input_cmd_topic,
                self.output_cmd_topic,
                self.explored_map_topic,
                self.odom_topic,
                self.camera_pan_topic,
                self.enabled,
            )
        )
        self.get_logger().info(
            '[FOG INSPECTION] slow_scale=%.2f pre_hold=%s/%.2fx/%.2fs hold=%.2fs cooldown=%.2fs distance=[%.2f, %.2f]m half_angle=%.1fdeg pan_period=%.2fs'
            % (
                self.slow_scale,
                self.pre_hold_decel_enabled,
                self.pre_hold_decel_scale,
                self.pre_hold_decel_duration_sec,
                self.hold_duration_sec,
                self.cooldown_sec,
                self.attention_min_distance_m,
                self.attention_max_distance_m,
                math.degrees(self.attention_half_angle),
                self.camera_command_period_sec,
            )
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def map_callback(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            self.get_logger().warn('[FOG INSPECTION] Ignoring malformed explored map.')
            return
        self.latest_grid = np.array(msg.data, dtype=np.int16).reshape((height, width))
        self.latest_info = msg.info
        self.latest_map_time_sec = self.now_sec()

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quaternion_to_yaw(msg.pose.pose.orientation),
        )
        self.latest_odom_time_sec = self.now_sec()

    def mission_progress_callback(self, msg: MissionProgress) -> None:
        self.mission_dwell_active = bool(msg.dwell_active)
        self.latest_mission_time_sec = self.now_sec()

    def positive_save_callback(self, msg: Bool) -> None:
        self.positive_save_active = bool(msg.data)
        self.latest_positive_save_time_sec = self.now_sec()

    def cmd_callback(self, msg: LeftRightFloat32) -> None:
        now = self.now_sec()
        if not self.enabled:
            self.publish_command(msg.left, msg.right)
            return

        priority_reason = self.capture_priority_reason(now)
        if priority_reason is not None:
            self.release('plant/capture priority: %s' % priority_reason, now)
            self.publish_command(msg.left, msg.right)
            return

        if not self.forward_motion_command(msg):
            self.release('raw command not forward motion', now)
            self.publish_command(msg.left, msg.right)
            return

        stale_reason = self.stale_input_reason(now)
        if stale_reason is not None:
            self.release(stale_reason, now)
            self.publish_command(msg.left, msg.right)
            return

        if self.state == 'COOLDOWN' and now < self.cooldown_until_sec:
            self.publish_command(msg.left, msg.right)
            return

        target = self.find_fog_target()
        if target is None:
            self.release('no nearby frontier fog', now)
            self.publish_command(msg.left, msg.right)
            return

        self.publish_camera_pan(target['bearing'], now)

        if self.state not in ('PRE_HOLD_DECEL', 'HOLD'):
            if self.state != 'SLOW':
                self.enter_state('SLOW', now, target)
            elif self.should_enter_hold(target, now):
                self.enter_pre_hold_or_hold(now, target)

        if self.state == 'PRE_HOLD_DECEL':
            if now < self.pre_hold_until_sec:
                left, right = self.scale_forward_command(
                    msg.left,
                    msg.right,
                    self.pre_hold_decel_scale,
                )
                self.publish_status_message(now, target)
                self.publish_command(left, right)
                return
            self.enter_state('HOLD', now, target)

        if self.state == 'HOLD':
            if now < self.hold_until_sec:
                self.publish_status_message(now, target)
                self.publish_command(0.0, 0.0)
                return
            self.enter_state('COOLDOWN', now, target)
            self.publish_command(msg.left, msg.right)
            return

        left, right = self.scale_forward_command(msg.left, msg.right)
        self.publish_status_message(now, target)
        self.publish_command(left, right)

    def capture_priority_reason(self, now: float) -> str | None:
        if (
            self.mission_dwell_active
            and self.latest_mission_time_sec is not None
            and (
                self.mission_progress_timeout_sec <= 0.0
                or now - self.latest_mission_time_sec
                <= self.mission_progress_timeout_sec
            )
        ):
            return 'mission dwell active'
        if (
            self.positive_save_active
            and self.latest_positive_save_time_sec is not None
            and (
                self.positive_save_timeout_sec <= 0.0
                or now - self.latest_positive_save_time_sec
                <= self.positive_save_timeout_sec
            )
        ):
            return 'positive image save active'
        return None

    def stale_input_reason(self, now: float) -> str | None:
        if self.latest_grid is None or self.latest_info is None:
            return 'waiting for explored_map'
        if self.latest_pose is None:
            return 'waiting for odometry'
        if (
            self.map_timeout_sec > 0.0
            and self.latest_map_time_sec is not None
            and now - self.latest_map_time_sec > self.map_timeout_sec
        ):
            return 'explored_map stale'
        if (
            self.odom_timeout_sec > 0.0
            and self.latest_odom_time_sec is not None
            and now - self.latest_odom_time_sec > self.odom_timeout_sec
        ):
            return 'odometry stale'
        return None

    def forward_motion_command(self, msg: LeftRightFloat32) -> bool:
        left = float(msg.left)
        right = float(msg.right)
        forward = 0.5 * (left + right)
        turn = 0.5 * (right - left)
        if left <= 0.0 or right <= 0.0:
            return False
        if forward <= self.forward_command_threshold_duty:
            return False
        if self.forward_turn_ratio_limit <= 0.0:
            return True
        return abs(turn) <= abs(forward) * self.forward_turn_ratio_limit

    def scale_forward_command(
        self,
        left: float,
        right: float,
        extra_scale: float = 1.0,
    ) -> tuple[float, float]:
        scale = self.slow_scale * self._clamp(float(extra_scale), 0.0, 1.0)
        return float(left) * scale, float(right) * scale

    def should_enter_hold(self, target: dict[str, float], now: float) -> bool:
        if self.hold_duration_sec <= 0.0:
            return False
        age = now - self.state_started_sec
        if age < self.slow_before_hold_sec:
            return False
        return (
            target['distance_m'] <= self.hold_trigger_distance_m
            or age >= self.max_slow_before_hold_sec
        )

    def enter_pre_hold_or_hold(
        self,
        now: float,
        target: dict[str, float],
    ) -> None:
        if (
            not self.pre_hold_decel_enabled
            or self.pre_hold_decel_duration_sec <= 0.0
            or self.pre_hold_decel_scale >= 1.0
        ):
            self.enter_state('HOLD', now, target)
            return
        self.enter_state('PRE_HOLD_DECEL', now, target)

    def enter_state(
        self,
        state: str,
        now: float,
        target: dict[str, float],
    ) -> None:
        if state == self.state:
            return
        self.state = state
        self.state_started_sec = now
        if state == 'PRE_HOLD_DECEL':
            self.pre_hold_until_sec = now + self.pre_hold_decel_duration_sec
            self.hold_until_sec = 0.0
        elif state == 'HOLD':
            self.pre_hold_until_sec = 0.0
            self.hold_until_sec = now + self.hold_duration_sec
        elif state == 'COOLDOWN':
            self.cooldown_until_sec = now + self.cooldown_sec
            self.pre_hold_until_sec = 0.0
            self.hold_until_sec = 0.0
        self.log_state_change(now, target)

    def release(self, reason: str, now: float) -> None:
        if self.state == 'IDLE':
            return
        self.state = 'IDLE'
        self.state_started_sec = now
        self.pre_hold_until_sec = 0.0
        self.hold_until_sec = 0.0
        self.cooldown_until_sec = 0.0
        if now - self.last_status_log_sec >= self.status_log_period_sec:
            self.last_status_log_sec = now
            self.get_logger().info('[FOG INSPECTION] Released: %s.' % reason)

    def log_state_change(self, now: float, target: dict[str, float]) -> None:
        if now - self.last_status_log_sec < self.status_log_period_sec:
            return
        self.last_status_log_sec = now
        self.get_logger().info(
            '[FOG INSPECTION] %s fog target bearing=%.1fdeg distance=%.2fm area=%.2fm2.'
            % (
                self.state,
                math.degrees(target['bearing']),
                target['distance_m'],
                target['area_m2'],
            )
        )

    def publish_status_message(
        self,
        now: float,
        target: dict[str, float],
    ) -> None:
        if self.status_pub is None:
            return
        if now - self.last_status_publish_sec < 0.25:
            return
        self.last_status_publish_sec = now
        msg = String()
        msg.data = json.dumps(
            {
                'state': self.state,
                'bearing_rad': target['bearing'],
                'bearing_deg': math.degrees(target['bearing']),
                'distance_m': target['distance_m'],
                'area_m2': target['area_m2'],
                'x': target['x'],
                'y': target['y'],
            },
            sort_keys=True,
        )
        self.status_pub.publish(msg)

    def publish_camera_pan(self, relative_bearing: float, now: float) -> None:
        if now - self.last_camera_command_sec < self.camera_command_period_sec:
            return
        limited_angle = self._clamp(
            relative_bearing,
            -self.fog_camera_pan_max_angle,
            self.fog_camera_pan_max_angle,
        )
        msg = ServoPulseWidth()
        msg.channel = int(self.camera_pan_channel)
        msg.pulse_width_in_microseconds = int(
            self.camera_angle_to_pan_us(limited_angle)
        )
        self.camera_pan_pub.publish(msg)
        self.last_camera_command_sec = now

    def camera_angle_to_pan_us(self, angle_left_positive: float) -> int:
        angle = (
            angle_left_positive
            if self.camera_pan_left_positive_is_decreasing
            else -angle_left_positive
        )
        ratio = min(abs(angle) / self.camera_pan_max_angle, 1.0)
        if angle >= 0.0:
            span = self.camera_pan_center_us - self.camera_pan_min_us
            pulse = self.camera_pan_center_us - ratio * span
        else:
            span = self.camera_pan_max_us - self.camera_pan_center_us
            pulse = self.camera_pan_center_us + ratio * span
        return int(round(self._clamp(pulse, self.camera_pan_min_us, self.camera_pan_max_us)))

    def find_fog_target(self) -> dict[str, float] | None:
        if self.latest_grid is None or self.latest_info is None or self.latest_pose is None:
            return None

        robot_x, robot_y, robot_yaw = self.latest_pose
        candidate_mask = self.frontier_candidate_mask(robot_x, robot_y, robot_yaw)
        if candidate_mask is None or not np.any(candidate_mask):
            return None

        clusters = self.connected_clusters(candidate_mask)
        if not clusters:
            return None

        resolution = float(self.latest_info.resolution)
        min_cells = max(1, int(math.ceil(self.min_frontier_area_m2 / (resolution ** 2))))
        best: dict[str, float] | None = None
        best_score = -float('inf')

        for cluster in clusters:
            if len(cluster) < min_cells:
                continue
            points = [self.cell_to_world(row, col) for row, col in cluster]
            target_x = sum(point[0] for point in points) / float(len(points))
            target_y = sum(point[1] for point in points) / float(len(points))
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.hypot(dx, dy)
            if distance <= 1.0e-6:
                continue
            bearing = wrap_to_pi(math.atan2(dy, dx) - robot_yaw)
            area = len(cluster) * resolution * resolution
            score = area / max(distance, 0.10)
            if score > best_score:
                best_score = score
                best = {
                    'x': target_x,
                    'y': target_y,
                    'bearing': bearing,
                    'distance_m': distance,
                    'area_m2': area,
                    'cell_count': float(len(cluster)),
                }

        return best

    def frontier_candidate_mask(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ) -> np.ndarray | None:
        grid = self.latest_grid
        if grid is None:
            return None

        unknown_mask = grid == self.UNKNOWN
        free_mask = grid == self.FREE
        if not np.any(unknown_mask) or not np.any(free_mask):
            return np.zeros(grid.shape, dtype=bool)

        neighbor_free = np.zeros(grid.shape, dtype=bool)
        neighbor_free[1:, :] |= free_mask[:-1, :]
        neighbor_free[:-1, :] |= free_mask[1:, :]
        neighbor_free[:, 1:] |= free_mask[:, :-1]
        neighbor_free[:, :-1] |= free_mask[:, 1:]
        frontier_mask = unknown_mask & neighbor_free

        rows, cols = np.where(frontier_mask)
        candidate_mask = np.zeros(grid.shape, dtype=bool)
        for row, col in zip(rows, cols):
            cell_x, cell_y = self.cell_to_world(int(row), int(col))
            dx = cell_x - robot_x
            dy = cell_y - robot_y
            distance = math.hypot(dx, dy)
            if (
                distance < self.attention_min_distance_m
                or distance > self.attention_max_distance_m
            ):
                continue
            bearing = wrap_to_pi(math.atan2(dy, dx) - robot_yaw)
            if abs(bearing) > self.attention_half_angle:
                continue
            if not self.line_of_sight_clear(robot_x, robot_y, cell_x, cell_y):
                continue
            candidate_mask[int(row), int(col)] = True
        return candidate_mask

    def connected_clusters(self, mask: np.ndarray) -> list[list[tuple[int, int]]]:
        height, width = mask.shape
        visited = np.zeros(mask.shape, dtype=bool)
        clusters: list[list[tuple[int, int]]] = []
        for row in range(height):
            for col in range(width):
                if visited[row, col] or not mask[row, col]:
                    continue
                cluster: list[tuple[int, int]] = []
                queue = deque([(row, col)])
                visited[row, col] = True
                while queue:
                    current_row, current_col = queue.popleft()
                    cluster.append((current_row, current_col))
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue
                            nr = current_row + dr
                            nc = current_col + dc
                            if (
                                0 <= nr < height
                                and 0 <= nc < width
                                and not visited[nr, nc]
                                and mask[nr, nc]
                            ):
                                visited[nr, nc] = True
                                queue.append((nr, nc))
                clusters.append(cluster)
        return clusters

    def line_of_sight_clear(
        self,
        source_x: float,
        source_y: float,
        target_x: float,
        target_y: float,
    ) -> bool:
        if self.latest_grid is None or self.latest_info is None:
            return False

        distance = math.hypot(target_x - source_x, target_y - source_y)
        if distance <= 1.0e-6:
            return True

        step_m = max(0.02, 0.5 * float(self.latest_info.resolution))
        steps = max(1, int(math.ceil(distance / step_m)))
        target_cell = self.world_to_cell(target_x, target_y)
        source_cell = self.world_to_cell(source_x, source_y)
        for step in range(1, steps + 1):
            ratio = min(1.0, step / steps)
            sample_x = source_x + ratio * (target_x - source_x)
            sample_y = source_y + ratio * (target_y - source_y)
            cell = self.world_to_cell(sample_x, sample_y)
            if cell is None:
                return False
            if cell == source_cell or cell == target_cell:
                continue
            row, col = cell
            if int(self.latest_grid[row, col]) >= self.blocked_cost_threshold:
                return False
        return True

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        if self.latest_info is None or self.latest_grid is None:
            return None
        resolution = float(self.latest_info.resolution)
        origin_x = float(self.latest_info.origin.position.x)
        origin_y = float(self.latest_info.origin.position.y)
        col = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))
        if not (0 <= row < self.latest_grid.shape[0] and 0 <= col < self.latest_grid.shape[1]):
            return None
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        resolution = float(self.latest_info.resolution)
        origin_x = float(self.latest_info.origin.position.x)
        origin_y = float(self.latest_info.origin.position.y)
        return (
            origin_x + (col + 0.5) * resolution,
            origin_y + (row + 0.5) * resolution,
        )

    def publish_command(self, left_cmd: float, right_cmd: float) -> None:
        msg = LeftRightFloat32()
        msg.left = float(left_cmd)
        msg.right = float(right_cmd)
        msg.seq_num = self.output_seq_num
        self.cmd_pub.publish(msg)
        self.output_seq_num += 1

    def _clamp(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FogInspectionMotorFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_command(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
