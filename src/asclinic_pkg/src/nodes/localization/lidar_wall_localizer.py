#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def wrap_line_angle_error(angle: float) -> float:
    return (angle + 0.5 * math.pi) % math.pi - 0.5 * math.pi


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rot2(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


@dataclass
class WallSpec:
    wall_id: str
    axis: str
    target: float
    along_min: float
    along_max: float
    expected_angle: float


@dataclass
class WallObservation:
    wall: WallSpec
    count: int
    centroid: np.ndarray
    direction: np.ndarray
    mean_position_correction: float
    yaw_correction: float
    residual_std: float
    span: float
    weight: float


class LidarWallLocalizer(Node):
    """Estimate conservative pose corrections from axis-aligned room walls."""

    def __init__(self) -> None:
        super().__init__('lidar_wall_localizer')

        self.declare_parameter('enabled', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', 'pitt_fused_odometry')
        self.declare_parameter('wall_odom_topic', 'pitt_lidar_wall_odometry')
        self.declare_parameter('debug_marker_topic', 'lidar_wall_debug')
        self.declare_parameter('frame_id', 'odom')

        self.declare_parameter('room_width_m', 15.0)
        self.declare_parameter('room_height_m', 10.8)
        self.declare_parameter('wall_margin_m', 0.20)
        self.declare_parameter('wall_update_radius_m', 2.0)
        self.declare_parameter('virtual_wall_ids', ['left'])

        self.declare_parameter('lidar_in_base_x', 0.0)
        self.declare_parameter('lidar_in_base_y', 0.0)
        self.declare_parameter('lidar_in_base_yaw_deg', 0.0)
        self.declare_parameter('lidar_scan_angle_multiplier', 1.0)
        self.declare_parameter('lidar_forward_is_backward', True)

        self.declare_parameter('beam_stride', 3)
        self.declare_parameter('min_range_m', 0.08)
        self.declare_parameter('max_range_m', 4.5)
        self.declare_parameter('association_distance_m', 0.45)
        self.declare_parameter('line_inlier_residual_m', 0.08)
        self.declare_parameter('max_fit_residual_std_m', 0.06)
        self.declare_parameter('min_wall_inliers', 25)
        self.declare_parameter('min_wall_span_m', 0.45)
        self.declare_parameter('max_line_angle_error_deg', 15.0)

        self.declare_parameter('update_period_sec', 1.0)
        self.declare_parameter('scan_timeout_sec', 0.40)
        self.declare_parameter('odom_timeout_sec', 0.50)
        self.declare_parameter('max_update_angular_speed_rad_s', 0.18)
        self.declare_parameter('post_turn_settle_sec', 0.45)
        self.declare_parameter('max_update_linear_speed_m_s', 0.30)
        self.declare_parameter('max_position_correction_m', 0.12)
        self.declare_parameter('max_yaw_correction_deg', 3.0)

        self.declare_parameter('position_variance_min', 0.010)
        self.declare_parameter('yaw_variance_min', 0.003)
        self.declare_parameter('unconstrained_variance', 100.0)
        self.declare_parameter('publish_debug_markers', True)
        self.declare_parameter('verbose', False)
        self.declare_parameter('skip_log_period_sec', 1.5)

        self.enabled = as_bool(self.get_parameter('enabled').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.wall_odom_topic = str(self.get_parameter('wall_odom_topic').value)
        self.debug_marker_topic = str(self.get_parameter('debug_marker_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.room_width_m = max(0.1, float(self.get_parameter('room_width_m').value))
        self.room_height_m = max(0.1, float(self.get_parameter('room_height_m').value))
        self.wall_margin_m = max(0.0, float(self.get_parameter('wall_margin_m').value))
        self.wall_update_radius_m = max(
            0.0,
            float(self.get_parameter('wall_update_radius_m').value),
        )
        self.virtual_wall_ids = self._parse_wall_ids(
            self.get_parameter('virtual_wall_ids').value
        )

        self.lidar_in_base_x = float(self.get_parameter('lidar_in_base_x').value)
        self.lidar_in_base_y = float(self.get_parameter('lidar_in_base_y').value)
        self.lidar_in_base_yaw = math.radians(
            float(self.get_parameter('lidar_in_base_yaw_deg').value)
        )
        self.lidar_scan_angle_multiplier = float(
            self.get_parameter('lidar_scan_angle_multiplier').value
        )
        self.lidar_forward_is_backward = as_bool(
            self.get_parameter('lidar_forward_is_backward').value
        )

        self.beam_stride = max(1, int(self.get_parameter('beam_stride').value))
        self.min_range_m = max(0.0, float(self.get_parameter('min_range_m').value))
        self.max_range_m = max(0.0, float(self.get_parameter('max_range_m').value))
        self.association_distance_m = max(
            0.02,
            float(self.get_parameter('association_distance_m').value),
        )
        self.line_inlier_residual_m = max(
            0.01,
            float(self.get_parameter('line_inlier_residual_m').value),
        )
        self.max_fit_residual_std_m = max(
            0.0,
            float(self.get_parameter('max_fit_residual_std_m').value),
        )
        self.min_wall_inliers = max(2, int(self.get_parameter('min_wall_inliers').value))
        self.min_wall_span_m = max(0.0, float(self.get_parameter('min_wall_span_m').value))
        self.max_line_angle_error = math.radians(
            abs(float(self.get_parameter('max_line_angle_error_deg').value))
        )

        self.update_period_sec = max(0.0, float(self.get_parameter('update_period_sec').value))
        self.scan_timeout_sec = max(0.0, float(self.get_parameter('scan_timeout_sec').value))
        self.odom_timeout_sec = max(0.0, float(self.get_parameter('odom_timeout_sec').value))
        self.max_update_angular_speed_rad_s = max(
            0.0,
            float(self.get_parameter('max_update_angular_speed_rad_s').value),
        )
        self.post_turn_settle_sec = max(
            0.0,
            float(self.get_parameter('post_turn_settle_sec').value),
        )
        self.max_update_linear_speed_m_s = max(
            0.0,
            float(self.get_parameter('max_update_linear_speed_m_s').value),
        )
        self.max_position_correction_m = max(
            0.0,
            float(self.get_parameter('max_position_correction_m').value),
        )
        self.max_yaw_correction = math.radians(
            max(0.0, float(self.get_parameter('max_yaw_correction_deg').value))
        )

        self.position_variance_min = max(
            1.0e-6,
            float(self.get_parameter('position_variance_min').value),
        )
        self.yaw_variance_min = max(
            1.0e-6,
            float(self.get_parameter('yaw_variance_min').value),
        )
        self.unconstrained_variance = max(
            1.0,
            float(self.get_parameter('unconstrained_variance').value),
        )
        self.publish_debug_markers = as_bool(
            self.get_parameter('publish_debug_markers').value
        )
        self.verbose = as_bool(self.get_parameter('verbose').value)
        self.skip_log_period_sec = max(
            0.1,
            float(self.get_parameter('skip_log_period_sec').value),
        )

        all_walls = self._build_room_walls()
        self.walls = [
            wall for wall in all_walls
            if wall.wall_id not in self.virtual_wall_ids
        ]
        self.latest_pose: Optional[np.ndarray] = None
        self.latest_odom_receive_sec: Optional[float] = None
        self.last_odom_pose: Optional[np.ndarray] = None
        self.last_odom_time_sec: Optional[float] = None
        self.linear_speed_m_s = 0.0
        self.angular_speed_rad_s = 0.0
        self.last_turn_time_sec: Optional[float] = None
        self.last_update_sec: Optional[float] = None
        self.last_skip_log_sec = 0.0

        self.wall_odom_pub = self.create_publisher(Odometry, self.wall_odom_topic, 10)
        self.debug_pub = self.create_publisher(MarkerArray, self.debug_marker_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info('==================================================')
        self.get_logger().info('[LIDAR WALL] Node started')
        self.get_logger().info(f'[LIDAR WALL] enabled            = {self.enabled}')
        self.get_logger().info(f'[LIDAR WALL] scan_topic         = {self.scan_topic}')
        self.get_logger().info(f'[LIDAR WALL] odom_topic         = {self.odom_topic}')
        self.get_logger().info(f'[LIDAR WALL] wall_odom_topic    = {self.wall_odom_topic}')
        self.get_logger().info(
            f'[LIDAR WALL] room               = '
            f'{self.room_width_m:.2f}m x {self.room_height_m:.2f}m'
        )
        self.get_logger().info(
            f'[LIDAR WALL] real walls         = '
            f'{",".join(wall.wall_id for wall in self.walls) or "none"} '
            f'(within {self.wall_update_radius_m:.2f}m)'
        )
        self.get_logger().info(
            f'[LIDAR WALL] motion gate        = '
            f'|w|<={self.max_update_angular_speed_rad_s:.2f}rad/s, '
            f'settle={self.post_turn_settle_sec:.2f}s'
        )
        self.get_logger().info('==================================================')

    def now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1.0e-9

    def stamp_to_sec(self, stamp) -> Optional[float]:
        sec = int(stamp.sec)
        nsec = int(stamp.nanosec)
        if sec == 0 and nsec == 0:
            return None
        return float(sec) + float(nsec) * 1.0e-9

    def _parse_wall_ids(self, value) -> set:
        if isinstance(value, str):
            raw_items = value.replace(';', ',').split(',')
        else:
            try:
                raw_items = list(value)
            except TypeError:
                raw_items = []
        return {
            str(item).strip().lower()
            for item in raw_items
            if str(item).strip()
        }

    def _build_room_walls(self) -> List[WallSpec]:
        return [
            WallSpec('left', 'x', 0.0, 0.0, self.room_height_m, 0.5 * math.pi),
            WallSpec('right', 'x', self.room_width_m, 0.0, self.room_height_m, 0.5 * math.pi),
            WallSpec('bottom', 'y', 0.0, 0.0, self.room_width_m, 0.0),
            WallSpec('top', 'y', self.room_height_m, 0.0, self.room_width_m, 0.0),
        ]

    def odom_callback(self, msg: Odometry) -> None:
        now = self.now_sec()
        stamp_sec = self.stamp_to_sec(msg.header.stamp) or now
        pose = np.array(
            [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                quaternion_to_yaw(msg.pose.pose.orientation),
            ],
            dtype=float,
        )
        if not np.isfinite(pose).all():
            return

        twist_v = math.hypot(
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )
        twist_w = abs(float(msg.twist.twist.angular.z))
        use_twist = math.isfinite(twist_v) and math.isfinite(twist_w) and (
            twist_v > 1.0e-5 or twist_w > 1.0e-5
        )

        if self.last_odom_pose is not None and self.last_odom_time_sec is not None:
            dt = max(1.0e-3, stamp_sec - self.last_odom_time_sec)
            delta_xy = pose[:2] - self.last_odom_pose[:2]
            delta_yaw = wrap_angle(float(pose[2] - self.last_odom_pose[2]))
            computed_v = float(np.linalg.norm(delta_xy)) / dt
            computed_w = abs(delta_yaw) / dt
            self.linear_speed_m_s = twist_v if use_twist else computed_v
            self.angular_speed_rad_s = twist_w if use_twist else computed_w
        else:
            self.linear_speed_m_s = twist_v if use_twist else 0.0
            self.angular_speed_rad_s = twist_w if use_twist else 0.0

        if self.angular_speed_rad_s > self.max_update_angular_speed_rad_s:
            self.last_turn_time_sec = now

        self.latest_pose = pose
        self.latest_odom_receive_sec = now
        self.last_odom_pose = pose
        self.last_odom_time_sec = stamp_sec

    def scan_callback(self, msg: LaserScan) -> None:
        if not self.enabled:
            return
        now = self.now_sec()
        if not self._update_due(now):
            return
        if self.latest_pose is None or self.latest_odom_receive_sec is None:
            self._log_skip(now, 'waiting for odometry')
            return
        if now - self.latest_odom_receive_sec > self.odom_timeout_sec:
            self._log_skip(now, 'odometry is stale')
            return
        scan_stamp = self.stamp_to_sec(msg.header.stamp)
        if scan_stamp is not None and now - scan_stamp > self.scan_timeout_sec:
            self._log_skip(now, 'scan is stale')
            return
        if self.angular_speed_rad_s > self.max_update_angular_speed_rad_s:
            self.last_turn_time_sec = now
            self._log_skip(
                now,
                f'rotating |w|={self.angular_speed_rad_s:.2f}rad/s',
            )
            return
        if (
            self.last_turn_time_sec is not None
            and now - self.last_turn_time_sec < self.post_turn_settle_sec
        ):
            self._log_skip(now, 'waiting after rotation')
            return
        if (
            self.max_update_linear_speed_m_s > 0.0
            and self.linear_speed_m_s > self.max_update_linear_speed_m_s
        ):
            self._log_skip(
                now,
                f'moving too fast v={self.linear_speed_m_s:.2f}m/s',
            )
            return
        if not self._is_near_real_wall():
            self._log_skip(
                now,
                f'not within {self.wall_update_radius_m:.1f}m of a real wall',
            )
            self._publish_debug([], msg.header.stamp)
            return

        points_base = self._scan_to_base_points(msg)
        if points_base.shape[0] < self.min_wall_inliers:
            self._log_skip(now, f'not enough scan points ({points_base.shape[0]})')
            self._publish_debug([], msg.header.stamp)
            return

        robot_x, robot_y, robot_yaw = self.latest_pose
        points_world = self._base_to_world(points_base, robot_x, robot_y, robot_yaw)
        observations = self._extract_wall_observations(points_world)
        if not observations:
            self._log_skip(now, 'no stable wall line')
            self._publish_debug([], msg.header.stamp)
            return

        measurement = self._build_pose_measurement(observations)
        if measurement is None:
            self._log_skip(now, 'wall correction was unconstrained')
            self._publish_debug(observations, msg.header.stamp)
            return

        pose_xy, pose_yaw, cov = measurement
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = float(pose_xy[0])
        out.pose.pose.position.y = float(pose_xy[1])
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation = yaw_to_quaternion(pose_yaw)
        out.pose.covariance = cov.reshape(-1).tolist()
        self.wall_odom_pub.publish(out)
        self._publish_debug(observations, msg.header.stamp)
        self.last_update_sec = now

        if self.verbose:
            ids = ','.join(obs.wall.wall_id for obs in observations)
            correction = pose_xy - self.latest_pose[:2]
            self.get_logger().info(
                f'[LIDAR WALL] update walls=[{ids}] '
                f'corr=({correction[0]:+.3f},{correction[1]:+.3f})m '
                f'yaw={math.degrees(wrap_angle(pose_yaw - robot_yaw)):+.2f}deg '
                f'n={sum(obs.count for obs in observations)}'
            )

    def _update_due(self, now: float) -> bool:
        if self.last_update_sec is None:
            return True
        if self.update_period_sec <= 0.0:
            return True
        return now - self.last_update_sec >= self.update_period_sec

    def _scan_to_base_points(self, msg: LaserScan) -> np.ndarray:
        if msg.angle_increment == 0.0:
            return np.zeros((0, 2), dtype=float)

        range_min = max(float(msg.range_min), self.min_range_m)
        range_max = float(msg.range_max) if msg.range_max > 0.0 else self.max_range_m
        if self.max_range_m > 0.0:
            range_max = min(range_max, self.max_range_m)

        points: List[Tuple[float, float]] = []
        forward_sign = -1.0 if self.lidar_forward_is_backward else 1.0
        c_yaw = math.cos(self.lidar_in_base_yaw)
        s_yaw = math.sin(self.lidar_in_base_yaw)

        for index, distance in enumerate(msg.ranges):
            if index % self.beam_stride != 0:
                continue
            distance = float(distance)
            if not math.isfinite(distance):
                continue
            if distance <= range_min or distance > range_max:
                continue

            scan_angle = self.lidar_scan_angle_multiplier * (
                float(msg.angle_min) + index * float(msg.angle_increment)
            )
            sensor_x = forward_sign * distance * math.cos(scan_angle)
            sensor_y = distance * math.sin(scan_angle)
            base_x = self.lidar_in_base_x + c_yaw * sensor_x - s_yaw * sensor_y
            base_y = self.lidar_in_base_y + s_yaw * sensor_x + c_yaw * sensor_y
            points.append((base_x, base_y))

        if not points:
            return np.zeros((0, 2), dtype=float)
        return np.array(points, dtype=float)

    def _is_near_real_wall(self) -> bool:
        if self.wall_update_radius_m <= 0.0:
            return True
        if self.latest_pose is None:
            return False

        robot_x, robot_y, _robot_yaw = self.latest_pose
        pose = np.array([robot_x, robot_y], dtype=float)
        for wall in self.walls:
            axis_index = 0 if wall.axis == 'x' else 1
            along_index = 1 - axis_index
            along = float(pose[along_index])
            if (
                along < wall.along_min - self.wall_margin_m
                or along > wall.along_max + self.wall_margin_m
            ):
                continue
            distance = abs(float(pose[axis_index]) - wall.target)
            if distance <= self.wall_update_radius_m:
                return True
        return False

    def _base_to_world(
        self,
        points_base: np.ndarray,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ) -> np.ndarray:
        return points_base @ rot2(robot_yaw).T + np.array([robot_x, robot_y], dtype=float)

    def _extract_wall_observations(self, points_world: np.ndarray) -> List[WallObservation]:
        observations: List[WallObservation] = []
        for wall in self.walls:
            axis_index = 0 if wall.axis == 'x' else 1
            along_index = 1 - axis_index
            wall_distance = np.abs(points_world[:, axis_index] - wall.target)
            along = points_world[:, along_index]
            mask = (
                (wall_distance <= self.association_distance_m)
                & (along >= wall.along_min - self.wall_margin_m)
                & (along <= wall.along_max + self.wall_margin_m)
            )
            wall_points = points_world[mask]
            obs = self._fit_wall(wall, wall_points)
            if obs is not None:
                observations.append(obs)
        observations.sort(key=lambda obs: obs.weight, reverse=True)
        return observations

    def _fit_wall(
        self,
        wall: WallSpec,
        points: np.ndarray,
    ) -> Optional[WallObservation]:
        if points.shape[0] < self.min_wall_inliers:
            return None

        first_fit = self._fit_line_svd(points)
        if first_fit is None:
            return None
        centroid, direction, residuals, projections = first_fit
        inliers = residuals <= self.line_inlier_residual_m
        if int(np.count_nonzero(inliers)) < self.min_wall_inliers:
            return None

        fit = self._fit_line_svd(points[inliers])
        if fit is None:
            return None
        centroid, direction, residuals, projections = fit
        span = float(np.max(projections) - np.min(projections)) if projections.size else 0.0
        if span < self.min_wall_span_m:
            return None

        residual_std = float(np.std(residuals)) if residuals.size else 0.0
        if self.max_fit_residual_std_m > 0.0 and residual_std > self.max_fit_residual_std_m:
            return None

        line_angle = math.atan2(float(direction[1]), float(direction[0]))
        angle_error = wrap_line_angle_error(line_angle - wall.expected_angle)
        if abs(angle_error) > self.max_line_angle_error:
            return None

        axis_index = 0 if wall.axis == 'x' else 1
        mean_coord = float(np.mean(points[inliers, axis_index]))
        mean_position_correction = wall.target - mean_coord
        yaw_correction = -angle_error
        count = int(points[inliers].shape[0])
        weight = float(count) * max(0.05, span) / max(0.01, residual_std + 0.01)

        return WallObservation(
            wall=wall,
            count=count,
            centroid=centroid,
            direction=direction,
            mean_position_correction=mean_position_correction,
            yaw_correction=yaw_correction,
            residual_std=residual_std,
            span=span,
            weight=weight,
        )

    def _fit_line_svd(
        self,
        points: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
        if points.shape[0] < 2:
            return None
        centroid = np.mean(points, axis=0)
        centered = points - centroid
        try:
            _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        direction = np.array(vh[0], dtype=float)
        norm = float(np.linalg.norm(direction))
        if norm <= 1.0e-9:
            return None
        direction = direction / norm
        normal = np.array([-direction[1], direction[0]], dtype=float)
        residuals = np.abs(centered @ normal)
        projections = centered @ direction
        return centroid, direction, residuals, projections

    def _build_pose_measurement(
        self,
        observations: List[WallObservation],
    ) -> Optional[Tuple[np.ndarray, float, np.ndarray]]:
        if self.latest_pose is None:
            return None

        pose_xy = np.array(self.latest_pose[:2], dtype=float)
        pose_yaw = float(self.latest_pose[2])
        cov = np.zeros((6, 6), dtype=float)
        cov[0, 0] = self.unconstrained_variance
        cov[1, 1] = self.unconstrained_variance
        cov[5, 5] = self.unconstrained_variance

        grouped: Dict[str, List[WallObservation]] = {'x': [], 'y': []}
        yaw_obs: List[WallObservation] = []
        for obs in observations:
            grouped[obs.wall.axis].append(obs)
            yaw_obs.append(obs)

        constrained = False
        for axis, index in (('x', 0), ('y', 1)):
            group = grouped[axis]
            if not group:
                continue
            correction = self._weighted_mean(
                [obs.mean_position_correction for obs in group],
                [obs.weight for obs in group],
            )
            correction = self._clip(correction, self.max_position_correction_m)
            pose_xy[index] += correction
            cov[index, index] = self._position_variance(group)
            constrained = True

        if yaw_obs:
            yaw_correction = self._weighted_mean(
                [obs.yaw_correction for obs in yaw_obs],
                [obs.weight for obs in yaw_obs],
            )
            yaw_correction = self._clip(yaw_correction, self.max_yaw_correction)
            pose_yaw = wrap_angle(pose_yaw + yaw_correction)
            cov[5, 5] = self._yaw_variance(yaw_obs)
            constrained = True

        if not constrained:
            return None
        return pose_xy, pose_yaw, cov

    def _position_variance(self, observations: List[WallObservation]) -> float:
        residual_terms = [
            obs.residual_std * obs.residual_std + self.position_variance_min
            for obs in observations
        ]
        weights = [obs.weight for obs in observations]
        return max(self.position_variance_min, self._weighted_mean(residual_terms, weights))

    def _yaw_variance(self, observations: List[WallObservation]) -> float:
        values = np.array([obs.yaw_correction for obs in observations], dtype=float)
        weights = np.array([max(1.0e-9, obs.weight) for obs in observations], dtype=float)
        weights = weights / float(np.sum(weights))
        mean = float(np.sum(weights * values))
        spread = float(np.sum(weights * (values - mean) * (values - mean)))
        return max(self.yaw_variance_min, spread + self.yaw_variance_min)

    def _weighted_mean(self, values: List[float], weights: List[float]) -> float:
        if not values:
            return 0.0
        weights_arr = np.array([max(1.0e-9, w) for w in weights], dtype=float)
        values_arr = np.array(values, dtype=float)
        return float(np.sum(weights_arr * values_arr) / np.sum(weights_arr))

    def _clip(self, value: float, limit: float) -> float:
        if limit <= 0.0:
            return float(value)
        return float(np.clip(value, -limit, limit))

    def _publish_debug(self, observations: List[WallObservation], stamp) -> None:
        if not self.publish_debug_markers:
            return

        markers = MarkerArray()
        delete = Marker()
        delete.header.frame_id = self.frame_id
        delete.header.stamp = stamp
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        for index, obs in enumerate(observations):
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = stamp
            line.ns = 'lidar_wall_fit'
            line.id = index
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.035
            line.color.r = 0.0
            line.color.g = 0.8
            line.color.b = 1.0
            line.color.a = 0.9

            half = 0.5 * obs.span
            p0 = obs.centroid - obs.direction * half
            p1 = obs.centroid + obs.direction * half
            line.points = [self._point(p0), self._point(p1)]
            markers.markers.append(line)

        self.debug_pub.publish(markers)

    def _point(self, xy: np.ndarray) -> Point:
        p = Point()
        p.x = float(xy[0])
        p.y = float(xy[1])
        p.z = 0.05
        return p

    def _log_skip(self, now: float, reason: str) -> None:
        if not self.verbose:
            return
        if now - self.last_skip_log_sec < self.skip_log_period_sec:
            return
        self.last_skip_log_sec = now
        self.get_logger().info(f'[LIDAR WALL] skip: {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarWallLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
