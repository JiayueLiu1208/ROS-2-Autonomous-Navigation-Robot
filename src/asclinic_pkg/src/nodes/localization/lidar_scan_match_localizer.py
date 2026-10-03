#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

try:
    from scipy.ndimage import distance_transform_edt
except Exception:  # pragma: no cover - runtime fallback for stripped installs
    distance_transform_edt = None


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


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


class LidarScanMatchLocalizer(Node):
    """Shadow localizer that aligns RPLIDAR hits against the static /map.

    This node intentionally does not feed the control loop. It searches near the
    current odometry prior, scores candidate poses with an occupancy-map distance
    field, and publishes the best pose on a comparison topic.
    """

    def __init__(self) -> None:
        super().__init__('lidar_scan_match_localizer')

        self.declare_parameter('enabled', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('prior_odom_topic', 'pitt_fused_odometry')
        self.declare_parameter('matched_odom_topic', 'pitt_lidar_scanmatch_odometry')
        self.declare_parameter('debug_points_topic', 'lidar_scanmatch_points')
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('lidar_in_base_x', 0.0)
        self.declare_parameter('lidar_in_base_y', 0.0)
        self.declare_parameter('lidar_in_base_yaw_deg', 0.0)
        self.declare_parameter('lidar_scan_angle_multiplier', 1.0)
        self.declare_parameter('lidar_forward_is_backward', True)

        self.declare_parameter('beam_stride', 4)
        self.declare_parameter('min_range_m', 0.10)
        self.declare_parameter('max_range_m', 4.5)
        self.declare_parameter('min_valid_points', 35)
        self.declare_parameter('occupied_threshold', 90)

        self.declare_parameter('update_period_sec', 0.75)
        self.declare_parameter('scan_timeout_sec', 0.50)
        self.declare_parameter('prior_timeout_sec', 0.60)

        self.declare_parameter('search_xy_radius_m', 0.35)
        self.declare_parameter('search_xy_step_m', 0.07)
        self.declare_parameter('search_yaw_radius_deg', 12.0)
        self.declare_parameter('search_yaw_step_deg', 3.0)
        self.declare_parameter('fine_xy_radius_m', 0.08)
        self.declare_parameter('fine_xy_step_m', 0.02)
        self.declare_parameter('fine_yaw_radius_deg', 3.0)
        self.declare_parameter('fine_yaw_step_deg', 0.75)

        self.declare_parameter('trim_fraction', 0.65)
        self.declare_parameter('distance_score_cap_m', 0.75)
        self.declare_parameter('out_of_map_penalty_m', 0.35)
        self.declare_parameter('max_accepted_score_m', 0.22)
        self.declare_parameter('prior_xy_weight', 0.015)
        self.declare_parameter('prior_yaw_weight', 0.025)

        self.declare_parameter('position_variance_min', 0.015)
        self.declare_parameter('yaw_variance_min', 0.006)
        self.declare_parameter('publish_debug_points', True)
        self.declare_parameter('debug_point_distance_gate_m', 0.18)
        self.declare_parameter('verbose', True)
        self.declare_parameter('skip_log_period_sec', 1.5)

        self.enabled = as_bool(self.get_parameter('enabled').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.prior_odom_topic = str(self.get_parameter('prior_odom_topic').value)
        self.matched_odom_topic = str(self.get_parameter('matched_odom_topic').value)
        self.debug_points_topic = str(self.get_parameter('debug_points_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

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
        self.min_valid_points = max(3, int(self.get_parameter('min_valid_points').value))
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)

        self.update_period_sec = max(
            0.0,
            float(self.get_parameter('update_period_sec').value),
        )
        self.scan_timeout_sec = max(
            0.0,
            float(self.get_parameter('scan_timeout_sec').value),
        )
        self.prior_timeout_sec = max(
            0.0,
            float(self.get_parameter('prior_timeout_sec').value),
        )

        self.search_xy_radius_m = max(
            0.0,
            float(self.get_parameter('search_xy_radius_m').value),
        )
        self.search_xy_step_m = max(
            1.0e-6,
            float(self.get_parameter('search_xy_step_m').value),
        )
        self.search_yaw_radius = math.radians(
            max(0.0, float(self.get_parameter('search_yaw_radius_deg').value))
        )
        self.search_yaw_step = math.radians(
            max(0.05, float(self.get_parameter('search_yaw_step_deg').value))
        )
        self.fine_xy_radius_m = max(
            0.0,
            float(self.get_parameter('fine_xy_radius_m').value),
        )
        self.fine_xy_step_m = max(
            1.0e-6,
            float(self.get_parameter('fine_xy_step_m').value),
        )
        self.fine_yaw_radius = math.radians(
            max(0.0, float(self.get_parameter('fine_yaw_radius_deg').value))
        )
        self.fine_yaw_step = math.radians(
            max(0.05, float(self.get_parameter('fine_yaw_step_deg').value))
        )

        self.trim_fraction = float(np.clip(float(self.get_parameter('trim_fraction').value), 0.05, 1.0))
        self.distance_score_cap_m = max(
            0.01,
            float(self.get_parameter('distance_score_cap_m').value),
        )
        self.out_of_map_penalty_m = max(
            0.0,
            float(self.get_parameter('out_of_map_penalty_m').value),
        )
        self.max_accepted_score_m = max(
            0.0,
            float(self.get_parameter('max_accepted_score_m').value),
        )
        self.prior_xy_weight = max(
            0.0,
            float(self.get_parameter('prior_xy_weight').value),
        )
        self.prior_yaw_weight = max(
            0.0,
            float(self.get_parameter('prior_yaw_weight').value),
        )
        self.position_variance_min = max(
            1.0e-6,
            float(self.get_parameter('position_variance_min').value),
        )
        self.yaw_variance_min = max(
            1.0e-6,
            float(self.get_parameter('yaw_variance_min').value),
        )
        self.publish_debug_points = as_bool(
            self.get_parameter('publish_debug_points').value
        )
        self.debug_point_distance_gate_m = max(
            0.0,
            float(self.get_parameter('debug_point_distance_gate_m').value),
        )
        self.verbose = as_bool(self.get_parameter('verbose').value)
        self.skip_log_period_sec = max(
            0.1,
            float(self.get_parameter('skip_log_period_sec').value),
        )

        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.distance_field: Optional[np.ndarray] = None
        self.map_info_logged = False

        self.latest_prior_pose: Optional[np.ndarray] = None
        self.latest_prior_receive_sec: Optional[float] = None
        self.last_update_sec: Optional[float] = None
        self.last_skip_log_sec = 0.0
        self.last_accept_log_sec = 0.0
        self.accepted_updates = 0
        self.rejected_updates = 0

        self.odom_pub = self.create_publisher(Odometry, self.matched_odom_topic, 10)
        self.debug_pub = self.create_publisher(Marker, self.debug_points_topic, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.create_subscription(Odometry, self.prior_odom_topic, self.prior_callback, 20)
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info('==================================================')
        self.get_logger().info('[LIDAR SCANMATCH] Node started')
        self.get_logger().info(f'[LIDAR SCANMATCH] enabled       = {self.enabled}')
        self.get_logger().info(f'[LIDAR SCANMATCH] scan_topic    = {self.scan_topic}')
        self.get_logger().info(f'[LIDAR SCANMATCH] map_topic     = {self.map_topic}')
        self.get_logger().info(f'[LIDAR SCANMATCH] prior_odom    = {self.prior_odom_topic}')
        self.get_logger().info(f'[LIDAR SCANMATCH] output_odom   = {self.matched_odom_topic}')
        self.get_logger().info(
            f'[LIDAR SCANMATCH] coarse search = +/-{self.search_xy_radius_m:.2f}m '
            f'@ {self.search_xy_step_m:.2f}m, +/-{math.degrees(self.search_yaw_radius):.1f}deg '
            f'@ {math.degrees(self.search_yaw_step):.1f}deg'
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

    def map_callback(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            self._log_skip(self.now_sec(), 'invalid map metadata')
            return
        if len(msg.data) != width * height:
            self._log_skip(self.now_sec(), 'map data size mismatch')
            return

        grid = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        occupied = grid >= self.occupied_threshold
        if not np.any(occupied):
            occupied = grid >= 80
        if not np.any(occupied):
            self._log_skip(self.now_sec(), 'map has no occupied cells')
            return

        self.map_resolution = resolution
        self.map_origin_x = float(msg.info.origin.position.x)
        self.map_origin_y = float(msg.info.origin.position.y)
        self.map_width = width
        self.map_height = height
        self.distance_field = self._distance_field_from_occupied(occupied, resolution)

        if self.verbose and not self.map_info_logged:
            self.map_info_logged = True
            self.get_logger().info(
                f'[LIDAR SCANMATCH] map ready {width}x{height} '
                f'@ {resolution:.3f}m, occupied={int(np.count_nonzero(occupied))}'
            )

    def prior_callback(self, msg: Odometry) -> None:
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
        self.latest_prior_pose = pose
        self.latest_prior_receive_sec = self.now_sec()

    def scan_callback(self, msg: LaserScan) -> None:
        if not self.enabled:
            return
        now = self.now_sec()
        if not self._update_due(now):
            return
        if self.distance_field is None:
            self._log_skip(now, 'waiting for /map')
            return
        if self.latest_prior_pose is None or self.latest_prior_receive_sec is None:
            self._log_skip(now, 'waiting for prior odometry')
            return
        if now - self.latest_prior_receive_sec > self.prior_timeout_sec:
            self._log_skip(now, 'prior odometry is stale')
            return
        scan_stamp = self.stamp_to_sec(msg.header.stamp)
        if scan_stamp is not None and now - scan_stamp > self.scan_timeout_sec:
            self._log_skip(now, 'scan is stale')
            return

        points_base = self._scan_to_base_points(msg)
        if points_base.shape[0] < self.min_valid_points:
            self._log_skip(now, f'not enough usable scan hits ({points_base.shape[0]})')
            return

        self.last_update_sec = now
        prior = self.latest_prior_pose.copy()
        coarse = self._search_pose_grid(
            points_base=points_base,
            center_pose=prior,
            prior_pose=prior,
            xy_offsets=self._offsets(self.search_xy_radius_m, self.search_xy_step_m),
            yaw_offsets=self._offsets(self.search_yaw_radius, self.search_yaw_step),
        )
        if coarse is None:
            self._log_skip(now, 'no valid coarse scan-map candidate')
            return

        fine = self._search_pose_grid(
            points_base=points_base,
            center_pose=coarse[0],
            prior_pose=prior,
            xy_offsets=self._offsets(self.fine_xy_radius_m, self.fine_xy_step_m),
            yaw_offsets=self._offsets(self.fine_yaw_radius, self.fine_yaw_step),
        )
        best_pose, best_score, best_valid_fraction, best_match_fraction = fine or coarse

        if self.max_accepted_score_m > 0.0 and best_score > self.max_accepted_score_m:
            self.rejected_updates += 1
            self._log_skip(
                now,
                f'rejected score={best_score:.3f}m > {self.max_accepted_score_m:.3f}m',
            )
            return

        self.accepted_updates += 1
        self._publish_odometry(
            stamp=msg.header.stamp,
            pose=best_pose,
            score=best_score,
            valid_fraction=best_valid_fraction,
            match_fraction=best_match_fraction,
        )
        if self.publish_debug_points:
            self._publish_debug_points(msg.header.stamp, points_base, best_pose)
        self._log_accept(now, prior, best_pose, best_score, best_valid_fraction)

    def _distance_field_from_occupied(
        self,
        occupied: np.ndarray,
        resolution: float,
    ) -> np.ndarray:
        if distance_transform_edt is not None:
            return distance_transform_edt(~occupied).astype(np.float32) * float(resolution)
        return self._chamfer_distance_field(occupied, resolution)

    def _chamfer_distance_field(self, occupied: np.ndarray, resolution: float) -> np.ndarray:
        height, width = occupied.shape
        inf = float(max(height, width)) * float(resolution) * 4.0
        dist = np.full((height, width), inf, dtype=np.float32)
        dist[occupied] = 0.0
        diag = math.sqrt(2.0) * resolution

        for r in range(height):
            for c in range(width):
                best = dist[r, c]
                if r > 0:
                    best = min(best, dist[r - 1, c] + resolution)
                    if c > 0:
                        best = min(best, dist[r - 1, c - 1] + diag)
                    if c + 1 < width:
                        best = min(best, dist[r - 1, c + 1] + diag)
                if c > 0:
                    best = min(best, dist[r, c - 1] + resolution)
                dist[r, c] = best

        for r in range(height - 1, -1, -1):
            for c in range(width - 1, -1, -1):
                best = dist[r, c]
                if r + 1 < height:
                    best = min(best, dist[r + 1, c] + resolution)
                    if c > 0:
                        best = min(best, dist[r + 1, c - 1] + diag)
                    if c + 1 < width:
                        best = min(best, dist[r + 1, c + 1] + diag)
                if c + 1 < width:
                    best = min(best, dist[r, c + 1] + resolution)
                dist[r, c] = best
        return dist

    def _scan_to_base_points(self, msg: LaserScan) -> np.ndarray:
        if msg.angle_increment == 0.0:
            return np.zeros((0, 2), dtype=float)

        range_min = max(float(msg.range_min), self.min_range_m)
        range_max = float(msg.range_max) if msg.range_max > 0.0 else self.max_range_m
        if self.max_range_m > 0.0:
            range_max = min(range_max, self.max_range_m)

        points = []
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
        return np.asarray(points, dtype=float)

    def _offsets(self, radius: float, step: float) -> np.ndarray:
        if radius <= 0.0:
            return np.array([0.0], dtype=float)
        count = int(math.ceil(radius / max(step, 1.0e-9)))
        values = np.arange(-count, count + 1, dtype=float) * step
        return values[np.abs(values) <= radius + 1.0e-9]

    def _search_pose_grid(
        self,
        points_base: np.ndarray,
        center_pose: np.ndarray,
        prior_pose: np.ndarray,
        xy_offsets: np.ndarray,
        yaw_offsets: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, float, float, float]]:
        best_pose = None
        best_score = float('inf')
        best_valid_fraction = 0.0
        best_match_fraction = 0.0

        for dyaw in yaw_offsets:
            yaw = wrap_angle(float(center_pose[2]) + float(dyaw))
            rotated = points_base @ rot2(yaw).T
            for dx in xy_offsets:
                x = float(center_pose[0]) + float(dx)
                for dy in xy_offsets:
                    y = float(center_pose[1]) + float(dy)
                    score, valid_fraction, match_fraction = self._score_world_points(
                        rotated + np.array([x, y], dtype=float),
                        candidate_pose=np.array([x, y, yaw], dtype=float),
                        prior_pose=prior_pose,
                    )
                    if score < best_score:
                        best_score = score
                        best_pose = np.array([x, y, yaw], dtype=float)
                        best_valid_fraction = valid_fraction
                        best_match_fraction = match_fraction

        if best_pose is None or not math.isfinite(best_score):
            return None
        return best_pose, best_score, best_valid_fraction, best_match_fraction

    def _score_world_points(
        self,
        points_world: np.ndarray,
        candidate_pose: np.ndarray,
        prior_pose: np.ndarray,
    ) -> Tuple[float, float, float]:
        if self.distance_field is None or points_world.shape[0] == 0:
            return float('inf'), 0.0, 0.0

        cols = np.floor((points_world[:, 0] - self.map_origin_x) / self.map_resolution).astype(np.int32)
        rows = np.floor((points_world[:, 1] - self.map_origin_y) / self.map_resolution).astype(np.int32)
        valid = (
            (rows >= 0)
            & (rows < self.map_height)
            & (cols >= 0)
            & (cols < self.map_width)
        )
        valid_count = int(np.count_nonzero(valid))
        if valid_count < self.min_valid_points:
            return float('inf'), 0.0, 0.0

        distances = self.distance_field[rows[valid], cols[valid]]
        distances = np.minimum(distances, self.distance_score_cap_m)
        keep_count = max(
            self.min_valid_points,
            int(math.ceil(self.trim_fraction * float(valid_count))),
        )
        keep_count = min(keep_count, valid_count)
        if keep_count < valid_count:
            kept = np.partition(distances, keep_count - 1)[:keep_count]
        else:
            kept = distances

        valid_fraction = float(valid_count) / float(points_world.shape[0])
        match_fraction = float(np.count_nonzero(kept <= self.debug_point_distance_gate_m)) / float(keep_count)
        robust_mean = float(np.mean(kept))
        score = robust_mean + self.out_of_map_penalty_m * (1.0 - valid_fraction)

        prior_xy_delta = float(np.linalg.norm(candidate_pose[:2] - prior_pose[:2]))
        prior_yaw_delta = abs(wrap_angle(float(candidate_pose[2] - prior_pose[2])))
        score += self.prior_xy_weight * prior_xy_delta
        score += self.prior_yaw_weight * prior_yaw_delta
        return score, valid_fraction, match_fraction

    def _publish_odometry(
        self,
        stamp,
        pose: np.ndarray,
        score: float,
        valid_fraction: float,
        match_fraction: float,
    ) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = 'base_link_lidar_scanmatch'
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(float(pose[2]))

        cov = np.zeros((6, 6), dtype=float)
        score_var = max(self.position_variance_min, score * score)
        cov[0, 0] = score_var
        cov[1, 1] = score_var
        yaw_var = max(
            self.yaw_variance_min,
            (self.fine_yaw_step + 0.20 * score) ** 2,
        )
        cov[5, 5] = yaw_var
        cov[0, 1] = cov[1, 0] = score_var * 0.25
        # Store quality hints in otherwise unused planar covariance slots for logs/tools.
        cov[2, 2] = float(valid_fraction)
        cov[3, 3] = float(match_fraction)
        cov[4, 4] = float(score)
        msg.pose.covariance = cov.reshape(-1).tolist()
        self.odom_pub.publish(msg)

    def _publish_debug_points(self, stamp, points_base: np.ndarray, pose: np.ndarray) -> None:
        if self.distance_field is None:
            return
        points_world = points_base @ rot2(float(pose[2])).T + pose[:2]
        distances = self._distances_for_world_points(points_world)
        if distances is None:
            return
        mask = distances <= self.debug_point_distance_gate_m
        points_world = points_world[mask]
        if points_world.shape[0] == 0:
            return

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'lidar_scanmatch_points'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.color.r = 0.0
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 0.75
        marker.points = [
            Point(x=float(x), y=float(y), z=0.08)
            for x, y in points_world[:: max(1, int(points_world.shape[0] / 240))]
        ]
        self.debug_pub.publish(marker)

    def _distances_for_world_points(self, points_world: np.ndarray) -> Optional[np.ndarray]:
        if self.distance_field is None:
            return None
        cols = np.floor((points_world[:, 0] - self.map_origin_x) / self.map_resolution).astype(np.int32)
        rows = np.floor((points_world[:, 1] - self.map_origin_y) / self.map_resolution).astype(np.int32)
        valid = (
            (rows >= 0)
            & (rows < self.map_height)
            & (cols >= 0)
            & (cols < self.map_width)
        )
        distances = np.full(points_world.shape[0], np.inf, dtype=float)
        distances[valid] = self.distance_field[rows[valid], cols[valid]]
        return distances

    def _update_due(self, now: float) -> bool:
        if self.last_update_sec is None:
            return True
        if self.update_period_sec <= 0.0:
            return True
        return now - self.last_update_sec >= self.update_period_sec

    def _log_skip(self, now: float, reason: str) -> None:
        if not self.verbose:
            return
        if now - self.last_skip_log_sec < self.skip_log_period_sec:
            return
        self.last_skip_log_sec = now
        self.get_logger().info(f'[LIDAR SCANMATCH] skip: {reason}')

    def _log_accept(
        self,
        now: float,
        prior: np.ndarray,
        best: np.ndarray,
        score: float,
        valid_fraction: float,
    ) -> None:
        if not self.verbose:
            return
        if now - self.last_accept_log_sec < self.skip_log_period_sec:
            return
        self.last_accept_log_sec = now
        dx = float(best[0] - prior[0])
        dy = float(best[1] - prior[1])
        dyaw = math.degrees(wrap_angle(float(best[2] - prior[2])))
        self.get_logger().info(
            '[LIDAR SCANMATCH] accepted '
            f'score={score:.3f}m valid={100.0 * valid_fraction:.0f}% '
            f'correction=({dx:+.3f},{dy:+.3f},{dyaw:+.1f}deg) '
            f'updates={self.accepted_updates} rejected={self.rejected_updates}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarScanMatchLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
