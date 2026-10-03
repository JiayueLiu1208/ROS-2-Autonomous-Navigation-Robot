#!/usr/bin/env python3

from collections import deque
import math
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from asclinic_pkg.msg import FiducialMarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rot2(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def parameter_to_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def rodrigues(rvec: np.ndarray) -> np.ndarray:
    r = np.asarray(rvec, dtype=float).reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3, dtype=float)

    k = r / theta
    K = np.array(
        [
            [0.0, -k[2], k[1]],
            [k[2], 0.0, -k[0]],
            [-k[1], k[0], 0.0],
        ],
        dtype=float,
    )
    return np.eye(3, dtype=float) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


class PittKalmanFilter2D:
    """Minimal EKF-style filter for planar pose [x, y, yaw]."""

    def __init__(self, initial_state: np.ndarray, initial_cov: np.ndarray):
        self.x = initial_state.astype(float).copy()  # [x, y, yaw]
        self.P = initial_cov.astype(float).copy()    # 3x3

    def predict_from_body_delta(self, delta_body: np.ndarray, delta_yaw: float, q_body: np.ndarray) -> None:
        yaw = float(self.x[2])
        R = rot2(yaw)
        world_delta = R @ delta_body.reshape(2)

        self.x[0] += world_delta[0]
        self.x[1] += world_delta[1]
        self.x[2] = wrap_angle(self.x[2] + float(delta_yaw))

        F = np.eye(3, dtype=float)
        F[0, 2] = -math.sin(yaw) * delta_body[0] - math.cos(yaw) * delta_body[1]
        F[1, 2] =  math.cos(yaw) * delta_body[0] - math.sin(yaw) * delta_body[1]

        Q = np.zeros((3, 3), dtype=float)
        Q[0, 0] = float(q_body[0])
        Q[1, 1] = float(q_body[1])
        Q[2, 2] = float(q_body[2])

        self.P = F @ self.P @ F.T + Q

    def update_xy(
        self,
        z_xy: np.ndarray,
        r_xy: np.ndarray,
        update_yaw_from_xy: bool = False,
    ) -> None:
        yaw_before = float(self.x[2])
        H = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        )

        y = z_xy.reshape(2) - H @ self.x
        S = H @ self.P @ H.T + r_xy
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        if update_yaw_from_xy:
            self.x[2] = wrap_angle(self.x[2])
        else:
            self.x[2] = yaw_before

        I = np.eye(3, dtype=float)
        self.P = (I - K @ H) @ self.P
        if not update_yaw_from_xy:
            self.P[0, 2] = 0.0
            self.P[1, 2] = 0.0
            self.P[2, 0] = 0.0
            self.P[2, 1] = 0.0

    def update_pose(self, z_pose: np.ndarray, r_pose: np.ndarray) -> None:
        """Update x, y, and yaw simultaneously from a full pose measurement.
        Implements the EKF correction step from Lecture 06 Slide 14.
        """
        H = np.eye(3, dtype=float)
        y = z_pose.reshape(3) - H @ self.x
        y[2] = wrap_angle(float(y[2]))
        S = H @ self.P @ H.T + r_pose
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(float(self.x[2]))
        self.P = (np.eye(3, dtype=float) - K @ H) @ self.P


class PittFusedOdemetry(Node):
    def __init__(self) -> None:
        super().__init__('pitt_fused_odemetry')

        # Topics
        self.declare_parameter('wheel_odom_topic', 'wheel_odometry')
        self.declare_parameter('aruco_topic', 'aruco_detections')
        self.declare_parameter('fused_odom_topic', 'pitt_fused_odometry')
        self.declare_parameter('vision_odom_topic', 'pitt_vision_odometry')

        # Toggle for using Kalman block.
        self.declare_parameter('use_pitt_kalman_filter', True)

        # Camera extrinsics: camera pose in base_link frame [m, m, rad].
        self.declare_parameter('camera_in_base_x', 0.00)
        self.declare_parameter('camera_in_base_y', 0.00)
        self.declare_parameter('camera_in_base_yaw', 0.041)

        # Process and measurement noise.
        self.declare_parameter('process_q_x', 2.0e-4)
        self.declare_parameter('process_q_y', 2.0e-4)
        self.declare_parameter('process_q_yaw', 1.0e-4)
        self.declare_parameter('vision_r_x', 0.05)
        self.declare_parameter('vision_r_y', 0.05)
        self.declare_parameter('vision_r_yaw', 0.08)
        self.declare_parameter('off_axis_skip_deg', 35.0)
        self.declare_parameter('incidence_skip_deg', 60.0)
        self.declare_parameter('vision_yaw_gate_deg', 25.0)
        self.declare_parameter('use_vision_yaw_for_position', False)
        self.declare_parameter('use_vision_yaw_update', True)
        self.declare_parameter('use_vision_xy_yaw_coupling', False)
        self.declare_parameter('use_multi_marker_fusion', True)
        self.declare_parameter('vision_min_markers', 1)
        self.declare_parameter('vision_max_marker_range_m', 7.0)
        self.declare_parameter('single_marker_max_range_m', 5.5)
        self.declare_parameter('vision_outlier_rejection_m', 2.50)
        self.declare_parameter('vision_update_period_s', 1.5)
        self.declare_parameter('vision_filter_window_s', 1.6)
        self.declare_parameter('vision_filter_min_samples', 2)
        self.declare_parameter('vision_filter_max_std_m', 0.35)
        self.declare_parameter('vision_filter_max_jump_m', 0.75)
        self.declare_parameter('vision_correction_step_limit_m', 0.65)

        # Optional LiDAR wall pose correction. A separate node estimates this
        # from stable wall scans; this filter only accepts small corrections.
        self.declare_parameter('use_lidar_wall_update', True)
        self.declare_parameter('use_lidar_wall_yaw_update', True)
        self.declare_parameter('lidar_wall_odom_topic', 'pitt_lidar_wall_odometry')
        self.declare_parameter('lidar_wall_update_period_s', 1.0)
        self.declare_parameter('lidar_wall_outlier_rejection_m', 0.35)
        self.declare_parameter('lidar_wall_yaw_gate_deg', 8.0)
        self.declare_parameter('lidar_wall_correction_step_limit_m', 0.08)
        self.declare_parameter('lidar_wall_yaw_step_limit_deg', 2.0)
        self.declare_parameter('lidar_wall_r_x', 0.025)
        self.declare_parameter('lidar_wall_r_y', 0.025)
        self.declare_parameter('lidar_wall_r_yaw', 0.006)
        self.declare_parameter('lidar_wall_unconstrained_variance', 100.0)

        # Optional LiDAR scan-to-static-map correction. This consumes the
        # shadow scan matcher conservatively: small XY steps only by default.
        self.declare_parameter('use_lidar_scanmatch_update', False)
        self.declare_parameter('use_lidar_scanmatch_yaw_update', False)
        self.declare_parameter(
            'lidar_scanmatch_odom_topic',
            'pitt_lidar_scanmatch_odometry',
        )
        self.declare_parameter('lidar_scanmatch_update_period_s', 1.0)
        self.declare_parameter('lidar_scanmatch_max_score_m', 0.06)
        self.declare_parameter('lidar_scanmatch_min_valid_fraction', 0.90)
        self.declare_parameter('lidar_scanmatch_min_match_fraction', 0.45)
        self.declare_parameter('lidar_scanmatch_outlier_rejection_m', 0.25)
        self.declare_parameter('lidar_scanmatch_yaw_gate_deg', 3.0)
        self.declare_parameter('lidar_scanmatch_correction_step_limit_m', 0.06)
        self.declare_parameter('lidar_scanmatch_yaw_step_limit_deg', 1.0)
        self.declare_parameter('lidar_scanmatch_filter_window_s', 2.5)
        self.declare_parameter('lidar_scanmatch_filter_min_samples', 3)
        self.declare_parameter('lidar_scanmatch_filter_max_std_m', 0.08)
        self.declare_parameter('lidar_scanmatch_filter_max_jump_m', 0.12)
        self.declare_parameter('lidar_scanmatch_r_x', 0.10)
        self.declare_parameter('lidar_scanmatch_r_y', 0.10)
        self.declare_parameter('lidar_scanmatch_r_yaw', 0.04)

        # Marker map string format: "id:x,y,z,phi_deg;..."  (phi_deg = world facing angle)
        # Example: "10:0.0,0.0,0.0;11:3.0,0.0,0.0"
        self.declare_parameter('marker_world_map', '')

        self.declare_parameter('verbosity', 1)

        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.fused_odom_topic = str(self.get_parameter('fused_odom_topic').value)
        self.vision_odom_topic = str(self.get_parameter('vision_odom_topic').value)

        self.use_pitt_kalman_filter = parameter_to_bool(
            self.get_parameter('use_pitt_kalman_filter').value
        )

        self.camera_in_base_x = float(self.get_parameter('camera_in_base_x').value)
        self.camera_in_base_y = float(self.get_parameter('camera_in_base_y').value)
        self.camera_in_base_yaw = float(self.get_parameter('camera_in_base_yaw').value)

        self.process_q = np.array(
            [
                float(self.get_parameter('process_q_x').value),
                float(self.get_parameter('process_q_y').value),
                float(self.get_parameter('process_q_yaw').value),
            ],
            dtype=float,
        )

        self.vision_r_diag = np.array(
            [
                float(self.get_parameter('vision_r_x').value),
                float(self.get_parameter('vision_r_y').value),
            ],
            dtype=float,
        )
        self.vision_r_yaw = float(self.get_parameter('vision_r_yaw').value)
        self.off_axis_skip = math.radians(float(self.get_parameter('off_axis_skip_deg').value))
        self.incidence_skip = math.radians(float(self.get_parameter('incidence_skip_deg').value))
        self.vision_yaw_gate = math.radians(float(self.get_parameter('vision_yaw_gate_deg').value))
        self.use_vision_yaw_for_position = parameter_to_bool(
            self.get_parameter('use_vision_yaw_for_position').value
        )
        self.use_vision_yaw_update = parameter_to_bool(
            self.get_parameter('use_vision_yaw_update').value
        )
        self.use_vision_xy_yaw_coupling = parameter_to_bool(
            self.get_parameter('use_vision_xy_yaw_coupling').value
        )
        self.use_multi_marker_fusion = parameter_to_bool(
            self.get_parameter('use_multi_marker_fusion').value
        )
        self.vision_min_markers = int(self.get_parameter('vision_min_markers').value)
        self.vision_max_marker_range_m = float(
            self.get_parameter('vision_max_marker_range_m').value
        )
        self.single_marker_max_range_m = max(
            0.0,
            float(self.get_parameter('single_marker_max_range_m').value),
        )
        self.vision_outlier_rejection_m = float(
            self.get_parameter('vision_outlier_rejection_m').value
        )
        self.vision_update_period_s = max(
            0.0,
            float(self.get_parameter('vision_update_period_s').value),
        )
        self.vision_filter_window_s = max(
            0.0,
            float(self.get_parameter('vision_filter_window_s').value),
        )
        self.vision_filter_min_samples = max(
            1,
            int(self.get_parameter('vision_filter_min_samples').value),
        )
        self.vision_filter_max_std_m = max(
            0.0,
            float(self.get_parameter('vision_filter_max_std_m').value),
        )
        self.vision_filter_max_jump_m = max(
            0.0,
            float(self.get_parameter('vision_filter_max_jump_m').value),
        )
        self.vision_correction_step_limit_m = max(
            0.0,
            float(self.get_parameter('vision_correction_step_limit_m').value),
        )
        if self.vision_min_markers < 1:
            self.vision_min_markers = 1

        self.use_lidar_wall_update = parameter_to_bool(
            self.get_parameter('use_lidar_wall_update').value
        )
        self.use_lidar_wall_yaw_update = parameter_to_bool(
            self.get_parameter('use_lidar_wall_yaw_update').value
        )
        self.lidar_wall_odom_topic = str(
            self.get_parameter('lidar_wall_odom_topic').value
        )
        self.lidar_wall_update_period_s = max(
            0.0,
            float(self.get_parameter('lidar_wall_update_period_s').value),
        )
        self.lidar_wall_outlier_rejection_m = max(
            0.0,
            float(self.get_parameter('lidar_wall_outlier_rejection_m').value),
        )
        self.lidar_wall_yaw_gate = math.radians(
            abs(float(self.get_parameter('lidar_wall_yaw_gate_deg').value))
        )
        self.lidar_wall_correction_step_limit_m = max(
            0.0,
            float(self.get_parameter('lidar_wall_correction_step_limit_m').value),
        )
        self.lidar_wall_yaw_step_limit = math.radians(
            max(0.0, float(self.get_parameter('lidar_wall_yaw_step_limit_deg').value))
        )
        self.lidar_wall_r_diag = np.array(
            [
                max(1.0e-6, float(self.get_parameter('lidar_wall_r_x').value)),
                max(1.0e-6, float(self.get_parameter('lidar_wall_r_y').value)),
            ],
            dtype=float,
        )
        self.lidar_wall_r_yaw = max(
            1.0e-6,
            float(self.get_parameter('lidar_wall_r_yaw').value),
        )
        self.lidar_wall_unconstrained_variance = max(
            1.0,
            float(self.get_parameter('lidar_wall_unconstrained_variance').value),
        )

        self.use_lidar_scanmatch_update = parameter_to_bool(
            self.get_parameter('use_lidar_scanmatch_update').value
        )
        self.use_lidar_scanmatch_yaw_update = parameter_to_bool(
            self.get_parameter('use_lidar_scanmatch_yaw_update').value
        )
        self.lidar_scanmatch_odom_topic = str(
            self.get_parameter('lidar_scanmatch_odom_topic').value
        )
        self.lidar_scanmatch_update_period_s = max(
            0.0,
            float(self.get_parameter('lidar_scanmatch_update_period_s').value),
        )
        self.lidar_scanmatch_max_score_m = max(
            0.0,
            float(self.get_parameter('lidar_scanmatch_max_score_m').value),
        )
        self.lidar_scanmatch_min_valid_fraction = float(np.clip(
            float(self.get_parameter('lidar_scanmatch_min_valid_fraction').value),
            0.0,
            1.0,
        ))
        self.lidar_scanmatch_min_match_fraction = float(np.clip(
            float(self.get_parameter('lidar_scanmatch_min_match_fraction').value),
            0.0,
            1.0,
        ))
        self.lidar_scanmatch_outlier_rejection_m = max(
            0.0,
            float(self.get_parameter('lidar_scanmatch_outlier_rejection_m').value),
        )
        self.lidar_scanmatch_yaw_gate = math.radians(
            abs(float(self.get_parameter('lidar_scanmatch_yaw_gate_deg').value))
        )
        self.lidar_scanmatch_correction_step_limit_m = max(
            0.0,
            float(self.get_parameter('lidar_scanmatch_correction_step_limit_m').value),
        )
        self.lidar_scanmatch_yaw_step_limit = math.radians(
            max(0.0, float(self.get_parameter('lidar_scanmatch_yaw_step_limit_deg').value))
        )
        self.lidar_scanmatch_filter_window_s = max(
            0.0,
            float(self.get_parameter('lidar_scanmatch_filter_window_s').value),
        )
        self.lidar_scanmatch_filter_min_samples = max(
            1,
            int(self.get_parameter('lidar_scanmatch_filter_min_samples').value),
        )
        self.lidar_scanmatch_filter_max_std_m = max(
            0.0,
            float(self.get_parameter('lidar_scanmatch_filter_max_std_m').value),
        )
        self.lidar_scanmatch_filter_max_jump_m = max(
            0.0,
            float(self.get_parameter('lidar_scanmatch_filter_max_jump_m').value),
        )
        self.lidar_scanmatch_r_diag = np.array(
            [
                max(1.0e-6, float(self.get_parameter('lidar_scanmatch_r_x').value)),
                max(1.0e-6, float(self.get_parameter('lidar_scanmatch_r_y').value)),
            ],
            dtype=float,
        )
        self.lidar_scanmatch_r_yaw = max(
            1.0e-6,
            float(self.get_parameter('lidar_scanmatch_r_yaw').value),
        )

        marker_map_text = str(self.get_parameter('marker_world_map').value)
        self.marker_world_map = self._parse_marker_map(marker_map_text)

        self.verbosity = int(self.get_parameter('verbosity').value)

        self.fused_pub = self.create_publisher(Odometry, self.fused_odom_topic, 10)
        self.vision_pub = self.create_publisher(Odometry, self.vision_odom_topic, 10)

        self.wheel_sub = self.create_subscription(
            Odometry,
            self.wheel_odom_topic,
            self.wheel_odom_callback,
            10,
        )

        self.aruco_sub = self.create_subscription(
            FiducialMarkerArray,
            self.aruco_topic,
            self.aruco_callback,
            10,
        )
        self.lidar_wall_sub = None
        if self.use_lidar_wall_update:
            self.lidar_wall_sub = self.create_subscription(
                Odometry,
                self.lidar_wall_odom_topic,
                self.lidar_wall_callback,
                10,
            )
        self.lidar_scanmatch_sub = None
        if self.use_lidar_scanmatch_update:
            self.lidar_scanmatch_sub = self.create_subscription(
                Odometry,
                self.lidar_scanmatch_odom_topic,
                self.lidar_scanmatch_callback,
                10,
            )

        self.kf: Optional[PittKalmanFilter2D] = None
        self.last_odom_pose: Optional[np.ndarray] = None  # [x, y, yaw] from wheel odom
        self.vision_buffer: Deque[dict] = deque()
        self.last_vision_update_time: Optional[float] = None
        self.last_vision_skip_log_time = 0.0
        self.last_lidar_wall_update_time: Optional[float] = None
        self.last_lidar_wall_skip_log_time = 0.0
        self.lidar_scanmatch_buffer: Deque[dict] = deque()
        self.last_lidar_scanmatch_update_time: Optional[float] = None
        self.last_lidar_scanmatch_skip_log_time = 0.0

        if self.verbosity >= 1:
            self.get_logger().info('[PITT FUSED ODEMETRY] Node started')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] wheel_odom_topic = {self.wheel_odom_topic}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] aruco_topic      = {self.aruco_topic}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] fused_odom_topic = {self.fused_odom_topic}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] vision_odom_topic= {self.vision_odom_topic}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] use_kalman      = {self.use_pitt_kalman_filter}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] camera extrinsic= (x={self.camera_in_base_x:.3f}, y={self.camera_in_base_y:.3f}, yaw={self.camera_in_base_yaw:.3f})')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] marker_map_size = {len(self.marker_world_map)}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] off_axis_skip   = {math.degrees(self.off_axis_skip):.1f} deg')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] incidence_skip  = {math.degrees(self.incidence_skip):.1f} deg')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] yaw_gate        = {math.degrees(self.vision_yaw_gate):.1f} deg')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] xy yaw source   = {"vision" if self.use_vision_yaw_for_position else "prior"}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] yaw update      = {"vision" if self.use_vision_yaw_update else "disabled"}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] xy-yaw coupling = {self.use_vision_xy_yaw_coupling}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] multi-marker    = {self.use_multi_marker_fusion}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] min markers     = {self.vision_min_markers}')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] max range       = {self.vision_max_marker_range_m:.2f} m')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] single range    = {self.single_marker_max_range_m:.2f} m')
            self.get_logger().info(f'[PITT FUSED ODEMETRY] outlier gate    = {self.vision_outlier_rejection_m:.2f} m')
            self.get_logger().info(
                f'[PITT FUSED ODEMETRY] update period   = {self.vision_update_period_s:.2f} s'
            )
            self.get_logger().info(
                f'[PITT FUSED ODEMETRY] temporal filter = window {self.vision_filter_window_s:.2f}s, '
                f'min {self.vision_filter_min_samples}, std {self.vision_filter_max_std_m:.2f}m, '
                f'jump {self.vision_filter_max_jump_m:.2f}m'
            )
            self.get_logger().info(
                f'[PITT FUSED ODEMETRY] correction cap  = {self.vision_correction_step_limit_m:.2f} m'
            )
            self.get_logger().info(
                f'[PITT FUSED ODEMETRY] lidar wall      = '
                f'{"enabled" if self.use_lidar_wall_update else "disabled"} '
                f'topic={self.lidar_wall_odom_topic}'
            )
            self.get_logger().info(
                f'[PITT FUSED ODEMETRY] lidar scan-map  = '
                f'{"enabled" if self.use_lidar_scanmatch_update else "disabled"} '
                f'topic={self.lidar_scanmatch_odom_topic}, '
                f'yaw_update={self.use_lidar_scanmatch_yaw_update}, '
                f'max_score={self.lidar_scanmatch_max_score_m:.3f}m'
            )

    def _parse_marker_map(self, text: str) -> Dict[int, Tuple]:
        """Parse marker map string. Format: 'id:x,y,z,phi_deg;...'
        phi_deg is the world-frame facing angle of the marker (from WORLD_COORDINATE_SYSTEM.md).
        The phi_deg field is optional for backward compatibility (will be None if missing).
        """
        result: Dict[int, Tuple] = {}
        if text.strip() == '':
            return result

        entries = [e.strip() for e in text.split(';') if e.strip()]
        for entry in entries:
            if ':' not in entry:
                self.get_logger().warn(f'[PITT FUSED ODEMETRY] Skip malformed marker entry: {entry}')
                continue
            marker_id_text, value_text = entry.split(':', 1)
            fields = [v.strip() for v in value_text.split(',')]
            if len(fields) < 3:
                self.get_logger().warn(f'[PITT FUSED ODEMETRY] Skip malformed marker entry: {entry}')
                continue
            try:
                marker_id = int(marker_id_text)
                xw = float(fields[0])
                yw = float(fields[1])
                zw = float(fields[2])
                phi_deg: Optional[float] = float(fields[3]) if len(fields) >= 4 else None
                result[marker_id] = (xw, yw, zw, phi_deg)
            except ValueError:
                self.get_logger().warn(f'[PITT FUSED ODEMETRY] Skip malformed marker entry: {entry}')

        return result

    def _ensure_filter_from_first_odom(self, odom_pose: np.ndarray) -> None:
        if self.kf is not None:
            return

        init_cov = np.diag([0.10, 0.10, 0.20]).astype(float)
        self.kf = PittKalmanFilter2D(initial_state=odom_pose, initial_cov=init_cov)

        if self.verbosity >= 1:
            self.get_logger().info('[PITT FUSED ODEMETRY] Kalman filter initialized from wheel odometry')

    def wheel_odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        odom_pose = np.array([x, y, yaw], dtype=float)

        self._ensure_filter_from_first_odom(odom_pose)

        if self.last_odom_pose is None:
            self.last_odom_pose = odom_pose
            self._publish_fused_from_state(msg.header.stamp)
            return

        dx_world = odom_pose[0] - self.last_odom_pose[0]
        dy_world = odom_pose[1] - self.last_odom_pose[1]
        dyaw = wrap_angle(odom_pose[2] - self.last_odom_pose[2])

        R_inv = rot2(-self.last_odom_pose[2])
        delta_body = R_inv @ np.array([dx_world, dy_world], dtype=float)

        if self.use_pitt_kalman_filter and self.kf is not None:
            self.kf.predict_from_body_delta(delta_body=delta_body, delta_yaw=dyaw, q_body=self.process_q)
        elif self.kf is not None:
            self.kf.x = odom_pose.copy()

        self.last_odom_pose = odom_pose
        self._publish_fused_from_state(msg.header.stamp)

    def aruco_callback(self, msg: FiducialMarkerArray) -> None:
        if self.kf is None:
            return

        if len(self.marker_world_map) == 0:
            if self.verbosity >= 1:
                self.get_logger().warn('[PITT FUSED ODEMETRY] marker_world_map is empty; cannot convert to world pose')
            return

        if msg.num_markers <= 0:
            return

        yaw_prior = float(self.kf.x[2])

        # Each entry: (off_axis_angle_rad, xy_est, yaw_est, marker_id,
        # incidence_rad, range_planar_m)
        candidates: List[tuple] = []

        for marker in msg.markers:
            marker_id = int(marker.id)
            if marker_id not in self.marker_world_map:
                continue

            marker_world = self.marker_world_map[marker_id]
            phi_deg = marker_world[3]  # None if not provided in marker map

            tvec_raw = np.array(marker.tvec, dtype=float)
            rvec_raw = np.array(marker.rvec, dtype=float)

            # Compute off-axis angle for quality scoring
            off_axis = math.atan2(abs(tvec_raw[0]), tvec_raw[2]) if tvec_raw[2] > 0.01 else math.pi

            # Skip markers that are too far off-axis; tvec[0] becomes unreliable.
            if off_axis > self.off_axis_skip:
                if self.verbosity >= 3:
                    self.get_logger().info(
                        f'[DBG] marker {marker_id:2d} skipped — off-axis '
                        f'{math.degrees(off_axis):.1f}deg > {math.degrees(self.off_axis_skip):.1f}deg'
                    )
                continue

            range_planar = math.hypot(float(tvec_raw[0]), float(tvec_raw[2]))
            if (
                self.vision_max_marker_range_m > 0.0
                and range_planar > self.vision_max_marker_range_m
            ):
                if self.verbosity >= 3:
                    self.get_logger().info(
                        f'[DBG] marker {marker_id:2d} skipped — range '
                        f'{range_planar:.2f}m > {self.vision_max_marker_range_m:.2f}m'
                    )
                continue

            result = self._estimate_base_pose_from_one_marker(
                tvec=tvec_raw,
                rvec=rvec_raw,
                marker_world_xy=np.array([marker_world[0], marker_world[1]], dtype=float),
                marker_phi_deg=phi_deg,
                base_yaw=yaw_prior,
            )

            if self.verbosity >= 3:
                if result is not None:
                    xy_est, yaw_est, incidence = result
                    yaw_str = f'{math.degrees(yaw_est):.1f}deg' if yaw_est is not None else 'None'
                    self.get_logger().info(
                        f'[DBG] marker {marker_id:2d} '
                        f'world=({marker_world[0]:.2f},{marker_world[1]:.2f}) phi={phi_deg}deg | '
                        f'tvec=({tvec_raw[0]:.3f},{tvec_raw[1]:.3f},{tvec_raw[2]:.3f}) '
                        f'off-axis={math.degrees(off_axis):.1f}deg '
                        f'incidence={math.degrees(incidence):.1f}deg | '
                        f'xy_est=({xy_est[0]:.3f},{xy_est[1]:.3f}) yaw_est={yaw_str}'
                    )
                else:
                    self.get_logger().info(f'[DBG] marker {marker_id:2d} — estimate returned None')

            if result is not None:
                xy_est, yaw_est, incidence = result
                if math.isfinite(incidence) and incidence > self.incidence_skip:
                    if self.verbosity >= 3:
                        self.get_logger().info(
                            f'[DBG] marker {marker_id:2d} skipped — incidence '
                            f'{math.degrees(incidence):.1f}deg > {math.degrees(self.incidence_skip):.1f}deg'
                        )
                    continue
                if yaw_est is not None:
                    yaw_error = abs(wrap_angle(yaw_est - yaw_prior))
                    if yaw_error > self.vision_yaw_gate:
                        if self.verbosity >= 3:
                            self.get_logger().info(
                                f'[DBG] marker {marker_id:2d} yaw ignored — innovation '
                                f'{math.degrees(yaw_error):.1f}deg > {math.degrees(self.vision_yaw_gate):.1f}deg'
                        )
                        yaw_est = None
                candidates.append((off_axis, xy_est, yaw_est, marker_id, incidence, range_planar))

        if len(candidates) == 0:
            return
        if len(candidates) < self.vision_min_markers:
            if self.verbosity >= 3:
                ids = [int(c[3]) for c in candidates]
                self.get_logger().info(
                    f'[DBG] skipping vision update — only {len(candidates)} marker candidates '
                    f'{ids}, need at least {self.vision_min_markers}'
                )
            return

        if (
            len(candidates) == 1
            and self.single_marker_max_range_m > 0.0
            and candidates[0][5] > self.single_marker_max_range_m
        ):
            now = self.get_clock().now()
            now_sec = float(now.nanoseconds) * 1.0e-9
            marker_id = int(candidates[0][3])
            range_planar = float(candidates[0][5])
            self._log_vision_filter_skip(
                now_sec,
                f'single marker {marker_id} range {range_planar:.2f}m > '
                f'{self.single_marker_max_range_m:.2f}m',
                min_interval=2.0,
            )
            return

        # Incidence is the main single-marker quality score: low incidence
        # means the marker is closer to face-on. Off-axis is a secondary
        # tie-breaker used when multi-marker fusion is disabled or rejects all
        # but one candidate.
        candidates.sort(key=lambda c: (c[4] if math.isfinite(c[4]) else math.pi, c[0]))
        best_off_axis, best_xy, best_yaw, best_id, best_incidence, best_range = candidates[0]

        if self.verbosity >= 3 and len(candidates) > 1:
            self.get_logger().info(
                f'[DBG] {len(candidates)} candidates — using marker {best_id} '
                f'(off-axis {math.degrees(best_off_axis):.1f}deg)'
            )

        used_candidates = [candidates[0]]
        if self.use_multi_marker_fusion and len(candidates) > 1:
            used_candidates = self._select_consistent_candidates(candidates)

        z_xy, z_yaw, spread_xy, spread_yaw, used_ids = self._fuse_marker_candidates(
            used_candidates
        )
        if z_yaw is None and self.use_vision_yaw_update and best_yaw is not None:
            z_yaw = best_yaw
            spread_yaw = 0.0

        now = self.get_clock().now()
        stamp = now.to_msg()
        now_sec = float(now.nanoseconds) * 1.0e-9
        prior_xy = np.array([float(self.kf.x[0]), float(self.kf.x[1])], dtype=float)
        innovation_xy = z_xy - prior_xy

        self._remember_vision_sample(
            now_sec=now_sec,
            innovation_xy=innovation_xy,
            z_yaw=z_yaw,
            spread_xy=spread_xy,
            spread_yaw=spread_yaw,
            used_ids=used_ids,
        )

        # Always publish the current vision-only pose for live debugging, even
        # when the temporal gate decides not to update the fused estimate yet.
        self._publish_vision_only(stamp=stamp, z_xy=z_xy)

        if not self._vision_update_due(now_sec):
            self._publish_fused_from_state(stamp)
            return

        filtered = self._filtered_vision_measurement(now_sec=now_sec, prior_xy=prior_xy)
        if filtered is None:
            self._publish_fused_from_state(stamp)
            return

        z_xy, z_yaw, spread_xy, spread_yaw, used_ids, filter_stats = filtered
        self.last_vision_update_time = now_sec

        r_xy = np.diag(self.vision_r_diag + spread_xy)

        if self.use_vision_yaw_update and z_yaw is not None:
            r_yaw = self.vision_r_yaw + spread_yaw

            if self.use_pitt_kalman_filter:
                z_pose = np.array([z_xy[0], z_xy[1], z_yaw], dtype=float)
                r_pose = np.diag([float(r_xy[0, 0]), float(r_xy[1, 1]), r_yaw])
                self.kf.update_pose(z_pose=z_pose, r_pose=r_pose)
            else:
                self.kf.x[0] = z_xy[0]
                self.kf.x[1] = z_xy[1]
                self.kf.x[2] = z_yaw
        else:
            # No usable/allowed marker yaw — fall back to xy-only update.
            if self.use_pitt_kalman_filter:
                self.kf.update_xy(
                    z_xy=z_xy,
                    r_xy=r_xy,
                    update_yaw_from_xy=self.use_vision_xy_yaw_coupling,
                )
            else:
                self.kf.x[0] = z_xy[0]
                self.kf.x[1] = z_xy[1]

        self._publish_fused_from_state(stamp)

        if self.verbosity >= 2:
            used_text = ','.join(str(mid) for mid in used_ids)
            self.get_logger().info(
                '[PITT FUSED ODEMETRY] vision update '
                f'using markers [{used_text}] best={best_id} '
                f'(off-axis {math.degrees(best_off_axis):.1f}deg, '
                f'incidence {math.degrees(best_incidence):.1f}deg, range {best_range:.2f}m, '
                f'{len(candidates)} candidates, '
                f'{int(filter_stats["samples"])} filtered samples, '
                f'corr={filter_stats["correction_norm"]:.3f}m, '
                f'std={filter_stats["std_norm"]:.3f}m): '
                f'vision_xy=({z_xy[0]:.3f}, {z_xy[1]:.3f}), '
                f'fused_xy=({self.kf.x[0]:.3f}, {self.kf.x[1]:.3f}), yaw={self.kf.x[2]:.3f}'
            )

    def lidar_wall_callback(self, msg: Odometry) -> None:
        if not self.use_lidar_wall_update or self.kf is None:
            return

        now = self.get_clock().now()
        now_sec = float(now.nanoseconds) * 1.0e-9
        if not self._lidar_wall_update_due(now_sec):
            return

        z_xy = np.array(
            [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            ],
            dtype=float,
        )
        z_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if not np.isfinite(z_xy).all() or not math.isfinite(z_yaw):
            self._log_lidar_wall_skip(now_sec, 'non-finite pose measurement')
            return

        prior_xy = np.array([float(self.kf.x[0]), float(self.kf.x[1])], dtype=float)
        prior_yaw = float(self.kf.x[2])
        correction_xy = z_xy - prior_xy
        correction_norm = float(np.linalg.norm(correction_xy))

        if (
            self.lidar_wall_outlier_rejection_m > 0.0
            and correction_norm > self.lidar_wall_outlier_rejection_m
        ):
            self._log_lidar_wall_skip(
                now_sec,
                f'xy correction {correction_norm:.3f}m > '
                f'{self.lidar_wall_outlier_rejection_m:.3f}m',
            )
            return

        clipped_xy = False
        if (
            self.lidar_wall_correction_step_limit_m > 0.0
            and correction_norm > self.lidar_wall_correction_step_limit_m
        ):
            scale = self.lidar_wall_correction_step_limit_m / max(correction_norm, 1.0e-9)
            correction_xy = correction_xy * scale
            z_xy = prior_xy + correction_xy
            correction_norm = float(np.linalg.norm(correction_xy))
            clipped_xy = True

        r_x, r_y, r_yaw = self._lidar_wall_covariance_diag(msg)
        yaw_error = wrap_angle(z_yaw - prior_yaw)
        yaw_constrained = r_yaw < 0.5 * self.lidar_wall_unconstrained_variance
        use_yaw = (
            self.use_lidar_wall_yaw_update
            and yaw_constrained
            and abs(yaw_error) <= self.lidar_wall_yaw_gate
        )
        clipped_yaw = False
        if use_yaw and self.lidar_wall_yaw_step_limit > 0.0:
            if abs(yaw_error) > self.lidar_wall_yaw_step_limit:
                yaw_error = math.copysign(self.lidar_wall_yaw_step_limit, yaw_error)
                z_yaw = wrap_angle(prior_yaw + yaw_error)
                clipped_yaw = True

        if yaw_constrained and not use_yaw:
            self._log_lidar_wall_skip(
                now_sec,
                f'yaw ignored innovation {math.degrees(abs(yaw_error)):.1f}deg',
                min_interval=2.0,
            )

        r_xy = np.diag([r_x, r_y])
        if self.use_pitt_kalman_filter:
            if use_yaw:
                z_pose = np.array([z_xy[0], z_xy[1], z_yaw], dtype=float)
                r_pose = np.diag([r_x, r_y, r_yaw])
                self.kf.update_pose(z_pose=z_pose, r_pose=r_pose)
            else:
                self.kf.update_xy(
                    z_xy=z_xy,
                    r_xy=r_xy,
                    update_yaw_from_xy=False,
                )
        else:
            self.kf.x[0] = z_xy[0]
            self.kf.x[1] = z_xy[1]
            if use_yaw:
                self.kf.x[2] = z_yaw

        self.last_lidar_wall_update_time = now_sec
        self._publish_fused_from_state(msg.header.stamp)

        if self.verbosity >= 2:
            detail = []
            if clipped_xy:
                detail.append('xy clipped')
            if clipped_yaw:
                detail.append('yaw clipped')
            detail_text = f' ({", ".join(detail)})' if detail else ''
            self.get_logger().info(
                '[PITT FUSED ODEMETRY] lidar wall update '
                f'corr={correction_norm:.3f}m yaw={math.degrees(yaw_error):+.2f}deg '
                f'use_yaw={use_yaw} r=({r_x:.4f},{r_y:.4f},{r_yaw:.4f})'
                f'{detail_text}: fused=({self.kf.x[0]:.3f}, {self.kf.x[1]:.3f}, '
                f'{math.degrees(self.kf.x[2]):.1f}deg)'
            )

    def lidar_scanmatch_callback(self, msg: Odometry) -> None:
        if not self.use_lidar_scanmatch_update or self.kf is None:
            return

        now = self.get_clock().now()
        now_sec = float(now.nanoseconds) * 1.0e-9
        if not self._lidar_scanmatch_update_due(now_sec):
            return

        z_xy = np.array(
            [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            ],
            dtype=float,
        )
        z_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if not np.isfinite(z_xy).all() or not math.isfinite(z_yaw):
            self._log_lidar_scanmatch_skip(now_sec, 'non-finite pose measurement')
            return

        score_m, valid_fraction, match_fraction = self._lidar_scanmatch_quality(msg)
        if (
            self.lidar_scanmatch_max_score_m > 0.0
            and score_m > self.lidar_scanmatch_max_score_m
        ):
            self._log_lidar_scanmatch_skip(
                now_sec,
                f'score {score_m:.3f}m > {self.lidar_scanmatch_max_score_m:.3f}m',
            )
            return
        if valid_fraction < self.lidar_scanmatch_min_valid_fraction:
            self._log_lidar_scanmatch_skip(
                now_sec,
                f'valid scan fraction {valid_fraction:.2f} < '
                f'{self.lidar_scanmatch_min_valid_fraction:.2f}',
            )
            return
        if match_fraction < self.lidar_scanmatch_min_match_fraction:
            self._log_lidar_scanmatch_skip(
                now_sec,
                f'map-hit fraction {match_fraction:.2f} < '
                f'{self.lidar_scanmatch_min_match_fraction:.2f}',
            )
            return

        prior_xy = np.array([float(self.kf.x[0]), float(self.kf.x[1])], dtype=float)
        prior_yaw = float(self.kf.x[2])
        innovation_xy = z_xy - prior_xy
        correction_norm = float(np.linalg.norm(innovation_xy))
        if (
            self.lidar_scanmatch_outlier_rejection_m > 0.0
            and correction_norm > self.lidar_scanmatch_outlier_rejection_m
        ):
            self._log_lidar_scanmatch_skip(
                now_sec,
                f'xy correction {correction_norm:.3f}m > '
                f'{self.lidar_scanmatch_outlier_rejection_m:.3f}m',
            )
            return

        yaw_error = wrap_angle(z_yaw - prior_yaw)
        if abs(yaw_error) > self.lidar_scanmatch_yaw_gate:
            z_yaw = None
        elif not self.use_lidar_scanmatch_yaw_update:
            z_yaw = None

        self._remember_lidar_scanmatch_sample(
            now_sec=now_sec,
            innovation_xy=innovation_xy,
            z_yaw=z_yaw,
            score_m=score_m,
        )

        filtered = self._filtered_lidar_scanmatch_measurement(
            now_sec=now_sec,
            prior_xy=prior_xy,
        )
        if filtered is None:
            return

        z_xy, z_yaw, stats = filtered
        filtered_correction = z_xy - prior_xy
        filtered_correction_norm = float(np.linalg.norm(filtered_correction))
        clipped_xy = False
        if (
            self.lidar_scanmatch_correction_step_limit_m > 0.0
            and filtered_correction_norm > self.lidar_scanmatch_correction_step_limit_m
        ):
            scale = self.lidar_scanmatch_correction_step_limit_m / max(
                filtered_correction_norm,
                1.0e-9,
            )
            filtered_correction = filtered_correction * scale
            z_xy = prior_xy + filtered_correction
            filtered_correction_norm = float(np.linalg.norm(filtered_correction))
            clipped_xy = True

        r_xy = np.diag(
            self.lidar_scanmatch_r_diag
            + np.array([stats['score_m'] ** 2, stats['score_m'] ** 2], dtype=float)
        )
        use_yaw = z_yaw is not None
        clipped_yaw = False
        if use_yaw and self.lidar_scanmatch_yaw_step_limit > 0.0:
            yaw_error = wrap_angle(float(z_yaw) - prior_yaw)
            if abs(yaw_error) > self.lidar_scanmatch_yaw_step_limit:
                yaw_error = math.copysign(self.lidar_scanmatch_yaw_step_limit, yaw_error)
                z_yaw = wrap_angle(prior_yaw + yaw_error)
                clipped_yaw = True

        if self.use_pitt_kalman_filter:
            if use_yaw:
                z_pose = np.array([z_xy[0], z_xy[1], float(z_yaw)], dtype=float)
                r_pose = np.diag([
                    float(r_xy[0, 0]),
                    float(r_xy[1, 1]),
                    self.lidar_scanmatch_r_yaw,
                ])
                self.kf.update_pose(z_pose=z_pose, r_pose=r_pose)
            else:
                self.kf.update_xy(
                    z_xy=z_xy,
                    r_xy=r_xy,
                    update_yaw_from_xy=False,
                )
        else:
            self.kf.x[0] = z_xy[0]
            self.kf.x[1] = z_xy[1]
            if use_yaw:
                self.kf.x[2] = float(z_yaw)

        self.last_lidar_scanmatch_update_time = now_sec
        self._publish_fused_from_state(msg.header.stamp)

        if self.verbosity >= 2:
            detail = []
            if clipped_xy:
                detail.append('xy clipped')
            if clipped_yaw:
                detail.append('yaw clipped')
            detail_text = f' ({", ".join(detail)})' if detail else ''
            self.get_logger().info(
                '[PITT FUSED ODEMETRY] lidar scan-map update '
                f'corr={filtered_correction_norm:.3f}m raw={stats["raw_norm"]:.3f}m '
                f'score={stats["score_m"]:.3f}m samples={int(stats["samples"])} '
                f'std={stats["std_norm"]:.3f}m use_yaw={use_yaw}'
                f'{detail_text}: fused=({self.kf.x[0]:.3f}, {self.kf.x[1]:.3f}, '
                f'{math.degrees(self.kf.x[2]):.1f}deg)'
            )

    def _lidar_scanmatch_quality(self, msg: Odometry) -> Tuple[float, float, float]:
        cov = list(msg.pose.covariance)
        score_m = self.lidar_scanmatch_max_score_m
        valid_fraction = 1.0
        match_fraction = 1.0
        if len(cov) >= 36:
            # Newer scan-map messages store the raw distance score in cov[28].
            # Fall back to cov[0] for older logs/messages, where score was
            # encoded as a variance and may include a covariance floor.
            if math.isfinite(cov[28]) and cov[28] > 0.0:
                score_m = float(cov[28])
            elif math.isfinite(cov[0]) and cov[0] > 0.0:
                score_m = math.sqrt(max(0.0, float(cov[0])))
            if math.isfinite(cov[14]) and cov[14] > 0.0:
                valid_fraction = float(np.clip(cov[14], 0.0, 1.0))
            if math.isfinite(cov[21]) and cov[21] > 0.0:
                match_fraction = float(np.clip(cov[21], 0.0, 1.0))
        return score_m, valid_fraction, match_fraction

    def _remember_lidar_scanmatch_sample(
        self,
        now_sec: float,
        innovation_xy: np.ndarray,
        z_yaw: Optional[float],
        score_m: float,
    ) -> None:
        self.lidar_scanmatch_buffer.append(
            {
                'time': now_sec,
                'innovation_xy': np.array(innovation_xy, dtype=float).reshape(2),
                'z_yaw': z_yaw,
                'score_m': float(score_m),
            }
        )
        self._trim_lidar_scanmatch_buffer(now_sec)

    def _trim_lidar_scanmatch_buffer(self, now_sec: float) -> None:
        if self.lidar_scanmatch_filter_window_s > 0.0:
            while (
                self.lidar_scanmatch_buffer
                and now_sec - float(self.lidar_scanmatch_buffer[0]['time'])
                > self.lidar_scanmatch_filter_window_s
            ):
                self.lidar_scanmatch_buffer.popleft()
        while len(self.lidar_scanmatch_buffer) > 100:
            self.lidar_scanmatch_buffer.popleft()

    def _filtered_lidar_scanmatch_measurement(
        self,
        now_sec: float,
        prior_xy: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, Optional[float], Dict[str, float]]]:
        self._trim_lidar_scanmatch_buffer(now_sec)
        samples = list(self.lidar_scanmatch_buffer)
        if len(samples) < self.lidar_scanmatch_filter_min_samples:
            self._log_lidar_scanmatch_skip(
                now_sec,
                f'need {self.lidar_scanmatch_filter_min_samples} stable samples, '
                f'have {len(samples)}',
            )
            return None

        corrections = np.array([s['innovation_xy'] for s in samples], dtype=float)
        median_correction = np.median(corrections, axis=0)
        distances = np.linalg.norm(corrections - median_correction, axis=1)

        keep_mask = np.ones(len(samples), dtype=bool)
        if self.lidar_scanmatch_filter_max_jump_m > 0.0:
            keep_mask = distances <= self.lidar_scanmatch_filter_max_jump_m

        kept_count = int(np.count_nonzero(keep_mask))
        if kept_count < self.lidar_scanmatch_filter_min_samples:
            self._log_lidar_scanmatch_skip(
                now_sec,
                f'only {kept_count}/{len(samples)} samples within jump gate '
                f'{self.lidar_scanmatch_filter_max_jump_m:.2f}m',
            )
            return None

        kept_corrections = corrections[keep_mask]
        correction = np.median(kept_corrections, axis=0)
        std_xy = np.std(kept_corrections, axis=0)
        std_norm = float(np.linalg.norm(std_xy))
        if (
            self.lidar_scanmatch_filter_max_std_m > 0.0
            and std_norm > self.lidar_scanmatch_filter_max_std_m
        ):
            self._log_lidar_scanmatch_skip(
                now_sec,
                f'unstable correction std={std_norm:.3f}m > '
                f'{self.lidar_scanmatch_filter_max_std_m:.3f}m',
            )
            return None

        raw_norm = float(np.linalg.norm(correction))
        if (
            self.lidar_scanmatch_outlier_rejection_m > 0.0
            and raw_norm > self.lidar_scanmatch_outlier_rejection_m
        ):
            self._log_lidar_scanmatch_skip(
                now_sec,
                f'filtered correction {raw_norm:.3f}m > '
                f'{self.lidar_scanmatch_outlier_rejection_m:.3f}m',
            )
            return None

        z_yaw: Optional[float] = None
        if self.use_lidar_scanmatch_yaw_update:
            yaw_values = [
                float(s['z_yaw'])
                for s, keep in zip(samples, keep_mask)
                if keep and s['z_yaw'] is not None
            ]
            if yaw_values:
                sin_sum = float(np.sum(np.sin(yaw_values)))
                cos_sum = float(np.sum(np.cos(yaw_values)))
                z_yaw = math.atan2(sin_sum, cos_sum)

        scores = [
            float(s['score_m'])
            for s, keep in zip(samples, keep_mask)
            if keep
        ]
        stats = {
            'samples': float(kept_count),
            'std_norm': std_norm,
            'raw_norm': raw_norm,
            'score_m': max(scores) if scores else self.lidar_scanmatch_max_score_m,
        }
        return prior_xy + correction, z_yaw, stats

    def _lidar_wall_covariance_diag(self, msg: Odometry) -> Tuple[float, float, float]:
        cov = list(msg.pose.covariance)
        r_x = self.lidar_wall_r_diag[0]
        r_y = self.lidar_wall_r_diag[1]
        r_yaw = self.lidar_wall_r_yaw
        if len(cov) >= 36:
            if math.isfinite(cov[0]) and cov[0] > 0.0:
                r_x = float(cov[0])
            if math.isfinite(cov[7]) and cov[7] > 0.0:
                r_y = float(cov[7])
            if math.isfinite(cov[35]) and cov[35] > 0.0:
                r_yaw = float(cov[35])
        max_var = self.lidar_wall_unconstrained_variance
        return (
            float(np.clip(r_x, 1.0e-6, max_var)),
            float(np.clip(r_y, 1.0e-6, max_var)),
            float(np.clip(r_yaw, 1.0e-6, max_var)),
        )

    def _lidar_wall_update_due(self, now_sec: float) -> bool:
        if self.last_lidar_wall_update_time is None:
            return True
        if self.lidar_wall_update_period_s <= 0.0:
            return True
        elapsed = now_sec - self.last_lidar_wall_update_time
        if elapsed >= self.lidar_wall_update_period_s:
            return True
        if self.verbosity >= 3:
            self._log_lidar_wall_skip(
                now_sec,
                f'cooldown {elapsed:.2f}s/{self.lidar_wall_update_period_s:.2f}s',
                min_interval=2.0,
            )
        return False

    def _log_lidar_wall_skip(
        self,
        now_sec: float,
        reason: str,
        min_interval: float = 1.0,
    ) -> None:
        if self.verbosity < 2:
            return
        if now_sec - self.last_lidar_wall_skip_log_time < min_interval:
            return
        self.last_lidar_wall_skip_log_time = now_sec
        self.get_logger().info(f'[PITT FUSED ODEMETRY] lidar wall update held: {reason}')

    def _lidar_scanmatch_update_due(self, now_sec: float) -> bool:
        if self.last_lidar_scanmatch_update_time is None:
            return True
        if self.lidar_scanmatch_update_period_s <= 0.0:
            return True
        elapsed = now_sec - self.last_lidar_scanmatch_update_time
        if elapsed >= self.lidar_scanmatch_update_period_s:
            return True
        if self.verbosity >= 3:
            self._log_lidar_scanmatch_skip(
                now_sec,
                f'cooldown {elapsed:.2f}s/{self.lidar_scanmatch_update_period_s:.2f}s',
                min_interval=2.0,
            )
        return False

    def _log_lidar_scanmatch_skip(
        self,
        now_sec: float,
        reason: str,
        min_interval: float = 1.0,
    ) -> None:
        if self.verbosity < 2:
            return
        if now_sec - self.last_lidar_scanmatch_skip_log_time < min_interval:
            return
        self.last_lidar_scanmatch_skip_log_time = now_sec
        self.get_logger().info(
            f'[PITT FUSED ODEMETRY] lidar scan-map update held: {reason}'
        )


    def _vision_update_due(self, now_sec: float) -> bool:
        if self.last_vision_update_time is None:
            return True
        if self.vision_update_period_s <= 0.0:
            return True
        elapsed = now_sec - self.last_vision_update_time
        if elapsed >= self.vision_update_period_s:
            return True
        if self.verbosity >= 3:
            self._log_vision_filter_skip(
                now_sec,
                f'cooldown {elapsed:.2f}s/{self.vision_update_period_s:.2f}s',
                min_interval=2.0,
            )
        return False

    def _remember_vision_sample(
        self,
        now_sec: float,
        innovation_xy: np.ndarray,
        z_yaw: Optional[float],
        spread_xy: np.ndarray,
        spread_yaw: float,
        used_ids: List[int],
    ) -> None:
        self.vision_buffer.append(
            {
                'time': now_sec,
                'innovation_xy': np.array(innovation_xy, dtype=float).reshape(2),
                'z_yaw': z_yaw,
                'spread_xy': np.array(spread_xy, dtype=float).reshape(2),
                'spread_yaw': float(spread_yaw),
                'used_ids': list(used_ids),
            }
        )
        self._trim_vision_buffer(now_sec)

    def _trim_vision_buffer(self, now_sec: float) -> None:
        if self.vision_filter_window_s > 0.0:
            while (
                self.vision_buffer
                and now_sec - float(self.vision_buffer[0]['time']) > self.vision_filter_window_s
            ):
                self.vision_buffer.popleft()

        while len(self.vision_buffer) > 100:
            self.vision_buffer.popleft()

    def _filtered_vision_measurement(
        self,
        now_sec: float,
        prior_xy: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, Optional[float], np.ndarray, float, List[int], Dict[str, float]]]:
        self._trim_vision_buffer(now_sec)
        samples = list(self.vision_buffer)
        if len(samples) < self.vision_filter_min_samples:
            self._log_vision_filter_skip(
                now_sec,
                f'need {self.vision_filter_min_samples} stable samples, have {len(samples)}',
            )
            return None

        corrections = np.array([s['innovation_xy'] for s in samples], dtype=float)
        median_correction = np.median(corrections, axis=0)
        distances = np.linalg.norm(corrections - median_correction, axis=1)

        keep_mask = np.ones(len(samples), dtype=bool)
        if self.vision_filter_max_jump_m > 0.0:
            keep_mask = distances <= self.vision_filter_max_jump_m

        kept_count = int(np.count_nonzero(keep_mask))
        if kept_count < self.vision_filter_min_samples:
            self._log_vision_filter_skip(
                now_sec,
                f'only {kept_count}/{len(samples)} samples within jump gate '
                f'{self.vision_filter_max_jump_m:.2f}m',
            )
            return None

        kept_corrections = corrections[keep_mask]
        correction = np.median(kept_corrections, axis=0)
        std_xy = np.std(kept_corrections, axis=0)
        std_norm = float(np.linalg.norm(std_xy))
        if self.vision_filter_max_std_m > 0.0 and std_norm > self.vision_filter_max_std_m:
            self._log_vision_filter_skip(
                now_sec,
                f'unstable correction std={std_norm:.3f}m > '
                f'{self.vision_filter_max_std_m:.3f}m',
            )
            return None

        correction_norm = float(np.linalg.norm(correction))
        raw_correction_norm = correction_norm
        if (
            self.vision_outlier_rejection_m > 0.0
            and raw_correction_norm > self.vision_outlier_rejection_m
        ):
            self._log_vision_filter_skip(
                now_sec,
                f'filtered correction={raw_correction_norm:.3f}m > '
                f'{self.vision_outlier_rejection_m:.3f}m',
            )
            return None

        clipped = False
        if (
            self.vision_correction_step_limit_m > 0.0
            and correction_norm > self.vision_correction_step_limit_m
        ):
            scale = self.vision_correction_step_limit_m / max(correction_norm, 1.0e-9)
            correction = correction * scale
            correction_norm = float(np.linalg.norm(correction))
            clipped = True

        kept_spreads = np.array(
            [s['spread_xy'] for s, keep in zip(samples, keep_mask) if keep],
            dtype=float,
        )
        temporal_spread_xy = np.var(kept_corrections, axis=0)
        marker_spread_xy = np.mean(kept_spreads, axis=0) if len(kept_spreads) else np.zeros(2)
        spread_xy = np.minimum(
            marker_spread_xy + temporal_spread_xy,
            np.array([0.25, 0.25], dtype=float),
        )

        z_yaw: Optional[float] = None
        spread_yaw = 0.0
        if self.use_vision_yaw_update:
            yaw_values = [
                float(s['z_yaw'])
                for s, keep in zip(samples, keep_mask)
                if keep and s['z_yaw'] is not None
            ]
            if yaw_values:
                sin_sum = float(np.sum(np.sin(yaw_values)))
                cos_sum = float(np.sum(np.cos(yaw_values)))
                z_yaw = math.atan2(sin_sum, cos_sum)
                yaw_errors = np.array([wrap_angle(y - z_yaw) for y in yaw_values], dtype=float)
                sample_yaw_spread = np.mean(
                    [
                        float(s['spread_yaw'])
                        for s, keep in zip(samples, keep_mask)
                        if keep and s['z_yaw'] is not None
                    ]
                )
                spread_yaw = min(0.25, float(np.var(yaw_errors) + sample_yaw_spread))

        used_ids: List[int] = []
        for sample, keep in zip(samples, keep_mask):
            if not keep:
                continue
            for marker_id in sample['used_ids']:
                if marker_id not in used_ids:
                    used_ids.append(int(marker_id))

        stats = {
            'samples': float(kept_count),
            'std_norm': std_norm,
            'correction_norm': correction_norm,
            'raw_correction_norm': raw_correction_norm,
            'clipped': 1.0 if clipped else 0.0,
        }
        z_xy = prior_xy + correction
        return z_xy, z_yaw, spread_xy, spread_yaw, used_ids, stats

    def _log_vision_filter_skip(
        self,
        now_sec: float,
        reason: str,
        min_interval: float = 1.0,
    ) -> None:
        if self.verbosity < 2:
            return
        if now_sec - self.last_vision_skip_log_time < min_interval:
            return
        self.last_vision_skip_log_time = now_sec
        self.get_logger().info(f'[PITT FUSED ODEMETRY] vision update held — {reason}')

    def _candidate_weight(self, candidate: tuple) -> float:
        off_axis, _xy_est, _yaw_est, _marker_id, incidence, range_planar = candidate

        range_scale = max(0.5, self.vision_max_marker_range_m if self.vision_max_marker_range_m > 0.0 else 7.0)
        range_weight = 1.0 / (1.0 + (range_planar / range_scale) ** 2)
        off_axis_weight = max(0.05, math.cos(min(abs(off_axis), math.pi / 2.0))) ** 2
        if math.isfinite(incidence):
            incidence_weight = max(0.05, math.cos(min(abs(incidence), math.pi / 2.0))) ** 2
        else:
            incidence_weight = 0.25

        return max(1.0e-6, range_weight * off_axis_weight * incidence_weight)

    def _select_consistent_candidates(self, candidates: List[tuple]) -> List[tuple]:
        if len(candidates) < 3 or self.vision_outlier_rejection_m <= 0.0:
            return candidates

        xy = np.array([c[1] for c in candidates], dtype=float)
        median_xy = np.median(xy, axis=0)
        distances = np.linalg.norm(xy - median_xy, axis=1)

        median_distance = float(np.median(distances))
        mad = float(np.median(np.abs(distances - median_distance)))
        robust_gate = max(self.vision_outlier_rejection_m, median_distance + 3.0 * 1.4826 * mad)

        kept = [c for c, d in zip(candidates, distances) if d <= robust_gate]
        if kept:
            if self.verbosity >= 3 and len(kept) < len(candidates):
                dropped = [c[3] for c, d in zip(candidates, distances) if d > robust_gate]
                self.get_logger().info(
                    f'[DBG] multi-marker outlier gate kept {len(kept)}/{len(candidates)} '
                    f'(gate={robust_gate:.2f}m, dropped={dropped})'
                )
            return kept

        return [candidates[0]]

    def _fuse_marker_candidates(self, candidates: List[tuple]) -> Tuple[np.ndarray, Optional[float], np.ndarray, float, List[int]]:
        weights = np.array([self._candidate_weight(c) for c in candidates], dtype=float)
        weights_sum = float(np.sum(weights))
        if weights_sum <= 0.0:
            weights = np.ones(len(candidates), dtype=float) / float(len(candidates))
        else:
            weights = weights / weights_sum

        xy_stack = np.array([c[1] for c in candidates], dtype=float)
        z_xy = np.average(xy_stack, axis=0, weights=weights)
        xy_diff = xy_stack - z_xy
        spread_xy = np.average(xy_diff * xy_diff, axis=0, weights=weights)
        spread_xy = np.minimum(spread_xy, np.array([0.25, 0.25], dtype=float))

        yaw_candidates = []
        yaw_weights = []
        for candidate, weight in zip(candidates, weights):
            yaw_est = candidate[2]
            if yaw_est is None or not self.use_vision_yaw_update:
                continue
            yaw_candidates.append(float(yaw_est))
            yaw_weights.append(float(weight))

        z_yaw: Optional[float] = None
        spread_yaw = 0.0
        if yaw_candidates:
            yaw_weights_arr = np.array(yaw_weights, dtype=float)
            yaw_weights_arr = yaw_weights_arr / float(np.sum(yaw_weights_arr))
            sin_sum = float(np.sum(yaw_weights_arr * np.sin(yaw_candidates)))
            cos_sum = float(np.sum(yaw_weights_arr * np.cos(yaw_candidates)))
            z_yaw = math.atan2(sin_sum, cos_sum)
            yaw_errors = np.array([wrap_angle(y - z_yaw) for y in yaw_candidates], dtype=float)
            spread_yaw = min(0.25, float(np.average(yaw_errors * yaw_errors, weights=yaw_weights_arr)))

        used_ids = [int(c[3]) for c in candidates]
        return z_xy, z_yaw, spread_xy, spread_yaw, used_ids


    def _compute_marker_yaw_in_cam(self, rvec: np.ndarray) -> Tuple[float, np.ndarray]:
        """Return marker normal yaw in the camera horizontal plane.

        OpenCV camera frame: x right, y down, z forward. For a face-on marker,
        the marker normal points approximately toward -z, which gives alpha=pi.
        """
        R_mc = rodrigues(rvec)
        nx = float(R_mc[0, 2])
        nz = float(R_mc[2, 2])
        alpha = math.atan2(nx, nz)
        return alpha, R_mc

    def _estimate_base_pose_from_one_marker(
        self,
        tvec: np.ndarray,
        rvec: np.ndarray,
        marker_world_xy: np.ndarray,
        marker_phi_deg: Optional[float],
        base_yaw: float,
    ) -> Optional[Tuple[np.ndarray, Optional[float], float]]:
        """Estimate robot (xy, yaw) from a single ArUco detection.

        Implements the derivation from Lecture 06 Slide 14:
          phi_B = phi_I_to_Mi - phi_Mi_to_C - phi_C_to_B
        where:
          phi_I_to_Mi = marker world-facing angle (phi_deg in marker map)
          phi_Mi_to_C = horizontal angle of the marker normal in the camera frame
          phi_C_to_B  = camera_in_base_yaw

        Returns (xy_estimate, vision_yaw, incidence) where vision_yaw is None if
        phi_deg is not available.
        """
        # OpenCV camera frame: x right, y down, z forward.
        # Convert to planar camera frame: x forward, y left.
        x_forward = float(tvec[2])
        y_left = -float(tvec[0])
        p_camera_to_marker_cam2d = np.array([x_forward, y_left], dtype=float)

        if not np.isfinite(p_camera_to_marker_cam2d).all():
            return None

        alpha, R_mc = self._compute_marker_yaw_in_cam(rvec)
        tvec_norm = float(np.linalg.norm(tvec))
        if tvec_norm > 1e-9:
            ray_unit = np.asarray(tvec, dtype=float) / tvec_norm
            normal_in = -R_mc[:, 2]
            cos_inc = float(np.clip(np.dot(ray_unit, normal_in), -1.0, 1.0))
            incidence = math.acos(cos_inc)
        else:
            incidence = float('nan')

        # Compute robot yaw from rvec and known marker world orientation
        vision_yaw: Optional[float] = None
        if marker_phi_deg is not None and np.isfinite(rvec).all():
            phi_marker_rad = math.radians(float(marker_phi_deg))
            vision_yaw = wrap_angle(phi_marker_rad - alpha - self.camera_in_base_yaw)

        # For position, the filter prior yaw is usually more stable than the
        # single-marker yaw estimate, especially for side-wall markers.
        if self.use_vision_yaw_for_position and vision_yaw is not None:
            effective_yaw = vision_yaw
        else:
            effective_yaw = base_yaw

        t_base_to_camera_in_base = np.array([self.camera_in_base_x, self.camera_in_base_y], dtype=float)

        p_world_camera = rot2(effective_yaw) @ t_base_to_camera_in_base
        p_world_cam_to_marker = rot2(effective_yaw + self.camera_in_base_yaw) @ p_camera_to_marker_cam2d

        p_world_base = marker_world_xy - p_world_camera - p_world_cam_to_marker
        return p_world_base, vision_yaw, incidence

    def _publish_fused_from_state(self, stamp) -> None:
        if self.kf is None:
            return

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = float(self.kf.x[0])
        msg.pose.pose.position.y = float(self.kf.x[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(float(self.kf.x[2]))

        cov = np.zeros((6, 6), dtype=float)
        cov[0, 0] = float(self.kf.P[0, 0])
        cov[1, 1] = float(self.kf.P[1, 1])
        cov[5, 5] = float(self.kf.P[2, 2])
        cov[0, 1] = float(self.kf.P[0, 1])
        cov[1, 0] = float(self.kf.P[1, 0])
        cov[0, 5] = float(self.kf.P[0, 2])
        cov[5, 0] = float(self.kf.P[2, 0])
        cov[1, 5] = float(self.kf.P[1, 2])
        cov[5, 1] = float(self.kf.P[2, 1])
        msg.pose.covariance = cov.reshape(-1).tolist()

        self.fused_pub.publish(msg)

    def _publish_vision_only(self, stamp, z_xy: np.ndarray) -> None:
        if self.kf is None:
            return

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = float(z_xy[0])
        msg.pose.pose.position.y = float(z_xy[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(float(self.kf.x[2]))

        cov = np.zeros((6, 6), dtype=float)
        cov[0, 0] = float(self.vision_r_diag[0])
        cov[1, 1] = float(self.vision_r_diag[1])
        cov[5, 5] = 10.0
        msg.pose.covariance = cov.reshape(-1).tolist()

        self.vision_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PittFusedOdemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
