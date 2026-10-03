#!/usr/bin/env python3
"""ArUco-based localization validator / diagnostic node.

Place the robot at a known ground-truth (gt_x, gt_y, gt_yaw) on the
WORLD_COORDINATE_SYSTEM map, run this node, and it will:

  1. Subscribe to ``aruco_detections`` (FiducialMarkerArray).
  2. For every detected marker (whose ID is in the world-map), reconstruct
     the robot pose in the world frame using the same single-marker math
     as ``pitt_fused_odemetry.py``.
  3. Per-detection log raw range / bearing / rvec / back-computed pose /
     errors versus ground truth.
  4. After ``num_samples`` total frames have been received (independent of
     how many markers each frame had), append a detail CSV row per
     detection, print a per-marker mean / std summary, then shut down.

This is purely diagnostic — it does not publish any odometry.
"""

import csv
import math
import os
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from asclinic_pkg.msg import FiducialMarkerArray


# Reuse the canonical ASClinic Final Demonstration 2026 marker world map.
DEFAULT_MARKER_MAP = (
    '1:0.00,1.00,0.30,0;'
    '2:0.00,3.60,0.30,0;'
    '3:0.00,6.60,0.30,0;'
    '4:0.00,9.60,0.30,0;'
    '5:15.00,1.00,0.30,180;'
    '6:15.00,3.60,0.30,180;'
    '7:15.00,6.60,0.30,180;'
    '8:15.00,9.60,0.30,180;'
    '9:5.00,0.00,0.30,90;'
    '10:8.00,0.00,0.30,90;'
    '11:11.00,0.00,0.30,90;'
    '12:14.00,0.00,0.30,90;'
    '13:5.00,10.80,0.30,-90;'
    '14:8.00,10.80,0.30,-90;'
    '15:11.00,10.80,0.30,-90;'
    '16:14.00,10.80,0.30,-90;'
    '18:8.20,5.20,0.30,0;'
    '19:7.80,5.20,0.30,180;'
    '20:8.00,5.40,0.30,90;'
    '22:8.00,5.00,0.30,-90;'
    '27:1.50,3.60,0.30,90;'
    '29:8.00,1.30,0.30,180'
)


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def rodrigues(rvec: np.ndarray) -> np.ndarray:
    """Rodrigues' rotation formula. Returns the 3x3 rotation matrix R such that
    a point p_marker in the marker frame maps to R @ p_marker in the camera frame.
    """
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


CSV_HEADERS = [
    'label', 'gt_x', 'gt_y', 'gt_yaw_deg',
    'frame_idx', 'marker_id',
    # Raw camera-frame measurements
    'tvec_x', 'tvec_y', 'tvec_z',
    'rvec_x', 'rvec_y', 'rvec_z',
    'raw_range_3d', 'raw_range_planar', 'raw_bearing_deg',
    # What the world map predicts the robot SHOULD see if placed at gt
    'expected_range_planar', 'expected_bearing_deg',
    # Marker normal angle in camera frame (horizontal); incidence between
    # camera-to-marker ray and -normal (0 = perfectly face-on, 90 = edge-on)
    'marker_normal_cam_deg', 'incidence_deg',
    # Back-computed robot pose
    'est_x', 'est_y', 'est_yaw_deg',
    # Errors
    'err_x', 'err_y', 'err_dist', 'err_yaw_deg',
    # Marker world reference
    'marker_world_x', 'marker_world_y', 'marker_phi_deg',
]


class PittArucoLocalizationValidator(Node):
    def __init__(self) -> None:
        super().__init__('pitt_aruco_localization_validator')

        # Ground truth pose (world frame) where the robot is placed.
        self.declare_parameter('gt_x', 0.0)
        self.declare_parameter('gt_y', 0.0)
        self.declare_parameter('gt_yaw_deg', 0.0)
        self.declare_parameter('label', 'P_unnamed')

        # Sampling control.
        self.declare_parameter('num_samples', 30)

        # Camera extrinsics in base frame [m, m, rad]. Defaults match the
        # user's confirmed setup: camera mounted at base origin (z height
        # only — irrelevant for planar back-projection).
        self.declare_parameter('camera_in_base_x', 0.0)
        self.declare_parameter('camera_in_base_y', 0.0)
        self.declare_parameter('camera_in_base_yaw', 0.041)
        self.declare_parameter('use_vision_yaw_for_position', False)

        # Marker world map (same string format as pitt_fused_odemetry).
        self.declare_parameter('marker_world_map', DEFAULT_MARKER_MAP)

        # Aruco topic and CSV output.
        self.declare_parameter('aruco_topic', 'aruco_detections')
        # Optional comma-separated allow-list, e.g. "2,3,4,6".
        # Empty means use every marker present in marker_world_map.
        self.declare_parameter('marker_ids', '')
        self.declare_parameter(
            'csv_path',
            os.path.expanduser(
                '~/asclinic-ros2/ros2_ws/src/asclinic_pkg/Pitt_A1/'
                'aruco_localization_validation.csv'
            ),
        )
        # If true, ignore detections with off-axis bearing > this threshold.
        self.declare_parameter('off_axis_skip_deg', 60.0)
        # Skip markers whose incidence angle (angle between camera ray and
        # marker normal) exceeds this threshold. ArUco can still 'detect'
        # markers viewed edge-on, but pose extraction is unreliable.
        self.declare_parameter('incidence_skip_deg', 70.0)

        self.gt_x = float(self.get_parameter('gt_x').value)
        self.gt_y = float(self.get_parameter('gt_y').value)
        self.gt_yaw = math.radians(float(self.get_parameter('gt_yaw_deg').value))
        self.label = str(self.get_parameter('label').value)
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.cam_x = float(self.get_parameter('camera_in_base_x').value)
        self.cam_y = float(self.get_parameter('camera_in_base_y').value)
        self.cam_yaw = float(self.get_parameter('camera_in_base_yaw').value)
        self.use_vision_yaw_for_position = self._parameter_to_bool(
            self.get_parameter('use_vision_yaw_for_position').value
        )
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.marker_ids = self._parse_marker_ids(
            str(self.get_parameter('marker_ids').value)
        )
        self.csv_path = str(self.get_parameter('csv_path').value)
        self.off_axis_skip = math.radians(
            float(self.get_parameter('off_axis_skip_deg').value)
        )
        self.incidence_skip = math.radians(
            float(self.get_parameter('incidence_skip_deg').value)
        )

        marker_map_text = str(self.get_parameter('marker_world_map').value)
        if not marker_map_text.strip():
            marker_map_text = DEFAULT_MARKER_MAP
            self.get_logger().info(
                '[VALIDATOR] marker_world_map param empty -> using built-in DEFAULT_MARKER_MAP'
            )
        self.marker_world_map = self._parse_marker_map(marker_map_text)

        # Per-detection rows and per-marker accumulators.
        self.rows: List[List] = []
        self.per_marker: Dict[int, List[Dict[str, float]]] = defaultdict(list)
        self.frame_count: int = 0
        self.done: bool = False

        self.sub = self.create_subscription(
            FiducialMarkerArray, self.aruco_topic, self.aruco_callback, 10
        )

        self.get_logger().info(
            f'[VALIDATOR] label={self.label} '
            f'gt=({self.gt_x:.3f}, {self.gt_y:.3f}, '
            f'{math.degrees(self.gt_yaw):.1f} deg)'
        )
        self.get_logger().info(
            f'[VALIDATOR] camera_in_base=({self.cam_x:.3f}, {self.cam_y:.3f}, '
            f'yaw={self.cam_yaw:.3f}) | num_samples={self.num_samples} | '
            f'topic={self.aruco_topic}'
        )
        self.get_logger().info(
            f'[VALIDATOR] xy yaw source='
            f'{"vision" if self.use_vision_yaw_for_position else "ground_truth"}'
        )
        self.get_logger().info(
            f'[VALIDATOR] marker_map_size={len(self.marker_world_map)} | '
            f'csv={self.csv_path}'
        )
        if self.marker_ids is not None:
            self.get_logger().info(
                f'[VALIDATOR] marker_ids={sorted(self.marker_ids)}'
            )

    # ------------------------------------------------------------------ utils
    def _parse_marker_map(self, text: str) -> Dict[int, Tuple[float, float, float, Optional[float]]]:
        result: Dict[int, Tuple[float, float, float, Optional[float]]] = {}
        if not text.strip():
            return result
        for entry in (e.strip() for e in text.split(';') if e.strip()):
            if ':' not in entry:
                continue
            id_text, val_text = entry.split(':', 1)
            fields = [v.strip() for v in val_text.split(',')]
            if len(fields) < 3:
                continue
            try:
                mid = int(id_text)
                xw = float(fields[0])
                yw = float(fields[1])
                zw = float(fields[2])
                phi: Optional[float] = float(fields[3]) if len(fields) >= 4 else None
                result[mid] = (xw, yw, zw, phi)
            except ValueError:
                self.get_logger().warn(f'[VALIDATOR] bad map entry: {entry}')
        return result

    def _parse_marker_ids(self, text: str) -> Optional[set]:
        if not text.strip():
            return None
        marker_ids = set()
        for item in text.split(','):
            item = item.strip()
            if not item:
                continue
            try:
                marker_ids.add(int(item))
            except ValueError:
                self.get_logger().warn(f'[VALIDATOR] bad marker id in marker_ids: {item}')
        return marker_ids if marker_ids else None

    def _parameter_to_bool(self, value) -> bool:
        if isinstance(value, str):
            return value.strip().lower() in ('true', '1', 'yes', 'on')
        return bool(value)

    def _compute_marker_yaw_in_cam(self, rvec: np.ndarray) -> Tuple[float, np.ndarray]:
        """Return (alpha, R_mc) where alpha is the horizontal angle of the marker's
        normal as seen in the camera frame.

        Camera frame: x right, y down, z forward.
        We project the marker normal (R_mc[:, 2]) onto the horizontal (x-z) plane
        and use atan2 of its x and z components. For a perfectly face-on marker
        the normal points back at the camera (-z), giving alpha = pi.
        """
        R_mc = rodrigues(rvec)
        nx = float(R_mc[0, 2])
        nz = float(R_mc[2, 2])
        alpha = math.atan2(nx, nz)
        return alpha, R_mc

    def _estimate_base_pose(
        self,
        tvec: np.ndarray,
        rvec: np.ndarray,
        marker_world_xy: np.ndarray,
        marker_phi_deg: Optional[float],
    ) -> Tuple[np.ndarray, Optional[float], float, float]:
        """Back-project a single ArUco detection to a robot world pose.

        Camera frame:  x right, y down, z forward.
        Planar camera frame: x forward, y left.

        Returns (xy_world, vision_yaw_or_None, marker_normal_cam_rad,
        incidence_rad).
        """
        x_forward = float(tvec[2])
        y_left = -float(tvec[0])
        p_cam_to_marker = np.array([x_forward, y_left], dtype=float)

        # Marker normal angle in camera frame (horizontal projection).
        alpha, R_mc = self._compute_marker_yaw_in_cam(rvec)

        # Incidence angle: angle between the camera-to-marker ray and the
        # *inward* marker normal (i.e. -R_mc[:,2]). 0 = face-on, 90 = edge-on.
        tvec_norm = float(np.linalg.norm(tvec))
        if tvec_norm > 1e-9:
            ray_unit = np.asarray(tvec, dtype=float) / tvec_norm
            normal_in = -R_mc[:, 2]
            cos_inc = float(np.clip(np.dot(ray_unit, normal_in), -1.0, 1.0))
            incidence = math.acos(cos_inc)
        else:
            incidence = float('nan')

        vision_yaw: Optional[float] = None
        if marker_phi_deg is not None and np.isfinite(rvec).all():
            phi_marker_rad = math.radians(float(marker_phi_deg))
            # marker_normal_world_angle = camera_world_yaw + alpha
            # camera_world_yaw = robot_yaw + cam_in_base_yaw
            # =>  robot_yaw = phi_marker - alpha - cam_in_base_yaw
            vision_yaw = wrap_angle(phi_marker_rad - alpha - self.cam_yaw)

        # For origin diagnostics, ground-truth yaw makes translational errors
        # visible without being mixed with single-marker yaw ambiguity.
        if self.use_vision_yaw_for_position and vision_yaw is not None:
            effective_yaw = vision_yaw
        else:
            effective_yaw = self.gt_yaw

        t_base_to_camera_in_base = np.array([self.cam_x, self.cam_y], dtype=float)
        p_world_camera = rot2(effective_yaw) @ t_base_to_camera_in_base
        p_world_cam_to_marker = rot2(effective_yaw + self.cam_yaw) @ p_cam_to_marker
        p_world_base = marker_world_xy - p_world_camera - p_world_cam_to_marker
        return p_world_base, vision_yaw, alpha, incidence

    # ---------------------------------------------------------------- callback
    def aruco_callback(self, msg: FiducialMarkerArray) -> None:
        if self.done:
            return

        self.frame_count += 1
        frame_idx = self.frame_count

        n_used_in_frame = 0
        for marker in msg.markers:
            mid = int(marker.id)
            if self.marker_ids is not None and mid not in self.marker_ids:
                continue
            if mid not in self.marker_world_map:
                continue
            mx, my, _mz, mphi = self.marker_world_map[mid]
            tvec = np.array(marker.tvec, dtype=float)
            rvec = np.array(marker.rvec, dtype=float)

            tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])
            range_3d = math.sqrt(tx * tx + ty * ty + tz * tz)
            range_planar = math.sqrt(tx * tx + tz * tz)
            bearing = math.atan2(-tx, tz) if tz > 1e-3 else float('nan')

            # What the map predicts the robot SHOULD see if it really is at gt.
            dx_world = mx - self.gt_x
            dy_world = my - self.gt_y
            expected_range = math.sqrt(dx_world * dx_world + dy_world * dy_world)
            expected_bearing = wrap_angle(
                math.atan2(dy_world, dx_world) - (self.gt_yaw + self.cam_yaw)
            )

            # Always run the geometry so we get marker_normal_cam_deg and
            # incidence_deg into the CSV, even if we ultimately reject.
            est_xy, vision_yaw, alpha, incidence = self._estimate_base_pose(
                tvec=tvec,
                rvec=rvec,
                marker_world_xy=np.array([mx, my], dtype=float),
                marker_phi_deg=mphi,
            )

            reject_reason = None
            if math.isfinite(bearing) and abs(bearing) > self.off_axis_skip:
                reject_reason = (
                    f'off_axis>{math.degrees(self.off_axis_skip):.0f}deg'
                )
            elif math.isfinite(incidence) and incidence > self.incidence_skip:
                reject_reason = (
                    f'incidence>{math.degrees(self.incidence_skip):.0f}deg'
                )

            if reject_reason is not None:
                est_xy = np.array([float('nan'), float('nan')])
                est_yaw_deg = float('nan')
                err_x = float('nan'); err_y = float('nan')
                err_dist = float('nan'); err_yaw_deg = float('nan')
                self.get_logger().info(
                    f'[VALIDATOR] reject id={mid} ({reject_reason}) '
                    f'bearing={math.degrees(bearing):.1f} '
                    f'incidence={math.degrees(incidence):.1f}'
                )
            else:
                est_yaw_deg = (
                    math.degrees(vision_yaw) if vision_yaw is not None else float('nan')
                )
                err_x = float(est_xy[0]) - self.gt_x
                err_y = float(est_xy[1]) - self.gt_y
                err_dist = math.sqrt(err_x * err_x + err_y * err_y)
                err_yaw_deg = (
                    math.degrees(wrap_angle(vision_yaw - self.gt_yaw))
                    if vision_yaw is not None
                    else float('nan')
                )

            row = [
                self.label, self.gt_x, self.gt_y, math.degrees(self.gt_yaw),
                frame_idx, mid,
                tx, ty, tz,
                float(rvec[0]), float(rvec[1]), float(rvec[2]),
                range_3d, range_planar,
                math.degrees(bearing) if math.isfinite(bearing) else float('nan'),
                expected_range, math.degrees(expected_bearing),
                math.degrees(alpha) if math.isfinite(alpha) else float('nan'),
                math.degrees(incidence) if math.isfinite(incidence) else float('nan'),
                float(est_xy[0]), float(est_xy[1]), est_yaw_deg,
                err_x, err_y, err_dist, err_yaw_deg,
                mx, my, mphi if mphi is not None else float('nan'),
            ]
            self.rows.append(row)

            self.per_marker[mid].append({
                'range_planar': range_planar,
                'bearing_deg': math.degrees(bearing) if math.isfinite(bearing) else float('nan'),
                'expected_range': expected_range,
                'expected_bearing_deg': math.degrees(expected_bearing),
                'incidence_deg': math.degrees(incidence) if math.isfinite(incidence) else float('nan'),
                'est_x': float(est_xy[0]),
                'est_y': float(est_xy[1]),
                'est_yaw_deg': est_yaw_deg,
                'err_x': err_x,
                'err_y': err_y,
                'err_dist': err_dist,
                'err_yaw_deg': err_yaw_deg,
            })
            if reject_reason is None:
                n_used_in_frame += 1

        self.get_logger().info(
            f'[VALIDATOR] frame {frame_idx}/{self.num_samples} '
            f'— {n_used_in_frame} mapped markers used (raw {msg.num_markers})'
        )

        if frame_idx >= self.num_samples:
            self.finalize()

    # ----------------------------------------------------------------- output
    def finalize(self) -> None:
        if self.done:
            return
        self.done = True

        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        write_header = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            if write_header:
                w.writerow(CSV_HEADERS)
            for row in self.rows:
                w.writerow(row)

        self.get_logger().info(
            f'[VALIDATOR] wrote {len(self.rows)} detection rows -> {self.csv_path}'
        )

        # Per-marker summary.
        self.get_logger().info(
            f'[VALIDATOR] === SUMMARY @ label={self.label} '
            f'gt=({self.gt_x:.3f}, {self.gt_y:.3f}, '
            f'{math.degrees(self.gt_yaw):.1f}deg) ==='
        )
        header = (
            f'{"id":>3} {"n":>3} '
            f'{"range":>7} {"exp_rng":>7} {"d_rng":>6} '
            f'{"brng":>6} {"exp_brng":>8} '
            f'{"incid":>6} '
            f'{"est_x":>7} {"est_y":>7} {"est_yaw":>7} '
            f'{"err_d":>6} {"err_yaw":>7} '
            f'{"std_d":>6} {"std_yaw":>7}'
        )
        self.get_logger().info(header)

        def _stat(values: List[float], op) -> float:
            arr = [v for v in values if math.isfinite(v)]
            return float(op(arr)) if arr else float('nan')

        for mid in sorted(self.per_marker.keys()):
            rows = self.per_marker[mid]
            n = len(rows)
            mean_range = _stat([r['range_planar'] for r in rows], np.mean)
            mean_exp_range = _stat([r['expected_range'] for r in rows], np.mean)
            d_range = (mean_range - mean_exp_range) if (math.isfinite(mean_range) and math.isfinite(mean_exp_range)) else float('nan')
            line = (
                f'{mid:>3d} {n:>3d} '
                f'{mean_range:>7.3f} '
                f'{mean_exp_range:>7.3f} '
                f'{d_range:>+6.3f} '
                f'{_stat([r["bearing_deg"] for r in rows], np.mean):>+6.1f} '
                f'{_stat([r["expected_bearing_deg"] for r in rows], np.mean):>+8.1f} '
                f'{_stat([r["incidence_deg"] for r in rows], np.mean):>6.1f} '
                f'{_stat([r["est_x"] for r in rows], np.mean):>+7.3f} '
                f'{_stat([r["est_y"] for r in rows], np.mean):>+7.3f} '
                f'{_stat([r["est_yaw_deg"] for r in rows], np.mean):>+7.2f} '
                f'{_stat([r["err_dist"] for r in rows], np.mean):>6.3f} '
                f'{_stat([r["err_yaw_deg"] for r in rows], np.mean):>+7.2f} '
                f'{_stat([r["err_dist"] for r in rows], np.std):>6.3f} '
                f'{_stat([r["err_yaw_deg"] for r in rows], np.std):>7.2f}'
            )
            self.get_logger().info(line)

        self.get_logger().info('[VALIDATOR] done — shutting down')
        # Trigger graceful exit on the next spin.
        self.create_timer(0.2, lambda: rclpy.shutdown())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PittArucoLocalizationValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
