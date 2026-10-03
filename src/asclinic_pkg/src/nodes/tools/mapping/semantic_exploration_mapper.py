#!/usr/bin/env python3

import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from asclinic_pkg.msg import FiducialMarkerArray, PlantDetections, ServoPulseWidth


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
    HEIGHT_M,
    LECTERNS,
    MARKERS,
    PLANTS,
    TABLES,
    WIDTH_M,
)


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


class SemanticExplorationMapper(Node):
    """Maintains a fog-of-war map and semantic plant/landmark discoveries."""

    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100

    def __init__(self) -> None:
        super().__init__('semantic_exploration_mapper')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_topic', 'pitt_fused_odometry')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('aruco_topic', 'aruco_detections')
        self.declare_parameter('plant_detections_topic', 'plant_detections')
        self.declare_parameter('unknown_plant_capture_topic', 'unknown_plant_capture')
        self.declare_parameter('camera_servo_topic', 'set_servo_pulse_width')
        self.declare_parameter('explored_map_topic', 'explored_map')
        self.declare_parameter('exploration_summary_topic', 'exploration_summary')
        self.declare_parameter('resolution_m', 0.10)
        self.declare_parameter('reveal_radius_m', 0.80)
        self.declare_parameter('marker_reveal_radius_m', 0.45)
        self.declare_parameter('camera_reveal_half_angle_deg', 30.0)
        self.declare_parameter('publish_period_sec', 0.50)
        self.declare_parameter('mark_static_prior_seen_on_start', True)
        self.declare_parameter('use_scan_for_fog', True)
        self.declare_parameter('mark_scan_obstacles', True)
        self.declare_parameter('scan_ray_stride', 4)
        self.declare_parameter('scan_free_step_m', 0.10)
        self.declare_parameter('scan_max_range_m', 5.0)
        self.declare_parameter('lidar_scan_angle_multiplier', 1.0)
        self.declare_parameter('lidar_in_base_yaw_deg', 0.0)
        self.declare_parameter('plant_yolo_min_confidence', 0.45)
        self.declare_parameter('plant_known_match_radius_m', 1.80)
        self.declare_parameter('plant_known_bearing_gate_deg', 35.0)
        self.declare_parameter('unknown_plant_merge_radius_m', 0.65)
        self.declare_parameter('unknown_plant_known_merge_radius_m', 1.00)
        self.declare_parameter('unknown_plant_assumed_range_m', 1.20)
        self.declare_parameter('unknown_plant_requires_unexplored_region', True)
        self.declare_parameter('unknown_plant_unexplored_check_radius_m', 0.30)
        self.declare_parameter('plant_bbox_image_width', 1920.0)
        self.declare_parameter('plant_bbox_fx_px', 1429.62)
        self.declare_parameter('camera_pan_channel', 14)
        self.declare_parameter('camera_pan_center_us', 1548)
        self.declare_parameter('camera_pan_min_us', 500)
        self.declare_parameter('camera_pan_max_us', 2500)
        self.declare_parameter('camera_pan_max_angle_deg', 180.0)
        self.declare_parameter('camera_pan_left_positive_is_decreasing', False)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.plant_detections_topic = str(
            self.get_parameter('plant_detections_topic').value
        )
        self.unknown_plant_capture_topic = str(
            self.get_parameter('unknown_plant_capture_topic').value
        ).strip()
        self.camera_servo_topic = str(self.get_parameter('camera_servo_topic').value)
        self.explored_map_topic = str(self.get_parameter('explored_map_topic').value)
        self.exploration_summary_topic = str(
            self.get_parameter('exploration_summary_topic').value
        )
        self.resolution = max(0.02, float(self.get_parameter('resolution_m').value))
        self.reveal_radius_m = max(0.0, float(self.get_parameter('reveal_radius_m').value))
        self.marker_reveal_radius_m = max(
            0.0,
            float(self.get_parameter('marker_reveal_radius_m').value),
        )
        self.camera_reveal_half_angle = math.radians(
            max(0.0, float(self.get_parameter('camera_reveal_half_angle_deg').value))
        )
        self.publish_period_sec = max(
            0.1,
            float(self.get_parameter('publish_period_sec').value),
        )
        self.mark_static_prior_seen_on_start = as_bool(
            self.get_parameter('mark_static_prior_seen_on_start').value
        )
        self.use_scan_for_fog = as_bool(self.get_parameter('use_scan_for_fog').value)
        self.mark_scan_obstacles = as_bool(
            self.get_parameter('mark_scan_obstacles').value
        )
        self.scan_ray_stride = max(1, int(self.get_parameter('scan_ray_stride').value))
        self.scan_free_step_m = max(0.02, float(self.get_parameter('scan_free_step_m').value))
        self.scan_max_range_m = max(0.05, float(self.get_parameter('scan_max_range_m').value))
        self.lidar_scan_angle_multiplier = float(
            self.get_parameter('lidar_scan_angle_multiplier').value
        )
        self.lidar_in_base_yaw = math.radians(
            float(self.get_parameter('lidar_in_base_yaw_deg').value)
        )
        self.plant_yolo_min_confidence = max(
            0.0,
            float(self.get_parameter('plant_yolo_min_confidence').value),
        )
        self.plant_known_match_radius_m = max(
            0.0,
            float(self.get_parameter('plant_known_match_radius_m').value),
        )
        self.plant_known_bearing_gate = math.radians(
            max(0.0, float(self.get_parameter('plant_known_bearing_gate_deg').value))
        )
        self.unknown_plant_merge_radius_m = max(
            0.05,
            float(self.get_parameter('unknown_plant_merge_radius_m').value),
        )
        self.unknown_plant_known_merge_radius_m = max(
            0.0,
            float(self.get_parameter('unknown_plant_known_merge_radius_m').value),
        )
        self.unknown_plant_assumed_range_m = max(
            0.10,
            float(self.get_parameter('unknown_plant_assumed_range_m').value),
        )
        self.unknown_plant_requires_unexplored_region = as_bool(
            self.get_parameter('unknown_plant_requires_unexplored_region').value
        )
        self.unknown_plant_unexplored_check_radius_m = max(
            0.0,
            float(
                self.get_parameter(
                    'unknown_plant_unexplored_check_radius_m'
                ).value
            ),
        )
        self.plant_bbox_image_width = max(
            1.0,
            float(self.get_parameter('plant_bbox_image_width').value),
        )
        self.plant_bbox_fx_px = max(
            1.0,
            float(self.get_parameter('plant_bbox_fx_px').value),
        )
        self.camera_pan_channel = int(self.get_parameter('camera_pan_channel').value)
        self.camera_pan_center_us = int(self.get_parameter('camera_pan_center_us').value)
        self.camera_pan_min_us = int(self.get_parameter('camera_pan_min_us').value)
        self.camera_pan_max_us = int(self.get_parameter('camera_pan_max_us').value)
        self.camera_pan_max_angle = math.radians(
            max(1.0, float(self.get_parameter('camera_pan_max_angle_deg').value))
        )
        self.camera_pan_left_positive_is_decreasing = as_bool(
            self.get_parameter('camera_pan_left_positive_is_decreasing').value
        )

        self.width_m = WIDTH_M
        self.height_m = HEIGHT_M
        self.width = int(math.ceil(self.width_m / self.resolution))
        self.height = int(math.ceil(self.height_m / self.resolution))
        self.prior_occupancy = np.zeros((self.height, self.width), dtype=np.int16)
        self.known_boundary_mask = np.zeros((self.height, self.width), dtype=bool)
        self.explored_grid = np.full(
            (self.height, self.width),
            self.UNKNOWN,
            dtype=np.int16,
        )
        self._build_static_prior()
        if self.mark_static_prior_seen_on_start:
            self.explored_grid[self.prior_occupancy == self.OCCUPIED] = self.OCCUPIED
        else:
            self.explored_grid[self.known_boundary_mask] = self.OCCUPIED

        self.marker_by_id = {
            int(marker_id): (float(x), float(y), float(z), float(yaw_deg))
            for marker_id, x, y, z, yaw_deg in MARKERS
        }
        self.plant_by_id = {
            str(plant_id): (float(x), float(y))
            for plant_id, x, y in PLANTS
        }

        self.latest_pose: tuple[float, float, float] | None = None
        self.latest_scan: LaserScan | None = None
        self.latest_scan_time_sec: float | None = None
        self.last_pan_us: int | None = None
        self.revealed_marker_ids: set[int] = set()
        self.confirmed_plants: dict[str, dict[str, float | str | int]] = {}
        self.unknown_plants: list[dict[str, float | str | int]] = []
        self.next_unknown_plant_index = 1
        self.last_yolo_observation: dict[str, float | str | int] | None = None

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.explored_map_topic,
            QoSProfile(depth=2),
        )
        self.summary_pub = self.create_publisher(
            String,
            self.exploration_summary_topic,
            QoSProfile(depth=5),
        )

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(
            FiducialMarkerArray,
            self.aruco_topic,
            self.aruco_callback,
            10,
        )
        self.create_subscription(
            PlantDetections,
            self.plant_detections_topic,
            self.plant_detections_callback,
            10,
        )
        if self.unknown_plant_capture_topic:
            self.create_subscription(
                String,
                self.unknown_plant_capture_topic,
                self.unknown_plant_capture_callback,
                10,
            )
        self.create_subscription(
            ServoPulseWidth,
            self.camera_servo_topic,
            self.camera_servo_callback,
            10,
        )
        if self.use_scan_for_fog:
            self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.scan_callback,
                qos_profile_sensor_data,
            )

        self.create_timer(self.publish_period_sec, self.publish_outputs)

        self.get_logger().info(
            '[EXPLORE] semantic fog map started map=%s summary=%s odom=%s scan=%s scan_max=%.2fm aruco=%s yolo=%s capture=%s camera_fov=+/-%.1fdeg'
            % (
                self.explored_map_topic,
                self.exploration_summary_topic,
                self.odom_topic,
                self.scan_topic if self.use_scan_for_fog else '<disabled>',
                self.scan_max_range_m,
                self.aruco_topic,
                self.plant_detections_topic,
                self.unknown_plant_capture_topic or '<disabled>',
                math.degrees(self.camera_reveal_half_angle),
            )
        )

    def _build_static_prior(self) -> None:
        wall_thickness = 0.10
        self._draw_rect(0.0, 0.0, self.width_m, wall_thickness, mark_boundary=True)
        self._draw_rect(
            0.0,
            self.height_m - wall_thickness,
            self.width_m,
            wall_thickness,
            mark_boundary=True,
        )
        self._draw_rect(0.0, 0.0, wall_thickness, self.height_m, mark_boundary=True)
        self._draw_rect(
            self.width_m - wall_thickness,
            0.0,
            wall_thickness,
            self.height_m,
            mark_boundary=True,
        )
        for _name, cx, cy, sx, sy in TABLES:
            self._draw_rect_center(cx, cy, sx, sy)
        for _name, cx, cy, sx, sy in LECTERNS:
            self._draw_rect_center(cx, cy, sx, sy)

    def _draw_rect_center(self, cx: float, cy: float, sx: float, sy: float) -> None:
        self._draw_rect(cx - sx / 2.0, cy - sy / 2.0, sx, sy)

    def _draw_rect(
        self,
        x: float,
        y: float,
        sx: float,
        sy: float,
        mark_boundary: bool = False,
    ) -> None:
        c0 = int(math.floor(x / self.resolution))
        c1 = int(math.ceil((x + sx) / self.resolution))
        r0 = int(math.floor(y / self.resolution))
        r1 = int(math.ceil((y + sy) / self.resolution))
        c0 = max(0, min(self.width, c0))
        c1 = max(0, min(self.width, c1))
        r0 = max(0, min(self.height, r0))
        r1 = max(0, min(self.height, r1))
        self.prior_occupancy[r0:r1, c0:c1] = self.OCCUPIED
        if mark_boundary:
            self.known_boundary_mask[r0:r1, c0:c1] = True

    def odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_pose = (x, y, yaw)
        self.reveal_camera_wedge(x, y, yaw, self.reveal_radius_m)

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if self.latest_pose is None:
            return
        x, y, yaw = self.latest_pose
        angle = float(msg.angle_min)
        for index, scan_range in enumerate(msg.ranges):
            if index % self.scan_ray_stride != 0:
                angle += float(msg.angle_increment)
                continue
            self.reveal_scan_ray(x, y, yaw, msg, angle, float(scan_range))
            angle += float(msg.angle_increment)

    def reveal_scan_ray(
        self,
        x: float,
        y: float,
        yaw: float,
        msg: LaserScan,
        scan_angle: float,
        scan_range: float,
    ) -> None:
        if not math.isfinite(scan_range) or scan_range <= 0.0:
            return
        max_range = min(
            self.scan_max_range_m,
            float(msg.range_max) if math.isfinite(float(msg.range_max)) else self.scan_max_range_m,
        )
        range_min = max(0.0, float(msg.range_min))
        endpoint_is_obstacle = range_min <= scan_range < max_range
        clipped_range = min(scan_range, max_range)
        global_angle = (
            yaw
            + self.lidar_in_base_yaw
            + self.lidar_scan_angle_multiplier * scan_angle
        )
        if not self.angle_in_camera_reveal_fov(global_angle, yaw):
            return
        free_range = max(0.0, clipped_range - 0.05)
        steps = max(1, int(math.ceil(free_range / self.scan_free_step_m)))
        for step in range(1, steps + 1):
            distance = min(free_range, step * self.scan_free_step_m)
            point_x = x + distance * math.cos(global_angle)
            point_y = y + distance * math.sin(global_angle)
            if self.prior_occupied_at(point_x, point_y):
                self.set_observed_occupied(point_x, point_y)
                return
            self.set_observed_free(point_x, point_y)
        if endpoint_is_obstacle and self.mark_scan_obstacles:
            self.set_observed_occupied(
                x + clipped_range * math.cos(global_angle),
                y + clipped_range * math.sin(global_angle),
            )

    def aruco_callback(self, msg: FiducialMarkerArray) -> None:
        for marker in msg.markers:
            marker_id = int(marker.id)
            if marker_id not in self.marker_by_id:
                continue
            self.revealed_marker_ids.add(marker_id)
            marker_x, marker_y, _z, _yaw = self.marker_by_id[marker_id]
            self.reveal_disk(
                marker_x,
                marker_y,
                self.marker_reveal_radius_m,
                camera_limited=True,
            )

    def camera_servo_callback(self, msg: ServoPulseWidth) -> None:
        if int(msg.channel) == self.camera_pan_channel:
            self.last_pan_us = int(msg.pulse_width_in_microseconds)
            if self.latest_pose is not None:
                x, y, yaw = self.latest_pose
                self.reveal_camera_wedge(x, y, yaw, self.reveal_radius_m)

    def plant_detections_callback(self, msg: PlantDetections) -> None:
        if self.latest_pose is None or int(msg.detection_count) <= 0 or not msg.detections:
            return
        detection = max(msg.detections, key=lambda item: float(item.confidence))
        confidence = float(detection.confidence)
        if confidence < self.plant_yolo_min_confidence:
            return

        x, y, yaw = self.latest_pose
        error_x = float(detection.centre_x) - 0.5 * self.plant_bbox_image_width
        bbox_bearing_left = -math.atan2(error_x, self.plant_bbox_fx_px)
        pan_left = self.pan_us_to_angle_left_positive(self.last_pan_us)
        global_bearing = wrap_to_pi(yaw + pan_left + bbox_bearing_left)
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9

        matched_plant = self.match_known_plant(x, y, global_bearing)
        if matched_plant is not None:
            plant_id, plant_x, plant_y, range_m, bearing_error = matched_plant
            self.confirm_known_plant(
                plant_id,
                plant_x,
                plant_y,
                confidence,
                now_sec,
                str(detection.class_name),
            )
            removed_count = self.remove_unknowns_near_known_plant(
                plant_id,
                plant_x,
                plant_y,
            )
            self.reveal_disk(plant_x, plant_y, 0.35, camera_limited=True)
            self.last_yolo_observation = {
                'type': 'known',
                'id': plant_id,
                'confidence': confidence,
                'range_m': range_m,
                'bearing_error_deg': math.degrees(bearing_error),
                'class_name': str(detection.class_name),
                'merged_unknown_count': removed_count,
            }
            return

        estimated_range = self.estimate_range_from_scan(global_bearing)
        if estimated_range is None:
            estimated_range = self.unknown_plant_assumed_range_m
        plant_x = x + estimated_range * math.cos(global_bearing)
        plant_y = y + estimated_range * math.sin(global_bearing)
        if not self.in_bounds_world(plant_x, plant_y):
            return

        known_by_position = self.nearest_known_plant_by_position(plant_x, plant_y)
        if (
            known_by_position is not None
            and known_by_position[3] <= self.unknown_plant_known_merge_radius_m
        ):
            plant_id, known_x, known_y, known_distance = known_by_position
            self.confirm_known_plant(
                plant_id,
                known_x,
                known_y,
                confidence,
                now_sec,
                str(detection.class_name),
            )
            removed_count = self.remove_unknowns_near_known_plant(
                plant_id,
                known_x,
                known_y,
            )
            self.reveal_disk(known_x, known_y, 0.35, camera_limited=True)
            self.last_yolo_observation = {
                'type': 'merged_unknown_with_known',
                'id': plant_id,
                'confidence': confidence,
                'x': known_x,
                'y': known_y,
                'candidate_x': plant_x,
                'candidate_y': plant_y,
                'known_distance_m': known_distance,
                'range_m': estimated_range,
                'class_name': str(detection.class_name),
                'merged_unknown_count': removed_count,
            }
            return

        nearby_unknown, nearby_unknown_distance = self.nearest_unknown_plant(
            plant_x,
            plant_y,
        )
        is_new_unknown = (
            nearby_unknown is None
            or nearby_unknown_distance > self.unknown_plant_merge_radius_m
        )
        if (
            is_new_unknown
            and self.unknown_plant_requires_unexplored_region
            and not self.has_unexplored_near(
                plant_x,
                plant_y,
                self.unknown_plant_unexplored_check_radius_m,
            )
        ):
            self.last_yolo_observation = {
                'type': 'ignored_unknown_in_explored_area',
                'confidence': confidence,
                'x': plant_x,
                'y': plant_y,
                'range_m': estimated_range,
                'class_name': str(detection.class_name),
            }
            return

        unknown = self.merge_unknown_plant(
            plant_x,
            plant_y,
            confidence,
            now_sec,
            str(detection.class_name),
        )
        self.reveal_disk(
            float(unknown['x']),
            float(unknown['y']),
            0.30,
            camera_limited=True,
        )
        self.last_yolo_observation = {
            'type': 'unknown_candidate',
            'id': str(unknown['id']),
            'confidence': confidence,
            'x': float(unknown['x']),
            'y': float(unknown['y']),
            'range_m': estimated_range,
            'class_name': str(detection.class_name),
        }

    def unknown_plant_capture_callback(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(
                '[EXPLORE] Ignoring malformed unknown plant capture JSON: %s' % exc
            )
            return
        if not isinstance(event, dict):
            return

        position = self.capture_event_world_position(event)
        if position is None:
            return
        plant_x, plant_y, range_m, range_source, global_bearing = position
        if not self.in_bounds_world(plant_x, plant_y):
            return

        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        confidence = self._event_float(event, 'confidence', 0.0) or 0.0
        class_name = str(event.get('class_name') or 'unknown')
        capture_id = str(event.get('id') or '').strip()
        known_by_position = self.nearest_known_plant_by_position(plant_x, plant_y)
        if (
            known_by_position is not None
            and known_by_position[3] <= self.unknown_plant_known_merge_radius_m
        ):
            plant_id, known_x, known_y, known_distance = known_by_position
            self.confirm_known_plant(
                plant_id,
                known_x,
                known_y,
                confidence,
                now_sec,
                class_name,
            )
            removed_count = self.remove_unknowns_near_known_plant(
                plant_id,
                known_x,
                known_y,
            )
            self.set_observed_occupied(known_x, known_y)
            self.reveal_disk(known_x, known_y, 0.35)
            self.last_yolo_observation = {
                'type': 'unknown_capture_merged_with_known',
                'id': plant_id,
                'capture_id': capture_id,
                'confidence': confidence,
                'x': known_x,
                'y': known_y,
                'candidate_x': plant_x,
                'candidate_y': plant_y,
                'known_distance_m': known_distance,
                'range_m': range_m,
                'range_source': range_source,
                'class_name': class_name,
                'merged_unknown_count': removed_count,
            }
            self.get_logger().info(
                '[EXPLORE] Unknown capture %s merged into known plant %s at (%.2f, %.2f), candidate distance=%.2fm.'
                % (
                    capture_id or '<unnamed>',
                    plant_id,
                    known_x,
                    known_y,
                    known_distance,
                )
            )
            return

        unknown = self.merge_unknown_plant(
            plant_x,
            plant_y,
            confidence,
            now_sec,
            class_name,
            preferred_id=capture_id,
            source='capture',
            range_m=range_m,
            range_source=range_source,
            global_bearing=global_bearing,
        )
        unknown['captures'] = int(unknown.get('captures', 0)) + 1
        unknown['last_capture_sec'] = now_sec
        unknown['capture_reason'] = str(event.get('reason') or '')
        self.set_observed_occupied(float(unknown['x']), float(unknown['y']))
        self.reveal_disk(float(unknown['x']), float(unknown['y']), 0.35)
        self.last_yolo_observation = {
            'type': 'unknown_capture',
            'id': str(unknown['id']),
            'confidence': confidence,
            'x': float(unknown['x']),
            'y': float(unknown['y']),
            'range_m': range_m,
            'range_source': range_source,
            'class_name': class_name,
        }
        capture_label = ''
        if capture_id and capture_id != str(unknown['id']):
            capture_label = ' from capture_id=%s' % capture_id
        self.get_logger().info(
            '[EXPLORE] Unknown plant %s captured%s at (%.2f, %.2f), range=%.2fm source=%s.'
            % (
                unknown['id'],
                capture_label,
                float(unknown['x']),
                float(unknown['y']),
                range_m,
                range_source,
            )
        )

    def capture_event_world_position(
        self,
        event: dict[str, object],
    ) -> tuple[float, float, float, str, float] | None:
        plant_x = self._event_float(event, 'x')
        plant_y = self._event_float(event, 'y')
        range_m = self._event_float(event, 'range_m')
        range_source = str(event.get('range_source') or 'event')
        global_bearing = self._event_float(event, 'global_bearing')
        if plant_x is not None and plant_y is not None:
            if range_m is None:
                robot_x = self._event_float(event, 'robot_x')
                robot_y = self._event_float(event, 'robot_y')
                if robot_x is not None and robot_y is not None:
                    range_m = math.hypot(plant_x - robot_x, plant_y - robot_y)
                else:
                    range_m = self.unknown_plant_assumed_range_m
            if global_bearing is None:
                robot_x = self._event_float(event, 'robot_x')
                robot_y = self._event_float(event, 'robot_y')
                if robot_x is not None and robot_y is not None:
                    global_bearing = math.atan2(plant_y - robot_y, plant_x - robot_x)
                else:
                    global_bearing = 0.0
            return plant_x, plant_y, range_m, range_source, global_bearing

        robot_x = self._event_float(event, 'robot_x')
        robot_y = self._event_float(event, 'robot_y')
        if robot_x is None or robot_y is None:
            return None
        if global_bearing is None:
            robot_yaw = self._event_float(event, 'robot_yaw', 0.0) or 0.0
            camera_pan_left = self._event_float(event, 'camera_pan_left_rad', 0.0) or 0.0
            bbox_bearing_left = self._event_float(event, 'bbox_bearing_left_rad', 0.0) or 0.0
            global_bearing = wrap_to_pi(robot_yaw + camera_pan_left + bbox_bearing_left)
        if range_m is None or not math.isfinite(range_m) or range_m <= 0.0:
            range_from_scan = self.estimate_range_from_scan(global_bearing)
            if range_from_scan is None:
                range_m = self.unknown_plant_assumed_range_m
                range_source = 'assumed'
            else:
                range_m = range_from_scan
                range_source = 'mapper_lidar'
        return (
            robot_x + range_m * math.cos(global_bearing),
            robot_y + range_m * math.sin(global_bearing),
            range_m,
            range_source,
            global_bearing,
        )

    def _event_float(
        self,
        event: dict[str, object],
        key: str,
        default: float | None = None,
    ) -> float | None:
        try:
            value = event.get(key, default)
            if value is None:
                return None
            return float(value)
        except (TypeError, ValueError):
            return default

    def pan_us_to_angle_left_positive(self, pulse_us: int | None) -> float:
        if pulse_us is None:
            return 0.0
        pulse = float(max(self.camera_pan_min_us, min(self.camera_pan_max_us, pulse_us)))
        center = float(self.camera_pan_center_us)
        if pulse <= center:
            denom = max(1.0, center - float(self.camera_pan_min_us))
            internal_angle = ((center - pulse) / denom) * self.camera_pan_max_angle
        else:
            denom = max(1.0, float(self.camera_pan_max_us) - center)
            internal_angle = -((pulse - center) / denom) * self.camera_pan_max_angle
        if self.camera_pan_left_positive_is_decreasing:
            return internal_angle
        return -internal_angle

    def camera_center_yaw(self, robot_yaw: float) -> float:
        return wrap_to_pi(
            robot_yaw + self.pan_us_to_angle_left_positive(self.last_pan_us)
        )

    def angle_in_camera_reveal_fov(
        self,
        global_angle: float,
        robot_yaw: float,
    ) -> bool:
        if self.camera_reveal_half_angle >= math.pi:
            return True
        camera_center = self.camera_center_yaw(robot_yaw)
        return (
            abs(wrap_to_pi(global_angle - camera_center))
            <= self.camera_reveal_half_angle
        )

    def point_in_camera_reveal_fov(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        target_x: float,
        target_y: float,
    ) -> bool:
        dx = target_x - robot_x
        dy = target_y - robot_y
        if math.hypot(dx, dy) <= 1.0e-6:
            return True
        return self.angle_in_camera_reveal_fov(math.atan2(dy, dx), robot_yaw)

    def match_known_plant(
        self,
        robot_x: float,
        robot_y: float,
        global_bearing: float,
    ) -> tuple[str, float, float, float, float] | None:
        best = None
        best_score = float('inf')
        for plant_id, (plant_x, plant_y) in self.plant_by_id.items():
            dx = plant_x - robot_x
            dy = plant_y - robot_y
            distance = math.hypot(dx, dy)
            if distance > self.plant_known_match_radius_m:
                continue
            plant_bearing = math.atan2(dy, dx)
            bearing_error = abs(wrap_to_pi(plant_bearing - global_bearing))
            if bearing_error > self.plant_known_bearing_gate:
                continue
            score = bearing_error + 0.25 * distance
            if score < best_score:
                best_score = score
                best = (plant_id, plant_x, plant_y, distance, bearing_error)
        return best

    def nearest_known_plant_by_position(
        self,
        x: float,
        y: float,
    ) -> tuple[str, float, float, float] | None:
        best = None
        best_distance = float('inf')
        for plant_id, (plant_x, plant_y) in self.plant_by_id.items():
            distance = math.hypot(plant_x - x, plant_y - y)
            if distance < best_distance:
                best_distance = distance
                best = (plant_id, plant_x, plant_y, distance)
        return best

    def confirm_known_plant(
        self,
        plant_id: str,
        plant_x: float,
        plant_y: float,
        confidence: float,
        now_sec: float,
        class_name: str = '',
    ) -> dict[str, float | str | int]:
        entry = self.confirmed_plants.setdefault(
            plant_id,
            {
                'id': plant_id,
                'x': plant_x,
                'y': plant_y,
                'observations': 0,
                'best_confidence': 0.0,
                'first_seen_sec': now_sec,
                'last_seen_sec': now_sec,
            },
        )
        entry['observations'] = int(entry['observations']) + 1
        entry['best_confidence'] = max(float(entry['best_confidence']), confidence)
        entry['last_seen_sec'] = now_sec
        if class_name:
            entry['class_name'] = class_name
        return entry

    def remove_unknowns_near_known_plant(
        self,
        plant_id: str,
        plant_x: float,
        plant_y: float,
    ) -> int:
        if self.unknown_plant_known_merge_radius_m <= 0.0 or not self.unknown_plants:
            return 0

        kept = []
        removed = []
        for unknown in self.unknown_plants:
            distance = math.hypot(float(unknown['x']) - plant_x, float(unknown['y']) - plant_y)
            if distance <= self.unknown_plant_known_merge_radius_m:
                removed.append((unknown, distance))
            else:
                kept.append(unknown)
        self.unknown_plants = kept
        if removed:
            ids = ', '.join(str(item[0].get('id', 'U?')) for item in removed)
            nearest = min(distance for _unknown, distance in removed)
            self.get_logger().info(
                '[EXPLORE] Merged unknown plant candidate(s) %s into known plant %s; nearest distance=%.2fm.'
                % (ids, plant_id, nearest)
            )
        return len(removed)

    def prune_unknowns_near_known_plants(self) -> int:
        if self.unknown_plant_known_merge_radius_m <= 0.0 or not self.unknown_plants:
            return 0

        total_removed = 0
        for plant_id, (plant_x, plant_y) in self.plant_by_id.items():
            total_removed += self.remove_unknowns_near_known_plant(
                plant_id,
                plant_x,
                plant_y,
            )
        return total_removed

    def estimate_range_from_scan(self, global_bearing: float) -> float | None:
        if self.latest_pose is None or self.latest_scan is None:
            return None
        if self.latest_scan_time_sec is not None:
            now_sec = self.get_clock().now().nanoseconds * 1.0e-9
            if now_sec - self.latest_scan_time_sec > 1.0:
                return None
        _x, _y, yaw = self.latest_pose
        scan = self.latest_scan
        local_angle = wrap_to_pi(global_bearing - yaw - self.lidar_in_base_yaw)
        if abs(self.lidar_scan_angle_multiplier) > 1.0e-6:
            local_angle /= self.lidar_scan_angle_multiplier
        index = int(round((local_angle - float(scan.angle_min)) / float(scan.angle_increment)))
        if index < 0 or index >= len(scan.ranges):
            return None
        scan_range = float(scan.ranges[index])
        if not math.isfinite(scan_range):
            return None
        if scan_range < float(scan.range_min) or scan_range > self.scan_max_range_m:
            return None
        return scan_range

    def has_unexplored_near(self, x: float, y: float, radius: float) -> bool:
        center = self.world_to_cell(x, y)
        if center is None:
            return False

        row_center, col_center = center
        radius_cells = int(math.ceil(radius / self.resolution))
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                row = row_center + dr
                col = col_center + dc
                if not self.in_bounds_cell(row, col):
                    continue
                if math.hypot(dr * self.resolution, dc * self.resolution) > radius:
                    continue
                if self.explored_grid[row, col] == self.UNKNOWN:
                    return True
        return False

    def nearest_unknown_plant(
        self,
        x: float,
        y: float,
    ) -> tuple[dict[str, float | str | int] | None, float]:
        best = None
        best_distance = float('inf')
        for candidate in self.unknown_plants:
            distance = math.hypot(float(candidate['x']) - x, float(candidate['y']) - y)
            if distance < best_distance:
                best_distance = distance
                best = candidate
        return best, best_distance

    def merge_unknown_plant(
        self,
        x: float,
        y: float,
        confidence: float,
        now_sec: float,
        class_name: str,
        *,
        preferred_id: str = '',
        source: str = 'yolo',
        range_m: float | None = None,
        range_source: str = '',
        global_bearing: float | None = None,
    ) -> dict[str, float | str | int]:
        best, best_distance = self.nearest_unknown_plant(x, y)
        if best is None or best_distance > self.unknown_plant_merge_radius_m:
            plant_id = preferred_id if preferred_id and self.unknown_id_is_available(preferred_id) else ''
            if not plant_id:
                while not self.unknown_id_is_available(f'U{self.next_unknown_plant_index}'):
                    self.next_unknown_plant_index += 1
                plant_id = f'U{self.next_unknown_plant_index}'
                self.next_unknown_plant_index += 1
            elif plant_id.startswith('U') and plant_id[1:].isdigit():
                self.next_unknown_plant_index = max(
                    self.next_unknown_plant_index,
                    int(plant_id[1:]) + 1,
                )
            best = {
                'id': plant_id,
                'x': x,
                'y': y,
                'observations': 0,
                'best_confidence': 0.0,
                'class_name': class_name,
                'source': source,
                'first_seen_sec': now_sec,
                'last_seen_sec': now_sec,
            }
            self.unknown_plants.append(best)

        observations = int(best['observations']) + 1
        alpha = 1.0 / float(observations)
        best['x'] = (1.0 - alpha) * float(best['x']) + alpha * x
        best['y'] = (1.0 - alpha) * float(best['y']) + alpha * y
        best['observations'] = observations
        best['best_confidence'] = max(float(best['best_confidence']), confidence)
        best['class_name'] = class_name
        best['source'] = source
        best['last_seen_sec'] = now_sec
        if range_m is not None:
            best['range_m'] = float(range_m)
        if range_source:
            best['range_source'] = range_source
        if global_bearing is not None:
            best['global_bearing'] = float(global_bearing)
        return best

    def unknown_id_is_available(self, plant_id: str) -> bool:
        return all(str(candidate.get('id')) != plant_id for candidate in self.unknown_plants)

    def reveal_camera_wedge(
        self,
        x: float,
        y: float,
        yaw: float,
        radius: float,
    ) -> None:
        center = self.world_to_cell(x, y)
        if center is None:
            return
        self.reveal_cell_from_prior(*center)
        row_center, col_center = center
        radius_cells = int(math.ceil(radius / self.resolution))
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                row = row_center + dr
                col = col_center + dc
                if not self.in_bounds_cell(row, col):
                    continue
                distance = math.hypot(dr * self.resolution, dc * self.resolution)
                if distance > radius:
                    continue
                cell_x, cell_y = self.cell_to_world(row, col)
                if not self.point_in_camera_reveal_fov(x, y, yaw, cell_x, cell_y):
                    continue
                if not self.static_line_of_sight_clear(x, y, cell_x, cell_y):
                    continue
                self.reveal_cell_from_prior(row, col)

    def reveal_disk(
        self,
        x: float,
        y: float,
        radius: float,
        *,
        camera_limited: bool = False,
    ) -> None:
        center = self.world_to_cell(x, y)
        if center is None:
            return
        camera_pose = self.latest_pose if camera_limited else None
        if camera_limited and camera_pose is None:
            return
        row_center, col_center = center
        radius_cells = int(math.ceil(radius / self.resolution))
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                row = row_center + dr
                col = col_center + dc
                if not self.in_bounds_cell(row, col):
                    continue
                distance = math.hypot(dr * self.resolution, dc * self.resolution)
                if distance > radius:
                    continue
                if camera_pose is not None:
                    robot_x, robot_y, robot_yaw = camera_pose
                    cell_x, cell_y = self.cell_to_world(row, col)
                    if not self.point_in_camera_reveal_fov(
                        robot_x,
                        robot_y,
                        robot_yaw,
                        cell_x,
                        cell_y,
                    ):
                        continue
                    if not self.static_line_of_sight_clear(
                        robot_x,
                        robot_y,
                        cell_x,
                        cell_y,
                    ):
                        continue
                self.reveal_cell_from_prior(row, col)

    def static_line_of_sight_clear(
        self,
        source_x: float,
        source_y: float,
        target_x: float,
        target_y: float,
    ) -> bool:
        target_cell = self.world_to_cell(target_x, target_y)
        if target_cell is None:
            return False

        distance = math.hypot(target_x - source_x, target_y - source_y)
        if distance <= 1.0e-6:
            return True

        step_m = max(0.02, 0.5 * self.resolution)
        steps = max(1, int(math.ceil(distance / step_m)))
        for step in range(1, steps + 1):
            ratio = min(1.0, step / steps)
            sample_x = source_x + ratio * (target_x - source_x)
            sample_y = source_y + ratio * (target_y - source_y)
            sample_cell = self.world_to_cell(sample_x, sample_y)
            if sample_cell is None:
                return False
            row, col = sample_cell
            if self.prior_occupancy[row, col] == self.OCCUPIED:
                return sample_cell == target_cell
        return True

    def reveal_cell_from_prior(self, row: int, col: int) -> None:
        self.explored_grid[row, col] = (
            self.OCCUPIED
            if self.prior_occupancy[row, col] == self.OCCUPIED
            else self.FREE
        )

    def set_observed_free(self, x: float, y: float) -> None:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return
        row, col = cell
        self.reveal_cell_from_prior(row, col)

    def set_observed_occupied(self, x: float, y: float) -> None:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return
        row, col = cell
        self.explored_grid[row, col] = self.OCCUPIED

    def prior_occupied_at(self, x: float, y: float) -> bool:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return True
        row, col = cell
        return bool(self.prior_occupancy[row, col] == self.OCCUPIED)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int(math.floor(x / self.resolution))
        row = int(math.floor(y / self.resolution))
        if not self.in_bounds_cell(row, col):
            return None
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        return (
            (col + 0.5) * self.resolution,
            (row + 0.5) * self.resolution,
        )

    def in_bounds_world(self, x: float, y: float) -> bool:
        return 0.0 <= x <= self.width_m and 0.0 <= y <= self.height_m

    def in_bounds_cell(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def publish_outputs(self) -> None:
        self.publish_map()
        self.publish_summary()

    def publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.explored_grid.astype(np.int8).flatten().tolist()
        self.map_pub.publish(msg)

    def publish_summary(self) -> None:
        self.prune_unknowns_near_known_plants()
        explored = self.explored_grid != self.UNKNOWN
        coverage_ratio = float(np.count_nonzero(explored)) / float(self.width * self.height)
        frontiers = self.compute_frontier_summary()
        summary = {
            'stamp_sec': self.get_clock().now().nanoseconds * 1.0e-9,
            'map_frame': self.map_frame,
            'coverage_ratio': coverage_ratio,
            'coverage_percent': 100.0 * coverage_ratio,
            'explored_cells': int(np.count_nonzero(explored)),
            'total_cells': int(self.width * self.height),
            'frontier_count': int(frontiers['count']),
            'nearest_frontier': frontiers.get('nearest'),
            'revealed_markers': sorted(int(item) for item in self.revealed_marker_ids),
            'confirmed_plants': sorted(
                self.confirmed_plants.values(),
                key=lambda item: str(item['id']),
            ),
            'unknown_plants': sorted(
                self.unknown_plants,
                key=lambda item: str(item['id']),
            ),
            'latest_yolo_observation': self.last_yolo_observation,
        }
        msg = String()
        msg.data = json.dumps(summary, sort_keys=True)
        self.summary_pub.publish(msg)

    def compute_frontier_summary(self) -> dict[str, object]:
        free_mask = self.explored_grid == self.FREE
        unknown_mask = self.explored_grid == self.UNKNOWN
        frontier_cells: list[tuple[int, int]] = []
        for row in range(1, self.height - 1):
            for col in range(1, self.width - 1):
                if not free_mask[row, col]:
                    continue
                if (
                    unknown_mask[row - 1, col]
                    or unknown_mask[row + 1, col]
                    or unknown_mask[row, col - 1]
                    or unknown_mask[row, col + 1]
                ):
                    frontier_cells.append((row, col))

        nearest = None
        if self.latest_pose is not None and frontier_cells:
            robot_x, robot_y, _yaw = self.latest_pose
            best_cell = min(
                frontier_cells,
                key=lambda cell: math.hypot(
                    self.cell_to_world(cell[0], cell[1])[0] - robot_x,
                    self.cell_to_world(cell[0], cell[1])[1] - robot_y,
                ),
            )
            fx, fy = self.cell_to_world(best_cell[0], best_cell[1])
            nearest = {
                'x': fx,
                'y': fy,
                'distance_m': math.hypot(fx - robot_x, fy - robot_y),
            }
        return {
            'count': len(frontier_cells),
            'nearest': nearest,
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticExplorationMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
