#!/usr/bin/env python3

import json
import math
import sys
import time
from pathlib import Path
from typing import Iterable

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from asclinic_pkg.msg import (
    FiducialMarkerArray,
    LeftRightFloat32,
    MissionProgress,
    PlantDetections,
    ReferencePath,
    RobotState,
    ServoPulseWidth,
)


_NODES_DIR = Path(__file__).resolve().parents[1]
_EXECUTABLE_DIR = Path(__file__).resolve().parent
for _candidate in (_NODES_DIR, _EXECUTABLE_DIR):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

try:
    from controllers.lqg_controller import LQGController
    from controllers.mpc_controller import MPCController
    from controllers.pid_controller import PIDController
except Exception as exc:  # pragma: no cover - logged after rclpy node exists.
    LQGController = None
    MPCController = None
    PIDController = None
    CONTROLLER_BACKEND_IMPORT_ERROR = exc
else:
    CONTROLLER_BACKEND_IMPORT_ERROR = None


DEFAULT_PLANT_WORLD_MAP = (
    'P1:1.00,10.00;P2:5.00,4.00;P3:8.40,8.00;'
    'P4:10.00,5.00;P5:12.40,1.00;P6:14.60,8.00'
)

DEFAULT_INSPECTION_STOP_MAP = (
    'P1:1.00,9.30;P2:5.50,5.30;P3:8.40,6.75;'
    'P4:10.50,6.20;P6:14.50,7.00;P5:10.90,1.00'
)

DEFAULT_INSPECTION_DWELL_MAP = 'P6:6.0'


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q) -> float:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-9:
        return 0.0
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class TrackingControllerNode(Node):
    """
    Track a reference path with fused odometry and publish RoboClaw duty commands.

    The controller intentionally keeps the hardware interface compatible with the
    existing motor driver: it publishes LeftRightFloat32 duty-cycle percentages
    on set_motor_duty_cycle. It also gates plant detection with a Bool topic so
    YOLO inference only runs when the robot is near known plant locations.
    """

    def __init__(self) -> None:
        super().__init__('tracking_controller_node')

        self.declare_parameter('odom_topic', 'pitt_fused_odometry')
        self.declare_parameter('reference_path_topic', 'reference_path')
        self.declare_parameter('reference_path_msg_topic', '')
        self.declare_parameter('target_pose_topic', 'target_pose')
        self.declare_parameter('motor_cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('robot_state_topic', 'robot_state')
        self.declare_parameter('mission_progress_topic', 'mission_progress')
        self.declare_parameter('plant_detector_enable_topic', 'plant_detector/enabled')
        self.declare_parameter(
            'plant_positive_save_gate_topic',
            'plant_detector/save_positive_enabled',
        )
        self.declare_parameter('plant_detections_topic', 'plant_detections')
        self.declare_parameter('unknown_plant_capture_topic', 'unknown_plant_capture')
        self.declare_parameter('aruco_topic', 'aruco_detections')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('reset_mission_progress_on_new_path', False)

        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('path_controller_backend', 'direct')
        self.declare_parameter('nominal_speed', 0.12)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 1.8)
        self.declare_parameter('lookahead_distance', 0.45)
        self.declare_parameter('goal_tolerance', 0.18)
        self.declare_parameter('waypoint_reached_distance', 0.20)
        self.declare_parameter('slowdown_distance', 0.70)
        self.declare_parameter('curvature_gain', 1.0)
        self.declare_parameter('heading_gain', 1.8)
        self.declare_parameter('right_angle_corner_mode', True)
        self.declare_parameter('corner_angle_threshold', 0.70)
        self.declare_parameter('corner_hold_distance', 0.85)
        self.declare_parameter('corner_search_points', 35)
        self.declare_parameter('point_to_point_mode', False)
        self.declare_parameter('point_to_point_rotate_tolerance', 0.06)
        self.declare_parameter('point_to_point_arrival_tolerance', 0.04)
        self.declare_parameter('point_to_point_min_segment_length', 0.05)
        self.declare_parameter('stop_after_point_rotate_complete', True)
        self.declare_parameter('turn_forward_stop_threshold_duty', 4.0)
        self.declare_parameter('preserve_replan_motion_enabled', True)
        self.declare_parameter('preserve_replan_heading_tolerance_deg', 12.0)
        self.declare_parameter('preserve_replan_start_tolerance_m', 0.45)
        self.declare_parameter('segment_stop_turn_mode', False)
        self.declare_parameter('corner_arrival_distance', 0.22)
        self.declare_parameter('corner_turn_tolerance', 0.08)
        self.declare_parameter('corner_turn_kp', 12.0)
        self.declare_parameter('corner_turn_min_duty', 8.0)
        self.declare_parameter('corner_turn_max_duty', 12.0)
        self.declare_parameter('segment_straight_duty', 20.0)
        self.declare_parameter('segment_min_straight_duty', 15.0)
        self.declare_parameter('segment_yaw_kp', 14.0)
        self.declare_parameter('segment_max_correction', 4.0)
        self.declare_parameter('segment_yaw_deadband', 0.02)
        self.declare_parameter('segment_lateral_kp', 0.0)
        self.declare_parameter('segment_lateral_deadband_m', 0.03)
        self.declare_parameter('segment_lateral_max_heading_deg', 18.0)
        self.declare_parameter('segment_deceleration_distance', 0.70)
        self.declare_parameter('segment_velocity_profile_enabled', False)
        self.declare_parameter('segment_profile_cruise_duty', 35.0)
        self.declare_parameter('segment_profile_approach_duty', 25.0)
        self.declare_parameter('segment_profile_accel_duty_per_sec', 50.0)
        self.declare_parameter('rotate_in_place_angle', 0.85)
        self.declare_parameter('rotate_in_place_min_angular_speed', 0.35)
        self.declare_parameter('backward_target_turn_direction', 1.0)

        self.declare_parameter('half_wheel_base', 0.109)
        self.declare_parameter('max_wheel_speed_at_full_duty', 1.50)
        self.declare_parameter('duty_cycle_limit', 28.0)
        self.declare_parameter('min_moving_duty', 18.0)
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 0.932)
        self.declare_parameter('duty_slew_rate_percent_per_sec', 55.0)
        self.declare_parameter('graceful_stop_enabled', True)
        self.declare_parameter('graceful_stop_decel_duty_per_sec', 35.0)
        self.declare_parameter('graceful_stop_turn_duty_per_sec', 120.0)
        self.declare_parameter('graceful_stop_zero_threshold_duty', 0.5)
        self.declare_parameter('command_filter_alpha', 0.0)

        self.declare_parameter('backend_delta_v_limit', 0.30)
        self.declare_parameter('backend_delta_omega_limit', 1.50)
        self.declare_parameter('lqg_q_lqr_diag', [5.0, 5.0, 1.0])
        self.declare_parameter('lqg_r_lqr_diag', [1.0, 0.5])
        self.declare_parameter('lqg_qn_diag', [1.0e-2, 1.0e-2, 1.0e-2])
        self.declare_parameter('lqg_rn_diag', [5.0e-2, 5.0e-2, 2.0e-2])
        self.declare_parameter('lqg_smooth_turn_enabled', True)
        self.declare_parameter('lqg_smooth_turn_angle_deg', 30.0)
        self.declare_parameter('lqg_stop_turn_angle_deg', 60.0)
        self.declare_parameter('lqg_min_smooth_turn_speed_scale', 0.45)
        self.declare_parameter('mpc_q_diag', [5.0, 5.0, 1.0])
        self.declare_parameter('mpc_r_diag', [1.0, 0.5])
        self.declare_parameter('mpc_prediction_horizon', 10)
        self.declare_parameter('pid_kp_x', 1.5)
        self.declare_parameter('pid_ki_x', 0.0)
        self.declare_parameter('pid_kd_x', 0.0)
        self.declare_parameter('pid_ky', 2.5)
        self.declare_parameter('pid_kphi', 1.4)
        self.declare_parameter('pid_kiy', 0.0)

        self.declare_parameter('odom_timeout_sec', 1.0)
        self.declare_parameter('path_timeout_sec', 5.0)
        self.declare_parameter('verbose', False)

        self.declare_parameter('visual_servo_enabled', True)
        self.declare_parameter(
            'visual_servo_guides',
            '9:5.00,-90;12:14.20,-90,1.65',
        )
        self.declare_parameter('visual_servo_lane_tolerance_m', 0.45)
        self.declare_parameter('visual_servo_heading_tolerance_deg', 12.0)
        self.declare_parameter('visual_servo_marker_timeout_sec', 0.75)
        self.declare_parameter('visual_servo_min_range_m', 0.40)
        self.declare_parameter('visual_servo_max_range_m', 7.00)
        self.declare_parameter('visual_servo_bearing_gain', 0.85)
        self.declare_parameter('visual_servo_max_bearing_correction_deg', 18.0)
        self.declare_parameter('visual_servo_speed_scale', 0.70)
        self.declare_parameter('visual_servo_acquire_speed_scale', 0.55)
        self.declare_parameter('visual_arrival_extra_distance_m', 0.80)
        self.declare_parameter('visual_arrival_hold_distance_m', 0.20)

        self.declare_parameter('inspection_stop_map', DEFAULT_INSPECTION_STOP_MAP)
        self.declare_parameter('inspection_dwell_sec', 3.0)
        self.declare_parameter('inspection_dwell_map', DEFAULT_INSPECTION_DWELL_MAP)
        self.declare_parameter('inspection_stop_heading_tolerance_deg', 35.0)
        self.declare_parameter('inspection_stop_capture_tolerance_m', 0.03)
        self.declare_parameter('inspection_stop_reached_tolerance_m', 0.20)
        self.declare_parameter('inspection_standoff_distance_m', 1.0)
        self.declare_parameter('inspection_standoff_tolerance_m', 0.25)

        self.declare_parameter('plant_world_map', DEFAULT_PLANT_WORLD_MAP)
        self.declare_parameter('plant_detection_enable_radius', 0.60)
        self.declare_parameter('plant_detection_disable_radius', 0.75)
        self.declare_parameter('plant_detector_hold_sec', 2.0)
        self.declare_parameter('plant_detector_always_enabled', True)
        self.declare_parameter('plant_completion_confidence_threshold', 0.60)
        self.declare_parameter('plant_completion_max_stop_distance_m', 1.60)
        self.declare_parameter('plant_completion_match_mode', 'any')
        self.declare_parameter('plant_completion_require_valid_capture', True)
        self.declare_parameter('plant_completion_require_dwell', True)

        self.declare_parameter('plant_bbox_servo_enabled', True)
        self.declare_parameter('plant_capture_radius_m', 0.60)
        self.declare_parameter('plant_detection_stale_timeout_sec', 2.50)
        self.declare_parameter('plant_bbox_image_width', 1920.0)
        self.declare_parameter('plant_bbox_image_height', 1080.0)
        self.declare_parameter('plant_bbox_fx_px', 1429.62)
        self.declare_parameter('plant_bbox_fy_px', 1429.00)
        self.declare_parameter('plant_bbox_center_tolerance_angle_deg', 15.0)
        self.declare_parameter('plant_bbox_fine_tolerance_angle_deg', 5.0)
        self.declare_parameter('plant_bbox_center_tolerance_x_px', 80.0)
        self.declare_parameter('plant_bbox_center_tolerance_y_px', 70.0)
        self.declare_parameter('plant_bbox_center_hold_sec', 1.00)
        self.declare_parameter('plant_capture_dwell_sec', 2.00)
        self.declare_parameter('plant_bbox_pan_gain_us_per_px', 0.18)
        self.declare_parameter('plant_bbox_tilt_gain_us_per_px', 0.14)
        self.declare_parameter('plant_bbox_fine_pan_gain_us_per_px', 0.04)
        self.declare_parameter('plant_bbox_fine_tilt_gain_us_per_px', 0.03)
        self.declare_parameter('plant_bbox_tilt_servo_enabled', False)
        self.declare_parameter('plant_search_pan_span_us', 500)
        self.declare_parameter('plant_search_pan_step_us', 170)
        self.declare_parameter('plant_search_update_period_sec', 1.50)
        self.declare_parameter('plant_bbox_update_period_sec', 0.45)
        self.declare_parameter('plant_opportunistic_capture_enabled', True)
        self.declare_parameter('plant_opportunistic_min_confidence', 0.50)
        self.declare_parameter('plant_opportunistic_assumed_distance_m', 1.20)
        self.declare_parameter('plant_opportunistic_known_plant_exclusion_m', 1.00)
        self.declare_parameter('plant_opportunistic_stop_exclusion_m', 1.00)
        self.declare_parameter('plant_opportunistic_duplicate_radius_m', 0.80)
        self.declare_parameter('plant_opportunistic_cooldown_sec', 8.0)
        self.declare_parameter('plant_opportunistic_lost_timeout_sec', 3.0)
        self.declare_parameter('plant_opportunistic_max_capture_sec', 12.0)
        self.declare_parameter('plant_opportunistic_initial_save_sec', 1.50)
        self.declare_parameter('plant_unknown_lidar_max_range_m', 4.0)
        self.declare_parameter('plant_unknown_lidar_neighbor_beams', 3)
        self.declare_parameter('plant_unknown_lidar_stale_timeout_sec', 0.75)
        self.declare_parameter('lidar_scan_angle_multiplier', 1.0)
        self.declare_parameter('lidar_in_base_yaw_deg', 0.0)

        self.declare_parameter('camera_auto_pan_enabled', True)
        self.declare_parameter('camera_pan_topic', 'set_servo_pulse_width')
        self.declare_parameter('camera_pan_channel', 14)
        self.declare_parameter('camera_pan_center_us', 1548)
        self.declare_parameter('camera_pan_min_us', 500)
        self.declare_parameter('camera_pan_max_us', 2500)
        self.declare_parameter('camera_pan_max_angle_deg', 180.0)
        self.declare_parameter('camera_pan_active_radius_m', 1.60)
        self.declare_parameter('camera_pan_enable_detector_gate', True)
        self.declare_parameter('camera_pan_update_period_sec', 0.20)
        self.declare_parameter('camera_pan_return_to_center', True)
        self.declare_parameter('camera_pan_idle_center_period_sec', 0.0)
        self.declare_parameter('camera_pan_left_positive_is_decreasing', False)
        self.declare_parameter('camera_tilt_channel', 13)
        self.declare_parameter('camera_tilt_center_us', 1400)
        self.declare_parameter('camera_tilt_min_us', 500)
        self.declare_parameter('camera_tilt_max_us', 2500)
        self.declare_parameter('camera_tilt_image_down_positive_increases_us', True)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.reference_path_topic = str(self.get_parameter('reference_path_topic').value)
        self.reference_path_msg_topic = str(
            self.get_parameter('reference_path_msg_topic').value
        )
        self.target_pose_topic = str(self.get_parameter('target_pose_topic').value)
        self.motor_cmd_topic = str(self.get_parameter('motor_cmd_topic').value)
        self.robot_state_topic = str(self.get_parameter('robot_state_topic').value)
        self.mission_progress_topic = str(
            self.get_parameter('mission_progress_topic').value
        )
        self.plant_detector_enable_topic = str(
            self.get_parameter('plant_detector_enable_topic').value
        )
        self.plant_positive_save_gate_topic = str(
            self.get_parameter('plant_positive_save_gate_topic').value
        ).strip()
        self.plant_detections_topic = str(
            self.get_parameter('plant_detections_topic').value
        )
        self.unknown_plant_capture_topic = str(
            self.get_parameter('unknown_plant_capture_topic').value
        ).strip()
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value).strip()
        self.reset_mission_progress_on_new_path = as_bool(
            self.get_parameter('reset_mission_progress_on_new_path').value
        )

        self.control_period = max(0.01, float(self.get_parameter('control_period').value))
        self.path_controller_backend = str(
            self.get_parameter('path_controller_backend').value
        ).strip().lower()
        if self.path_controller_backend in ('direct_control', 'default'):
            self.path_controller_backend = 'direct'
        if self.path_controller_backend not in ('direct', 'lqg', 'mpc', 'pid'):
            self.get_logger().warn(
                '[TRACKING] Unknown path_controller_backend=%s; using direct.'
                % self.path_controller_backend
            )
            self.path_controller_backend = 'direct'
        self.nominal_speed = float(self.get_parameter('nominal_speed').value)
        self.max_linear_speed = abs(float(self.get_parameter('max_linear_speed').value))
        self.max_angular_speed = abs(float(self.get_parameter('max_angular_speed').value))
        self.lookahead_distance = max(
            0.05, float(self.get_parameter('lookahead_distance').value)
        )
        self.goal_tolerance = max(0.02, float(self.get_parameter('goal_tolerance').value))
        self.waypoint_reached_distance = max(
            0.02, float(self.get_parameter('waypoint_reached_distance').value)
        )
        self.slowdown_distance = max(0.05, float(self.get_parameter('slowdown_distance').value))
        self.curvature_gain = max(0.0, float(self.get_parameter('curvature_gain').value))
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.right_angle_corner_mode = as_bool(
            self.get_parameter('right_angle_corner_mode').value
        )
        self.corner_angle_threshold = max(
            0.05, abs(float(self.get_parameter('corner_angle_threshold').value))
        )
        self.corner_hold_distance = max(
            self.waypoint_reached_distance,
            float(self.get_parameter('corner_hold_distance').value),
        )
        self.corner_search_points = max(
            3, int(self.get_parameter('corner_search_points').value)
        )
        self.point_to_point_mode = as_bool(
            self.get_parameter('point_to_point_mode').value
        )
        self.point_to_point_rotate_tolerance = max(
            0.02, abs(float(self.get_parameter('point_to_point_rotate_tolerance').value))
        )
        self.point_to_point_arrival_tolerance = max(
            0.02, abs(float(self.get_parameter('point_to_point_arrival_tolerance').value))
        )
        self.point_to_point_min_segment_length = max(
            0.01, abs(float(self.get_parameter('point_to_point_min_segment_length').value))
        )
        self.stop_after_point_rotate_complete = as_bool(
            self.get_parameter('stop_after_point_rotate_complete').value
        )
        self.turn_forward_stop_threshold_duty = max(
            0.0,
            abs(float(self.get_parameter('turn_forward_stop_threshold_duty').value)),
        )
        self.preserve_replan_motion_enabled = as_bool(
            self.get_parameter('preserve_replan_motion_enabled').value
        )
        self.preserve_replan_heading_tolerance = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter('preserve_replan_heading_tolerance_deg').value
                ),
            )
        )
        self.preserve_replan_start_tolerance_m = max(
            0.0,
            float(self.get_parameter('preserve_replan_start_tolerance_m').value),
        )
        self.segment_stop_turn_mode = as_bool(
            self.get_parameter('segment_stop_turn_mode').value
        )
        self.corner_arrival_distance = max(
            self.waypoint_reached_distance,
            float(self.get_parameter('corner_arrival_distance').value),
        )
        self.corner_turn_tolerance = max(
            0.02, abs(float(self.get_parameter('corner_turn_tolerance').value))
        )
        self.corner_turn_kp = max(0.0, float(self.get_parameter('corner_turn_kp').value))
        self.corner_turn_min_duty = max(
            0.0, abs(float(self.get_parameter('corner_turn_min_duty').value))
        )
        self.corner_turn_max_duty = max(
            self.corner_turn_min_duty,
            abs(float(self.get_parameter('corner_turn_max_duty').value)),
        )
        self.segment_straight_duty = max(
            0.0, abs(float(self.get_parameter('segment_straight_duty').value))
        )
        self.segment_min_straight_duty = max(
            0.0, abs(float(self.get_parameter('segment_min_straight_duty').value))
        )
        self.segment_min_straight_duty = min(
            self.segment_min_straight_duty,
            self.segment_straight_duty,
        )
        self.segment_yaw_kp = float(self.get_parameter('segment_yaw_kp').value)
        self.segment_max_correction = max(
            0.0, abs(float(self.get_parameter('segment_max_correction').value))
        )
        self.segment_yaw_deadband = max(
            0.0, abs(float(self.get_parameter('segment_yaw_deadband').value))
        )
        self.segment_lateral_kp = float(self.get_parameter('segment_lateral_kp').value)
        self.segment_lateral_deadband_m = max(
            0.0,
            abs(float(self.get_parameter('segment_lateral_deadband_m').value)),
        )
        self.segment_lateral_max_heading = math.radians(
            max(
                0.0,
                abs(float(self.get_parameter('segment_lateral_max_heading_deg').value)),
            )
        )
        self.segment_deceleration_distance = max(
            0.0, float(self.get_parameter('segment_deceleration_distance').value)
        )
        self.segment_velocity_profile_enabled = as_bool(
            self.get_parameter('segment_velocity_profile_enabled').value
        )
        self.segment_profile_cruise_duty = max(
            0.0, abs(float(self.get_parameter('segment_profile_cruise_duty').value))
        )
        self.segment_profile_approach_duty = max(
            0.0, abs(float(self.get_parameter('segment_profile_approach_duty').value))
        )
        self.segment_profile_approach_duty = min(
            self.segment_profile_approach_duty,
            self.segment_profile_cruise_duty,
        )
        self.segment_profile_accel_duty_per_sec = max(
            0.0,
            float(self.get_parameter('segment_profile_accel_duty_per_sec').value),
        )
        self.rotate_in_place_angle = abs(
            float(self.get_parameter('rotate_in_place_angle').value)
        )
        self.rotate_in_place_min_angular_speed = abs(
            float(self.get_parameter('rotate_in_place_min_angular_speed').value)
        )
        backward_turn_direction = float(
            self.get_parameter('backward_target_turn_direction').value
        )
        self.backward_target_turn_direction = 1.0 if backward_turn_direction >= 0.0 else -1.0

        self.half_wheel_base = abs(float(self.get_parameter('half_wheel_base').value))
        self.max_wheel_speed_at_full_duty = max(
            0.05, abs(float(self.get_parameter('max_wheel_speed_at_full_duty').value))
        )
        self.duty_cycle_limit = abs(float(self.get_parameter('duty_cycle_limit').value))
        self.min_moving_duty = max(0.0, abs(float(self.get_parameter('min_moving_duty').value)))
        self.left_trim = float(self.get_parameter('left_trim').value)
        self.right_trim = float(self.get_parameter('right_trim').value)
        self.duty_slew_rate = max(
            0.0, float(self.get_parameter('duty_slew_rate_percent_per_sec').value)
        )
        self.graceful_stop_enabled = as_bool(
            self.get_parameter('graceful_stop_enabled').value
        )
        self.graceful_stop_decel_duty_per_sec = max(
            0.0,
            float(self.get_parameter('graceful_stop_decel_duty_per_sec').value),
        )
        self.graceful_stop_turn_duty_per_sec = max(
            self.graceful_stop_decel_duty_per_sec,
            float(self.get_parameter('graceful_stop_turn_duty_per_sec').value),
        )
        self.graceful_stop_zero_threshold_duty = max(
            0.0,
            float(self.get_parameter('graceful_stop_zero_threshold_duty').value),
        )
        if (
            self.segment_velocity_profile_enabled
            and self.segment_profile_accel_duty_per_sec > 0.0
            and 0.0 < self.duty_slew_rate < self.segment_profile_accel_duty_per_sec
        ):
            self.get_logger().warn(
                '[TRACKING] Raising duty slew %.1f -> %.1f so segment velocity '
                'profile acceleration is not clipped.'
                % (self.duty_slew_rate, self.segment_profile_accel_duty_per_sec)
            )
            self.duty_slew_rate = self.segment_profile_accel_duty_per_sec
        if self.segment_velocity_profile_enabled:
            self.get_logger().info(
                '[TRACKING] Segment velocity profile enabled: approach=%.1f, '
                'cruise=%.1f, accel=%.1f duty/s, decel_distance=%.2fm.'
                % (
                    self.segment_profile_approach_duty,
                    self.segment_profile_cruise_duty,
                    self.segment_profile_accel_duty_per_sec,
                    self.segment_deceleration_distance,
                )
            )
        self.command_filter_alpha = self._clamp(
            float(self.get_parameter('command_filter_alpha').value),
            0.0,
            0.98,
        )
        self.backend_delta_v_limit = max(
            0.0, float(self.get_parameter('backend_delta_v_limit').value)
        )
        self.backend_delta_omega_limit = max(
            0.0, float(self.get_parameter('backend_delta_omega_limit').value)
        )
        self.lqg_q_lqr_diag = self._float_list_parameter(
            'lqg_q_lqr_diag',
            [5.0, 5.0, 1.0],
            3,
        )
        self.lqg_r_lqr_diag = self._float_list_parameter(
            'lqg_r_lqr_diag',
            [1.0, 0.5],
            2,
        )
        self.lqg_qn_diag = self._float_list_parameter(
            'lqg_qn_diag',
            [1.0e-2, 1.0e-2, 1.0e-2],
            3,
        )
        self.lqg_rn_diag = self._float_list_parameter(
            'lqg_rn_diag',
            [5.0e-2, 5.0e-2, 2.0e-2],
            3,
        )
        self.lqg_smooth_turn_enabled = as_bool(
            self.get_parameter('lqg_smooth_turn_enabled').value
        )
        self.lqg_smooth_turn_angle = math.radians(
            max(0.0, float(self.get_parameter('lqg_smooth_turn_angle_deg').value))
        )
        self.lqg_stop_turn_angle = math.radians(
            max(
                math.degrees(self.lqg_smooth_turn_angle) + 1.0,
                float(self.get_parameter('lqg_stop_turn_angle_deg').value),
            )
        )
        self.lqg_min_smooth_turn_speed_scale = self._clamp(
            float(self.get_parameter('lqg_min_smooth_turn_speed_scale').value),
            0.10,
            1.0,
        )
        self.mpc_q_diag = self._float_list_parameter(
            'mpc_q_diag',
            [5.0, 5.0, 1.0],
            3,
        )
        self.mpc_r_diag = self._float_list_parameter(
            'mpc_r_diag',
            [1.0, 0.5],
            2,
        )
        self.mpc_prediction_horizon = max(
            1, int(self.get_parameter('mpc_prediction_horizon').value)
        )
        self.pid_kp_x = float(self.get_parameter('pid_kp_x').value)
        self.pid_ki_x = float(self.get_parameter('pid_ki_x').value)
        self.pid_kd_x = float(self.get_parameter('pid_kd_x').value)
        self.pid_ky = float(self.get_parameter('pid_ky').value)
        self.pid_kphi = float(self.get_parameter('pid_kphi').value)
        self.pid_kiy = float(self.get_parameter('pid_kiy').value)

        self.odom_timeout_sec = max(0.0, float(self.get_parameter('odom_timeout_sec').value))
        self.path_timeout_sec = max(0.0, float(self.get_parameter('path_timeout_sec').value))
        self.verbose = as_bool(self.get_parameter('verbose').value)

        self.visual_servo_enabled = as_bool(
            self.get_parameter('visual_servo_enabled').value
        )
        self.visual_servo_guides = self._parse_visual_servo_guides(
            str(self.get_parameter('visual_servo_guides').value)
        )
        self.visual_servo_lane_tolerance_m = max(
            0.0,
            float(self.get_parameter('visual_servo_lane_tolerance_m').value),
        )
        self.visual_servo_heading_tolerance = math.radians(
            max(0.0, float(self.get_parameter('visual_servo_heading_tolerance_deg').value))
        )
        self.visual_servo_marker_timeout_sec = max(
            0.0,
            float(self.get_parameter('visual_servo_marker_timeout_sec').value),
        )
        self.visual_servo_min_range_m = max(
            0.0,
            float(self.get_parameter('visual_servo_min_range_m').value),
        )
        self.visual_servo_max_range_m = max(
            self.visual_servo_min_range_m,
            float(self.get_parameter('visual_servo_max_range_m').value),
        )
        self.visual_servo_bearing_gain = self._clamp(
            float(self.get_parameter('visual_servo_bearing_gain').value),
            0.0,
            1.0,
        )
        self.visual_servo_max_bearing_correction = math.radians(
            max(
                0.0,
                float(self.get_parameter('visual_servo_max_bearing_correction_deg').value),
            )
        )
        self.visual_servo_speed_scale = self._clamp(
            float(self.get_parameter('visual_servo_speed_scale').value),
            0.10,
            1.0,
        )
        self.visual_servo_acquire_speed_scale = self._clamp(
            float(self.get_parameter('visual_servo_acquire_speed_scale').value),
            0.10,
            1.0,
        )
        self.visual_arrival_extra_distance_m = max(
            0.0,
            float(self.get_parameter('visual_arrival_extra_distance_m').value),
        )
        self.visual_arrival_hold_distance_m = max(
            0.05,
            float(self.get_parameter('visual_arrival_hold_distance_m').value),
        )

        self.inspection_stops = self._parse_named_xy_map(
            str(self.get_parameter('inspection_stop_map').value),
            'inspection_stop_map',
        )
        self.inspection_dwell_sec = max(
            0.0,
            float(self.get_parameter('inspection_dwell_sec').value),
        )
        self.inspection_dwell_by_id = self._parse_named_float_map(
            str(self.get_parameter('inspection_dwell_map').value),
            'inspection_dwell_map',
        )
        self.inspection_stop_heading_tolerance = math.radians(
            max(
                0.0,
                float(self.get_parameter('inspection_stop_heading_tolerance_deg').value),
            )
        )
        self.inspection_stop_capture_tolerance_m = max(
            0.0,
            float(self.get_parameter('inspection_stop_capture_tolerance_m').value),
        )
        self.inspection_stop_reached_tolerance_m = max(
            self.inspection_stop_capture_tolerance_m,
            float(self.get_parameter('inspection_stop_reached_tolerance_m').value),
        )
        self.inspection_standoff_distance_m = max(
            0.0,
            float(self.get_parameter('inspection_standoff_distance_m').value),
        )
        self.inspection_standoff_tolerance_m = max(
            0.0,
            float(self.get_parameter('inspection_standoff_tolerance_m').value),
        )

        self.plants = self._parse_plant_world_map(
            str(self.get_parameter('plant_world_map').value)
        )
        self.plant_by_id = {plant_id: (x, y) for plant_id, x, y in self.plants}
        self.plant_detection_enable_radius = max(
            0.0, float(self.get_parameter('plant_detection_enable_radius').value)
        )
        self.plant_detection_disable_radius = max(
            self.plant_detection_enable_radius,
            float(self.get_parameter('plant_detection_disable_radius').value),
        )
        self.plant_detector_hold_sec = max(
            0.0, float(self.get_parameter('plant_detector_hold_sec').value)
        )
        self.plant_detector_always_enabled = as_bool(
            self.get_parameter('plant_detector_always_enabled').value
        )
        self.plant_completion_confidence_threshold = self._clamp(
            float(self.get_parameter('plant_completion_confidence_threshold').value),
            0.0,
            1.0,
        )
        self.plant_completion_max_stop_distance_m = max(
            0.0,
            float(self.get_parameter('plant_completion_max_stop_distance_m').value),
        )
        self.plant_completion_match_mode = str(
            self.get_parameter('plant_completion_match_mode').value
        ).strip().lower()
        if self.plant_completion_match_mode not in ('any', 'stop_id'):
            self.get_logger().warn(
                '[TRACKING] Invalid plant_completion_match_mode=%s; using any.'
                % self.plant_completion_match_mode
            )
            self.plant_completion_match_mode = 'any'
        self.plant_completion_require_valid_capture = as_bool(
            self.get_parameter('plant_completion_require_valid_capture').value
        )
        self.plant_completion_require_dwell = as_bool(
            self.get_parameter('plant_completion_require_dwell').value
        )

        self.plant_bbox_servo_enabled = as_bool(
            self.get_parameter('plant_bbox_servo_enabled').value
        )
        self.plant_capture_radius_m = max(
            0.0, float(self.get_parameter('plant_capture_radius_m').value)
        )
        self.plant_detection_stale_timeout_sec = max(
            0.0, float(self.get_parameter('plant_detection_stale_timeout_sec').value)
        )
        self.plant_bbox_image_width = max(
            1.0, float(self.get_parameter('plant_bbox_image_width').value)
        )
        self.plant_bbox_image_height = max(
            1.0, float(self.get_parameter('plant_bbox_image_height').value)
        )
        self.plant_bbox_fx_px = max(
            1.0, float(self.get_parameter('plant_bbox_fx_px').value)
        )
        self.plant_bbox_fy_px = max(
            1.0, float(self.get_parameter('plant_bbox_fy_px').value)
        )
        self.plant_bbox_center_tolerance_angle = math.radians(
            max(
                0.0,
                float(self.get_parameter('plant_bbox_center_tolerance_angle_deg').value),
            )
        )
        self.plant_bbox_fine_tolerance_angle = math.radians(
            max(
                0.0,
                float(self.get_parameter('plant_bbox_fine_tolerance_angle_deg').value),
            )
        )
        self.plant_bbox_center_tolerance_x_px = max(
            0.0,
            float(self.get_parameter('plant_bbox_center_tolerance_x_px').value),
        )
        self.plant_bbox_center_tolerance_y_px = max(
            0.0,
            float(self.get_parameter('plant_bbox_center_tolerance_y_px').value),
        )
        self.plant_bbox_center_hold_sec = max(
            0.0, float(self.get_parameter('plant_bbox_center_hold_sec').value)
        )
        self.plant_capture_dwell_sec = max(
            0.0, float(self.get_parameter('plant_capture_dwell_sec').value)
        )
        self.plant_bbox_pan_gain_us_per_px = max(
            0.0, float(self.get_parameter('plant_bbox_pan_gain_us_per_px').value)
        )
        self.plant_bbox_tilt_gain_us_per_px = max(
            0.0, float(self.get_parameter('plant_bbox_tilt_gain_us_per_px').value)
        )
        self.plant_bbox_fine_pan_gain_us_per_px = max(
            0.0, float(self.get_parameter('plant_bbox_fine_pan_gain_us_per_px').value)
        )
        self.plant_bbox_fine_tilt_gain_us_per_px = max(
            0.0, float(self.get_parameter('plant_bbox_fine_tilt_gain_us_per_px').value)
        )
        self.plant_bbox_tilt_servo_enabled = as_bool(
            self.get_parameter('plant_bbox_tilt_servo_enabled').value
        )
        self.plant_search_pan_span_us = max(
            0, int(self.get_parameter('plant_search_pan_span_us').value)
        )
        self.plant_search_pan_step_us = max(
            1, int(self.get_parameter('plant_search_pan_step_us').value)
        )
        self.plant_search_update_period_sec = max(
            0.05, float(self.get_parameter('plant_search_update_period_sec').value)
        )
        self.plant_bbox_update_period_sec = max(
            0.05, float(self.get_parameter('plant_bbox_update_period_sec').value)
        )
        self.plant_opportunistic_capture_enabled = as_bool(
            self.get_parameter('plant_opportunistic_capture_enabled').value
        )
        self.plant_opportunistic_min_confidence = self._clamp(
            float(self.get_parameter('plant_opportunistic_min_confidence').value),
            0.0,
            1.0,
        )
        self.plant_opportunistic_assumed_distance_m = max(
            0.20,
            float(self.get_parameter('plant_opportunistic_assumed_distance_m').value),
        )
        self.plant_opportunistic_known_plant_exclusion_m = max(
            0.0,
            float(
                self.get_parameter(
                    'plant_opportunistic_known_plant_exclusion_m'
                ).value
            ),
        )
        self.plant_opportunistic_stop_exclusion_m = max(
            0.0,
            float(self.get_parameter('plant_opportunistic_stop_exclusion_m').value),
        )
        self.plant_opportunistic_duplicate_radius_m = max(
            0.0,
            float(self.get_parameter('plant_opportunistic_duplicate_radius_m').value),
        )
        self.plant_opportunistic_cooldown_sec = max(
            0.0,
            float(self.get_parameter('plant_opportunistic_cooldown_sec').value),
        )
        self.plant_opportunistic_lost_timeout_sec = max(
            0.0,
            float(self.get_parameter('plant_opportunistic_lost_timeout_sec').value),
        )
        self.plant_opportunistic_max_capture_sec = max(
            0.0,
            float(self.get_parameter('plant_opportunistic_max_capture_sec').value),
        )
        self.plant_opportunistic_initial_save_sec = max(
            0.0,
            float(self.get_parameter('plant_opportunistic_initial_save_sec').value),
        )
        self.plant_unknown_lidar_max_range_m = max(
            0.10,
            float(self.get_parameter('plant_unknown_lidar_max_range_m').value),
        )
        self.plant_unknown_lidar_neighbor_beams = max(
            0,
            int(self.get_parameter('plant_unknown_lidar_neighbor_beams').value),
        )
        self.plant_unknown_lidar_stale_timeout_sec = max(
            0.0,
            float(self.get_parameter('plant_unknown_lidar_stale_timeout_sec').value),
        )
        self.lidar_scan_angle_multiplier = float(
            self.get_parameter('lidar_scan_angle_multiplier').value
        )
        self.lidar_in_base_yaw = math.radians(
            float(self.get_parameter('lidar_in_base_yaw_deg').value)
        )

        self.camera_auto_pan_enabled = as_bool(
            self.get_parameter('camera_auto_pan_enabled').value
        )
        self.camera_pan_topic = str(self.get_parameter('camera_pan_topic').value)
        self.camera_pan_channel = max(
            0, int(self.get_parameter('camera_pan_channel').value)
        )
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
        self.camera_pan_active_radius_m = max(
            0.0, float(self.get_parameter('camera_pan_active_radius_m').value)
        )
        self.camera_pan_enable_detector_gate = as_bool(
            self.get_parameter('camera_pan_enable_detector_gate').value
        )
        self.camera_pan_update_period_sec = max(
            0.05, float(self.get_parameter('camera_pan_update_period_sec').value)
        )
        self.camera_pan_return_to_center = as_bool(
            self.get_parameter('camera_pan_return_to_center').value
        )
        self.camera_pan_idle_center_period_sec = max(
            0.0,
            float(self.get_parameter('camera_pan_idle_center_period_sec').value),
        )
        self.camera_pan_left_positive_is_decreasing = as_bool(
            self.get_parameter('camera_pan_left_positive_is_decreasing').value
        )
        self.camera_tilt_channel = max(
            0, int(self.get_parameter('camera_tilt_channel').value)
        )
        self.camera_tilt_center_us = int(self.get_parameter('camera_tilt_center_us').value)
        self.camera_tilt_min_us = int(self.get_parameter('camera_tilt_min_us').value)
        self.camera_tilt_max_us = int(self.get_parameter('camera_tilt_max_us').value)
        if self.camera_tilt_min_us > self.camera_tilt_max_us:
            self.camera_tilt_min_us, self.camera_tilt_max_us = (
                self.camera_tilt_max_us,
                self.camera_tilt_min_us,
            )
        self.camera_tilt_center_us = int(
            self._clamp(
                float(self.camera_tilt_center_us),
                float(self.camera_tilt_min_us),
                float(self.camera_tilt_max_us),
            )
        )
        self.camera_tilt_image_down_positive_increases_us = as_bool(
            self.get_parameter('camera_tilt_image_down_positive_increases_us').value
        )

        self.path: list[tuple[float, float, float]] = []
        self.path_index = 0
        self.goal_reached = False
        self.latest_pose: tuple[float, float, float] | None = None
        self.latest_scan: LaserScan | None = None
        self.latest_scan_time: float | None = None
        self.latest_twist: tuple[float, float] = (0.0, 0.0)
        self.latest_covariance = [0.0] * 9
        self.latest_target_pose: tuple[float, float, float] | None = None
        self.last_odom_time: float | None = None
        self.last_path_time: float | None = None
        self.last_target_pose_time: float | None = None
        self.last_left_cmd = 0.0
        self.last_right_cmd = 0.0
        self.command_seq_num = 1
        self.detector_enabled = False
        self.detector_hold_until = 0.0
        self.last_status_log = 0.0
        self.rotate_in_place_direction = 0.0
        self.corner_turn_index: int | None = None
        self.corner_turn_target_yaw: float | None = None
        self.segment_recovery_rotate_active = False
        self.point_route: list[tuple[float, float, float]] = []
        self.point_segment_index = 0
        self.point_segment_mode = 'rotate'
        self.point_segment_initialized = False
        self.point_segment_start_x = 0.0
        self.point_segment_start_y = 0.0
        self.point_segment_heading = 0.0
        self.point_segment_length = 0.0
        self.point_segment_route_start_x = 0.0
        self.point_segment_route_start_y = 0.0
        self.point_segment_route_target_x = 0.0
        self.point_segment_route_target_y = 0.0
        self.point_segment_route_target_yaw = 0.0
        self.segment_profile_forward_duty = 0.0
        self.latest_visual_markers: dict[int, dict[str, float]] = {}
        self.last_visual_servo_log = 0.0
        self.last_visual_arrival_log = 0.0
        self.point_dwell_until = 0.0
        self.point_dwell_stop_id = ''
        self.completed_inspection_stops: set[str] = set()
        self.pending_path_update: tuple[list[tuple[float, float, float]], str] | None = None
        self.last_camera_pan_publish_time = 0.0
        self.last_camera_pan_log_time = 0.0
        self.last_camera_pan_us: int | None = None
        self.last_camera_pan_target_id = ''
        self.last_camera_tilt_publish_time = 0.0
        self.last_camera_tilt_log_time = 0.0
        self.last_camera_tilt_us: int | None = self.camera_tilt_center_us
        self.latest_plant_detections: PlantDetections | None = None
        self.last_plant_detections_time: float | None = None
        self.active_plant_focus_id = ''
        self.plant_bbox_centered_since = 0.0
        self.plant_focus_started_time = 0.0
        self.plant_capture_stop_id = ''
        self.plant_capture_until = 0.0
        self.plant_positive_save_enabled = False
        self.plant_search_pan_us = self.camera_pan_center_us
        self.plant_search_direction = 1
        self.last_plant_search_time = 0.0
        self.last_plant_bbox_servo_time = 0.0
        self.last_plant_focus_log_time = 0.0
        self.next_opportunistic_capture_index = 1
        self.opportunistic_capture_estimates: dict[str, tuple[float, float]] = {}
        self.known_opportunistic_focus_ids: set[str] = set()
        self.completed_opportunistic_captures: dict[str, tuple[float, float]] = {}
        self.published_opportunistic_capture_events: set[str] = set()
        self.last_opportunistic_capture_time = -float('inf')
        self.opportunistic_initial_save_focus_id = ''
        self.opportunistic_initial_save_until = 0.0
        self.path_controller = self._create_path_controller_backend()
        self.path_controller_label = self.path_controller_backend.upper()

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            reliable_qos,
        )
        self.aruco_sub = self.create_subscription(
            FiducialMarkerArray,
            self.aruco_topic,
            self.aruco_callback,
            reliable_qos,
        )
        self.plant_detections_sub = self.create_subscription(
            PlantDetections,
            self.plant_detections_topic,
            self.plant_detections_callback,
            reliable_qos,
        )
        self.scan_sub = None
        if self.scan_topic:
            self.scan_sub = self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.scan_callback,
                qos_profile_sensor_data,
            )
        self.path_sub = self.create_subscription(
            RosPath,
            self.reference_path_topic,
            self.path_callback,
            reliable_qos,
        )
        self.reference_path_msg_sub = None
        if self.reference_path_msg_topic:
            self.reference_path_msg_sub = self.create_subscription(
                ReferencePath,
                self.reference_path_msg_topic,
                self.reference_path_msg_callback,
                reliable_qos,
            )
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.target_pose_callback,
            reliable_qos,
        )

        self.motor_pub = self.create_publisher(
            LeftRightFloat32,
            self.motor_cmd_topic,
            reliable_qos,
        )
        self.robot_state_pub = self.create_publisher(
            RobotState,
            self.robot_state_topic,
            reliable_qos,
        )
        self.mission_progress_pub = self.create_publisher(
            MissionProgress,
            self.mission_progress_topic,
            reliable_qos,
        )
        self.plant_detector_enable_pub = self.create_publisher(
            Bool,
            self.plant_detector_enable_topic,
            reliable_qos,
        )
        self.plant_positive_save_gate_pub = None
        if self.plant_positive_save_gate_topic:
            self.plant_positive_save_gate_pub = self.create_publisher(
                Bool,
                self.plant_positive_save_gate_topic,
                reliable_qos,
            )
        self.unknown_plant_capture_pub = None
        if self.unknown_plant_capture_topic:
            self.unknown_plant_capture_pub = self.create_publisher(
                String,
                self.unknown_plant_capture_topic,
                reliable_qos,
            )
        self.camera_pan_pub = None
        if self.camera_auto_pan_enabled or self.plant_bbox_servo_enabled:
            self.camera_pan_pub = self.create_publisher(
                ServoPulseWidth,
                self.camera_pan_topic,
                reliable_qos,
            )

        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(
            '[TRACKING] odom=%s reference_path=%s motor_cmd=%s detector_gate=%s detections=%s'
            % (
                self.odom_topic,
                self.reference_path_topic,
                self.motor_cmd_topic,
                self.plant_detector_enable_topic,
                self.plant_detections_topic,
            )
        )
        if self.plant_positive_save_gate_pub is not None:
            self.get_logger().info(
                '[TRACKING] plant positive save gate=%s capture_dwell=%.1fs'
                % (
                    self.plant_positive_save_gate_topic,
                    self.plant_capture_dwell_sec,
                )
            )
        if self.visual_servo_enabled:
            guide_text = ', '.join(
                (
                    'ID%d@x=%.2f,%.0fdeg,arrive<=%.2fm'
                    % (
                        guide['marker_id'],
                        guide['lane_x'],
                        math.degrees(guide['heading']),
                        guide['arrival_range'],
                    )
                    if guide.get('arrival_range', 0.0) > 0.0
                    else 'ID%d@x=%.2f,%.0fdeg'
                    % (
                        guide['marker_id'],
                        guide['lane_x'],
                        math.degrees(guide['heading']),
                    )
                )
                for guide in self.visual_servo_guides
            )
            self.get_logger().info(
                '[TRACKING] visual marker servo on %s guides=[%s]'
                % (self.aruco_topic, guide_text)
            )
        if self.inspection_dwell_sec > 0.0 and self.inspection_stops:
            dwell_overrides = ', '.join(
                '%s=%.1fs' % (stop_id, dwell_sec)
                for stop_id, dwell_sec in sorted(self.inspection_dwell_by_id.items())
            )
            override_text = ' overrides=[%s]' % dwell_overrides if dwell_overrides else ''
            self.get_logger().info(
                '[TRACKING] inspection stops=%d dwell=%.1fs%s'
                % (len(self.inspection_stops), self.inspection_dwell_sec, override_text)
            )
        if self.camera_auto_pan_enabled:
            self.get_logger().info(
                '[TRACKING] camera auto-pan topic=%s channel=%d center=%dus range=[%d,%d] max=%.0fdeg'
                % (
                    self.camera_pan_topic,
                    self.camera_pan_channel,
                    self.camera_pan_center_us,
                    self.camera_pan_min_us,
                    self.camera_pan_max_us,
                    math.degrees(self.camera_pan_max_angle),
                )
            )
        if self.plant_bbox_servo_enabled:
            self.get_logger().info(
                '[TRACKING] plant bbox servo radius=%.2fm image=%.0fx%.0f tol=%.1fdeg fine=%.1fdeg pan_ch=%d tilt_ch=%d tilt_center=%dus tilt_servo=%s'
                % (
                    self.plant_capture_radius_m,
                    self.plant_bbox_image_width,
                    self.plant_bbox_image_height,
                    math.degrees(self.plant_bbox_center_tolerance_angle),
                    math.degrees(self.plant_bbox_fine_tolerance_angle),
                    self.camera_pan_channel,
                    self.camera_tilt_channel,
                    self.camera_tilt_center_us,
                    'on' if self.plant_bbox_tilt_servo_enabled else 'off',
                )
            )
        if self.plant_detector_always_enabled:
            self.get_logger().info('[TRACKING] plant detector gate forced on full-time.')
        if self.plant_opportunistic_capture_enabled:
            self.get_logger().info(
                '[TRACKING] opportunistic YOLO capture on: conf>=%.2f duplicate_radius=%.2fm cooldown=%.1fs initial_save=%.1fs lidar=%s event=%s'
                % (
                    self.plant_opportunistic_min_confidence,
                    self.plant_opportunistic_duplicate_radius_m,
                    self.plant_opportunistic_cooldown_sec,
                    self.plant_opportunistic_initial_save_sec,
                    self.scan_topic if self.scan_topic else '<disabled>',
                    self.unknown_plant_capture_topic or '<disabled>',
                )
            )
        self.get_logger().info(
            '[TRACKING] path controller backend=%s.'
            % self.path_controller_backend
        )
        if self.path_controller_backend == 'lqg' and self.lqg_smooth_turn_enabled:
            self.get_logger().info(
                '[TRACKING] LQG smooth turns enabled: smooth<=%.1fdeg, stop-turn>%.1fdeg, min_speed_scale=%.2f.'
                % (
                    math.degrees(self.lqg_smooth_turn_angle),
                    math.degrees(self.lqg_stop_turn_angle),
                    self.lqg_min_smooth_turn_speed_scale,
                )
            )
        self.get_logger().info(
            '[TRACKING] v_nom=%.2f m/s lookahead=%.2f m duty_limit=%.1f%% plants=%d'
            % (
                self.nominal_speed,
                self.lookahead_distance,
                self.duty_cycle_limit,
                len(self.plants),
            )
        )
        if self.graceful_stop_enabled:
            self.get_logger().info(
                '[TRACKING] graceful stop decel=%.1f duty/s turn=%.1f duty/s'
                % (
                    self.graceful_stop_decel_duty_per_sec,
                    self.graceful_stop_turn_duty_per_sec,
                )
            )

    def odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        linear_velocity = float(msg.twist.twist.linear.x)
        angular_velocity = float(msg.twist.twist.angular.z)

        self.latest_pose = (x, y, yaw)
        self.latest_twist = (linear_velocity, angular_velocity)
        self.latest_covariance = self._extract_planar_covariance(msg.pose.covariance)
        self.last_odom_time = time.monotonic()

        state_msg = RobotState()
        state_msg.header = msg.header
        state_msg.pose.x = x
        state_msg.pose.y = y
        state_msg.pose.theta = yaw
        state_msg.linear_velocity = linear_velocity
        state_msg.angular_velocity = angular_velocity
        state_msg.covariance = self.latest_covariance
        self.robot_state_pub.publish(state_msg)

    def aruco_callback(self, msg: FiducialMarkerArray) -> None:
        now = time.monotonic()
        for marker in msg.markers:
            marker_id = int(marker.id)
            tx = float(marker.tvec[0])
            tz = float(marker.tvec[2])
            if tz <= 1.0e-6:
                continue

            bearing_left = math.atan2(-tx, tz)
            marker_range = math.hypot(tx, tz)
            self.latest_visual_markers[marker_id] = {
                'time': now,
                'bearing_left': bearing_left,
                'range': marker_range,
                'tx': tx,
                'tz': tz,
            }

    def plant_detections_callback(self, msg: PlantDetections) -> None:
        self.latest_plant_detections = msg
        self.last_plant_detections_time = time.monotonic()

        if self.plant_bbox_servo_enabled:
            return

        stop_id = self._active_inspection_stop_id()
        if not stop_id:
            return

        if stop_id in self.completed_inspection_stops:
            return

        if self.plant_completion_require_dwell and not self._dwell_matches_stop(stop_id):
            return

        if not self._plant_detection_is_completion(msg, stop_id):
            return

        if self._is_search_focus_id(stop_id):
            detection = self._best_plant_detection(msg, stop_id)
            if detection is not None:
                self._publish_unknown_plant_capture_event(
                    stop_id,
                    detection,
                    msg,
                    reason='hidden plant valid capture',
                )
            self._complete_inspection_stop(stop_id, reason='hidden plant valid capture')
            return

        self._complete_inspection_stop(stop_id, reason='YOLO valid capture')

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = time.monotonic()

    def path_callback(self, msg: RosPath) -> None:
        path = self._poses_to_path(msg.poses)
        if not path:
            self.get_logger().warn('[TRACKING] Ignoring empty nav_msgs/Path.')
            return
        self._set_new_path(path, 'nav_msgs/Path')

    def reference_path_msg_callback(self, msg: ReferencePath) -> None:
        path = self._poses_to_path(msg.poses)
        if not path:
            self.get_logger().warn('[TRACKING] Ignoring empty ReferencePath.')
            return
        if msg.nominal_speed > 0.0:
            self.nominal_speed = min(float(msg.nominal_speed), self.max_linear_speed)
        self._set_new_path(path, 'ReferencePath')

    def target_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_target_pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            quaternion_to_yaw(msg.pose.orientation),
        )
        self.last_target_pose_time = time.monotonic()

    def control_loop(self) -> None:
        now = time.monotonic()
        self._update_plant_detector_gate(now)
        self._publish_mission_progress()

        if self.latest_pose is None:
            self._publish_stop()
            self._log_status(
                now,
                '[TRACKING] Waiting for first odometry message on %s.' % self.odom_topic,
            )
            return

        if self._odom_is_stale(now):
            if self._handle_inspection_stop_with_stale_odom(now):
                return
            self._publish_stop()
            self._log_status(now, '[TRACKING] Waiting for fresh odometry.')
            return

        if self._handle_plant_bbox_focus(now):
            return

        self._update_camera_pan(now)

        if not self.path:
            if not self._target_pose_is_fresh(now):
                self._publish_stop()
                self._log_status(now, '[TRACKING] Waiting for reference path.')
                return
            self._track_single_target(self.latest_target_pose)
            return

        if self._path_is_stale(now):
            if self._target_pose_is_fresh(now):
                self._track_single_target(self.latest_target_pose)
                return
            self._publish_stop()
            self._log_status(now, '[TRACKING] Reference path is stale; stopping.')
            return

        if self.goal_reached:
            self._publish_stop()
            return

        rx, ry, ryaw = self.latest_pose

        if self.point_to_point_mode:
            self._publish_point_to_point_command(rx, ry, ryaw)
            return

        self._advance_path_index(
            rx,
            ry,
            stop_at_corners=self.segment_stop_turn_mode,
        )
        remaining = self._remaining_path_distance(rx, ry)
        gx, gy, _ = self.path[-1]

        # The final demo route starts and finishes at the same global pose.
        # Do not stop just because the robot is geometrically near the final
        # point; it must also have consumed almost all of the reference path.
        if (
            math.hypot(gx - rx, gy - ry) <= self.goal_tolerance
            and remaining <= max(self.goal_tolerance, self.waypoint_reached_distance)
        ):
            self.goal_reached = True
            self._publish_stop()
            self.get_logger().info('[TRACKING] Goal reached; motors stopped.')
            return

        if self.segment_stop_turn_mode:
            self._publish_segment_stop_turn_command(rx, ry, ryaw, remaining)
            return

        target, control_remaining = self._select_control_target(rx, ry, remaining)
        self._publish_velocity_command_for_target(target, control_remaining)

    def _track_single_target(self, target: tuple[float, float, float] | None) -> None:
        if target is None:
            self._publish_stop()
            return
        rx, ry, _ = self.latest_pose
        distance = math.hypot(target[0] - rx, target[1] - ry)
        if distance <= self.goal_tolerance:
            self._publish_stop()
            return
        self._publish_velocity_command_for_target(target, distance)

    def _handle_inspection_stop_with_stale_odom(self, now: float) -> bool:
        if self._inspection_stop_hold_active():
            if self._is_search_focus_id(self.point_dwell_stop_id):
                return self._handle_plant_bbox_focus(now)
            return False

        if not self.path:
            return False

        stop_id = self._path_final_stop_id(self.path)
        if not stop_id or stop_id in self.completed_inspection_stops:
            return False
        if not self._is_search_focus_id(stop_id):
            return False
        if self.latest_pose is None:
            return False

        rx, ry, _ = self.latest_pose
        gx, gy, _ = self.path[-1]
        path_goal_distance = math.hypot(gx - rx, gy - ry)
        stop_distance = self._inspection_stop_distance(stop_id)
        reached_distance = min(
            path_goal_distance,
            stop_distance if stop_distance is not None else float('inf'),
        )
        reached_threshold = max(
            self.inspection_stop_reached_tolerance_m,
            self.goal_tolerance,
            self.point_to_point_arrival_tolerance + 0.05,
        )
        if reached_distance > reached_threshold:
            return False

        dwell_sec = self._inspection_dwell_duration(stop_id)
        if dwell_sec <= 0.0:
            self._complete_inspection_stop(stop_id, reason='stale odometry at stop')
            return True

        self.point_segment_mode = 'dwell'
        self.point_dwell_stop_id = stop_id
        self.point_dwell_until = now + dwell_sec
        self.point_segment_initialized = True
        self.get_logger().warn(
            '[TRACKING] Odom stale at %s; holding stop and searching with camera for up to %.1fs.'
            % (stop_id, dwell_sec)
        )
        self._publish_stop()
        return self._handle_plant_bbox_focus(now) or True

    def _create_path_controller_backend(self):
        if self.path_controller_backend == 'direct':
            return None

        if CONTROLLER_BACKEND_IMPORT_ERROR is not None:
            self.get_logger().warn(
                '[TRACKING] Could not import controller backends (%s); using direct.'
                % CONTROLLER_BACKEND_IMPORT_ERROR
            )
            self.path_controller_backend = 'direct'
            return None

        delta_limits = (
            [-self.backend_delta_v_limit, -self.backend_delta_omega_limit],
            [self.backend_delta_v_limit, self.backend_delta_omega_limit],
        )

        if self.path_controller_backend == 'lqg':
            return LQGController(
                dt=self.control_period,
                Q_lqr=np.diag(self.lqg_q_lqr_diag),
                R_lqr=np.diag(self.lqg_r_lqr_diag),
                Qn=np.diag(self.lqg_qn_diag),
                Rn=np.diag(self.lqg_rn_diag),
                u_min=delta_limits[0],
                u_max=delta_limits[1],
            )

        if self.path_controller_backend == 'mpc':
            return MPCController(
                dt=self.control_period,
                Q=np.diag(self.mpc_q_diag),
                R=np.diag(self.mpc_r_diag),
                Np=self.mpc_prediction_horizon,
                u_min=delta_limits[0],
                u_max=delta_limits[1],
            )

        if self.path_controller_backend == 'pid':
            return PIDController(
                kp_x=self.pid_kp_x,
                ki_x=self.pid_ki_x,
                kd_x=self.pid_kd_x,
                ky=self.pid_ky,
                kphi=self.pid_kphi,
                kiy=self.pid_kiy,
            )

        return None

    def _path_controller_backend_active(self) -> bool:
        return self.path_controller_backend != 'direct' and self.path_controller is not None

    def _reset_path_controller_backend(self) -> None:
        if self.path_controller is not None and hasattr(self.path_controller, 'reset'):
            self.path_controller.reset()

    def _path_controller_delta(
        self,
        e_hat: list[float] | tuple[float, float, float],
        v_ref: float,
        omega_ref: float,
    ) -> tuple[float, float, str]:
        if not self._path_controller_backend_active():
            return 0.0, 0.0, 'direct'

        if self.path_controller_backend == 'pid':
            delta_u, aux = self.path_controller.step(
                np.array(e_hat, dtype=float),
                self.control_period,
            )
        else:
            delta_u, aux = self.path_controller.step(e_hat, v_ref, omega_ref)

        delta_v = self._clamp(
            float(delta_u[0]),
            -self.backend_delta_v_limit,
            self.backend_delta_v_limit,
        )
        delta_omega = self._clamp(
            float(delta_u[1]),
            -self.backend_delta_omega_limit,
            self.backend_delta_omega_limit,
        )
        label = str(aux.get('label', self.path_controller_backend.upper()))
        return delta_v, delta_omega, label

    def _publish_velocity_command_for_target(
        self,
        target: tuple[float, float, float],
        remaining_distance: float,
    ) -> None:
        rx, ry, ryaw = self.latest_pose
        tx, ty, tyaw = target

        dx = tx - rx
        dy = ty - ry
        distance = math.hypot(dx, dy)
        path_heading = math.atan2(dy, dx) if distance > 1e-6 else tyaw
        alpha = wrap_to_pi(path_heading - ryaw)
        target_heading_error = wrap_to_pi(tyaw - ryaw)

        if self._path_controller_backend_active():
            rotate_threshold = (
                self.lqg_stop_turn_angle
                if self._lqg_smooth_turn_active()
                else self.rotate_in_place_angle
            )
            if abs(alpha) > rotate_threshold:
                rotate_error = self._stable_rotate_error(alpha)
                omega_cmd = self._clamp(
                    self.heading_gain * rotate_error,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )
                if abs(omega_cmd) < self.rotate_in_place_min_angular_speed:
                    omega_cmd = math.copysign(
                        self.rotate_in_place_min_angular_speed,
                        omega_cmd if abs(omega_cmd) > 1e-9 else rotate_error,
                    )
                left_duty, right_duty = self._publish_velocity_command(0.0, omega_cmd)
                if self.verbose:
                    self.get_logger().info(
                        '[TRACKING] %s rotate alpha=%.1fdeg w=%.3f duty=(%.1f, %.1f)'
                        % (
                            self.path_controller_label,
                            math.degrees(alpha),
                            omega_cmd,
                            left_duty,
                            right_duty,
                        )
                    )
                return

            slowdown_scale = min(1.0, max(0.0, remaining_distance / self.slowdown_distance))
            if self._lqg_smooth_turn_active():
                turn_scale = self._lqg_smooth_turn_speed_scale(alpha)
            else:
                turn_scale = max(
                    0.25,
                    1.0 - min(abs(alpha), self.rotate_in_place_angle) / self.rotate_in_place_angle,
                )
            v_ref = min(self.nominal_speed, self.max_linear_speed) * slowdown_scale * turn_scale
            omega_ref = 2.0 * v_ref * math.sin(alpha) / max(
                self.lookahead_distance,
                distance,
                0.05,
            )
            heading_weight = 0.35 if remaining_distance < self.lookahead_distance else 0.10
            omega_ref += heading_weight * self.heading_gain * target_heading_error
            omega_ref = self._clamp(
                omega_ref,
                -self.max_angular_speed,
                self.max_angular_speed,
            )

            x_e = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            y_e = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
            # LQG/MPC/PID models use actual-minus-reference error.  The
            # lookahead geometry above is reference-minus-actual in the robot
            # frame, so flip all three components before handing it to the
            # backend.
            delta_v, delta_omega, label = self._path_controller_delta(
                [-x_e, -y_e, -target_heading_error],
                v_ref,
                omega_ref,
            )
            v_cmd = self._clamp(
                v_ref + delta_v,
                -self.max_linear_speed,
                self.max_linear_speed,
            )
            omega_cmd = self._clamp(
                omega_ref + delta_omega,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            left_duty, right_duty = self._publish_velocity_command(v_cmd, omega_cmd)

            if self.verbose:
                self.get_logger().info(
                    '[TRACKING] %s target=(%.2f, %.2f) e=(%.2f, %.2f, %.1fdeg) '
                    'v=%.3f+%.3f w=%.3f+%.3f duty=(%.1f, %.1f)'
                    % (
                        label,
                        tx,
                        ty,
                        x_e,
                        y_e,
                        math.degrees(target_heading_error),
                        v_ref,
                        delta_v,
                        omega_ref,
                        delta_omega,
                        left_duty,
                        right_duty,
                    )
                )
            return

        if abs(alpha) > self.rotate_in_place_angle:
            v_cmd = 0.0
            rotate_error = self._stable_rotate_error(alpha)
            omega_cmd = self._clamp(
                self.heading_gain * rotate_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            if abs(omega_cmd) < self.rotate_in_place_min_angular_speed:
                omega_cmd = math.copysign(
                    self.rotate_in_place_min_angular_speed,
                    omega_cmd if abs(omega_cmd) > 1e-9 else rotate_error,
                )
        else:
            self.rotate_in_place_direction = 0.0
            slowdown_scale = min(1.0, max(0.0, remaining_distance / self.slowdown_distance))
            turn_scale = max(0.25, 1.0 - min(abs(alpha), self.rotate_in_place_angle) / self.rotate_in_place_angle)
            v_cmd = min(self.nominal_speed, self.max_linear_speed) * slowdown_scale * turn_scale
            curvature_omega = self.curvature_gain * 2.0 * v_cmd * math.sin(alpha) / max(
                self.lookahead_distance,
                distance,
                0.05,
            )
            heading_weight = 0.35 if remaining_distance < self.lookahead_distance else 0.10
            omega_cmd = curvature_omega + heading_weight * self.heading_gain * target_heading_error
            omega_cmd = self._clamp(
                omega_cmd,
                -self.max_angular_speed,
                self.max_angular_speed,
            )

        left_duty, right_duty = self._publish_velocity_command(v_cmd, omega_cmd)

        if self.verbose:
            self.get_logger().info(
                '[TRACKING] target=(%.2f, %.2f) dist=%.2f alpha=%.2fdeg '
                'v=%.3f w=%.3f duty=(%.1f, %.1f)'
                % (
                    tx,
                    ty,
                    distance,
                    math.degrees(alpha),
                    v_cmd,
                    omega_cmd,
                    left_duty,
                    right_duty,
                )
            )

    def _publish_velocity_command(
        self,
        v_cmd: float,
        omega_cmd: float,
    ) -> tuple[float, float]:
        left_speed = v_cmd - self.half_wheel_base * omega_cmd
        right_speed = v_cmd + self.half_wheel_base * omega_cmd

        left_duty = self._wheel_speed_to_duty(left_speed) * self.left_trim
        right_duty = self._wheel_speed_to_duty(right_speed) * self.right_trim
        left_duty = self._clamp(left_duty, -self.duty_cycle_limit, self.duty_cycle_limit)
        right_duty = self._clamp(right_duty, -self.duty_cycle_limit, self.duty_cycle_limit)

        left_duty, right_duty = self._apply_command_filter(left_duty, right_duty)
        left_duty, right_duty = self._apply_slew_limit(left_duty, right_duty)
        self._publish_motor_command(left_duty, right_duty)
        return left_duty, right_duty

    def _poses_to_path(self, poses: Iterable[PoseStamped]) -> list[tuple[float, float, float]]:
        path: list[tuple[float, float, float]] = []
        pose_list = list(poses)
        for index, pose_stamped in enumerate(pose_list):
            pose = pose_stamped.pose
            x = float(pose.position.x)
            y = float(pose.position.y)
            yaw = quaternion_to_yaw(pose.orientation)
            if self._quaternion_is_empty(pose.orientation):
                yaw = self._yaw_from_neighbours(pose_list, index)
            path.append((x, y, yaw))
        return path

    def _set_new_path(self, path: list[tuple[float, float, float]], source: str) -> None:
        now = time.monotonic()
        if self._same_path(path, self.path):
            self.path = path
            self.last_path_time = now
            return

        if self._inspection_stop_hold_active():
            if self._path_final_stop_id(path) == self.point_dwell_stop_id:
                self.last_path_time = now
                return
            self.pending_path_update = (path, source)
            self.last_path_time = now
            self._log_status(
                now,
                '[TRACKING] Queued %s update until inspection stop completes.'
                % source,
            )
            return

        self._apply_new_path(path, source)

    def _apply_new_path(self, path: list[tuple[float, float, float]], source: str) -> None:
        now = time.monotonic()
        point_route = self._build_point_to_point_route(path)
        if self._try_preserve_replan_motion(path, point_route, source, now):
            return

        self.path = path
        self.point_route = point_route
        self.path_index = 0
        self.goal_reached = False
        self.rotate_in_place_direction = 0.0
        self.corner_turn_index = None
        self.corner_turn_target_yaw = None
        self.segment_recovery_rotate_active = False
        self.point_segment_index = 0
        self.point_segment_mode = 'rotate'
        self.point_segment_initialized = False
        self.segment_profile_forward_duty = 0.0
        self.point_dwell_until = 0.0
        self.point_dwell_stop_id = ''
        if self.reset_mission_progress_on_new_path:
            self.completed_inspection_stops.clear()
        self._reset_path_controller_backend()
        self.last_path_time = now
        self.get_logger().info(
            '[TRACKING] New %s received with %d poses.' % (source, len(path))
        )
        if self.point_to_point_mode:
            self.get_logger().info(
                '[TRACKING] Point-to-point route has %d key points.'
                % len(self.point_route)
            )

    def _try_preserve_replan_motion(
        self,
        path: list[tuple[float, float, float]],
        point_route: list[tuple[float, float, float]],
        source: str,
        now: float,
    ) -> bool:
        if (
            not self.preserve_replan_motion_enabled
            or not self.point_to_point_mode
            or self.latest_pose is None
            or len(point_route) < 2
            or not self.point_segment_initialized
            or self.point_segment_mode != 'straight'
        ):
            return False

        segment = self._first_point_route_segment(point_route)
        if segment is None:
            return False
        (
            segment_index,
            start_x,
            start_y,
            target_x,
            target_y,
            target_yaw,
            segment_heading,
            segment_length,
        ) = segment

        robot_x, robot_y, robot_yaw = self.latest_pose
        heading_error = abs(wrap_to_pi(segment_heading - robot_yaw))
        if heading_error > self.preserve_replan_heading_tolerance:
            return False

        start_distance = math.hypot(start_x - robot_x, start_y - robot_y)
        segment_distance = self._distance_to_segment(
            robot_x,
            robot_y,
            start_x,
            start_y,
            target_x,
            target_y,
        )
        if (
            min(start_distance, segment_distance)
            > self.preserve_replan_start_tolerance_m
        ):
            return False

        progress = (
            (robot_x - start_x) * math.cos(segment_heading)
            + (robot_y - start_y) * math.sin(segment_heading)
        )
        progress = self._clamp(progress, 0.0, segment_length)
        remaining = segment_length - progress
        if remaining <= self.point_to_point_arrival_tolerance:
            return False

        self.path = path
        self.point_route = point_route
        self.path_index = 0
        self.goal_reached = False
        self.rotate_in_place_direction = 0.0
        self.corner_turn_index = None
        self.corner_turn_target_yaw = None
        self.segment_recovery_rotate_active = False
        self.point_segment_index = segment_index
        self.point_segment_mode = 'straight'
        self.point_segment_initialized = True
        self.point_segment_start_x = robot_x
        self.point_segment_start_y = robot_y
        self.point_segment_heading = segment_heading
        self.point_segment_length = remaining
        self.point_segment_route_start_x = start_x
        self.point_segment_route_start_y = start_y
        self.point_segment_route_target_x = target_x
        self.point_segment_route_target_y = target_y
        self.point_segment_route_target_yaw = target_yaw
        self.point_dwell_until = 0.0
        self.point_dwell_stop_id = ''
        if self.reset_mission_progress_on_new_path:
            self.completed_inspection_stops.clear()
        self.last_path_time = now

        self.get_logger().info(
            '[TRACKING] New %s received with %d poses; preserving straight '
            'motion on aligned replan (heading error %.1fdeg, remaining %.2fm).'
            % (source, len(path), math.degrees(heading_error), remaining)
        )
        if self.verbose:
            self.get_logger().info(
                '[TRACKING] Point-to-point route has %d key points.'
                % len(self.point_route)
            )
        return True

    def _first_point_route_segment(
        self,
        point_route: list[tuple[float, float, float]],
    ) -> tuple[int, float, float, float, float, float, float, float] | None:
        if len(point_route) < 2:
            return None

        start_x, start_y, _ = point_route[0]
        target_x, target_y, target_yaw = point_route[1]
        length = math.hypot(target_x - start_x, target_y - start_y)
        if length < self.point_to_point_min_segment_length:
            return None
        heading = math.atan2(target_y - start_y, target_x - start_x)
        return (
            0,
            start_x,
            start_y,
            target_x,
            target_y,
            target_yaw,
            heading,
            length,
        )

    @staticmethod
    def _distance_to_segment(
        px: float,
        py: float,
        start_x: float,
        start_y: float,
        target_x: float,
        target_y: float,
    ) -> float:
        vx = target_x - start_x
        vy = target_y - start_y
        length_sq = vx * vx + vy * vy
        if length_sq <= 1.0e-9:
            return math.hypot(px - start_x, py - start_y)
        t = ((px - start_x) * vx + (py - start_y) * vy) / length_sq
        t = max(0.0, min(1.0, t))
        closest_x = start_x + t * vx
        closest_y = start_y + t * vy
        return math.hypot(px - closest_x, py - closest_y)

    def _advance_path_index(
        self,
        x: float,
        y: float,
        stop_at_corners: bool = False,
    ) -> None:
        if not self.path:
            return

        search_end = min(len(self.path), self.path_index + 25)
        closest_index = self.path_index
        closest_distance = float('inf')
        for index in range(self.path_index, search_end):
            px, py, _ = self.path[index]
            distance = math.hypot(px - x, py - y)
            if distance < closest_distance:
                closest_index = index
                closest_distance = distance
        if stop_at_corners:
            next_corner = self._find_next_sharp_corner_index()
            if next_corner is not None:
                closest_index = min(closest_index, next_corner)
        self.path_index = max(self.path_index, closest_index)

        while self.path_index < len(self.path) - 1:
            if stop_at_corners and self._is_sharp_corner_index(self.path_index):
                break
            px, py, _ = self.path[self.path_index]
            if math.hypot(px - x, py - y) <= self.waypoint_reached_distance:
                self.path_index += 1
            else:
                break

    def _select_lookahead_target(self, x: float, y: float) -> tuple[float, float, float]:
        if not self.path:
            raise RuntimeError('Cannot select target from empty path')

        target = self.path[-1]
        for index in range(self.path_index, len(self.path)):
            px, py, pyaw = self.path[index]
            if math.hypot(px - x, py - y) >= self.lookahead_distance:
                target = (px, py, pyaw)
                break
        return target

    def _select_control_target(
        self,
        x: float,
        y: float,
        remaining_distance: float,
    ) -> tuple[tuple[float, float, float], float]:
        corner_index = self._find_next_sharp_corner_index()
        if corner_index is not None:
            cx, cy, _ = self.path[corner_index]
            corner_distance = math.hypot(cx - x, cy - y)
            if (
                self.waypoint_reached_distance < corner_distance
                <= self.corner_hold_distance
            ):
                heading_to_corner = math.atan2(cy - y, cx - x)
                return (
                    (cx, cy, heading_to_corner),
                    min(remaining_distance, corner_distance),
                )

        return self._select_lookahead_target(x, y), remaining_distance

    def _build_point_to_point_route(
        self,
        path: list[tuple[float, float, float]],
    ) -> list[tuple[float, float, float]]:
        if len(path) <= 2:
            return list(path)

        route = [path[0]]
        included_stop_ids: set[str] = set()
        for index in range(1, len(path) - 1):
            px, py, pyaw = path[index]
            stop_id = self._inspection_stop_id(
                px,
                py,
                self.inspection_stop_capture_tolerance_m,
                yaw=pyaw,
                require_heading=True,
            )
            include_stop = bool(stop_id and stop_id not in included_stop_ids)
            if self._is_sharp_corner(path, index) or include_stop:
                last_x, last_y, _ = route[-1]
                if math.hypot(px - last_x, py - last_y) >= self.point_to_point_min_segment_length:
                    route.append((px, py, pyaw))
                    if stop_id:
                        included_stop_ids.add(stop_id)

        last_x, last_y, last_yaw = path[-1]
        prev_x, prev_y, _ = route[-1]
        if math.hypot(last_x - prev_x, last_y - prev_y) >= self.point_to_point_min_segment_length:
            route.append((last_x, last_y, last_yaw))

        return route

    def _publish_point_to_point_command(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> None:
        if len(self.point_route) < 2:
            self._publish_stop()
            return

        if self.point_segment_index >= len(self.point_route) - 1:
            self.goal_reached = True
            self._publish_stop()
            self.get_logger().info('[TRACKING] Point-to-point route complete.')
            return

        if not self.point_segment_initialized:
            if not self._initialize_point_segment():
                self.goal_reached = True
                self._publish_stop()
                return
            if self.verbose:
                self.get_logger().info(
                    '[TRACKING] P2P segment %d/%d heading=%.1fdeg length=%.2f'
                    % (
                        self.point_segment_index + 1,
                        len(self.point_route) - 1,
                        math.degrees(self.point_segment_heading),
                        self.point_segment_length,
                    )
                )

        if self.point_segment_mode == 'dwell':
            now = time.monotonic()
            self._publish_stop()
            if now >= self.point_dwell_until:
                stop_id = self.point_dwell_stop_id
                self._complete_inspection_stop(stop_id, reason='YOLO wait timeout')
            else:
                self._log_status(
                    now,
                    '[TRACKING] Waiting for YOLO at %s for %.1fs.'
                    % (self.point_dwell_stop_id, self.point_dwell_until - now),
                )
            return

        if self.point_segment_mode == 'capture_rotate':
            stop_id = self.point_dwell_stop_id
            desired_yaw = self._inspection_capture_yaw(stop_id)
            if not stop_id or desired_yaw is None:
                self.point_dwell_until = 0.0
                self.point_dwell_stop_id = ''
                self._finish_point_segment()
                return

            yaw_error = wrap_to_pi(desired_yaw - yaw)
            if abs(yaw_error) > self.point_to_point_rotate_tolerance:
                now = time.monotonic()
                self._log_status(
                    now,
                    '[TRACKING] Rotating at %s to plant heading %.1fdeg.'
                    % (stop_id, math.degrees(desired_yaw)),
                )
                self._publish_corner_turn_command(yaw_error)
                return

            dwell_sec = self._inspection_dwell_duration(stop_id)
            if dwell_sec <= 0.0:
                self._complete_inspection_stop(stop_id, reason='YOLO wait timeout')
                return
            self.point_segment_mode = 'dwell'
            self.point_dwell_until = time.monotonic() + dwell_sec
            self.get_logger().info(
                '[TRACKING] Waiting for YOLO at %s for up to %.1fs.'
                % (stop_id, dwell_sec)
            )
            self._publish_stop()
            return

        if self.point_segment_mode == 'rotate':
            yaw_error = wrap_to_pi(self.point_segment_heading - yaw)
            if self._lqg_can_smooth_turn(yaw_error):
                self._start_point_segment_straight_from_current_pose(x, y)
                if self.verbose:
                    self.get_logger().info(
                        '[TRACKING] LQG smooth segment entry yaw_error=%.1fdeg; tracking without stop-turn.'
                        % math.degrees(yaw_error)
                    )
                self._publish_segment_straight_command(
                    self.point_segment_heading,
                    self.point_segment_length,
                )
                return
            if abs(yaw_error) <= self.point_to_point_rotate_tolerance:
                self.point_segment_start_x = x
                self.point_segment_start_y = y
                self.point_segment_mode = 'straight'
                if self.verbose:
                    self.get_logger().info('[TRACKING] P2P rotate complete; straight start.')
                if self.stop_after_point_rotate_complete:
                    self._publish_stop()
                    return
                self._publish_segment_straight_command(
                    self.point_segment_heading,
                    self.point_segment_length,
                )
                return

            self._publish_corner_turn_command(yaw_error)
            return

        travelled = self._point_segment_travelled(x, y)
        remaining = self.point_segment_length - travelled
        if remaining <= self.point_to_point_arrival_tolerance:
            visual_hold = self._visual_arrival_hold(travelled)
            if visual_hold is not None:
                guide, visual_marker = visual_hold
                now = time.monotonic()
                if now - self.last_visual_arrival_log >= 1.0:
                    self.last_visual_arrival_log = now
                    self.get_logger().info(
                        '[TRACKING] visual arrival hold ID%d range=%.2fm '
                        '> %.2fm; overrun=%.2fm'
                        % (
                            guide['marker_id'],
                            float(visual_marker['range']),
                            guide['arrival_range'],
                            max(0.0, travelled - self.point_segment_length),
                        )
                    )
                self._publish_segment_straight_command(
                    self.point_segment_heading,
                    self.visual_arrival_hold_distance_m,
                )
                return

            stop_id = self._inspection_stop_id(
                self.point_segment_route_target_x,
                self.point_segment_route_target_y,
                self.inspection_stop_reached_tolerance_m,
                yaw=self.point_segment_route_target_yaw,
                require_heading=True,
            )
            dwell_sec = self._inspection_dwell_duration(stop_id)
            if self._lqg_try_smooth_advance_to_next_segment(x, y, yaw, stop_id):
                return

            self._publish_stop()
            if self.verbose:
                self.get_logger().info(
                    '[TRACKING] P2P straight complete; travelled=%.2f/%.2f'
                    % (travelled, self.point_segment_length)
                )
            if (
                stop_id
                and stop_id not in self.completed_inspection_stops
            ):
                desired_yaw = self._inspection_capture_yaw(stop_id)
                if desired_yaw is not None:
                    yaw_error = wrap_to_pi(desired_yaw - yaw)
                    if abs(yaw_error) > self.point_to_point_rotate_tolerance:
                        self.point_segment_mode = 'capture_rotate'
                        self.point_dwell_stop_id = stop_id
                        self.point_dwell_until = 0.0
                        self.get_logger().info(
                            '[TRACKING] Rotating at %s to face plant heading %.1fdeg.'
                            % (stop_id, math.degrees(desired_yaw))
                        )
                        self._publish_corner_turn_command(yaw_error)
                        return
                if dwell_sec <= 0.0:
                    self._complete_inspection_stop(stop_id, reason='YOLO wait timeout')
                    return
                self.point_segment_mode = 'dwell'
                self.point_dwell_stop_id = stop_id
                self.point_dwell_until = time.monotonic() + dwell_sec
                self.get_logger().info(
                    '[TRACKING] Waiting for YOLO at %s for up to %.1fs.'
                    % (stop_id, dwell_sec)
                )
                return
            self._finish_point_segment()
            return

        self._publish_segment_straight_command(
            self.point_segment_heading,
            max(0.0, remaining),
        )

    def _initialize_point_segment(self) -> bool:
        while self.point_segment_index < len(self.point_route) - 1:
            sx, sy, _ = self.point_route[self.point_segment_index]
            tx, ty, tyaw = self.point_route[self.point_segment_index + 1]
            length = math.hypot(tx - sx, ty - sy)
            if length >= self.point_to_point_min_segment_length:
                self.point_segment_heading = math.atan2(ty - sy, tx - sx)
                self.point_segment_length = length
                self.point_segment_mode = 'rotate'
                self.point_segment_initialized = True
                self.point_segment_route_start_x = sx
                self.point_segment_route_start_y = sy
                self.point_segment_route_target_x = tx
                self.point_segment_route_target_y = ty
                self.point_segment_route_target_yaw = tyaw
                self.segment_profile_forward_duty = 0.0
                self._reset_path_controller_backend()
                return True

            self.point_segment_index += 1

        return False

    def _finish_point_segment(self) -> None:
        self.point_segment_index += 1
        self.point_segment_initialized = False
        self.point_segment_mode = 'rotate'
        self.segment_profile_forward_duty = 0.0
        self.segment_recovery_rotate_active = False

    def _lqg_smooth_turn_active(self) -> bool:
        return (
            self.path_controller_backend == 'lqg'
            and self.lqg_smooth_turn_enabled
            and self._path_controller_backend_active()
        )

    def _lqg_can_smooth_turn(self, yaw_error: float) -> bool:
        return (
            self._lqg_smooth_turn_active()
            and abs(yaw_error) <= self.lqg_stop_turn_angle
        )

    def _lqg_smooth_turn_speed_scale(self, yaw_error: float) -> float:
        abs_error = abs(yaw_error)
        if abs_error <= self.lqg_smooth_turn_angle:
            return 1.0

        transition_span = max(
            self.lqg_stop_turn_angle - self.lqg_smooth_turn_angle,
            1.0e-6,
        )
        blend = self._clamp(
            (abs_error - self.lqg_smooth_turn_angle) / transition_span,
            0.0,
            1.0,
        )
        return (
            1.0
            - (1.0 - self.lqg_min_smooth_turn_speed_scale) * blend
        )

    def _start_point_segment_straight_from_current_pose(
        self,
        x: float,
        y: float,
    ) -> None:
        dx = self.point_segment_route_target_x - x
        dy = self.point_segment_route_target_y - y
        projected_remaining = (
            dx * math.cos(self.point_segment_heading)
            + dy * math.sin(self.point_segment_heading)
        )
        target_distance = math.hypot(dx, dy)
        remaining = projected_remaining if projected_remaining > 0.0 else target_distance

        self.point_segment_start_x = x
        self.point_segment_start_y = y
        self.point_segment_length = max(
            self.point_to_point_arrival_tolerance,
            remaining,
        )
        self.point_segment_mode = 'straight'
        self.segment_recovery_rotate_active = False
        self.segment_profile_forward_duty = 0.0

    def _lqg_try_smooth_advance_to_next_segment(
        self,
        x: float,
        y: float,
        yaw: float,
        stop_id: str | None,
    ) -> bool:
        if not self._lqg_smooth_turn_active():
            return False
        if stop_id and stop_id not in self.completed_inspection_stops:
            return False

        next_index = self.point_segment_index + 1
        if next_index >= len(self.point_route) - 1:
            return False

        start_x, start_y, _ = self.point_route[next_index]
        target_x, target_y, target_yaw = self.point_route[next_index + 1]
        length = math.hypot(target_x - start_x, target_y - start_y)
        if length < self.point_to_point_min_segment_length:
            return False

        next_heading = math.atan2(target_y - start_y, target_x - start_x)
        yaw_error = wrap_to_pi(next_heading - yaw)
        if not self._lqg_can_smooth_turn(yaw_error):
            return False

        self.point_segment_index = next_index
        self.point_segment_initialized = True
        self.point_segment_heading = next_heading
        self.point_segment_route_start_x = start_x
        self.point_segment_route_start_y = start_y
        self.point_segment_route_target_x = target_x
        self.point_segment_route_target_y = target_y
        self.point_segment_route_target_yaw = target_yaw
        self._reset_path_controller_backend()
        self._start_point_segment_straight_from_current_pose(x, y)

        if self.verbose:
            self.get_logger().info(
                '[TRACKING] LQG smooth segment transition %d/%d yaw_error=%.1fdeg; no stop-turn.'
                % (
                    self.point_segment_index + 1,
                    len(self.point_route) - 1,
                    math.degrees(yaw_error),
                )
            )

        self._publish_segment_straight_command(
            self.point_segment_heading,
            self.point_segment_length,
        )
        return True

    def _apply_pending_path_update(self) -> None:
        if self.pending_path_update is None:
            return
        path, source = self.pending_path_update
        self.pending_path_update = None
        self._apply_new_path(path, source)

    def _inspection_dwell_active(self) -> bool:
        return self.point_segment_mode == 'dwell' and bool(self.point_dwell_stop_id)

    def _inspection_stop_hold_active(self) -> bool:
        return self.point_segment_mode in ('capture_rotate', 'dwell') and bool(
            self.point_dwell_stop_id
        )

    def _dwell_matches_stop(self, stop_id: str) -> bool:
        return self._inspection_dwell_active() and self.point_dwell_stop_id == stop_id

    def _point_segment_travelled(self, x: float, y: float) -> float:
        dx = x - self.point_segment_start_x
        dy = y - self.point_segment_start_y
        travelled = (
            dx * math.cos(self.point_segment_heading)
            + dy * math.sin(self.point_segment_heading)
        )
        return max(0.0, travelled)

    def _publish_segment_stop_turn_command(
        self,
        x: float,
        y: float,
        yaw: float,
        remaining_distance: float,
    ) -> None:
        if self.corner_turn_target_yaw is not None:
            yaw_error = wrap_to_pi(self.corner_turn_target_yaw - yaw)
            if abs(yaw_error) <= self.corner_turn_tolerance:
                completed_index = self.corner_turn_index
                self.corner_turn_index = None
                self.corner_turn_target_yaw = None
                if completed_index is not None:
                    self.path_index = max(
                        self.path_index,
                        min(completed_index + 1, len(self.path) - 1),
                    )
                self._publish_stop()
                if self.verbose:
                    self.get_logger().info('[TRACKING] Corner turn complete.')
                return

            self._publish_corner_turn_command(yaw_error)
            return

        corner_index = self._find_next_sharp_corner_index()
        if corner_index is None:
            target = self._select_lookahead_target(x, y)
            self._publish_velocity_command_for_target(target, remaining_distance)
            return

        cx, cy, _ = self.path[corner_index]
        corner_distance = math.hypot(cx - x, cy - y)
        if corner_distance <= self.corner_arrival_distance:
            self.corner_turn_index = corner_index
            self.corner_turn_target_yaw = self._heading_after_index(corner_index)
            self._publish_stop()
            if self.verbose:
                self.get_logger().info(
                    '[TRACKING] Corner reached at index=%d; turning to %.1f deg.'
                    % (corner_index, math.degrees(self.corner_turn_target_yaw))
                )
            return

        self._publish_segment_straight_command(
            self._heading_before_index(corner_index),
            min(remaining_distance, corner_distance),
        )

    def _publish_segment_straight_command(
        self,
        segment_heading: float,
        distance_remaining: float,
    ) -> None:
        x, y, yaw = self.latest_pose
        # Positive heading error commands left lower / right higher to rotate
        # back toward the desired segment heading.
        yaw_error = wrap_to_pi(segment_heading - yaw)

        if self.segment_recovery_rotate_active:
            if abs(yaw_error) > self.corner_turn_tolerance:
                self._publish_corner_turn_command(yaw_error)
                return

            self.segment_recovery_rotate_active = False
            self._publish_stop()
            if self.verbose:
                self.get_logger().info('[TRACKING] Segment recovery rotate complete.')
            return

        stop_turn_angle = (
            self.lqg_stop_turn_angle
            if self._lqg_smooth_turn_active()
            else self.rotate_in_place_angle
        )
        if abs(yaw_error) > stop_turn_angle:
            self.segment_recovery_rotate_active = True
            self._publish_corner_turn_command(yaw_error)
            return

        if abs(yaw_error) < self.segment_yaw_deadband:
            yaw_error = 0.0

        lateral_error, lateral_heading_correction = self._segment_lateral_correction(
            x,
            y,
        )
        if lateral_heading_correction != 0.0:
            yaw_error = wrap_to_pi(yaw_error - lateral_heading_correction)

        guide = self._active_visual_servo_guide()
        visual_marker = self._fresh_visual_marker(guide['marker_id']) if guide else None

        if self.segment_deceleration_distance > 0.0:
            speed_scale = self._smoothstep(
                distance_remaining / self.segment_deceleration_distance
            )
        else:
            speed_scale = 1.0

        visual_text = ''
        lateral_text = ''
        if (
            self.segment_lateral_kp != 0.0
            and abs(lateral_error) > self.segment_lateral_deadband_m
        ):
            lateral_text = (
                ' lateral=%.2fm lateral_cmd=%.1fdeg'
                % (lateral_error, math.degrees(lateral_heading_correction))
            )
        if guide is not None:
            if visual_marker is not None:
                visual_error = self._clamp(
                    float(visual_marker['bearing_left']),
                    -self.visual_servo_max_bearing_correction,
                    self.visual_servo_max_bearing_correction,
                )
                yaw_error = (
                    (1.0 - self.visual_servo_bearing_gain) * yaw_error
                    + self.visual_servo_bearing_gain * visual_error
                )
                speed_scale *= self.visual_servo_speed_scale
                visual_text = (
                    ' visual=ID%d bearing=%.1fdeg range=%.2fm'
                    % (
                        guide['marker_id'],
                        math.degrees(float(visual_marker['bearing_left'])),
                        float(visual_marker['range']),
                    )
                )
            else:
                speed_scale *= self.visual_servo_acquire_speed_scale
                visual_text = ' visual=ID%d acquiring' % guide['marker_id']

        if self._lqg_smooth_turn_active():
            smooth_speed_scale = self._lqg_smooth_turn_speed_scale(yaw_error)
            speed_scale *= smooth_speed_scale
            if smooth_speed_scale < 0.999:
                lateral_text += ' smooth_turn_scale=%.2f' % smooth_speed_scale

        if self._path_controller_backend_active():
            self._publish_segment_backend_command(
                segment_heading,
                distance_remaining,
                yaw_error,
                lateral_error,
                speed_scale,
                visual_text,
                lateral_text,
            )
            return

        forward_duty = self._segment_forward_duty(speed_scale)

        left_base = forward_duty * self.left_trim
        right_base = forward_duty * self.right_trim

        correction = self.segment_yaw_kp * yaw_error
        correction = self._clamp(
            correction,
            -self.segment_max_correction,
            self.segment_max_correction,
        )
        correction = self._clamp(correction, -forward_duty, forward_duty)

        left_duty = left_base - correction
        right_duty = right_base + correction
        left_duty = self._clamp(left_duty, -self.duty_cycle_limit, self.duty_cycle_limit)
        right_duty = self._clamp(right_duty, -self.duty_cycle_limit, self.duty_cycle_limit)
        left_duty, right_duty = self._apply_command_filter(left_duty, right_duty)
        left_duty, right_duty = self._apply_slew_limit(left_duty, right_duty)
        self._publish_motor_command(left_duty, right_duty)

        if self.verbose:
            self.get_logger().info(
                '[TRACKING] segment_straight heading=%.1fdeg remaining=%.2f '
                'yaw_error=%.1fdeg forward=%.1f correction=%.1f duty=(%.1f, %.1f)%s%s'
                % (
                    math.degrees(segment_heading),
                    distance_remaining,
                    math.degrees(yaw_error),
                    forward_duty,
                    correction,
                    left_duty,
                    right_duty,
                    lateral_text,
                    visual_text,
                )
            )
        elif visual_text:
            now = time.monotonic()
            if now - self.last_visual_servo_log >= 1.0:
                self.last_visual_servo_log = now
                self.get_logger().info(
                    '[TRACKING] visual servo%s yaw_error=%.1fdeg forward=%.1f'
                    % (visual_text, math.degrees(yaw_error), forward_duty)
            )

    def _segment_forward_duty(
        self,
        speed_scale: float,
    ) -> float:
        speed_scale = self._clamp(speed_scale, 0.0, 1.0)
        if not self.segment_velocity_profile_enabled:
            duty_span = self.segment_straight_duty - self.segment_min_straight_duty
            forward_duty = self.segment_min_straight_duty + duty_span * speed_scale
            return self._clamp(forward_duty, 0.0, self.duty_cycle_limit)

        approach_duty = min(
            self.segment_profile_approach_duty,
            self.segment_profile_cruise_duty,
            self.duty_cycle_limit,
        )
        cruise_duty = min(self.segment_profile_cruise_duty, self.duty_cycle_limit)
        target_duty = approach_duty + (cruise_duty - approach_duty) * speed_scale
        target_duty = self._clamp(target_duty, 0.0, self.duty_cycle_limit)

        if self.segment_profile_forward_duty <= 0.0:
            self.segment_profile_forward_duty = approach_duty

        if self.segment_profile_accel_duty_per_sec <= 0.0:
            self.segment_profile_forward_duty = target_duty
        else:
            max_delta = self.segment_profile_accel_duty_per_sec * self.control_period
            self.segment_profile_forward_duty += self._clamp(
                target_duty - self.segment_profile_forward_duty,
                -max_delta,
                max_delta,
            )

        return self._clamp(
            self.segment_profile_forward_duty,
            0.0,
            self.duty_cycle_limit,
        )

    def _publish_segment_backend_command(
        self,
        segment_heading: float,
        distance_remaining: float,
        yaw_error: float,
        lateral_error: float,
        speed_scale: float,
        visual_text: str,
        lateral_text: str,
    ) -> None:
        v_ref = min(self.nominal_speed, self.max_linear_speed) * self._clamp(
            speed_scale,
            0.0,
            1.0,
        )
        omega_ref = 0.0
        longitudinal_error = min(
            max(distance_remaining, 0.0),
            self.lookahead_distance,
        )
        # Longitudinal distance and yaw_error are reference-minus-actual, while
        # lateral_error is already actual-minus-reference relative to the
        # segment line.  Convert to the backend model convention.
        delta_v, delta_omega, label = self._path_controller_delta(
            [-longitudinal_error, lateral_error, -yaw_error],
            v_ref,
            omega_ref,
        )
        v_cmd = self._clamp(
            v_ref + delta_v,
            0.0,
            self.max_linear_speed,
        )
        omega_cmd = self._clamp(
            omega_ref + delta_omega,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        left_duty, right_duty = self._publish_velocity_command(v_cmd, omega_cmd)

        if self.verbose:
            self.get_logger().info(
                '[TRACKING] %s segment heading=%.1fdeg remaining=%.2f '
                'e=(%.2f, %.2f, %.1fdeg) v=%.3f+%.3f w=%.3f duty=(%.1f, %.1f)%s%s'
                % (
                    label,
                    math.degrees(segment_heading),
                    distance_remaining,
                    longitudinal_error,
                    lateral_error,
                    math.degrees(yaw_error),
                    v_ref,
                    delta_v,
                    omega_cmd,
                    left_duty,
                    right_duty,
                    lateral_text,
                    visual_text,
                )
            )
        elif visual_text:
            now = time.monotonic()
            if now - self.last_visual_servo_log >= 1.0:
                self.last_visual_servo_log = now
                self.get_logger().info(
                    '[TRACKING] %s visual servo%s yaw_error=%.1fdeg v=%.3f w=%.3f'
                    % (
                        label,
                        visual_text,
                        math.degrees(yaw_error),
                        v_cmd,
                        omega_cmd,
                    )
                )

    def _segment_lateral_correction(self, x: float, y: float) -> tuple[float, float]:
        if (
            self.segment_lateral_kp == 0.0
            or self.segment_lateral_max_heading <= 0.0
            or not self.point_to_point_mode
            or not self.point_segment_initialized
            or self.point_segment_mode != 'straight'
        ):
            return 0.0, 0.0

        dx = x - self.point_segment_route_start_x
        dy = y - self.point_segment_route_start_y
        lateral_error = (
            -math.sin(self.point_segment_heading) * dx
            + math.cos(self.point_segment_heading) * dy
        )
        if abs(lateral_error) <= self.segment_lateral_deadband_m:
            return lateral_error, 0.0

        active_error = lateral_error - math.copysign(
            self.segment_lateral_deadband_m,
            lateral_error,
        )
        heading_correction = self.segment_lateral_kp * active_error
        heading_correction = self._clamp(
            heading_correction,
            -self.segment_lateral_max_heading,
            self.segment_lateral_max_heading,
        )
        return lateral_error, heading_correction

    def _publish_corner_turn_command(self, yaw_error: float) -> None:
        self.segment_profile_forward_duty = 0.0
        if self._corner_turn_needs_forward_settle():
            forward_duty = 0.5 * (self.last_left_cmd + self.last_right_cmd)
            self._publish_turn_entry_stop()
            if self.verbose:
                self.get_logger().info(
                    '[TRACKING] corner_turn settling forward duty %.1f before rotate.'
                    % forward_duty
                )
            return

        turn_duty = self.corner_turn_kp * abs(yaw_error)
        turn_duty = self._clamp(
            turn_duty,
            self.corner_turn_min_duty,
            self.corner_turn_max_duty,
        )
        turn_sign = math.copysign(1.0, yaw_error)
        left_duty = -turn_sign * turn_duty * self.left_trim
        right_duty = turn_sign * turn_duty * self.right_trim
        left_duty, right_duty = self._apply_command_filter(left_duty, right_duty)
        left_duty, right_duty = self._apply_slew_limit(left_duty, right_duty)
        self._publish_motor_command(left_duty, right_duty)

        if self.verbose:
            self.get_logger().info(
                '[TRACKING] corner_turn yaw_error=%.1fdeg duty=(%.1f, %.1f)'
                % (
                    math.degrees(yaw_error),
                    left_duty,
                    right_duty,
                )
            )

    def _find_next_sharp_corner_index(self) -> int | None:
        if (
            not self.right_angle_corner_mode
            or self.corner_turn_target_yaw is not None
            or len(self.path) < 3
        ):
            return None

        start = max(1, self.path_index)
        end = min(len(self.path) - 1, self.path_index + self.corner_search_points)
        for index in range(start, end):
            if self._is_sharp_corner_index(index):
                return index

        return None

    def _is_sharp_corner_index(self, index: int) -> bool:
        return self._is_sharp_corner(self.path, index)

    def _is_sharp_corner(
        self,
        path: list[tuple[float, float, float]],
        index: int,
    ) -> bool:
        if index <= 0 or index >= len(path) - 1:
            return False

        prev_x, prev_y, _ = path[index - 1]
        curr_x, curr_y, _ = path[index]
        next_x, next_y, _ = path[index + 1]

        in_len = math.hypot(curr_x - prev_x, curr_y - prev_y)
        out_len = math.hypot(next_x - curr_x, next_y - curr_y)
        if in_len < 0.03 or out_len < 0.03:
            return False

        heading_in = math.atan2(curr_y - prev_y, curr_x - prev_x)
        heading_out = math.atan2(next_y - curr_y, next_x - curr_x)
        return abs(wrap_to_pi(heading_out - heading_in)) >= self.corner_angle_threshold

    def _heading_after_index(self, index: int) -> float:
        if index + 1 < len(self.path):
            cx, cy, _ = self.path[index]
            nx, ny, nyaw = self.path[index + 1]
            if math.hypot(nx - cx, ny - cy) > 1.0e-6:
                return math.atan2(ny - cy, nx - cx)
            return nyaw
        return self.path[index][2]

    def _heading_before_index(self, index: int) -> float:
        if index > 0:
            px, py, pyaw = self.path[index - 1]
            cx, cy, _ = self.path[index]
            if math.hypot(cx - px, cy - py) > 1.0e-6:
                return math.atan2(cy - py, cx - px)
            return pyaw
        return self.path[index][2]

    def _parse_visual_servo_guides(self, text: str) -> list[dict[str, float]]:
        guides: list[dict[str, float]] = []
        for entry in text.split(';'):
            entry = entry.strip()
            if not entry:
                continue
            if ':' not in entry:
                self.get_logger().warn('[TRACKING] Bad visual_servo_guides entry: %s' % entry)
                continue
            marker_text, value_text = entry.split(':', 1)
            fields = [field.strip() for field in value_text.split(',')]
            if len(fields) < 2:
                self.get_logger().warn('[TRACKING] Bad visual_servo_guides entry: %s' % entry)
                continue
            try:
                arrival_range = 0.0
                if len(fields) >= 3 and fields[2]:
                    arrival_range = max(0.0, float(fields[2]))
                guides.append(
                    {
                        'marker_id': int(marker_text),
                        'lane_x': float(fields[0]),
                        'heading': math.radians(float(fields[1])),
                        'arrival_range': arrival_range,
                    }
                )
            except ValueError:
                self.get_logger().warn('[TRACKING] Bad visual_servo_guides entry: %s' % entry)
        return guides

    def _active_visual_servo_guide(self) -> dict[str, float] | None:
        if not self.visual_servo_enabled or not self.visual_servo_guides:
            return None
        if self.point_segment_mode != 'straight':
            return None

        lane_x = 0.5 * (
            self.point_segment_route_start_x + self.point_segment_route_target_x
        )
        for guide in self.visual_servo_guides:
            heading_error = abs(wrap_to_pi(self.point_segment_heading - guide['heading']))
            lane_error = abs(lane_x - guide['lane_x'])
            if (
                heading_error <= self.visual_servo_heading_tolerance
                and lane_error <= self.visual_servo_lane_tolerance_m
            ):
                return guide

        return None

    def _visual_arrival_hold(
        self,
        travelled: float,
    ) -> tuple[dict[str, float], dict[str, float]] | None:
        guide = self._active_visual_servo_guide()
        if guide is None:
            return None

        arrival_range = float(guide.get('arrival_range', 0.0))
        if arrival_range <= 0.0:
            return None

        visual_marker = self._fresh_visual_marker(int(guide['marker_id']))
        if visual_marker is None:
            return None

        marker_range = float(visual_marker['range'])
        if marker_range <= arrival_range:
            return None

        overrun = max(0.0, travelled - self.point_segment_length)
        if overrun >= self.visual_arrival_extra_distance_m:
            now = time.monotonic()
            if now - self.last_visual_arrival_log >= 1.0:
                self.last_visual_arrival_log = now
                self.get_logger().warn(
                    '[TRACKING] visual arrival override released after %.2fm '
                    'overrun: ID%d range=%.2fm > %.2fm'
                    % (
                        overrun,
                        guide['marker_id'],
                        marker_range,
                        arrival_range,
                    )
                )
            return None

        return guide, visual_marker

    def _fresh_visual_marker(self, marker_id: int) -> dict[str, float] | None:
        sample = self.latest_visual_markers.get(marker_id)
        if sample is None:
            return None

        now = time.monotonic()
        if (
            self.visual_servo_marker_timeout_sec > 0.0
            and now - float(sample['time']) > self.visual_servo_marker_timeout_sec
        ):
            return None

        marker_range = float(sample['range'])
        if (
            marker_range < self.visual_servo_min_range_m
            or marker_range > self.visual_servo_max_range_m
        ):
            return None

        return sample

    def _remaining_path_distance(self, x: float, y: float) -> float:
        if not self.path:
            return 0.0
        index = min(self.path_index, len(self.path) - 1)
        px, py, _ = self.path[index]
        distance = math.hypot(px - x, py - y)
        for current, nxt in zip(self.path[index:], self.path[index + 1:]):
            cx, cy, _ = current
            nx, ny, _ = nxt
            distance += math.hypot(nx - cx, ny - cy)
        return distance

    def _handle_plant_bbox_focus(self, now: float) -> bool:
        if (
            not self.plant_bbox_servo_enabled
            or self.camera_pan_pub is None
            or self.latest_pose is None
        ):
            return False

        focus_id = self._plant_focus_target_id()
        if not focus_id:
            focus_id = self._opportunistic_plant_focus_target_id(now)
        if not focus_id:
            self._reset_plant_focus()
            return False

        if focus_id in self.completed_inspection_stops:
            self._reset_plant_focus()
            return False

        opportunistic_focus = self._is_opportunistic_focus_id(focus_id)
        known_opportunistic_focus = self._is_known_opportunistic_focus_id(focus_id)
        search_focus = self._is_search_focus_id(focus_id)
        if focus_id != self.active_plant_focus_id:
            self.active_plant_focus_id = focus_id
            self.plant_bbox_centered_since = 0.0
            self.plant_focus_started_time = now
            self.plant_capture_stop_id = ''
            self.plant_capture_until = 0.0
            self.opportunistic_initial_save_focus_id = ''
            self.opportunistic_initial_save_until = 0.0
            self.plant_search_pan_us = (
                self._camera_search_min_pan_us()
                if search_focus
                else self._camera_pan_us_for_focus(focus_id)
            )
            self.plant_search_direction = 1
            self.last_plant_search_time = 0.0
            if opportunistic_focus:
                estimate = self.opportunistic_capture_estimates.get(focus_id)
                estimate_text = (
                    ' estimate=(%.2f, %.2f)' % estimate if estimate is not None else ''
                )
                self.get_logger().info(
                    '[TRACKING] Opportunistic plant %s detected%s; stopping for YOLO bbox centering.'
                    % (focus_id, estimate_text)
                )
                if self.plant_opportunistic_initial_save_sec > 0.0:
                    self.opportunistic_initial_save_focus_id = focus_id
                    self.opportunistic_initial_save_until = (
                        now + self.plant_opportunistic_initial_save_sec
                    )
                    self._publish_positive_save_enabled(True, force=True)
                    self.get_logger().info(
                        '[TRACKING] Opportunistic plant %s initial image save window started for %.1fs.'
                        % (focus_id, self.plant_opportunistic_initial_save_sec)
                    )
                else:
                    self._publish_positive_save_enabled(False)
            elif search_focus:
                self.get_logger().info(
                    '[TRACKING] Coverage search %s active; sweeping camera for hidden plants.'
                    % focus_id
                )
                self._publish_positive_save_enabled(False)
            else:
                self.get_logger().info(
                    '[TRACKING] Plant focus %s active; stopping for YOLO bbox centering.'
                    % focus_id
                )
                self._publish_positive_save_enabled(False)
            self._publish_camera_pan(
                self.plant_search_pan_us,
                now,
                target_id='%s:map' % focus_id,
                relative_angle=None,
                saturated=False,
            )
            if self.plant_bbox_tilt_servo_enabled:
                self._publish_camera_tilt(
                    self.camera_tilt_center_us,
                    now,
                    target_id='%s:center' % focus_id,
                    force=True,
                )

        self._publish_detector_enabled(True)
        self._publish_stop()
        self._update_opportunistic_initial_save_gate(
            now,
            focus_id,
            opportunistic_focus,
        )
        search_deadline_expired = (
            search_focus
            and self.point_dwell_stop_id == focus_id
            and self.point_dwell_until > 0.0
            and now >= self.point_dwell_until
        )

        if self.plant_capture_stop_id:
            if self.plant_capture_stop_id != focus_id:
                self.plant_capture_stop_id = ''
                self.plant_capture_until = 0.0
                self._publish_positive_save_enabled(False)
            elif now < self.plant_capture_until:
                self._publish_positive_save_enabled(True, force=True)
                self._log_status(
                    now,
                    '[TRACKING] Capturing stable plant %s images for %.1fs.'
                    % (focus_id, self.plant_capture_until - now),
                )
                return True
            else:
                self._publish_positive_save_enabled(False)
                if search_focus:
                    self._complete_inspection_stop(
                        focus_id,
                        reason='hidden plant stable capture complete',
                    )
                elif opportunistic_focus:
                    self._complete_opportunistic_capture(
                        focus_id,
                        reason='YOLO stable capture complete',
                    )
                else:
                    self._complete_inspection_stop(
                        focus_id,
                        reason='YOLO stable capture complete',
                    )
                self._reset_plant_focus()
                return True

        if search_deadline_expired:
            self._complete_inspection_stop(
                focus_id,
                reason='coverage search dwell complete',
            )
            self._reset_plant_focus()
            return True

        if (
            (opportunistic_focus or known_opportunistic_focus)
            and self.plant_opportunistic_max_capture_sec > 0.0
            and now - self.plant_focus_started_time
            > self.plant_opportunistic_max_capture_sec
        ):
            if opportunistic_focus:
                self._abort_opportunistic_capture(now, focus_id, 'capture timeout')
            else:
                self._abort_known_opportunistic_capture(now, focus_id, 'capture timeout')
            return False

        detections_msg = self._fresh_plant_detections(now)
        detection = (
            self._best_plant_detection(detections_msg, focus_id)
            if detections_msg is not None
            else None
        )

        if detection is None:
            if (
                (opportunistic_focus or known_opportunistic_focus)
                and self.plant_opportunistic_lost_timeout_sec > 0.0
                and now - self.plant_focus_started_time
                > self.plant_opportunistic_lost_timeout_sec
            ):
                if opportunistic_focus:
                    self._abort_opportunistic_capture(now, focus_id, 'bbox lost')
                else:
                    self._abort_known_opportunistic_capture(now, focus_id, 'bbox lost')
                return False
            self.plant_bbox_centered_since = 0.0
            self.plant_capture_stop_id = ''
            self.plant_capture_until = 0.0
            self._update_opportunistic_initial_save_gate(
                now,
                focus_id,
                opportunistic_focus,
            )
            self._search_plant_with_camera(now, focus_id)
            self._log_status(
                now,
                '[TRACKING] Searching camera for plant %s bbox (%s).'
                % (focus_id, self._plant_detection_status(now)),
            )
            return True

        error_x, error_y, centered = self._bbox_center_errors(detection)
        if not centered:
            self.plant_bbox_centered_since = 0.0
            self.plant_capture_stop_id = ''
            self.plant_capture_until = 0.0
            self._update_opportunistic_initial_save_gate(
                now,
                focus_id,
                opportunistic_focus,
            )
            self._servo_camera_to_bbox(now, focus_id, error_x, error_y)
            self._log_status(
                now,
                '[TRACKING] Centering plant %s bbox dx=%.0fpx dy=%.0fpx.'
                % (focus_id, error_x, error_y),
            )
            return True

        if not self._bbox_within_angle_tolerance(
            error_x,
            error_y,
            self.plant_bbox_fine_tolerance_angle,
        ):
            self._servo_camera_to_bbox(
                now,
                focus_id,
                error_x,
                error_y,
                pan_gain=self.plant_bbox_fine_pan_gain_us_per_px,
                tilt_gain=self.plant_bbox_fine_tilt_gain_us_per_px,
            )

        if self.plant_bbox_centered_since <= 0.0:
            self.plant_bbox_centered_since = now
            self.get_logger().info(
                '[TRACKING] Plant %s bbox centered; holding for %.1fs.'
                % (focus_id, self.plant_bbox_center_hold_sec)
            )
            return True

        if now - self.plant_bbox_centered_since < self.plant_bbox_center_hold_sec:
            return True

        if not self._plant_centered_detection_can_complete(detections_msg, detection):
            self._log_status(
                now,
                '[TRACKING] Plant %s bbox centered but capture is not valid yet '
                '(confidence=%.2f decision=%s).'
                % (
                    focus_id,
                    float(detection.confidence),
                    str(detections_msg.decision_hint),
                ),
            )
            return True

        if self.plant_capture_dwell_sec <= 0.0:
            if opportunistic_focus or search_focus:
                self._publish_unknown_plant_capture_event(
                    focus_id,
                    detection,
                    detections_msg,
                    reason='YOLO bbox centered',
                )
            if search_focus:
                self._complete_inspection_stop(focus_id, reason='hidden plant captured')
                self._reset_plant_focus()
                return True
            if opportunistic_focus:
                self._complete_opportunistic_capture(focus_id, reason='YOLO bbox centered')
            else:
                self._complete_inspection_stop(focus_id, reason='YOLO bbox centered')
            self._reset_plant_focus()
            return True

        self.plant_capture_stop_id = focus_id
        self.plant_capture_until = now + self.plant_capture_dwell_sec
        self._publish_positive_save_enabled(True, force=True)
        if opportunistic_focus or search_focus:
            self._publish_unknown_plant_capture_event(
                focus_id,
                detection,
                detections_msg,
                reason='stable capture window started',
            )
        self.get_logger().info(
            '[TRACKING] Plant %s stable capture window started for %.1fs.'
            % (focus_id, self.plant_capture_dwell_sec)
        )
        return True

    def _plant_focus_target_id(self) -> str | None:
        if (
            self.active_plant_focus_id
            and self.active_plant_focus_id not in self.completed_inspection_stops
        ):
            return self.active_plant_focus_id

        if self.point_segment_mode in ('capture_rotate', 'dwell') and self.point_dwell_stop_id:
            if self.point_dwell_stop_id not in self.completed_inspection_stops:
                return self.point_dwell_stop_id

        if self.latest_pose is None:
            return None

        x, y, _ = self.latest_pose
        nearest_id: str | None = None
        nearest_distance = float('inf')
        for plant_id, plant_x, plant_y in self.plants:
            if plant_id in self.completed_inspection_stops:
                continue
            distance = math.hypot(plant_x - x, plant_y - y)
            if distance < nearest_distance:
                nearest_id = plant_id
                nearest_distance = distance

        if nearest_id and nearest_distance <= self.plant_capture_radius_m:
            return nearest_id
        return None

    def _opportunistic_plant_focus_target_id(self, now: float) -> str | None:
        if (
            not self.plant_opportunistic_capture_enabled
            or self._inspection_stop_hold_active()
            or self.latest_pose is None
        ):
            return None

        if (
            self.plant_opportunistic_cooldown_sec > 0.0
            and now - self.last_opportunistic_capture_time
            < self.plant_opportunistic_cooldown_sec
        ):
            return None

        detections_msg = self._fresh_plant_detections(now)
        if detections_msg is None:
            return None

        detection = self._best_plant_detection(detections_msg, '')
        if detection is None:
            return None
        if float(detection.confidence) < self.plant_opportunistic_min_confidence:
            return None

        estimate = self._estimate_detection_world_position(detection)
        if estimate is None:
            return None
        known_focus_id = self._known_plant_id_for_detection_estimate(
            detection,
            estimate,
        )
        if known_focus_id:
            if known_focus_id not in self.known_opportunistic_focus_ids:
                self.get_logger().info(
                    '[TRACKING] Known plant %s seen along path; stopping for normal capture.'
                    % known_focus_id
                )
            self.known_opportunistic_focus_ids.add(known_focus_id)
            return known_focus_id
        if not self._opportunistic_estimate_is_new(estimate):
            return None

        focus_id = 'U%d' % self.next_opportunistic_capture_index
        self.next_opportunistic_capture_index += 1
        self.opportunistic_capture_estimates[focus_id] = estimate
        return focus_id

    def _known_plant_id_for_detection_estimate(
        self,
        detection,
        estimate: tuple[float, float],
    ) -> str:
        if (
            not self.plant_by_id
            or self.plant_opportunistic_known_plant_exclusion_m <= 0.0
        ):
            return ''

        ex, ey = estimate
        detection_class = str(detection.class_name).strip().lower()
        association_radius = max(
            self.plant_opportunistic_known_plant_exclusion_m,
            self.plant_capture_radius_m,
        )
        candidates: list[tuple[float, str, bool]] = []
        for plant_id, (plant_x, plant_y) in self.plant_by_id.items():
            if plant_id in self.completed_inspection_stops:
                continue
            distance = math.hypot(ex - plant_x, ey - plant_y)
            if distance > association_radius:
                continue
            class_match = (
                bool(detection_class)
                and detection_class == self._expected_plant_class_for_stop(plant_id)
            )
            candidates.append((distance, plant_id, class_match))

        if not candidates:
            return ''

        matching = [candidate for candidate in candidates if candidate[2]]
        pool = matching if matching else candidates
        pool.sort(key=lambda item: item[0])
        return pool[0][1]

    def _estimate_detection_world_position(self, detection) -> tuple[float, float] | None:
        if self.latest_pose is None:
            return None

        x, y, _yaw = self.latest_pose
        components = self._detection_bearing_components(detection)
        if components is None:
            return None
        global_bearing, _camera_pan_left, _bbox_bearing_left, _pan_us = components
        distance, _source = self._estimate_detection_range_from_scan(global_bearing)
        return (
            x + distance * math.cos(global_bearing),
            y + distance * math.sin(global_bearing),
        )

    def _detection_bearing_components(
        self,
        detection,
    ) -> tuple[float, float, float, int] | None:
        if self.latest_pose is None:
            return None

        _x, _y, yaw = self.latest_pose
        error_x = float(detection.centre_x) - 0.5 * self.plant_bbox_image_width
        bbox_bearing_left = -math.atan2(error_x, self.plant_bbox_fx_px)
        pan_us = (
            self.last_camera_pan_us
            if self.last_camera_pan_us is not None
            else self.camera_pan_center_us
        )
        camera_pan_left = self._camera_pan_left_angle_from_us(pan_us)
        global_bearing = wrap_to_pi(yaw + camera_pan_left + bbox_bearing_left)
        return global_bearing, camera_pan_left, bbox_bearing_left, int(pan_us)

    def _estimate_detection_range_from_scan(
        self,
        global_bearing: float,
    ) -> tuple[float, str]:
        if self.latest_pose is None or self.latest_scan is None:
            return self.plant_opportunistic_assumed_distance_m, 'assumed_no_scan'

        now = time.monotonic()
        if (
            self.latest_scan_time is not None
            and self.plant_unknown_lidar_stale_timeout_sec > 0.0
            and now - self.latest_scan_time > self.plant_unknown_lidar_stale_timeout_sec
        ):
            return self.plant_opportunistic_assumed_distance_m, 'assumed_stale_scan'

        _x, _y, yaw = self.latest_pose
        scan = self.latest_scan
        angle_increment = float(scan.angle_increment)
        if abs(angle_increment) < 1.0e-9:
            return self.plant_opportunistic_assumed_distance_m, 'assumed_bad_scan'

        local_angle = wrap_to_pi(global_bearing - yaw - self.lidar_in_base_yaw)
        if abs(self.lidar_scan_angle_multiplier) > 1.0e-6:
            local_angle /= self.lidar_scan_angle_multiplier

        center_index = int(round((local_angle - float(scan.angle_min)) / angle_increment))
        if center_index < 0 or center_index >= len(scan.ranges):
            return self.plant_opportunistic_assumed_distance_m, 'assumed_outside_scan'

        range_min = max(0.0, float(scan.range_min))
        range_max = self.plant_unknown_lidar_max_range_m
        if math.isfinite(float(scan.range_max)):
            range_max = min(range_max, float(scan.range_max))

        valid_ranges: list[float] = []
        radius = self.plant_unknown_lidar_neighbor_beams
        for index in range(center_index - radius, center_index + radius + 1):
            if index < 0 or index >= len(scan.ranges):
                continue
            scan_range = float(scan.ranges[index])
            if not math.isfinite(scan_range):
                continue
            if scan_range < range_min or scan_range > range_max:
                continue
            valid_ranges.append(scan_range)

        if not valid_ranges:
            return self.plant_opportunistic_assumed_distance_m, 'assumed_no_lidar_hit'

        valid_ranges.sort()
        return valid_ranges[len(valid_ranges) // 2], 'lidar'

    def _publish_unknown_plant_capture_event(
        self,
        focus_id: str,
        detection,
        detections_msg: PlantDetections,
        *,
        reason: str,
    ) -> None:
        if (
            self.unknown_plant_capture_pub is None
            or focus_id in self.published_opportunistic_capture_events
            or self.latest_pose is None
        ):
            return

        components = self._detection_bearing_components(detection)
        if components is None:
            return

        robot_x, robot_y, robot_yaw = self.latest_pose
        global_bearing, camera_pan_left, bbox_bearing_left, pan_us = components
        distance, range_source = self._estimate_detection_range_from_scan(global_bearing)
        plant_x = robot_x + distance * math.cos(global_bearing)
        plant_y = robot_y + distance * math.sin(global_bearing)
        event_id = focus_id if self._is_opportunistic_focus_id(focus_id) else ''

        estimate = (plant_x, plant_y)
        if self._is_opportunistic_focus_id(focus_id):
            self.opportunistic_capture_estimates[focus_id] = estimate
        msg = String()
        msg.data = json.dumps(
            {
                'id': event_id,
                'focus_id': focus_id,
                'type': 'unknown_plant_capture',
                'reason': reason,
                'stamp_sec': time.time(),
                'robot_x': robot_x,
                'robot_y': robot_y,
                'robot_yaw': robot_yaw,
                'x': plant_x,
                'y': plant_y,
                'global_bearing': global_bearing,
                'camera_pan_us': pan_us,
                'camera_pan_left_rad': camera_pan_left,
                'bbox_bearing_left_rad': bbox_bearing_left,
                'range_m': distance,
                'range_source': range_source,
                'confidence': float(detection.confidence),
                'class_name': str(detection.class_name),
                'decision_hint': str(detections_msg.decision_hint),
                'detection_count': int(detections_msg.detection_count),
            },
            sort_keys=True,
        )
        self.unknown_plant_capture_pub.publish(msg)
        self.published_opportunistic_capture_events.add(focus_id)
        self.get_logger().info(
            '[TRACKING] Unknown plant %s map event: pos=(%.2f, %.2f) range=%.2fm source=%s bearing=%.1fdeg.'
            % (
                focus_id,
                plant_x,
                plant_y,
                distance,
                range_source,
                math.degrees(global_bearing),
            )
        )

    def _opportunistic_estimate_is_new(self, estimate: tuple[float, float]) -> bool:
        ex, ey = estimate
        if (
            self.plant_opportunistic_known_plant_exclusion_m > 0.0
            and self.plant_by_id
        ):
            nearest_plant = min(
                math.hypot(ex - px, ey - py)
                for px, py in self.plant_by_id.values()
            )
            if nearest_plant <= self.plant_opportunistic_known_plant_exclusion_m:
                return False

        if (
            self.plant_opportunistic_stop_exclusion_m > 0.0
            and self.inspection_stops
        ):
            nearest_stop = min(
                math.hypot(ex - sx, ey - sy)
                for sx, sy in self.inspection_stops.values()
            )
            if nearest_stop <= self.plant_opportunistic_stop_exclusion_m:
                return False

            if self.latest_pose is not None:
                rx, ry, _ = self.latest_pose
                robot_stop_distance = min(
                    math.hypot(rx - sx, ry - sy)
                    for sx, sy in self.inspection_stops.values()
                )
                if robot_stop_distance <= self.plant_opportunistic_stop_exclusion_m:
                    return False

        if (
            self.plant_opportunistic_duplicate_radius_m > 0.0
            and self.completed_opportunistic_captures
        ):
            nearest_captured = min(
                math.hypot(ex - cx, ey - cy)
                for cx, cy in self.completed_opportunistic_captures.values()
            )
            if nearest_captured <= self.plant_opportunistic_duplicate_radius_m:
                return False

        return True

    def _is_opportunistic_focus_id(self, focus_id: str) -> bool:
        return focus_id in self.opportunistic_capture_estimates

    def _is_known_opportunistic_focus_id(self, focus_id: str) -> bool:
        return focus_id in self.known_opportunistic_focus_ids

    def _is_search_focus_id(self, focus_id: str) -> bool:
        return (
            bool(focus_id)
            and focus_id in self.inspection_stops
            and focus_id not in self.plant_by_id
            and not self._is_opportunistic_focus_id(focus_id)
        )

    def _abort_opportunistic_capture(
        self,
        now: float,
        focus_id: str,
        reason: str,
    ) -> None:
        self.last_opportunistic_capture_time = now
        self._publish_positive_save_enabled(False)
        self.get_logger().info(
            '[TRACKING] Opportunistic plant %s aborted: %s; resuming path.'
            % (focus_id, reason)
        )
        self._reset_plant_focus()
        self._center_camera_after_inspection(now)

    def _abort_known_opportunistic_capture(
        self,
        now: float,
        focus_id: str,
        reason: str,
    ) -> None:
        self.last_opportunistic_capture_time = now
        self.known_opportunistic_focus_ids.discard(focus_id)
        self._publish_positive_save_enabled(False)
        self.get_logger().info(
            '[TRACKING] Known plant %s along-path capture aborted: %s; resuming path.'
            % (focus_id, reason)
        )
        self._reset_plant_focus()
        self._center_camera_after_inspection(now)

    def _complete_opportunistic_capture(self, focus_id: str, reason: str) -> None:
        now = time.monotonic()
        estimate = self.opportunistic_capture_estimates.get(focus_id)
        if estimate is not None:
            self.completed_opportunistic_captures[focus_id] = estimate
        self.last_opportunistic_capture_time = now
        self._publish_positive_save_enabled(False)
        self._center_camera_after_inspection(now)
        if estimate is None:
            estimate_text = ''
        else:
            estimate_text = ' estimate=(%.2f, %.2f)' % estimate
        self.get_logger().info(
            '[TRACKING] Opportunistic plant %s captured by %s%s; resuming path.'
            % (focus_id, reason, estimate_text)
        )
        self._publish_stop()

    def _update_opportunistic_initial_save_gate(
        self,
        now: float,
        focus_id: str,
        opportunistic_focus: bool,
    ) -> bool:
        if self.plant_capture_stop_id:
            return False

        save_active = (
            opportunistic_focus
            and focus_id == self.opportunistic_initial_save_focus_id
            and now < self.opportunistic_initial_save_until
        )
        if save_active:
            self._publish_positive_save_enabled(True)
            return True

        if focus_id == self.opportunistic_initial_save_focus_id:
            self.opportunistic_initial_save_focus_id = ''
            self.opportunistic_initial_save_until = 0.0

        self._publish_positive_save_enabled(False)
        return False

    def _reset_plant_focus(self) -> None:
        self.active_plant_focus_id = ''
        self.plant_bbox_centered_since = 0.0
        self.plant_focus_started_time = 0.0
        self.plant_capture_stop_id = ''
        self.plant_capture_until = 0.0
        self.opportunistic_initial_save_focus_id = ''
        self.opportunistic_initial_save_until = 0.0
        self._publish_positive_save_enabled(False)
        self.last_plant_search_time = 0.0
        self.last_plant_bbox_servo_time = 0.0

    def _fresh_plant_detections(self, now: float) -> PlantDetections | None:
        if self.latest_plant_detections is None or self.last_plant_detections_time is None:
            return None
        if (
            self.plant_detection_stale_timeout_sec > 0.0
            and now - self.last_plant_detections_time
            > self.plant_detection_stale_timeout_sec
        ):
            return None
        return self.latest_plant_detections

    def _plant_detection_status(self, now: float) -> str:
        if self.latest_plant_detections is None or self.last_plant_detections_time is None:
            return 'no detection message yet'
        age = now - self.last_plant_detections_time
        count = int(self.latest_plant_detections.detection_count)
        decision = str(self.latest_plant_detections.decision_hint)
        if (
            self.plant_detection_stale_timeout_sec > 0.0
            and age > self.plant_detection_stale_timeout_sec
        ):
            return 'last message stale %.2fs count=%d decision=%s' % (
                age,
                count,
                decision,
            )
        return 'fresh message %.2fs count=%d decision=%s' % (age, count, decision)

    def _best_plant_detection(self, msg: PlantDetections, stop_id: str):
        if int(msg.detection_count) <= 0 or not msg.detections:
            return None

        detections = list(msg.detections)
        if self.plant_completion_match_mode == 'stop_id':
            expected_class = self._expected_plant_class_for_stop(stop_id)
            matching = [
                detection
                for detection in detections
                if str(detection.class_name).strip().lower() == expected_class
            ]
            if matching:
                detections = matching

        return max(detections, key=lambda detection: float(detection.confidence))

    def _bbox_center_errors(self, detection) -> tuple[float, float, bool]:
        error_x = float(detection.centre_x) - 0.5 * self.plant_bbox_image_width
        error_y = float(detection.centre_y) - 0.5 * self.plant_bbox_image_height
        centered = self._bbox_within_capture_tolerance(error_x, error_y)
        return error_x, error_y, centered

    def _bbox_within_capture_tolerance(self, error_x: float, error_y: float) -> bool:
        tolerance_x = self.plant_bbox_center_tolerance_x_px
        tolerance_y = self.plant_bbox_center_tolerance_y_px
        if self.plant_bbox_center_tolerance_angle > 0.0:
            tolerance_x = max(
                tolerance_x,
                self.plant_bbox_fx_px * math.tan(self.plant_bbox_center_tolerance_angle),
            )
            tolerance_y = max(
                tolerance_y,
                self.plant_bbox_fy_px * math.tan(self.plant_bbox_center_tolerance_angle),
            )
        return (
            abs(error_x) <= tolerance_x
            and (
                not self.plant_bbox_tilt_servo_enabled
                or abs(error_y) <= tolerance_y
            )
        )

    def _bbox_within_angle_tolerance(
        self,
        error_x: float,
        error_y: float,
        tolerance_angle: float,
    ) -> bool:
        if tolerance_angle <= 0.0:
            return True
        tolerance_x = self.plant_bbox_fx_px * math.tan(tolerance_angle)
        tolerance_y = self.plant_bbox_fy_px * math.tan(tolerance_angle)
        return (
            abs(error_x) <= tolerance_x
            and (
                not self.plant_bbox_tilt_servo_enabled
                or abs(error_y) <= tolerance_y
            )
        )

    def _servo_camera_to_bbox(
        self,
        now: float,
        stop_id: str,
        error_x: float,
        error_y: float,
        pan_gain: float | None = None,
        tilt_gain: float | None = None,
    ) -> None:
        if now - self.last_plant_bbox_servo_time < self.plant_bbox_update_period_sec:
            return
        self.last_plant_bbox_servo_time = now

        pan_gain = self.plant_bbox_pan_gain_us_per_px if pan_gain is None else pan_gain
        tilt_gain = self.plant_bbox_tilt_gain_us_per_px if tilt_gain is None else tilt_gain
        current_pan = (
            self.last_camera_pan_us
            if self.last_camera_pan_us is not None
            else self.camera_pan_center_us
        )
        pan_sign = 1.0 if self.camera_pan_left_positive_is_decreasing else -1.0
        pan_us = int(
            round(
                self._clamp(
                    current_pan + pan_sign * error_x * pan_gain,
                    self.camera_pan_min_us,
                    self.camera_pan_max_us,
                )
            )
        )

        self._publish_camera_pan(
            pan_us,
            now,
            target_id='%s:bbox' % stop_id,
            relative_angle=None,
            saturated=False,
        )
        if self.plant_bbox_tilt_servo_enabled:
            current_tilt = (
                self.last_camera_tilt_us
                if self.last_camera_tilt_us is not None
                else self.camera_tilt_center_us
            )
            tilt_sign = (
                1.0 if self.camera_tilt_image_down_positive_increases_us else -1.0
            )
            tilt_us = int(
                round(
                    self._clamp(
                        current_tilt
                        + tilt_sign * error_y * tilt_gain,
                        self.camera_tilt_min_us,
                        self.camera_tilt_max_us,
                    )
                )
            )
            self._publish_camera_tilt(
                tilt_us,
                now,
                target_id='%s:bbox' % stop_id,
            )
        self.plant_search_pan_us = pan_us

    def _search_plant_with_camera(self, now: float, stop_id: str) -> None:
        if now - self.last_plant_search_time < self.plant_search_update_period_sec:
            return
        self.last_plant_search_time = now

        span = max(0, self.plant_search_pan_span_us)
        search_min = max(self.camera_pan_min_us, self.camera_pan_center_us - span)
        search_max = min(self.camera_pan_max_us, self.camera_pan_center_us + span)
        if search_min >= search_max:
            search_min = self.camera_pan_min_us
            search_max = self.camera_pan_max_us

        next_pan = self.plant_search_pan_us + (
            self.plant_search_direction * self.plant_search_pan_step_us
        )
        if next_pan >= search_max:
            next_pan = search_max
            self.plant_search_direction = -1
        elif next_pan <= search_min:
            next_pan = search_min
            self.plant_search_direction = 1

        self.plant_search_pan_us = int(next_pan)
        self._publish_camera_pan(
            self.plant_search_pan_us,
            now,
            target_id='%s:search' % stop_id,
            relative_angle=None,
            saturated=False,
        )
        if self.plant_bbox_tilt_servo_enabled:
            self._publish_camera_tilt(
                self.camera_tilt_center_us,
                now,
                target_id='%s:search' % stop_id,
            )

    def _camera_pan_us_for_focus(self, focus_id: str) -> int:
        if (
            self._is_opportunistic_focus_id(focus_id)
            or self._is_known_opportunistic_focus_id(focus_id)
        ):
            return (
                self.last_camera_pan_us
                if self.last_camera_pan_us is not None
                else self.camera_pan_center_us
            )
        if self._is_search_focus_id(focus_id):
            return self._camera_search_min_pan_us()
        return self._camera_pan_us_for_plant(focus_id)

    def _camera_search_min_pan_us(self) -> int:
        span = max(0, self.plant_search_pan_span_us)
        search_min = max(self.camera_pan_min_us, self.camera_pan_center_us - span)
        search_max = min(self.camera_pan_max_us, self.camera_pan_center_us + span)
        if search_min >= search_max:
            return self.camera_pan_min_us
        return int(search_min)

    def _camera_pan_us_for_plant(self, stop_id: str) -> int:
        plant = self.plant_by_id.get(stop_id)
        if self.latest_pose is None or plant is None:
            return self.camera_pan_center_us

        x, y, yaw = self.latest_pose
        plant_x, plant_y = plant
        desired_bearing = math.atan2(plant_y - y, plant_x - x)
        relative_angle = wrap_to_pi(desired_bearing - yaw)
        limited_angle = self._clamp(
            relative_angle,
            -self.camera_pan_max_angle,
            self.camera_pan_max_angle,
        )
        return self._camera_angle_to_pan_us(limited_angle)

    def _camera_pan_left_angle_from_us(self, pulse_us: int) -> float:
        pulse = self._clamp(
            float(pulse_us),
            float(self.camera_pan_min_us),
            float(self.camera_pan_max_us),
        )
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

    def _plant_centered_detection_can_complete(
        self,
        msg: PlantDetections,
        detection,
    ) -> bool:
        if float(detection.confidence) < self.plant_completion_confidence_threshold:
            return False
        if (
            self.plant_completion_require_valid_capture
            and str(msg.decision_hint).strip().lower() != 'valid_capture'
        ):
            return False
        return True

    def _update_plant_detector_gate(self, now: float) -> None:
        if self.plant_detector_always_enabled:
            self.detector_hold_until = now + self.plant_detector_hold_sec
            self._publish_detector_enabled(True)
            return

        if self.latest_pose is None or not self.plants:
            self._publish_detector_enabled(False)
            return

        x, y, _ = self.latest_pose
        active_dwell_id = (
            self.point_dwell_stop_id
            if self.point_segment_mode == 'dwell' and self.point_dwell_stop_id
            else ''
        )
        pending_plant_ids = {
            plant_id
            for plant_id, _px, _py in self.plants
            if plant_id == active_dwell_id
            or plant_id not in self.completed_inspection_stops
        }
        nearest_plant_distance = min(
            (
                math.hypot(px - x, py - y)
                for name, px, py in self.plants
                if name in pending_plant_ids
            ),
            default=float('inf'),
        )
        nearest_stop_distance = float('inf')
        if self.inspection_stops:
            nearest_stop_distance = min(
                (
                    math.hypot(stop_x - x, stop_y - y)
                    for stop_id, (stop_x, stop_y) in self.inspection_stops.items()
                    if stop_id == active_dwell_id
                    or stop_id not in self.completed_inspection_stops
                ),
                default=float('inf'),
            )
        inspection_dwell_active = (
            self.point_segment_mode == 'dwell' and bool(self.point_dwell_stop_id)
        )
        camera_pan_capture_active = (
            self.camera_pan_enable_detector_gate
            and self.camera_auto_pan_enabled
            and self._camera_pan_target_id() is not None
        )
        plant_bbox_focus_active = (
            self.plant_bbox_servo_enabled
            and self._plant_focus_target_id() is not None
        )

        should_enable = self.detector_enabled
        if (
            inspection_dwell_active
            or camera_pan_capture_active
            or plant_bbox_focus_active
            or nearest_plant_distance <= self.plant_detection_enable_radius
            or nearest_stop_distance <= self.inspection_stop_reached_tolerance_m
        ):
            should_enable = True
            self.detector_hold_until = now + self.plant_detector_hold_sec
        elif (
            nearest_plant_distance >= self.plant_detection_disable_radius
            and nearest_stop_distance > self.inspection_stop_reached_tolerance_m
            and not inspection_dwell_active
            and not camera_pan_capture_active
            and not plant_bbox_focus_active
            and now >= self.detector_hold_until
        ):
            should_enable = False

        self._publish_detector_enabled(should_enable)

    def _publish_detector_enabled(self, enabled: bool) -> None:
        if enabled != self.detector_enabled:
            state_text = 'enabled' if enabled else 'disabled'
            self.get_logger().info('[TRACKING] Plant detector %s.' % state_text)
        self.detector_enabled = bool(enabled)
        msg = Bool()
        msg.data = self.detector_enabled
        self.plant_detector_enable_pub.publish(msg)

    def _publish_positive_save_enabled(
        self,
        enabled: bool,
        force: bool = False,
    ) -> None:
        if self.plant_positive_save_gate_pub is None:
            return
        enabled = bool(enabled)
        if not force and enabled == self.plant_positive_save_enabled:
            return
        if enabled != self.plant_positive_save_enabled:
            state_text = 'enabled' if enabled else 'disabled'
            self.get_logger().info('[TRACKING] Plant positive image save %s.' % state_text)
        self.plant_positive_save_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.plant_positive_save_gate_pub.publish(msg)

    def _plant_detection_is_completion(
        self,
        msg: PlantDetections,
        stop_id: str,
    ) -> bool:
        if self.latest_pose is None:
            return False

        stop_distance = self._inspection_stop_distance(stop_id)
        if (
            stop_distance is None
            or stop_distance > self.plant_completion_max_stop_distance_m
        ):
            return False

        if (
            self.plant_completion_require_valid_capture
            and str(msg.decision_hint).strip().lower() != 'valid_capture'
        ):
            return False

        if int(msg.detection_count) <= 0 or not msg.detections:
            return False

        if self.plant_completion_match_mode == 'any':
            return float(msg.best_confidence) >= self.plant_completion_confidence_threshold

        expected_class = self._expected_plant_class_for_stop(stop_id)
        if not expected_class:
            return False

        for detection in msg.detections:
            if (
                str(detection.class_name).strip().lower() == expected_class
                and float(detection.confidence)
                >= self.plant_completion_confidence_threshold
            ):
                return True
        return False

    def _complete_inspection_stop(self, stop_id: str, reason: str) -> None:
        if not stop_id:
            self.point_dwell_until = 0.0
            self.point_dwell_stop_id = ''
            self._finish_point_segment()
            return

        if stop_id not in self.completed_inspection_stops:
            self.completed_inspection_stops.add(stop_id)
            self.known_opportunistic_focus_ids.discard(stop_id)
            self._center_camera_after_inspection(time.monotonic())
            self.get_logger().info(
                '[TRACKING] Inspection stop %s completed by %s.'
                % (stop_id, reason)
            )

        if self.point_dwell_stop_id == stop_id:
            self.point_dwell_until = 0.0
            self.point_dwell_stop_id = ''

        if self.point_to_point_mode:
            route_stop_index = self._point_route_stop_index(stop_id)
            if route_stop_index is not None:
                self.point_segment_index = max(
                    self.point_segment_index,
                    route_stop_index,
                )
            elif self._path_final_stop_id(self.path) == stop_id:
                self.point_segment_index = max(0, len(self.point_route) - 1)
            self.point_segment_initialized = False
            self.point_segment_mode = 'rotate'
            self.goal_reached = self.point_segment_index >= len(self.point_route) - 1
        elif self._path_final_stop_id(self.path) == stop_id:
            self.goal_reached = True

        self._publish_stop()
        self._publish_mission_progress()

        if self.pending_path_update is not None:
            self._apply_pending_path_update()

    def _active_inspection_stop_id(self) -> str:
        if self.point_segment_mode == 'dwell' and self.point_dwell_stop_id:
            return self.point_dwell_stop_id

        if (
            self.point_to_point_mode
            and self.point_segment_initialized
            and self.point_segment_index < len(self.point_route) - 1
        ):
            stop_id = self._inspection_stop_id(
                self.point_segment_route_target_x,
                self.point_segment_route_target_y,
                self.inspection_stop_reached_tolerance_m,
                yaw=self.point_segment_route_target_yaw,
                require_heading=True,
            )
            if stop_id and stop_id not in self.completed_inspection_stops:
                return stop_id

        stop_id = self._path_final_stop_id(self.path)
        if stop_id and stop_id not in self.completed_inspection_stops:
            return stop_id
        return ''

    def _point_route_stop_index(self, stop_id: str) -> int | None:
        for index, (x, y, yaw) in enumerate(self.point_route):
            route_stop_id = self._inspection_stop_id(
                x,
                y,
                self.inspection_stop_reached_tolerance_m,
                yaw=yaw,
                require_heading=True,
            )
            if route_stop_id == stop_id:
                return index
        return None

    def _inspection_stop_distance(self, stop_id: str) -> float | None:
        if self.latest_pose is None:
            return None
        x, y, _ = self.latest_pose
        distances: list[float] = []

        stop = self.inspection_stops.get(stop_id)
        if stop is not None:
            stop_x, stop_y = stop
            distances.append(math.hypot(stop_x - x, stop_y - y))

        plant = self.plant_by_id.get(stop_id)
        if plant is not None and self.inspection_standoff_distance_m > 0.0:
            plant_x, plant_y = plant
            plant_distance = math.hypot(plant_x - x, plant_y - y)
            distances.append(abs(plant_distance - self.inspection_standoff_distance_m))

        if not distances:
            return None
        return min(distances)

    def _expected_plant_class_for_stop(self, stop_id: str) -> str:
        if len(stop_id) < 2 or stop_id[0].upper() != 'P':
            return ''
        try:
            plant_number = int(stop_id[1:])
        except ValueError:
            return ''
        return 'plant_%02d' % plant_number

    def _publish_mission_progress(self) -> None:
        msg = MissionProgress()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.current_stop_id = self._current_mission_stop_id()
        msg.completed_stop_ids = sorted(self.completed_inspection_stops)
        msg.dwell_active = self._inspection_dwell_active()
        msg.mission_complete = (
            self.goal_reached
            and bool(self.inspection_stops)
            and len(self.completed_inspection_stops) >= len(self.inspection_stops)
        )
        self.mission_progress_pub.publish(msg)

    def _current_mission_stop_id(self) -> str:
        if self._inspection_dwell_active():
            return self.point_dwell_stop_id

        if (
            self.point_to_point_mode
            and self.point_segment_initialized
            and self.point_segment_index < len(self.point_route) - 1
        ):
            stop_id = self._inspection_stop_id(
                self.point_segment_route_target_x,
                self.point_segment_route_target_y,
                self.inspection_stop_reached_tolerance_m,
                yaw=self.point_segment_route_target_yaw,
                require_heading=True,
            )
            if stop_id:
                return stop_id

        stop_id = self._path_final_stop_id(self.path)
        if stop_id and stop_id not in self.completed_inspection_stops:
            return stop_id

        return ''

    def _path_final_stop_id(self, path: list[tuple[float, float, float]]) -> str:
        if not path:
            return ''
        x, y, yaw = path[-1]
        return self._inspection_stop_id(
            x,
            y,
            self.inspection_stop_reached_tolerance_m,
            yaw=yaw,
            require_heading=True,
        ) or ''

    def _update_camera_pan(self, now: float) -> None:
        if (
            not self.camera_auto_pan_enabled
            or self.camera_pan_pub is None
            or self.latest_pose is None
        ):
            return

        target_id = self._camera_pan_target_id()
        if not target_id:
            if self.camera_pan_return_to_center:
                if (
                    self.last_camera_pan_target_id == 'center'
                    and self.last_camera_pan_us is not None
                    and abs(self.camera_pan_center_us - self.last_camera_pan_us) < 8
                    and (
                        self.camera_pan_idle_center_period_sec <= 0.0
                        or now - self.last_camera_pan_publish_time
                        < self.camera_pan_idle_center_period_sec
                    )
                ):
                    return
                self._publish_camera_pan(
                    self.camera_pan_center_us,
                    now,
                    target_id='center',
                    relative_angle=None,
                    saturated=False,
                )
            return

        plant = self.plant_by_id.get(target_id)
        if plant is None:
            return

        x, y, yaw = self.latest_pose
        plant_x, plant_y = plant
        desired_bearing = math.atan2(plant_y - y, plant_x - x)
        relative_angle = wrap_to_pi(desired_bearing - yaw)
        limited_angle = self._clamp(
            relative_angle,
            -self.camera_pan_max_angle,
            self.camera_pan_max_angle,
        )
        saturated = abs(wrap_to_pi(relative_angle - limited_angle)) > math.radians(1.0)
        pulse_us = self._camera_angle_to_pan_us(limited_angle)
        self._publish_camera_pan(
            pulse_us,
            now,
            target_id=target_id,
            relative_angle=relative_angle,
            saturated=saturated,
        )

    def _camera_pan_target_id(self) -> str | None:
        if self.point_segment_mode == 'dwell' and self.point_dwell_stop_id:
            return self.point_dwell_stop_id

        if self.latest_pose is None or not self.inspection_stops:
            return None

        x, y, _ = self.latest_pose
        nearest_stop_id: str | None = None
        nearest_distance = float('inf')
        for stop_id, (stop_x, stop_y) in self.inspection_stops.items():
            if stop_id in self.completed_inspection_stops:
                continue
            if stop_id not in self.plant_by_id:
                continue
            distance = math.hypot(stop_x - x, stop_y - y)
            if distance < nearest_distance:
                nearest_stop_id = stop_id
                nearest_distance = distance

        if (
            nearest_stop_id
            and nearest_distance <= self.camera_pan_active_radius_m
            and nearest_stop_id in self.plant_by_id
        ):
            return nearest_stop_id

        return None

    def _center_camera_after_inspection(self, now: float) -> None:
        if (
            not (self.camera_auto_pan_enabled or self.plant_bbox_servo_enabled)
            or self.camera_pan_pub is None
            or not self.camera_pan_return_to_center
        ):
            return
        self._publish_camera_pan(
            self.camera_pan_center_us,
            now,
            target_id='center',
            relative_angle=None,
            saturated=False,
            force=True,
        )
        if self.plant_bbox_tilt_servo_enabled:
            self._publish_camera_tilt(
                self.camera_tilt_center_us,
                now,
                target_id='center',
                force=True,
            )

    def _camera_angle_to_pan_us(self, angle_left_positive: float) -> int:
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

    def _publish_camera_pan(
        self,
        pulse_us: int,
        now: float,
        target_id: str,
        relative_angle: float | None,
        saturated: bool,
        force: bool = False,
    ) -> None:
        force_target_log = target_id != self.last_camera_pan_target_id
        if (
            not force
            and self.last_camera_pan_us is not None
            and abs(pulse_us - self.last_camera_pan_us) < 8
            and now - self.last_camera_pan_publish_time < 1.0
            and not force_target_log
        ):
            return

        if (
            not force
            and now - self.last_camera_pan_publish_time
            < self.camera_pan_update_period_sec
            and not force_target_log
        ):
            return

        msg = ServoPulseWidth()
        msg.channel = int(self.camera_pan_channel)
        msg.pulse_width_in_microseconds = int(pulse_us)
        self.camera_pan_pub.publish(msg)
        self.last_camera_pan_us = int(pulse_us)
        self.last_camera_pan_publish_time = now

        if (
            force_target_log
            or (saturated and now - self.last_camera_pan_log_time >= 1.0)
            or (self.verbose and now - self.last_camera_pan_log_time >= 1.0)
        ):
            self.last_camera_pan_log_time = now
            self.last_camera_pan_target_id = target_id
            if relative_angle is None:
                if target_id == 'center':
                    self.get_logger().info(
                        '[TRACKING] Camera pan center -> %dus.' % pulse_us
                    )
                else:
                    self.get_logger().info(
                        '[TRACKING] Camera pan target=%s -> %dus.'
                        % (target_id, pulse_us)
                    )
            else:
                suffix = ' (clamped)' if saturated else ''
                self.get_logger().info(
                    '[TRACKING] Camera pan target=%s rel=%.1fdeg -> %dus%s'
                    % (target_id, math.degrees(relative_angle), pulse_us, suffix)
                )

    def _publish_camera_tilt(
        self,
        pulse_us: int,
        now: float,
        target_id: str,
        force: bool = False,
    ) -> None:
        if self.camera_pan_pub is None:
            return

        pulse_us = int(
            round(
                self._clamp(
                    float(pulse_us),
                    float(self.camera_tilt_min_us),
                    float(self.camera_tilt_max_us),
                )
            )
        )
        if (
            not force
            and self.last_camera_tilt_us is not None
            and abs(pulse_us - self.last_camera_tilt_us) < 8
            and now - self.last_camera_tilt_publish_time < 1.0
        ):
            return

        if (
            not force
            and now - self.last_camera_tilt_publish_time
            < self.camera_pan_update_period_sec
        ):
            return

        msg = ServoPulseWidth()
        msg.channel = int(self.camera_tilt_channel)
        msg.pulse_width_in_microseconds = int(pulse_us)
        self.camera_pan_pub.publish(msg)
        self.last_camera_tilt_us = int(pulse_us)
        self.last_camera_tilt_publish_time = now

        if force or self.verbose or now - self.last_camera_tilt_log_time >= 1.0:
            self.last_camera_tilt_log_time = now
            self.get_logger().info(
                '[TRACKING] Camera tilt target=%s -> %dus.'
                % (target_id, pulse_us)
            )

    def _publish_motor_command(self, left: float, right: float) -> None:
        msg = LeftRightFloat32()
        msg.left = float(left)
        msg.right = float(right)
        msg.seq_num = self.command_seq_num
        self.motor_pub.publish(msg)
        self.command_seq_num += 1
        self.last_left_cmd = float(left)
        self.last_right_cmd = float(right)

    def _publish_stop(self) -> None:
        self.segment_profile_forward_duty = 0.0
        if not self.graceful_stop_enabled:
            self._publish_motor_command(0.0, 0.0)
            return

        left_duty, right_duty = self._graceful_stop_command()
        self._publish_motor_command(left_duty, right_duty)

    def _graceful_stop_command(self) -> tuple[float, float]:
        if (
            max(abs(self.last_left_cmd), abs(self.last_right_cmd))
            <= self.graceful_stop_zero_threshold_duty
        ):
            return 0.0, 0.0

        forward_duty = 0.5 * (self.last_left_cmd + self.last_right_cmd)
        turn_duty = 0.5 * (self.last_right_cmd - self.last_left_cmd)

        forward_duty = self._ramp_to_zero(
            forward_duty,
            self.graceful_stop_decel_duty_per_sec,
        )
        turn_duty = self._ramp_to_zero(
            turn_duty,
            self.graceful_stop_turn_duty_per_sec,
        )

        left_duty = forward_duty - turn_duty
        right_duty = forward_duty + turn_duty
        if (
            max(abs(left_duty), abs(right_duty))
            <= self.graceful_stop_zero_threshold_duty
        ):
            return 0.0, 0.0
        return left_duty, right_duty

    def _corner_turn_needs_forward_settle(self) -> bool:
        if self.turn_forward_stop_threshold_duty <= 0.0:
            return False

        forward_duty = 0.5 * (self.last_left_cmd + self.last_right_cmd)
        same_direction = self.last_left_cmd * self.last_right_cmd > 0.0
        return same_direction and abs(forward_duty) > self.turn_forward_stop_threshold_duty

    def _publish_turn_entry_stop(self) -> None:
        self.segment_profile_forward_duty = 0.0
        if not self.graceful_stop_enabled:
            self._publish_motor_command(0.0, 0.0)
            return

        forward_duty = 0.5 * (self.last_left_cmd + self.last_right_cmd)
        turn_duty = 0.5 * (self.last_right_cmd - self.last_left_cmd)
        fast_stop_rate = max(
            self.graceful_stop_decel_duty_per_sec,
            self.graceful_stop_turn_duty_per_sec,
        )
        forward_duty = self._ramp_to_zero(forward_duty, fast_stop_rate)
        turn_duty = self._ramp_to_zero(turn_duty, self.graceful_stop_turn_duty_per_sec)

        left_duty = self._clamp(
            forward_duty - turn_duty,
            -self.duty_cycle_limit,
            self.duty_cycle_limit,
        )
        right_duty = self._clamp(
            forward_duty + turn_duty,
            -self.duty_cycle_limit,
            self.duty_cycle_limit,
        )
        if max(abs(left_duty), abs(right_duty)) <= self.graceful_stop_zero_threshold_duty:
            left_duty = 0.0
            right_duty = 0.0
        self._publish_motor_command(left_duty, right_duty)

    def _ramp_to_zero(self, value: float, rate_per_sec: float) -> float:
        if rate_per_sec <= 0.0 or value == 0.0:
            return 0.0
        max_delta = rate_per_sec * self.control_period
        if abs(value) <= max_delta:
            return 0.0
        return value - math.copysign(max_delta, value)

    def _apply_slew_limit(self, target_left: float, target_right: float) -> tuple[float, float]:
        if self.duty_slew_rate <= 0.0:
            return target_left, target_right

        max_delta = self.duty_slew_rate * self.control_period
        left = self.last_left_cmd + self._clamp(
            target_left - self.last_left_cmd,
            -max_delta,
            max_delta,
        )
        right = self.last_right_cmd + self._clamp(
            target_right - self.last_right_cmd,
            -max_delta,
            max_delta,
        )
        return left, right

    def _apply_command_filter(
        self,
        target_left: float,
        target_right: float,
    ) -> tuple[float, float]:
        if self.command_filter_alpha <= 0.0:
            return target_left, target_right

        alpha = self.command_filter_alpha
        left = alpha * self.last_left_cmd + (1.0 - alpha) * target_left
        right = alpha * self.last_right_cmd + (1.0 - alpha) * target_right
        return left, right

    def _same_path(
        self,
        new_path: list[tuple[float, float, float]],
        old_path: list[tuple[float, float, float]],
    ) -> bool:
        if len(new_path) != len(old_path):
            return False
        if not new_path:
            return False

        for new_pose, old_pose in zip(new_path, old_path):
            if (
                abs(new_pose[0] - old_pose[0]) > 1.0e-4
                or abs(new_pose[1] - old_pose[1]) > 1.0e-4
                or abs(wrap_to_pi(new_pose[2] - old_pose[2])) > 1.0e-4
            ):
                return False
        return True

    def _stable_rotate_error(self, alpha: float) -> float:
        if abs(abs(alpha) - math.pi) <= 0.35:
            if self.rotate_in_place_direction == 0.0:
                self.rotate_in_place_direction = self.backward_target_turn_direction
            return self.rotate_in_place_direction * abs(alpha)

        self.rotate_in_place_direction = math.copysign(1.0, alpha)
        return alpha

    def _wheel_speed_to_duty(self, wheel_speed: float) -> float:
        if abs(wheel_speed) < 1e-4:
            return 0.0

        ratio = min(abs(wheel_speed) / self.max_wheel_speed_at_full_duty, 1.0)
        duty_span = max(0.0, self.duty_cycle_limit - self.min_moving_duty)
        duty = self.min_moving_duty + duty_span * ratio
        return math.copysign(duty, wheel_speed)

    def _odom_is_stale(self, now: float) -> bool:
        return (
            self.odom_timeout_sec > 0.0
            and self.last_odom_time is not None
            and now - self.last_odom_time > self.odom_timeout_sec
        )

    def _path_is_stale(self, now: float) -> bool:
        return (
            self.path_timeout_sec > 0.0
            and self.last_path_time is not None
            and now - self.last_path_time > self.path_timeout_sec
        )

    def _target_pose_is_fresh(self, now: float) -> bool:
        return (
            self.latest_target_pose is not None
            and self.last_target_pose_time is not None
            and (self.path_timeout_sec <= 0.0 or now - self.last_target_pose_time <= self.path_timeout_sec)
        )

    def _log_status(self, now: float, message: str) -> None:
        if now - self.last_status_log >= 2.0:
            self.get_logger().info(message)
            self.last_status_log = now

    def _extract_planar_covariance(self, pose_covariance) -> list[float]:
        covariance = [0.0] * 9
        covariance[0] = float(pose_covariance[0])
        covariance[1] = float(pose_covariance[1])
        covariance[2] = float(pose_covariance[5])
        covariance[3] = float(pose_covariance[6])
        covariance[4] = float(pose_covariance[7])
        covariance[5] = float(pose_covariance[11])
        covariance[6] = float(pose_covariance[30])
        covariance[7] = float(pose_covariance[31])
        covariance[8] = float(pose_covariance[35])
        return covariance

    def _inspection_stop_id(
        self,
        x: float,
        y: float,
        tolerance: float,
        yaw: float | None = None,
        require_heading: bool = False,
    ) -> str | None:
        if tolerance <= 0.0:
            return None

        standoff_stop_id = self._inspection_standoff_stop_id(
            x,
            y,
            tolerance,
            yaw=yaw,
            require_heading=require_heading,
        )
        if standoff_stop_id:
            return standoff_stop_id

        closest_stop_id: str | None = None
        closest_distance = float('inf')
        for stop_id, (stop_x, stop_y) in self.inspection_stops.items():
            distance = math.hypot(x - stop_x, y - stop_y)
            if distance > tolerance:
                continue
            if require_heading and yaw is not None:
                stop_yaw = self._inspection_stop_yaw(stop_id)
                if (
                    stop_yaw is not None
                    and abs(wrap_to_pi(yaw - stop_yaw))
                    > self.inspection_stop_heading_tolerance
                ):
                    continue
            if distance < closest_distance:
                closest_distance = distance
                closest_stop_id = stop_id
        return closest_stop_id

    def _inspection_standoff_stop_id(
        self,
        x: float,
        y: float,
        tolerance: float,
        yaw: float | None = None,
        require_heading: bool = False,
    ) -> str | None:
        if self.inspection_standoff_distance_m <= 0.0:
            return None

        match_tolerance = max(tolerance, self.inspection_standoff_tolerance_m)
        closest_stop_id: str | None = None
        closest_standoff_error = float('inf')
        for stop_id, (plant_x, plant_y) in self.plant_by_id.items():
            plant_distance = math.hypot(plant_x - x, plant_y - y)
            standoff_error = abs(plant_distance - self.inspection_standoff_distance_m)
            if standoff_error > match_tolerance:
                continue

            if require_heading and yaw is not None:
                if plant_distance <= 1.0e-6:
                    continue
                stop_yaw = math.atan2(plant_y - y, plant_x - x)
                if (
                    abs(wrap_to_pi(yaw - stop_yaw))
                    > self.inspection_stop_heading_tolerance
                ):
                    continue

            if standoff_error < closest_standoff_error:
                closest_standoff_error = standoff_error
                closest_stop_id = stop_id
        return closest_stop_id

    def _inspection_stop_yaw(
        self,
        stop_id: str,
        stop_x: float | None = None,
        stop_y: float | None = None,
    ) -> float | None:
        plant = self.plant_by_id.get(stop_id)
        if plant is None:
            return None
        if stop_x is None or stop_y is None:
            stop = self.inspection_stops.get(stop_id)
            if stop is None:
                return None
            stop_x, stop_y = stop
        plant_x, plant_y = plant
        return math.atan2(plant_y - stop_y, plant_x - stop_x)

    def _inspection_capture_yaw(self, stop_id: str) -> float | None:
        plant = self.plant_by_id.get(stop_id)
        if plant is not None:
            if self.point_segment_initialized:
                return self._inspection_stop_yaw(
                    stop_id,
                    self.point_segment_route_target_x,
                    self.point_segment_route_target_y,
                )
            if self.path:
                path_x, path_y, _path_yaw = self.path[-1]
                if (
                    self._inspection_standoff_stop_id(
                        path_x,
                        path_y,
                        self.inspection_standoff_tolerance_m,
                    )
                    == stop_id
                ):
                    return self._inspection_stop_yaw(stop_id, path_x, path_y)
            if self.latest_pose is not None and self.point_dwell_stop_id == stop_id:
                robot_x, robot_y, _robot_yaw = self.latest_pose
                return self._inspection_stop_yaw(stop_id, robot_x, robot_y)

        stop_yaw = self._inspection_stop_yaw(stop_id)
        if stop_yaw is not None:
            return stop_yaw
        if self.point_segment_initialized:
            return self.point_segment_route_target_yaw
        return None

    def _inspection_dwell_duration(self, stop_id: str | None) -> float:
        if not stop_id:
            return 0.0
        return max(
            0.0,
            self.inspection_dwell_by_id.get(stop_id, self.inspection_dwell_sec),
        )

    def _parse_named_xy_map(
        self,
        text: str,
        param_name: str,
    ) -> dict[str, tuple[float, float]]:
        points: dict[str, tuple[float, float]] = {}
        for item in text.split(';'):
            item = item.strip()
            if not item:
                continue
            if ':' not in item:
                self.get_logger().warn('[TRACKING] Bad %s entry: %s' % (param_name, item))
                continue
            name, values = item.split(':', 1)
            parts = [part.strip() for part in values.split(',')]
            if len(parts) < 2:
                self.get_logger().warn('[TRACKING] Bad %s entry: %s' % (param_name, item))
                continue
            try:
                points[name.strip()] = (float(parts[0]), float(parts[1]))
            except ValueError:
                self.get_logger().warn('[TRACKING] Bad %s entry: %s' % (param_name, item))
        return points

    def _parse_named_float_map(
        self,
        text: str,
        param_name: str,
    ) -> dict[str, float]:
        values: dict[str, float] = {}
        for item in text.split(';'):
            item = item.strip()
            if not item:
                continue
            if ':' not in item:
                self.get_logger().warn('[TRACKING] Bad %s entry: %s' % (param_name, item))
                continue
            name, value_text = item.split(':', 1)
            try:
                values[name.strip()] = max(0.0, float(value_text.strip()))
            except ValueError:
                self.get_logger().warn('[TRACKING] Bad %s entry: %s' % (param_name, item))
        return values

    def _parse_plant_world_map(self, text: str) -> list[tuple[str, float, float]]:
        plants: list[tuple[str, float, float]] = []
        for item in text.split(';'):
            item = item.strip()
            if not item or ':' not in item:
                continue
            name, values = item.split(':', 1)
            parts = [part.strip() for part in values.split(',')]
            if len(parts) < 2:
                continue
            try:
                plants.append((name.strip(), float(parts[0]), float(parts[1])))
            except ValueError:
                self.get_logger().warn('[TRACKING] Bad plant coordinate entry: %s' % item)
        return plants

    def _yaw_from_neighbours(self, poses: list[PoseStamped], index: int) -> float:
        current = poses[index].pose.position
        if index + 1 < len(poses):
            nxt = poses[index + 1].pose.position
            return math.atan2(float(nxt.y - current.y), float(nxt.x - current.x))
        if index > 0:
            prev = poses[index - 1].pose.position
            return math.atan2(float(current.y - prev.y), float(current.x - prev.x))
        return 0.0

    def _quaternion_is_empty(self, q) -> bool:
        return abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) < 1e-9

    def _float_list_parameter(
        self,
        name: str,
        default: list[float],
        expected_length: int,
    ) -> list[float]:
        raw_value = self.get_parameter(name).value
        try:
            if isinstance(raw_value, str):
                values = [
                    float(item.strip())
                    for item in raw_value.split(',')
                    if item.strip()
                ]
            elif isinstance(raw_value, Iterable):
                values = [float(item) for item in raw_value]
            else:
                values = list(default)
        except (TypeError, ValueError):
            values = list(default)

        if len(values) != expected_length:
            self.get_logger().warn(
                '[TRACKING] Parameter %s expected %d values, got %d; using default %s.'
                % (name, expected_length, len(values), default)
            )
            return list(default)
        return values

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _smoothstep(self, value: float) -> float:
        value = self._clamp(value, 0.0, 1.0)
        return value * value * (3.0 - 2.0 * value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
