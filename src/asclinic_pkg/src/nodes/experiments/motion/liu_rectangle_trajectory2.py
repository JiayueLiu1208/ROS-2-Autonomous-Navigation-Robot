#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from asclinic_pkg.msg import FiducialMarkerArray
from asclinic_pkg.msg import LeftRightFloat32
from nav_msgs.msg import Odometry


class LiuRectangleTrajectory(Node):
    def __init__(self):
        super().__init__('liu_rectangle_trajectory')

        # =========================================================
        # Parameters
        # =========================================================
        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('command_publish_period', 0.05)
        self.declare_parameter('verbose', True)

        # Rectangle size
        self.declare_parameter('length_long_m', 6.0)
        self.declare_parameter('length_short_m', 3.0)

        # Straight segment parameters
        self.declare_parameter('straight_duty_cycle_percent', 20.0)
        self.declare_parameter('straight_direction', 1)  # +1 forward, -1 backward
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 0.96)

        # Straight heading P control
        self.declare_parameter('yaw_kp', 8.0)
        self.declare_parameter('max_correction', 4.0)
        self.declare_parameter('yaw_deadband', 0.02)

        # Smooth straight-line speed control. The ramp starts at the minimum
        # duty so the robot can break static friction, then eases up/down.
        self.declare_parameter('min_straight_duty_cycle_percent', 15.0)
        self.declare_parameter('ramp_up_distance_m', 0.60)
        self.declare_parameter('deceleration_zone_m', 0.90)
        self.declare_parameter('heading_speed_scale_k', 1.4)
        self.declare_parameter('duty_slew_rate_percent_per_sec', 45.0)
        self.declare_parameter('command_filter_alpha', 0.65)

        # Optional temporary ArUco heading assist. Encoder odometry still owns
        # distance and turn completion; ArUco only nudges straight-line heading.
        self.declare_parameter('use_aruco_heading_assist', False)
        self.declare_parameter('aruco_topic', 'aruco_detections')
        self.declare_parameter('aruco_heading_marker_ids', '4,7,11,14')
        self.declare_parameter('aruco_heading_marker_groups', '')
        self.declare_parameter('aruco_heading_kp', 8.0)
        self.declare_parameter('aruco_heading_max_correction', 1.2)
        self.declare_parameter('aruco_heading_deadband_deg', 2.0)
        self.declare_parameter('aruco_heading_stale_timeout_sec', 0.6)
        self.declare_parameter('aruco_heading_max_bearing_deg', 32.0)
        self.declare_parameter('aruco_heading_filter_alpha', 0.55)
        self.declare_parameter('aruco_heading_error_sign', 1.0)
        self.declare_parameter('use_aruco_turn_distance_trigger', False)
        self.declare_parameter('aruco_turn_marker_ids', '')
        self.declare_parameter('aruco_turn_distance_m', 0.80)
        self.declare_parameter('aruco_turn_min_encoder_fraction', 0.55)
        self.declare_parameter('aruco_turn_max_bearing_deg', 15.0)

        # Startup yaw averaging
        self.declare_parameter('startup_yaw_sample_count', 10)

        # Optional startup self-localisation. When enabled, the robot first
        # waits for a stable ArUco-derived global pose, drives to the requested
        # start pose, then releases the existing rectangle controller unchanged.
        self.declare_parameter('use_startup_self_localization', False)
        self.declare_parameter('startup_global_pose_topic', 'pitt_vision_odometry')
        self.declare_parameter('startup_target_x_m', 0.0)
        self.declare_parameter('startup_target_y_m', 0.0)
        self.declare_parameter('startup_target_yaw_deg', 0.0)
        self.declare_parameter('startup_pose_filter_alpha', 0.30)
        self.declare_parameter('startup_pose_stable_sample_count', 8)
        self.declare_parameter('startup_pose_stale_timeout_sec', 0.75)
        self.declare_parameter('startup_position_tolerance_m', 0.05)
        self.declare_parameter('startup_heading_tolerance_deg', 2.0)
        self.declare_parameter('startup_completion_position_tolerance_m', 0.02)
        self.declare_parameter('startup_completion_heading_tolerance_deg', 1.0)
        self.declare_parameter('startup_settle_sample_count', 6)
        self.declare_parameter('startup_near_target_distance_m', 0.25)
        self.declare_parameter('startup_final_heading_slowdown_deg', 12.0)
        self.declare_parameter('startup_rotate_to_target_tolerance_deg', 8.0)
        self.declare_parameter('startup_drive_heading_tolerance_deg', 6.0)
        self.declare_parameter('startup_realign_heading_tolerance_deg', 12.0)
        self.declare_parameter('startup_max_duty_cycle_percent', 16.0)
        self.declare_parameter('startup_min_duty_cycle_percent', 5.0)
        self.declare_parameter('startup_near_target_min_duty_cycle_percent', 2.5)
        self.declare_parameter('startup_linear_kp', 14.0)
        self.declare_parameter('startup_heading_kp', 10.0)
        self.declare_parameter('startup_final_heading_kp', 10.0)

        # Turn segment parameters
        self.declare_parameter('turn_angle_deg', 90.0)
        self.declare_parameter('turn_direction', 1)  # +1 one way, -1 the other way
        self.declare_parameter('turn_kp', 12.0)
        self.declare_parameter('turn_max_cmd', 8.0)
        self.declare_parameter('turn_min_cmd', 4.0)
        self.declare_parameter('turn_angle_tolerance_deg', 2.0)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.command_publish_period = float(self.get_parameter('command_publish_period').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.length_long_m = float(self.get_parameter('length_long_m').value)
        self.length_short_m = float(self.get_parameter('length_short_m').value)

        self.straight_duty_cycle_percent = float(self.get_parameter('straight_duty_cycle_percent').value)
        self.straight_direction = int(self.get_parameter('straight_direction').value)
        self.left_trim = float(self.get_parameter('left_trim').value)
        self.right_trim = float(self.get_parameter('right_trim').value)

        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_correction = float(self.get_parameter('max_correction').value)
        self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)

        self.min_straight_duty_cycle_percent = abs(
            float(self.get_parameter('min_straight_duty_cycle_percent').value)
        )
        self.ramp_up_distance_m = max(
            0.0,
            float(self.get_parameter('ramp_up_distance_m').value),
        )
        self.deceleration_zone_m = max(
            0.0,
            float(self.get_parameter('deceleration_zone_m').value),
        )
        self.heading_speed_scale_k = max(
            0.0,
            float(self.get_parameter('heading_speed_scale_k').value),
        )
        self.duty_slew_rate_percent_per_sec = max(
            0.0,
            float(self.get_parameter('duty_slew_rate_percent_per_sec').value),
        )
        self.command_filter_alpha = self.clamp(
            float(self.get_parameter('command_filter_alpha').value),
            0.0,
            1.0,
        )

        self.use_aruco_heading_assist = self.parameter_to_bool(
            self.get_parameter('use_aruco_heading_assist').value
        )
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.aruco_heading_marker_ids = self.parse_marker_id_sequence(
            str(self.get_parameter('aruco_heading_marker_ids').value)
        )
        self.aruco_heading_marker_groups = self.parse_marker_group_sequence(
            str(self.get_parameter('aruco_heading_marker_groups').value)
        )
        if not self.aruco_heading_marker_groups:
            self.aruco_heading_marker_groups = [
                [marker_id] for marker_id in self.aruco_heading_marker_ids
            ]
        self.aruco_heading_kp = float(self.get_parameter('aruco_heading_kp').value)
        self.aruco_heading_max_correction = abs(
            float(self.get_parameter('aruco_heading_max_correction').value)
        )
        self.aruco_heading_deadband = math.radians(
            abs(float(self.get_parameter('aruco_heading_deadband_deg').value))
        )
        self.aruco_heading_stale_timeout_sec = float(
            self.get_parameter('aruco_heading_stale_timeout_sec').value
        )
        self.aruco_heading_max_bearing = math.radians(
            abs(float(self.get_parameter('aruco_heading_max_bearing_deg').value))
        )
        self.aruco_heading_filter_alpha = self.clamp(
            float(self.get_parameter('aruco_heading_filter_alpha').value),
            0.0,
            1.0,
        )
        self.aruco_heading_error_sign = float(
            self.get_parameter('aruco_heading_error_sign').value
        )
        self.use_aruco_turn_distance_trigger = self.parameter_to_bool(
            self.get_parameter('use_aruco_turn_distance_trigger').value
        )
        self.aruco_turn_marker_ids = self.parse_marker_id_sequence(
            str(self.get_parameter('aruco_turn_marker_ids').value)
        )
        if not self.aruco_turn_marker_ids:
            self.aruco_turn_marker_ids = [
                group[0] for group in self.aruco_heading_marker_groups if group
            ]
        self.aruco_turn_distance_m = abs(
            float(self.get_parameter('aruco_turn_distance_m').value)
        )
        self.aruco_turn_min_encoder_fraction = self.clamp(
            float(self.get_parameter('aruco_turn_min_encoder_fraction').value),
            0.0,
            1.0,
        )
        self.aruco_turn_max_bearing = math.radians(
            abs(float(self.get_parameter('aruco_turn_max_bearing_deg').value))
        )
        self.use_aruco_feedback = (
            self.use_aruco_heading_assist or self.use_aruco_turn_distance_trigger
        )
        self.aruco_wanted_ids = set(self.aruco_turn_marker_ids)
        for group in self.aruco_heading_marker_groups:
            self.aruco_wanted_ids.update(group)

        self.startup_yaw_sample_count = int(self.get_parameter('startup_yaw_sample_count').value)
        self.use_startup_self_localization = self.parameter_to_bool(
            self.get_parameter('use_startup_self_localization').value
        )
        self.startup_global_pose_topic = str(
            self.get_parameter('startup_global_pose_topic').value
        )
        self.startup_target_x_m = float(self.get_parameter('startup_target_x_m').value)
        self.startup_target_y_m = float(self.get_parameter('startup_target_y_m').value)
        self.startup_target_yaw = math.radians(
            float(self.get_parameter('startup_target_yaw_deg').value)
        )
        self.startup_pose_filter_alpha = self.clamp(
            float(self.get_parameter('startup_pose_filter_alpha').value),
            0.0,
            1.0,
        )
        self.startup_pose_stable_sample_count = int(
            self.get_parameter('startup_pose_stable_sample_count').value
        )
        self.startup_pose_stale_timeout_sec = float(
            self.get_parameter('startup_pose_stale_timeout_sec').value
        )
        self.startup_position_tolerance_m = abs(
            float(self.get_parameter('startup_position_tolerance_m').value)
        )
        self.startup_heading_tolerance = math.radians(
            abs(float(self.get_parameter('startup_heading_tolerance_deg').value))
        )
        self.startup_completion_position_tolerance_m = abs(
            float(self.get_parameter('startup_completion_position_tolerance_m').value)
        )
        self.startup_completion_heading_tolerance = math.radians(
            abs(float(self.get_parameter('startup_completion_heading_tolerance_deg').value))
        )
        self.startup_settle_sample_count = int(
            self.get_parameter('startup_settle_sample_count').value
        )
        self.startup_near_target_distance_m = max(
            0.0,
            float(self.get_parameter('startup_near_target_distance_m').value)
        )
        self.startup_final_heading_slowdown = math.radians(
            abs(float(self.get_parameter('startup_final_heading_slowdown_deg').value))
        )
        self.startup_rotate_to_target_tolerance = math.radians(
            abs(float(self.get_parameter('startup_rotate_to_target_tolerance_deg').value))
        )
        self.startup_drive_heading_tolerance = math.radians(
            abs(float(self.get_parameter('startup_drive_heading_tolerance_deg').value))
        )
        self.startup_realign_heading_tolerance = math.radians(
            abs(float(self.get_parameter('startup_realign_heading_tolerance_deg').value))
        )
        self.startup_max_duty_cycle_percent = abs(
            float(self.get_parameter('startup_max_duty_cycle_percent').value)
        )
        self.startup_min_duty_cycle_percent = abs(
            float(self.get_parameter('startup_min_duty_cycle_percent').value)
        )
        self.startup_near_target_min_duty_cycle_percent = abs(
            float(self.get_parameter('startup_near_target_min_duty_cycle_percent').value)
        )
        self.startup_linear_kp = float(self.get_parameter('startup_linear_kp').value)
        self.startup_heading_kp = float(self.get_parameter('startup_heading_kp').value)
        self.startup_final_heading_kp = float(
            self.get_parameter('startup_final_heading_kp').value
        )

        self.turn_angle_deg = float(self.get_parameter('turn_angle_deg').value)
        self.turn_direction = int(self.get_parameter('turn_direction').value)
        self.turn_kp = float(self.get_parameter('turn_kp').value)
        self.turn_max_cmd = float(self.get_parameter('turn_max_cmd').value)
        self.turn_min_cmd = float(self.get_parameter('turn_min_cmd').value)
        self.turn_angle_tolerance_deg = float(self.get_parameter('turn_angle_tolerance_deg').value)

        if self.straight_direction not in [1, -1]:
            self.get_logger().warn('straight_direction must be +1 or -1, defaulting to +1')
            self.straight_direction = 1

        if self.turn_direction not in [1, -1]:
            self.get_logger().warn('turn_direction must be +1 or -1, defaulting to +1')
            self.turn_direction = 1

        if self.straight_duty_cycle_percent < 0.0:
            self.straight_duty_cycle_percent = abs(self.straight_duty_cycle_percent)

        self.min_straight_duty_cycle_percent = min(
            self.min_straight_duty_cycle_percent,
            self.straight_duty_cycle_percent,
        )

        if self.turn_max_cmd < 0.0:
            self.turn_max_cmd = abs(self.turn_max_cmd)

        if self.turn_min_cmd < 0.0:
            self.turn_min_cmd = abs(self.turn_min_cmd)

        if self.turn_min_cmd > self.turn_max_cmd:
            self.get_logger().warn('turn_min_cmd > turn_max_cmd, swapping them')
            self.turn_min_cmd, self.turn_max_cmd = self.turn_max_cmd, self.turn_min_cmd

        if self.startup_yaw_sample_count < 1:
            self.startup_yaw_sample_count = 1
        if self.startup_pose_stable_sample_count < 1:
            self.startup_pose_stable_sample_count = 1
        if self.startup_settle_sample_count < 1:
            self.startup_settle_sample_count = 1
        if self.startup_completion_position_tolerance_m > self.startup_position_tolerance_m:
            self.startup_completion_position_tolerance_m = self.startup_position_tolerance_m
        if self.startup_completion_heading_tolerance > self.startup_heading_tolerance:
            self.startup_completion_heading_tolerance = self.startup_heading_tolerance
        self.startup_min_duty_cycle_percent = min(
            self.startup_min_duty_cycle_percent,
            self.startup_max_duty_cycle_percent,
        )
        self.startup_near_target_min_duty_cycle_percent = min(
            self.startup_near_target_min_duty_cycle_percent,
            self.startup_min_duty_cycle_percent,
        )

        self.turn_angle_rad = abs(self.turn_angle_deg) * math.pi / 180.0
        self.turn_angle_tolerance_rad = abs(self.turn_angle_tolerance_deg) * math.pi / 180.0

        # =========================================================
        # State
        # =========================================================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.odom_received = False
        self.finished = False
        self.command_seq_num = 1

        # startup yaw averaging
        self.yaw_samples = []
        self.ready_to_move = False
        self.control_mode = 'startup_localize' if self.use_startup_self_localization else 'run'
        self.startup_filtered_pose = None
        self.startup_pose_sample_count = 0
        self.startup_pose_stamp_sec = None
        self.startup_last_log_sec = None
        self.startup_recent_pose_window = []
        self.startup_in_tolerance_count = 0
        self.startup_nav_phase = 'localize'
        self.startup_target_heading = None

        # straight state reference
        self.segment_start_x = None
        self.segment_start_y = None
        self.segment_start_yaw = None

        # turn state reference
        self.prev_yaw = None
        self.accumulated_yaw = 0.0

        # plan
        self.plan = [
            ('straight', self.length_long_m),
            ('turn', self.turn_angle_rad),
            ('straight', self.length_short_m),
            ('turn', self.turn_angle_rad),
            ('straight', self.length_long_m),
            ('turn', self.turn_angle_rad),
            ('straight', self.length_short_m),
            ('turn', self.turn_angle_rad),
        ]
        self.state_index = 0
        self.state_initialized = False
        self.aruco_heading_observations = {}
        self.last_control_left_cmd = 0.0
        self.last_control_right_cmd = 0.0
        # =========================================================
        # ROS interfaces
        # =========================================================
        self.cmd_pub = self.create_publisher(
            LeftRightFloat32,
            self.cmd_topic,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.startup_global_pose_sub = None
        if self.use_startup_self_localization:
            self.startup_global_pose_sub = self.create_subscription(
                Odometry,
                self.startup_global_pose_topic,
                self.startup_global_pose_callback,
                10
            )

        self.aruco_sub = None
        if self.use_aruco_feedback:
            self.aruco_sub = self.create_subscription(
                FiducialMarkerArray,
                self.aruco_topic,
                self.aruco_callback,
                10
            )

        self.cmd_timer = self.create_timer(
            self.command_publish_period,
            self.command_timer_callback
        )

        self.get_logger().info('==================================================')
        self.get_logger().info('[RECT] Node started')
        self.get_logger().info(f'[RECT] odom_topic                 = {self.odom_topic}')
        self.get_logger().info(f'[RECT] cmd_topic                  = {self.cmd_topic}')
        self.get_logger().info(f'[RECT] length_long_m              = {self.length_long_m:.3f}')
        self.get_logger().info(f'[RECT] length_short_m             = {self.length_short_m:.3f}')
        self.get_logger().info(f'[RECT] straight_duty             = {self.straight_duty_cycle_percent:.3f}')
        self.get_logger().info(f'[RECT] straight_direction        = {self.straight_direction}')
        self.get_logger().info(f'[RECT] left_trim                 = {self.left_trim:.3f}')
        self.get_logger().info(f'[RECT] right_trim                = {self.right_trim:.3f}')
        self.get_logger().info(f'[RECT] yaw_kp                    = {self.yaw_kp:.3f}')
        self.get_logger().info(f'[RECT] max_correction            = {self.max_correction:.3f}')
        self.get_logger().info(f'[RECT] yaw_deadband              = {self.yaw_deadband:.4f}')
        self.get_logger().info(f'[RECT] min_straight_duty         = {self.min_straight_duty_cycle_percent:.3f}')
        self.get_logger().info(f'[RECT] ramp_up_distance_m        = {self.ramp_up_distance_m:.3f}')
        self.get_logger().info(f'[RECT] deceleration_zone_m       = {self.deceleration_zone_m:.3f}')
        self.get_logger().info(f'[RECT] heading_speed_scale_k     = {self.heading_speed_scale_k:.3f}')
        self.get_logger().info(f'[RECT] duty_slew_rate_percent_s  = {self.duty_slew_rate_percent_per_sec:.3f}')
        self.get_logger().info(f'[RECT] command_filter_alpha      = {self.command_filter_alpha:.3f}')
        self.get_logger().info(f'[RECT] aruco_heading_assist      = {self.use_aruco_heading_assist}')
        if self.use_aruco_heading_assist:
            self.get_logger().info(f'[RECT] aruco_topic               = {self.aruco_topic}')
            self.get_logger().info(f'[RECT] aruco_heading_markers     = {self.aruco_heading_marker_ids}')
            self.get_logger().info(f'[RECT] aruco_heading_groups      = {self.aruco_heading_marker_groups}')
            self.get_logger().info(f'[RECT] aruco_heading_kp          = {self.aruco_heading_kp:.3f}')
            self.get_logger().info(f'[RECT] aruco_heading_max_corr    = {self.aruco_heading_max_correction:.3f}')
            self.get_logger().info(f'[RECT] aruco_heading_deadband    = {math.degrees(self.aruco_heading_deadband):.2f} deg')
            self.get_logger().info(f'[RECT] aruco_heading_error_sign  = {self.aruco_heading_error_sign:.1f}')
            if not self.aruco_wanted_ids:
                self.get_logger().warn('[RECT][ARUCO] heading assist enabled but no marker ids configured')
        self.get_logger().info(f'[RECT] aruco_turn_trigger       = {self.use_aruco_turn_distance_trigger}')
        if self.use_aruco_turn_distance_trigger:
            self.get_logger().info(f'[RECT] aruco_turn_markers       = {self.aruco_turn_marker_ids}')
            self.get_logger().info(f'[RECT] aruco_turn_distance_m    = {self.aruco_turn_distance_m:.3f}')
            self.get_logger().info(f'[RECT] aruco_turn_min_fraction  = {self.aruco_turn_min_encoder_fraction:.3f}')
            self.get_logger().info(f'[RECT] aruco_turn_max_bearing   = {math.degrees(self.aruco_turn_max_bearing):.2f} deg')
        self.get_logger().info(f'[RECT] startup_yaw_samples       = {self.startup_yaw_sample_count}')
        self.get_logger().info(f'[RECT] startup_self_localization = {self.use_startup_self_localization}')
        if self.use_startup_self_localization:
            self.get_logger().info(f'[RECT] startup_global_pose_topic= {self.startup_global_pose_topic}')
            self.get_logger().info(
                f'[RECT] startup_target_pose      = '
                f'({self.startup_target_x_m:.3f}, {self.startup_target_y_m:.3f}, '
                f'{math.degrees(self.startup_target_yaw):.2f} deg)'
            )
            self.get_logger().info(f'[RECT] startup_pose_samples     = {self.startup_pose_stable_sample_count}')
            self.get_logger().info(f'[RECT] startup_pos_tolerance    = {self.startup_position_tolerance_m:.3f} m')
            self.get_logger().info(f'[RECT] startup_yaw_tolerance    = {math.degrees(self.startup_heading_tolerance):.2f} deg')
            self.get_logger().info(
                f'[RECT] startup_finish_tolerance = '
                f'{self.startup_completion_position_tolerance_m:.3f} m, '
                f'{math.degrees(self.startup_completion_heading_tolerance):.2f} deg'
            )
            self.get_logger().info(
                f'[RECT] startup_drive_heading  = '
                f'{math.degrees(self.startup_drive_heading_tolerance):.2f} deg, '
                f'realign={math.degrees(self.startup_realign_heading_tolerance):.2f} deg'
            )
            self.get_logger().info(f'[RECT] startup_settle_samples   = {self.startup_settle_sample_count}')
            self.get_logger().info(f'[RECT] startup_max_duty         = {self.startup_max_duty_cycle_percent:.3f}')
        self.get_logger().info(f'[RECT] turn_angle_deg            = {self.turn_angle_deg:.3f}')
        self.get_logger().info(f'[RECT] turn_direction            = {self.turn_direction}')
        self.get_logger().info(f'[RECT] turn_kp                   = {self.turn_kp:.3f}')
        self.get_logger().info(f'[RECT] turn_max_cmd              = {self.turn_max_cmd:.3f}')
        self.get_logger().info(f'[RECT] turn_min_cmd              = {self.turn_min_cmd:.3f}')
        self.get_logger().info(f'[RECT] turn_tolerance_deg        = {self.turn_angle_tolerance_deg:.3f}')
        self.get_logger().info('[RECT] Waiting for odometry...')
        self.get_logger().info('==================================================')

    # =========================================================
    # Utility
    # =========================================================
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def angle_diff(self, current, previous):
        return self.wrap_angle(current - previous)

    def clamp(self, value, vmin, vmax):
        return max(vmin, min(vmax, value))

    def parameter_to_bool(self, value):
        if isinstance(value, str):
            return value.strip().lower() in ('true', '1', 'yes', 'on')
        return bool(value)

    def parse_marker_id_sequence(self, text):
        marker_ids = []
        for item in text.split(','):
            item = item.strip()
            if not item:
                continue
            try:
                marker_ids.append(int(item))
            except ValueError:
                self.get_logger().warn(f'[RECT][ARUCO] bad marker id: {item}')
        return marker_ids

    def parse_marker_group_sequence(self, text):
        groups = []
        for group_text in text.split(';'):
            group = self.parse_marker_id_sequence(group_text)
            if group:
                groups.append(group)
        return groups

    def average_angles(self, angles):
        sin_sum = 0.0
        cos_sum = 0.0
        for a in angles:
            sin_sum += math.sin(a)
            cos_sum += math.cos(a)
        return math.atan2(sin_sum, cos_sum)

    def sign(self, x):
        if x > 0.0:
            return 1.0
        if x < 0.0:
            return -1.0
        return 0.0

    def publish_motor_command(self, left_cmd, right_cmd):
        msg = LeftRightFloat32()
        msg.left = float(left_cmd)
        msg.right = float(right_cmd)
        msg.seq_num = self.command_seq_num
        self.cmd_pub.publish(msg)
        self.command_seq_num += 1

    def publish_smoothed_control_command(self, target_left_cmd, target_right_cmd):
        """Apply slew-rate limiting and low-pass filtering to drive commands.

        State transitions and shutdown still publish zero directly. This helper
        is only for normal timer-driven motion, where command continuity matters.
        """
        left_cmd = float(target_left_cmd)
        right_cmd = float(target_right_cmd)

        if self.duty_slew_rate_percent_per_sec > 0.0 and self.command_publish_period > 0.0:
            max_delta = self.duty_slew_rate_percent_per_sec * self.command_publish_period
            left_cmd = self.last_control_left_cmd + self.clamp(
                left_cmd - self.last_control_left_cmd,
                -max_delta,
                max_delta,
            )
            right_cmd = self.last_control_right_cmd + self.clamp(
                right_cmd - self.last_control_right_cmd,
                -max_delta,
                max_delta,
            )

        if self.command_filter_alpha < 1.0:
            alpha = self.command_filter_alpha
            left_cmd = alpha * left_cmd + (1.0 - alpha) * self.last_control_left_cmd
            right_cmd = alpha * right_cmd + (1.0 - alpha) * self.last_control_right_cmd

        self.last_control_left_cmd = left_cmd
        self.last_control_right_cmd = right_cmd
        self.publish_motor_command(left_cmd, right_cmd)

    def reset_smoothed_control_command(self):
        self.last_control_left_cmd = 0.0
        self.last_control_right_cmd = 0.0

    def smoothstep(self, x):
        x = self.clamp(x, 0.0, 1.0)
        return x * x * (3.0 - 2.0 * x)

    def get_straight_distance_state(self, target_distance):
        dx = self.current_x - self.segment_start_x
        dy = self.current_y - self.segment_start_y
        distance_travelled = math.sqrt(dx * dx + dy * dy)
        encoder_remaining = max(0.0, target_distance - distance_travelled)
        aruco_remaining = self.get_aruco_turn_distance_remaining()
        distance_remaining = encoder_remaining
        if aruco_remaining is not None:
            distance_remaining = min(distance_remaining, aruco_remaining)
        return distance_travelled, distance_remaining, encoder_remaining, aruco_remaining

    def straight_speed_scale(self, distance_travelled, distance_remaining, heading_error):
        """Combine acceleration, corner deceleration, and heading-error scaling."""
        if self.ramp_up_distance_m > 0.0:
            start_scale = self.smoothstep(distance_travelled / self.ramp_up_distance_m)
        else:
            start_scale = 1.0

        if self.deceleration_zone_m > 0.0:
            stop_scale = self.smoothstep(distance_remaining / self.deceleration_zone_m)
        else:
            stop_scale = 1.0

        heading_scale = 1.0 - self.heading_speed_scale_k * abs(heading_error)
        heading_scale = self.clamp(heading_scale, 0.0, 1.0)

        return self.clamp(start_scale * stop_scale * heading_scale, 0.0, 1.0)

    def scaled_straight_duty(self, speed_scale):
        if self.straight_duty_cycle_percent <= 0.0:
            return 0.0

        # speed_scale=0 means "minimum moving duty", not zero. That prevents a
        # distance-based ramp from getting stuck before the robot starts moving.
        speed_scale = self.clamp(speed_scale, 0.0, 1.0)
        duty_span = self.straight_duty_cycle_percent - self.min_straight_duty_cycle_percent
        return self.min_straight_duty_cycle_percent + duty_span * speed_scale

    def scaled_startup_duty(self, raw_cmd, max_duty=None, min_duty=None):
        """Scale startup alignment commands smoothly near zero error."""
        if max_duty is None:
            max_duty = self.startup_max_duty_cycle_percent
        if min_duty is None:
            min_duty = self.startup_min_duty_cycle_percent

        max_duty = abs(float(max_duty))
        min_duty = abs(float(min_duty))
        min_duty = min(min_duty, max_duty)

        raw_cmd = self.clamp(
            raw_cmd,
            -max_duty,
            max_duty,
        )
        if abs(raw_cmd) <= 1e-6:
            return 0.0

        magnitude = abs(raw_cmd)
        magnitude = self.clamp(
            magnitude,
            0.0,
            max_duty,
        )
        if magnitude > 0.0:
            magnitude = max(magnitude, min_duty)
        return self.sign(raw_cmd) * magnitude

    def startup_pose_is_fresh(self):
        if self.startup_pose_stamp_sec is None:
            return False
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        return (now_sec - self.startup_pose_stamp_sec) <= self.startup_pose_stale_timeout_sec

    def startup_global_pose_callback(self, msg: Odometry):
        """Low-pass filter the ArUco-derived global pose used at startup."""
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(yaw):
            return

        if self.startup_filtered_pose is None:
            self.startup_filtered_pose = [x, y, yaw]
        else:
            alpha = self.startup_pose_filter_alpha
            old_x, old_y, old_yaw = self.startup_filtered_pose
            self.startup_filtered_pose = [
                alpha * x + (1.0 - alpha) * old_x,
                alpha * y + (1.0 - alpha) * old_y,
                self.wrap_angle(old_yaw + alpha * self.wrap_angle(yaw - old_yaw)),
            ]

        self.startup_pose_sample_count += 1
        self.startup_pose_stamp_sec = self.get_clock().now().nanoseconds * 1.0e-9
        self.startup_recent_pose_window.append(tuple(self.startup_filtered_pose))
        max_window = max(
            self.startup_pose_stable_sample_count,
            self.startup_settle_sample_count,
        )
        if len(self.startup_recent_pose_window) > max_window:
            self.startup_recent_pose_window.pop(0)

    def startup_pose_is_stable(self):
        return (
            self.startup_filtered_pose is not None
            and self.startup_pose_sample_count >= self.startup_pose_stable_sample_count
            and self.startup_pose_is_fresh()
        )

    def get_startup_reference_pose(self):
        if not self.startup_recent_pose_window:
            return None

        count = min(len(self.startup_recent_pose_window), self.startup_settle_sample_count)
        window = self.startup_recent_pose_window[-count:]
        avg_x = sum(pose[0] for pose in window) / count
        avg_y = sum(pose[1] for pose in window) / count
        avg_yaw = self.average_angles([pose[2] for pose in window])
        return avg_x, avg_y, avg_yaw

    def transition_to_rectangle_mode(self):
        self.control_mode = 'run'
        self.ready_to_move = True
        self.yaw_samples = [self.current_yaw]
        self.state_initialized = False
        self.publish_motor_command(0.0, 0.0)
        self.reset_smoothed_control_command()
        self.get_logger().info('==================================================')
        self.get_logger().info('[RECT][INIT] Startup alignment complete; starting rectangle trajectory')
        self.get_logger().info(f'[RECT][INIT] wheel_odom_x   = {self.current_x:.4f}')
        self.get_logger().info(f'[RECT][INIT] wheel_odom_y   = {self.current_y:.4f}')
        self.get_logger().info(f'[RECT][INIT] wheel_odom_yaw = {self.current_yaw:.4f}')
        if self.startup_filtered_pose is not None:
            x, y, yaw = self.startup_filtered_pose
            self.get_logger().info(
                f'[RECT][INIT] global_pose  = ({x:.4f}, {y:.4f}, '
                f'{math.degrees(yaw):.2f} deg)'
            )
        self.get_logger().info('==================================================')

    def update_startup_alignment(self):
        """Drive from the filtered global pose to the configured start pose."""
        if not self.startup_pose_is_stable():
            self.startup_nav_phase = 'localize'
            self.startup_target_heading = None
            self.startup_in_tolerance_count = 0
            now_sec = self.get_clock().now().nanoseconds * 1.0e-9
            if self.startup_last_log_sec is None or (now_sec - self.startup_last_log_sec) > 1.0:
                self.startup_last_log_sec = now_sec
                self.get_logger().info(
                    f'[RECT][INIT] Waiting for stable global pose: '
                    f'{self.startup_pose_sample_count}/{self.startup_pose_stable_sample_count}'
                )
            self.publish_motor_command(0.0, 0.0)
            self.reset_smoothed_control_command()
            return

        reference_pose = self.get_startup_reference_pose()
        if reference_pose is None:
            self.publish_motor_command(0.0, 0.0)
            self.reset_smoothed_control_command()
            return

        x, y, yaw = reference_pose
        dx = self.startup_target_x_m - x
        dy = self.startup_target_y_m - y
        position_error = math.sqrt(dx * dx + dy * dy)
        final_yaw_error = self.wrap_angle(self.startup_target_yaw - yaw)
        target_heading = math.atan2(dy, dx) if position_error > 1.0e-6 else self.startup_target_yaw

        if (
            position_error <= self.startup_completion_position_tolerance_m
            and abs(final_yaw_error) <= self.startup_completion_heading_tolerance
        ):
            self.startup_in_tolerance_count += 1
            if self.startup_in_tolerance_count >= self.startup_settle_sample_count:
                self.transition_to_rectangle_mode()
                return
        else:
            self.startup_in_tolerance_count = 0

        target_heading_text = 'None'
        if self.startup_target_heading is not None:
            target_heading_text = f'{math.degrees(self.startup_target_heading):+.2f}deg'

        left_cmd = 0.0
        right_cmd = 0.0
        phase = self.startup_nav_phase
        limited_max_duty = self.startup_max_duty_cycle_percent
        limited_min_duty = self.startup_min_duty_cycle_percent

        if position_error > self.startup_position_tolerance_m:
            if self.startup_nav_phase in ('localize', 'final_heading') or self.startup_target_heading is None:
                self.startup_target_heading = target_heading
                self.startup_nav_phase = 'rotate_to_target'

            heading_to_target_error = self.wrap_angle(self.startup_target_heading - yaw)
            if self.startup_near_target_distance_m > 0.0:
                near_scale = self.clamp(
                    position_error / self.startup_near_target_distance_m,
                    0.0,
                    1.0,
                )
                limited_max_duty = (
                    self.startup_near_target_min_duty_cycle_percent
                    + (self.startup_max_duty_cycle_percent - self.startup_near_target_min_duty_cycle_percent)
                    * near_scale
                )
                limited_min_duty = min(
                    self.startup_near_target_min_duty_cycle_percent,
                    limited_max_duty,
                )

            if (
                self.startup_nav_phase == 'drive_to_target'
                and abs(heading_to_target_error) > self.startup_realign_heading_tolerance
            ):
                self.startup_target_heading = target_heading
                self.startup_nav_phase = 'rotate_to_target'
                heading_to_target_error = self.wrap_angle(self.startup_target_heading - yaw)

            if self.startup_nav_phase == 'rotate_to_target':
                phase = 'rotate_to_target'
                if abs(heading_to_target_error) <= self.startup_drive_heading_tolerance:
                    self.startup_nav_phase = 'drive_to_target'
                    self.startup_target_heading = target_heading
                    heading_to_target_error = self.wrap_angle(self.startup_target_heading - yaw)
                    phase = 'drive_to_target'
                else:
                    turn_cmd = self.scaled_startup_duty(
                        self.startup_heading_kp * heading_to_target_error,
                        max_duty=limited_max_duty,
                        min_duty=limited_min_duty,
                    )
                    left_cmd = -turn_cmd
                    right_cmd = turn_cmd

            if self.startup_nav_phase == 'drive_to_target':
                phase = 'drive_to_target'
                self.startup_target_heading = target_heading
                heading_to_target_error = self.wrap_angle(self.startup_target_heading - yaw)
                if abs(heading_to_target_error) > self.startup_realign_heading_tolerance:
                    self.startup_nav_phase = 'rotate_to_target'
                    phase = 'rotate_to_target'
                    turn_cmd = self.scaled_startup_duty(
                        self.startup_heading_kp * heading_to_target_error,
                        max_duty=limited_max_duty,
                        min_duty=limited_min_duty,
                    )
                    left_cmd = -turn_cmd
                    right_cmd = turn_cmd
                else:
                    forward_cmd = self.scaled_startup_duty(
                        self.startup_linear_kp * position_error,
                        max_duty=limited_max_duty,
                        min_duty=limited_min_duty,
                    )
                    correction = self.scaled_startup_duty(
                        self.startup_heading_kp * heading_to_target_error,
                        max_duty=limited_max_duty,
                        min_duty=0.0,
                    )
                    correction = self.clamp(
                        correction,
                        -abs(forward_cmd),
                        abs(forward_cmd),
                    )
                    left_cmd = forward_cmd - correction
                    right_cmd = forward_cmd + correction
        else:
            self.startup_nav_phase = 'final_heading'
            self.startup_target_heading = None
            phase = 'final_heading'
            if self.startup_final_heading_slowdown > 0.0:
                final_scale = self.clamp(
                    abs(final_yaw_error) / self.startup_final_heading_slowdown,
                    0.0,
                    1.0,
                )
                limited_max_duty = (
                    self.startup_near_target_min_duty_cycle_percent
                    + (self.startup_max_duty_cycle_percent - self.startup_near_target_min_duty_cycle_percent)
                    * final_scale
                )
                limited_min_duty = min(
                    self.startup_near_target_min_duty_cycle_percent,
                    limited_max_duty,
                )
            if abs(final_yaw_error) > self.startup_completion_heading_tolerance:
                turn_cmd = self.scaled_startup_duty(
                    self.startup_final_heading_kp * final_yaw_error,
                    max_duty=limited_max_duty,
                    min_duty=limited_min_duty,
                )
                left_cmd = -turn_cmd
                right_cmd = turn_cmd

        if self.verbose:
            self.get_logger().info(
                f'[RECT][INIT CMD] phase={phase}, '
                f'global=({x:.3f}, {y:.3f}, {math.degrees(yaw):+.2f}deg), '
                f'dx={dx:+.3f}, dy={dy:+.3f}, '
                f'pos_err={position_error:.3f}m, '
                f'yaw_err={math.degrees(final_yaw_error):+.2f}deg, '
                f'target_heading={target_heading_text}, '
                f'settled={self.startup_in_tolerance_count}/{self.startup_settle_sample_count}, '
                f'left_cmd={left_cmd:.3f}, right_cmd={right_cmd:.3f}'
            )

        self.publish_smoothed_control_command(left_cmd, right_cmd)

    def current_mode_and_target(self):
        if self.state_index >= len(self.plan):
            return 'done', 0.0
        return self.plan[self.state_index]

    def target_aruco_marker_ids(self):
        mode, _target = self.current_mode_and_target()
        if mode != 'straight':
            return []
        straight_index = self.state_index // 2
        if straight_index < 0 or straight_index >= len(self.aruco_heading_marker_groups):
            return []
        return self.aruco_heading_marker_groups[straight_index]

    def target_aruco_turn_marker_id(self):
        mode, _target = self.current_mode_and_target()
        if mode != 'straight':
            return None
        straight_index = self.state_index // 2
        if straight_index < 0 or straight_index >= len(self.aruco_turn_marker_ids):
            return None
        return self.aruco_turn_marker_ids[straight_index]

    def get_aruco_heading_assist(self):
        if not self.use_aruco_heading_assist:
            return None

        target_ids = self.target_aruco_marker_ids()
        if not target_ids:
            return None

        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        best = None
        best_score = None

        for priority, target_id in enumerate(target_ids):
            obs = self.aruco_heading_observations.get(target_id)
            if obs is None:
                continue

            age = now_sec - obs['stamp_sec']
            if age > self.aruco_heading_stale_timeout_sec:
                continue

            heading_error = obs['heading_error']
            if abs(heading_error) < self.aruco_heading_deadband:
                heading_error = 0.0

            score = (priority, abs(obs['bearing']), obs['range'])
            if best_score is None or score < best_score:
                best_score = score
                best = {
                    'marker_id': target_id,
                    'target_ids': target_ids,
                    'priority': priority,
                    'age': age,
                    'bearing': obs['bearing'],
                    'heading_error': heading_error,
                    'range': obs['range'],
                }

        return best

    def get_aruco_turn_distance_trigger(self, distance_travelled, target_distance):
        if not self.use_aruco_turn_distance_trigger:
            return None

        target_id = self.target_aruco_turn_marker_id()
        if target_id is None:
            return None

        min_encoder_distance = target_distance * self.aruco_turn_min_encoder_fraction
        if distance_travelled < min_encoder_distance:
            return None

        obs = self.aruco_heading_observations.get(target_id)
        if obs is None:
            return None

        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        age = now_sec - obs['stamp_sec']
        if age > self.aruco_heading_stale_timeout_sec:
            return None

        if abs(obs['bearing']) > self.aruco_turn_max_bearing:
            return None

        if obs['range'] > self.aruco_turn_distance_m:
            return None

        return {
            'marker_id': target_id,
            'age': age,
            'bearing': obs['bearing'],
            'range': obs['range'],
            'forward_range': obs['forward_range'],
            'target_distance': self.aruco_turn_distance_m,
            'distance_travelled': distance_travelled,
        }

    def get_aruco_turn_distance_remaining(self):
        if not self.use_aruco_turn_distance_trigger:
            return None

        target_id = self.target_aruco_turn_marker_id()
        if target_id is None:
            return None

        obs = self.aruco_heading_observations.get(target_id)
        if obs is None:
            return None

        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if (now_sec - obs['stamp_sec']) > self.aruco_heading_stale_timeout_sec:
            return None

        if abs(obs['bearing']) > self.aruco_turn_max_bearing:
            return None

        return max(0.0, obs['forward_range'] - self.aruco_turn_distance_m)

    def initialize_state(self):
        mode, target = self.current_mode_and_target()

        if mode == 'straight':
            self.segment_start_x = self.current_x
            self.segment_start_y = self.current_y
            self.segment_start_yaw = self.current_yaw
            self.get_logger().info(
                f'[RECT] Init STRAIGHT {self.state_index + 1}/{len(self.plan)} | '
                f'target={target:.3f} m, x0={self.segment_start_x:.4f}, '
                f'y0={self.segment_start_y:.4f}, yaw0={self.segment_start_yaw:.4f}'
            )

        elif mode == 'turn':
            self.prev_yaw = self.current_yaw
            self.accumulated_yaw = 0.0
            self.get_logger().info(
                f'[RECT] Init TURN {self.state_index + 1}/{len(self.plan)} | '
                f'target={target * 180.0 / math.pi:.2f} deg, yaw0={self.current_yaw:.4f}'
            )

        self.state_initialized = True

    def advance_state(self):
        self.publish_motor_command(0.0, 0.0)
        self.reset_smoothed_control_command()
        self.state_index += 1
        self.state_initialized = False

        if self.state_index >= len(self.plan):
            self.finished = True
            self.publish_motor_command(0.0, 0.0)
            self.reset_smoothed_control_command()
            self.get_logger().info('==================================================')
            self.get_logger().info('[RECT] Rectangle trajectory completed')
            self.get_logger().info(f'[RECT] final_x   = {self.current_x:.4f}')
            self.get_logger().info(f'[RECT] final_y   = {self.current_y:.4f}')
            self.get_logger().info(f'[RECT] final_yaw = {self.current_yaw:.4f}')
            self.get_logger().info('==================================================')
            rclpy.shutdown()

    # =========================================================
    # Odom callback
    # =========================================================
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('[RECT] First odometry received.')

        if self.finished:
            return

        if self.control_mode != 'run':
            return

        if not self.ready_to_move:
            self.yaw_samples.append(self.current_yaw)

            if self.verbose:
                self.get_logger().info(
                    f'[RECT] Collecting startup yaw: '
                    f'{len(self.yaw_samples)}/{self.startup_yaw_sample_count}, '
                    f'current_yaw={self.current_yaw:.4f}'
                )

            if len(self.yaw_samples) >= self.startup_yaw_sample_count:
                avg_yaw = self.average_angles(self.yaw_samples)
                self.current_yaw = avg_yaw
                self.ready_to_move = True
                self.get_logger().info(
                    f'[RECT] Startup yaw ready, avg_yaw={avg_yaw:.4f}'
                )
            return

        mode, target = self.current_mode_and_target()
        if mode == 'done':
            return

        if not self.state_initialized:
            self.initialize_state()

        if mode == 'straight':
            dx = self.current_x - self.segment_start_x
            dy = self.current_y - self.segment_start_y
            distance_travelled = math.sqrt(dx * dx + dy * dy)
            yaw_error = self.wrap_angle(self.current_yaw - self.segment_start_yaw)
            aruco_turn = self.get_aruco_turn_distance_trigger(distance_travelled, target)

            if self.verbose:
                aruco_turn_text = 'none'
                if aruco_turn is not None:
                    aruco_turn_text = (
                        f'id={aruco_turn["marker_id"]}, '
                        f'range={aruco_turn["range"]:.3f}m, '
                        f'forward={aruco_turn["forward_range"]:.3f}m, '
                        f'bearing={math.degrees(aruco_turn["bearing"]):+.2f}deg, '
                        f'age={aruco_turn["age"]:.2f}s'
                    )
                self.get_logger().info(
                    f'[RECT][STRAIGHT] idx={self.state_index}, '
                    f'x={self.current_x:.4f}, y={self.current_y:.4f}, '
                    f'yaw={self.current_yaw:.4f}, yaw_error={yaw_error:.4f}, '
                    f'travelled={distance_travelled:.4f}/{target:.4f} m, '
                    f'aruco_targets={self.target_aruco_marker_ids()}, '
                    f'aruco_turn=({aruco_turn_text})'
                )

            if aruco_turn is not None:
                self.get_logger().info(
                    f'[RECT] STRAIGHT finished by ArUco distance | '
                    f'marker={aruco_turn["marker_id"]}, '
                    f'range={aruco_turn["range"]:.3f} m <= {aruco_turn["target_distance"]:.3f} m, '
                    f'bearing={math.degrees(aruco_turn["bearing"]):+.2f} deg, '
                    f'encoder_travelled={distance_travelled:.4f}/{target:.4f} m'
                )
                self.advance_state()

            elif distance_travelled >= target:
                self.get_logger().info(
                    f'[RECT] STRAIGHT finished by encoder fallback | '
                    f'travelled={distance_travelled:.4f} m'
                )
                self.advance_state()

        elif mode == 'turn':
            delta_yaw = self.angle_diff(self.current_yaw, self.prev_yaw)
            self.accumulated_yaw += delta_yaw
            self.prev_yaw = self.current_yaw

            travelled_angle = abs(self.accumulated_yaw)
            remaining_angle = target - travelled_angle

            if self.verbose:
                self.get_logger().info(
                    f'[RECT][TURN] idx={self.state_index}, '
                    f'yaw={self.current_yaw:.4f}, accumulated={self.accumulated_yaw:.4f} rad '
                    f'({self.accumulated_yaw * 180.0 / math.pi:.2f} deg), '
                    f'remaining={remaining_angle * 180.0 / math.pi:.2f} deg'
                )

            if remaining_angle <= self.turn_angle_tolerance_rad:
                self.get_logger().info(
                    f'[RECT] TURN finished | actual={self.accumulated_yaw * 180.0 / math.pi:.2f} deg'
                )
                self.advance_state()

    # =========================================================
    # ArUco callback
    # =========================================================
    def aruco_callback(self, msg: FiducialMarkerArray):
        if not self.use_aruco_feedback:
            return

        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        wanted_ids = self.aruco_wanted_ids

        for marker in msg.markers:
            marker_id = int(marker.id)
            if marker_id not in wanted_ids:
                continue

            tx = float(marker.tvec[0])
            tz = float(marker.tvec[2])
            if not math.isfinite(tx) or not math.isfinite(tz) or tz <= 0.05:
                continue

            bearing = math.atan2(-tx, tz)
            if abs(bearing) > self.aruco_heading_max_bearing:
                continue

            heading_error = self.aruco_heading_error_sign * bearing
            range_planar = math.sqrt(tx * tx + tz * tz)

            old = self.aruco_heading_observations.get(marker_id)
            if old is not None and (now_sec - old['stamp_sec']) <= self.aruco_heading_stale_timeout_sec:
                alpha = self.aruco_heading_filter_alpha
                heading_error = alpha * heading_error + (1.0 - alpha) * old['heading_error']
                bearing = alpha * bearing + (1.0 - alpha) * old['bearing']

            self.aruco_heading_observations[marker_id] = {
                'stamp_sec': now_sec,
                'bearing': bearing,
                'heading_error': heading_error,
                'range': range_planar,
                'forward_range': tz,
            }

    # =========================================================
    # Control timer
    # =========================================================
    def command_timer_callback(self):
        if self.finished:
            return

        if not self.odom_received:
            self.publish_motor_command(0.0, 0.0)
            self.reset_smoothed_control_command()
            return

        if self.control_mode != 'run':
            self.update_startup_alignment()
            return

        if not self.ready_to_move:
            self.publish_motor_command(0.0, 0.0)
            self.reset_smoothed_control_command()
            return

        mode, target = self.current_mode_and_target()
        if mode == 'done':
            self.publish_motor_command(0.0, 0.0)
            self.reset_smoothed_control_command()
            return

        if not self.state_initialized:
            self.publish_motor_command(0.0, 0.0)
            self.reset_smoothed_control_command()
            return

        if mode == 'straight':
            (
                distance_travelled,
                distance_remaining,
                encoder_remaining,
                aruco_remaining,
            ) = self.get_straight_distance_state(target)

            yaw_error = self.wrap_angle(self.current_yaw - self.segment_start_yaw)

            if abs(yaw_error) < self.yaw_deadband:
                yaw_error = 0.0

            aruco_correction = 0.0
            aruco_assist = self.get_aruco_heading_assist()
            speed_heading_error = abs(yaw_error)
            if aruco_assist is not None:
                speed_heading_error = max(
                    speed_heading_error,
                    abs(aruco_assist['heading_error']),
                )
                aruco_correction = self.aruco_heading_kp * aruco_assist['heading_error']
                aruco_correction = self.clamp(
                    aruco_correction,
                    -self.aruco_heading_max_correction,
                    self.aruco_heading_max_correction,
                )

            speed_scale = self.straight_speed_scale(
                distance_travelled,
                distance_remaining,
                speed_heading_error,
            )
            forward_duty = self.scaled_straight_duty(speed_scale)
            base_cmd = self.straight_direction * forward_duty

            left_base = base_cmd * self.left_trim
            right_base = base_cmd * self.right_trim

            encoder_correction = self.yaw_kp * yaw_error
            correction = encoder_correction + aruco_correction
            correction = self.clamp(correction, -self.max_correction, self.max_correction)
            # As forward duty falls near a corner, also limit correction so the
            # straight controller does not become a sudden pivot.
            correction_limit = min(self.max_correction, forward_duty)
            correction = self.clamp(correction, -correction_limit, correction_limit)

            if self.straight_direction == 1:
                left_cmd = left_base - correction
                right_cmd = right_base + correction
            else:
                left_cmd = left_base + correction
                right_cmd = right_base - correction

            if self.verbose:
                aruco_text = 'none'
                if aruco_assist is not None:
                    aruco_text = (
                        f'id={aruco_assist["marker_id"]}, '
                        f'targets={aruco_assist["target_ids"]}, '
                        f'priority={aruco_assist["priority"]}, '
                        f'bearing={math.degrees(aruco_assist["bearing"]):+.2f}deg, '
                        f'err={math.degrees(aruco_assist["heading_error"]):+.2f}deg, '
                        f'range={aruco_assist["range"]:.3f}m, '
                        f'corr={aruco_correction:+.3f}, '
                        f'age={aruco_assist["age"]:.2f}s'
                    )
                self.get_logger().info(
                    f'[RECT][STRAIGHT CMD] idx={self.state_index}, '
                    f'travelled={distance_travelled:.3f}, '
                    f'remaining={distance_remaining:.3f}, '
                    f'encoder_remaining={encoder_remaining:.3f}, '
                    f'aruco_remaining={aruco_remaining}, '
                    f'speed_scale={speed_scale:.3f}, '
                    f'forward_duty={forward_duty:.3f}, '
                    f'left_base={left_base:.3f}, right_base={right_base:.3f}, '
                    f'yaw_error={yaw_error:.4f}, '
                    f'encoder_corr={encoder_correction:.3f}, '
                    f'aruco=({aruco_text}), '
                    f'correction={correction:.3f}, '
                    f'left_cmd={left_cmd:.3f}, right_cmd={right_cmd:.3f}'
                )

            self.publish_smoothed_control_command(left_cmd, right_cmd)

        elif mode == 'turn':
            travelled_angle = abs(self.accumulated_yaw)
            remaining_angle = target - travelled_angle

            if remaining_angle <= 0.0:
                self.publish_motor_command(0.0, 0.0)
                self.reset_smoothed_control_command()
                return

            turn_cmd = self.turn_kp * remaining_angle
            turn_cmd = self.clamp(turn_cmd, self.turn_min_cmd, self.turn_max_cmd)

            if self.turn_direction == 1:
                left_cmd = turn_cmd
                right_cmd = -turn_cmd
            else:
                left_cmd = -turn_cmd
                right_cmd = turn_cmd

            if self.verbose:
                self.get_logger().info(
                    f'[RECT][TURN CMD] idx={self.state_index}, '
                    f'remaining_deg={remaining_angle * 180.0 / math.pi:.2f}, '
                    f'turn_cmd={turn_cmd:.3f}, '
                    f'left_cmd={left_cmd:.3f}, right_cmd={right_cmd:.3f}'
                )

            self.publish_smoothed_control_command(left_cmd, right_cmd)

    def destroy_node(self):
        try:
            self.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiuRectangleTrajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[RECT] Keyboard interrupt, stopping.')
    finally:
        try:
            node.publish_motor_command(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()
