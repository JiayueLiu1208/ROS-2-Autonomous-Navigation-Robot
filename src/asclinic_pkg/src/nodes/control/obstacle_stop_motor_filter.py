#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from asclinic_pkg.msg import LeftRightFloat32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ObstacleStopMotorFilter(Node):
    """Forward motor commands with local obstacle avoidance (steer or hard-stop)."""

    def __init__(self):
        super().__init__('obstacle_stop_motor_filter')

        self.declare_parameter('input_cmd_topic', 'set_motor_duty_cycle_raw')
        self.declare_parameter('output_cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', 'pitt_fused_odometry')
        self.declare_parameter('enabled', True)
        self.declare_parameter('stop_distance_m', 0.35)
        self.declare_parameter('clear_distance_m', 0.55)
        self.declare_parameter('front_sector_half_angle_deg', 30.0)
        self.declare_parameter('front_sector_center_angle_deg', 0.0)
        self.declare_parameter('lidar_in_base_yaw_deg', 0.0)
        self.declare_parameter('scan_angle_multiplier', 1.0)
        self.declare_parameter('scan_timeout_sec', 0.75)
        self.declare_parameter('stop_on_stale_scan', False)
        self.declare_parameter('stop_publish_period_sec', 0.10)
        self.declare_parameter('status_log_period_sec', 1.0)
        self.declare_parameter('lidar_verbose', False)
        self.declare_parameter('lidar_verbose_period_sec', 0.5)
        # avoidance_mode: 'steer' (lateral nudge) or 'stop' (hard zero)
        self.declare_parameter('avoidance_mode', 'steer')
        # Side-wall avoidance: bearings between front_sector_half_angle and this angle
        self.declare_parameter('side_sector_half_angle_deg', 90.0)
        # Closer threshold for side obstacles (walls can legitimately be nearby)
        self.declare_parameter('side_stop_distance_m', 0.20)
        # Max duty-cycle differential applied per side when obstacle is at 0 m
        self.declare_parameter('avoidance_max_steer_duty', 6.0)
        # In steer mode, still allow a final hard-stop if the forward sector is
        # already too close for a steering nudge to be useful.
        self.declare_parameter('steer_emergency_stop_distance_m', 0.0)
        self.declare_parameter('front_recovery_enabled', False)
        self.declare_parameter('front_recovery_trigger_distance_m', 0.0)
        self.declare_parameter('front_recovery_center_half_angle_deg', 15.0)
        self.declare_parameter('front_recovery_distance_m', 0.20)
        self.declare_parameter('front_recovery_reverse_duty', 22.0)
        self.declare_parameter('front_recovery_timeout_sec', 1.5)
        self.declare_parameter('front_recovery_steer_duration_sec', 0.8)
        self.declare_parameter('front_recovery_cooldown_sec', 1.0)
        self.declare_parameter('pre_stop_decel_enabled', True)
        self.declare_parameter('pre_stop_decel_scale', 0.70)
        self.declare_parameter('pre_stop_decel_distance_m', 0.60)
        # Latency compensation: predict obstacle distance forward by v*t to
        # counteract scan→motor pipeline delay without widening the static threshold.
        self.declare_parameter('latency_compensation_sec', 0.25)
        self.declare_parameter('forward_speed_m_s', 0.12)

        self.input_cmd_topic = str(self.get_parameter('input_cmd_topic').value)
        self.output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.enabled = self._to_bool(self.get_parameter('enabled').value)
        self.stop_distance_m = max(0.0, float(self.get_parameter('stop_distance_m').value))
        self.clear_distance_m = max(
            self.stop_distance_m,
            float(self.get_parameter('clear_distance_m').value),
        )
        self.front_sector_half_angle = math.radians(
            abs(float(self.get_parameter('front_sector_half_angle_deg').value))
        )
        front_center_deg = float(self.get_parameter('front_sector_center_angle_deg').value)
        lidar_yaw_deg = float(self.get_parameter('lidar_in_base_yaw_deg').value)
        self.front_sector_center_angle = math.radians(front_center_deg)
        self.lidar_in_base_yaw = math.radians(lidar_yaw_deg)
        self.lidar_in_base_yaw_deg = lidar_yaw_deg
        self.scan_angle_multiplier = float(
            self.get_parameter('scan_angle_multiplier').value
        )
        self.scan_timeout_sec = max(0.0, float(self.get_parameter('scan_timeout_sec').value))
        self.stop_on_stale_scan = self._to_bool(self.get_parameter('stop_on_stale_scan').value)
        self.stop_publish_period_sec = max(
            0.02, float(self.get_parameter('stop_publish_period_sec').value)
        )
        self.status_log_period_sec = max(
            0.1, float(self.get_parameter('status_log_period_sec').value)
        )
        self.lidar_verbose = self._to_bool(self.get_parameter('lidar_verbose').value)
        self.lidar_verbose_period_sec = max(
            0.1, float(self.get_parameter('lidar_verbose_period_sec').value)
        )
        self.avoidance_mode = str(self.get_parameter('avoidance_mode').value).strip().lower()
        self.avoidance_max_steer_duty = max(
            0.0, float(self.get_parameter('avoidance_max_steer_duty').value)
        )
        self.steer_emergency_stop_distance_m = max(
            0.0,
            float(self.get_parameter('steer_emergency_stop_distance_m').value),
        )
        self.front_recovery_enabled = self._to_bool(
            self.get_parameter('front_recovery_enabled').value
        )
        self.front_recovery_trigger_distance_m = max(
            0.0,
            float(self.get_parameter('front_recovery_trigger_distance_m').value),
        )
        self.front_recovery_center_half_angle = math.radians(
            abs(float(self.get_parameter('front_recovery_center_half_angle_deg').value))
        )
        self.front_recovery_distance_m = max(
            0.0,
            float(self.get_parameter('front_recovery_distance_m').value),
        )
        self.front_recovery_reverse_duty = max(
            0.0,
            min(100.0, float(self.get_parameter('front_recovery_reverse_duty').value)),
        )
        self.front_recovery_timeout_sec = max(
            0.1,
            float(self.get_parameter('front_recovery_timeout_sec').value),
        )
        self.front_recovery_steer_duration_sec = max(
            0.0,
            float(self.get_parameter('front_recovery_steer_duration_sec').value),
        )
        self.front_recovery_cooldown_sec = max(
            0.0,
            float(self.get_parameter('front_recovery_cooldown_sec').value),
        )
        self.pre_stop_decel_enabled = self._to_bool(
            self.get_parameter('pre_stop_decel_enabled').value
        )
        self.pre_stop_decel_scale = max(
            0.0,
            min(1.0, float(self.get_parameter('pre_stop_decel_scale').value)),
        )
        self.pre_stop_decel_distance_m = max(
            0.0,
            float(self.get_parameter('pre_stop_decel_distance_m').value),
        )
        self.latency_compensation_sec = max(
            0.0, float(self.get_parameter('latency_compensation_sec').value)
        )
        self.forward_speed_m_s = max(
            0.0, float(self.get_parameter('forward_speed_m_s').value)
        )
        self.side_sector_half_angle = math.radians(
            abs(float(self.get_parameter('side_sector_half_angle_deg').value))
        )
        self.side_stop_distance_m = max(
            0.0, float(self.get_parameter('side_stop_distance_m').value)
        )

        # Scan state
        self.latest_scan_time_sec = None
        self.latest_front_min_m = math.inf
        self.latest_left_min_m = math.inf    # bearing > 0  (left of robot), front sector
        self.latest_right_min_m = math.inf   # bearing <= 0 (right of robot), front sector
        self.latest_side_left_min_m = math.inf   # left side sector
        self.latest_side_right_min_m = math.inf  # right side sector
        self.latest_closest_bearing_rad = 0.0
        self.front_obstacle_active = False   # hysteresis flag (used in stop mode)
        self.latest_pose_xy = None

        self.recovery_phase = None
        self.recovery_start_time_sec = None
        self.recovery_start_xy = None
        self.recovery_steer_until_sec = None
        self.recovery_steer_direction = 1.0
        self.last_recovery_end_sec = None

        self.output_seq_num = 1
        self.last_status_log_sec = None
        self.last_lidar_log_sec = None

        self.cmd_pub = self.create_publisher(LeftRightFloat32, self.output_cmd_topic, 10)
        self.cmd_sub = self.create_subscription(
            LeftRightFloat32, self.input_cmd_topic, self.cmd_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        # Timer only needed in stop mode to keep re-publishing zeros
        if self.avoidance_mode == 'stop':
            self.stop_timer = self.create_timer(
                self.stop_publish_period_sec, self.stop_timer_callback
            )
        elif self.front_recovery_enabled:
            self.recovery_timer = self.create_timer(
                self.stop_publish_period_sec, self.recovery_timer_callback
            )

        self.get_logger().info('==================================================')
        self.get_logger().info('[OBSTACLE FILTER] Node started')
        self.get_logger().info(
            f'[OBSTACLE FILTER] avoidance_mode           = {self.avoidance_mode}'
        )
        self.get_logger().info(f'[OBSTACLE FILTER] enabled                  = {self.enabled}')
        self.get_logger().info(
            f'[OBSTACLE FILTER] input → output           = '
            f'{self.input_cmd_topic} → {self.output_cmd_topic}'
        )
        self.get_logger().info(f'[OBSTACLE FILTER] scan_topic               = {self.scan_topic}')
        self.get_logger().info(f'[OBSTACLE FILTER] odom_topic               = {self.odom_topic}')
        self.get_logger().info(
            f'[OBSTACLE FILTER] stop_distance_m          = {self.stop_distance_m:.3f}'
        )
        self.get_logger().info(
            f'[OBSTACLE FILTER] clear_distance_m         = {self.clear_distance_m:.3f}'
        )
        self.get_logger().info(
            f'[OBSTACLE FILTER] front_sector_half_angle  = '
            f'{math.degrees(self.front_sector_half_angle):.1f} deg'
        )
        self.get_logger().info(
            f'[OBSTACLE FILTER] lidar_in_base_yaw_deg    = '
            f'{self.lidar_in_base_yaw_deg:.1f}'
        )
        self.get_logger().info(
            f'[OBSTACLE FILTER] scan_angle_multiplier    = '
            f'{self.scan_angle_multiplier:.1f}'
        )
        if self.avoidance_mode == 'steer':
            self.get_logger().info(
                f'[OBSTACLE FILTER] avoidance_max_steer     = '
                f'{self.avoidance_max_steer_duty:.1f} duty'
            )
            self.get_logger().info(
                f'[OBSTACLE FILTER] steer_emergency_stop  = '
                f'{self.steer_emergency_stop_distance_m:.2f}m'
            )
            self.get_logger().info(
                f'[OBSTACLE FILTER] front_recovery        = '
                f'{self.front_recovery_enabled} '
                f'trigger={self.front_recovery_trigger_distance_m:.2f}m '
                f'center={math.degrees(self.front_recovery_center_half_angle):.1f}deg '
                f'backup={self.front_recovery_distance_m:.2f}m '
                f'duty={self.front_recovery_reverse_duty:.1f}'
            )
            self.get_logger().info(
                f'[OBSTACLE FILTER] pre_stop_decel          = '
                f'{self.pre_stop_decel_enabled} '
                f'scale={self.pre_stop_decel_scale:.2f} '
                f'distance={self.pre_stop_decel_distance_m:.2f}m'
            )
            self.get_logger().info(
                f'[OBSTACLE FILTER] side_sector_half_angle  = '
                f'{math.degrees(self.side_sector_half_angle):.1f} deg'
            )
            self.get_logger().info(
                f'[OBSTACLE FILTER] side_stop_distance_m    = {self.side_stop_distance_m:.3f}'
            )
        self.get_logger().info('==================================================')

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def _to_bool(self, value):
        if isinstance(value, str):
            return value.strip().lower() in ('true', '1', 'yes', 'on')
        return bool(value)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # LaserScan processing
    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        pose = msg.pose.pose.position
        self.latest_pose_xy = (float(pose.x), float(pose.y))

    def scan_callback(self, msg):
        front_min, left_min, right_min, closest_bearing, side_left_min, side_right_min = \
            self._front_sector_stats(msg)
        self.latest_front_min_m = front_min
        self.latest_left_min_m = left_min
        self.latest_right_min_m = right_min
        self.latest_side_left_min_m = side_left_min
        self.latest_side_right_min_m = side_right_min
        self.latest_closest_bearing_rad = closest_bearing
        self.latest_scan_time_sec = self.now_sec()

        advance = self.forward_speed_m_s * self.latency_compensation_sec
        pred_front = max(0.0, front_min - advance)

        if self.lidar_verbose:
            now = self.now_sec()
            if (
                self.last_lidar_log_sec is None
                or now - self.last_lidar_log_sec >= self.lidar_verbose_period_sec
            ):
                self.last_lidar_log_sec = now
                near = pred_front <= self.stop_distance_m
                self.get_logger().info(
                    f'[LIDAR] front={front_min:.3f}m (pred={pred_front:.3f}m)  '
                    f'L={left_min:.3f}m  R={right_min:.3f}m  '
                    f'sideL={side_left_min:.3f}m  sideR={side_right_min:.3f}m  '
                    f'bearing={math.degrees(closest_bearing):+.1f}°  '
                    f'thresh={self.stop_distance_m:.2f}m  '
                    f'{"NEAR" if near else "clear"}'
                )

        was_active = self.front_obstacle_active
        if pred_front <= self.stop_distance_m:
            self.front_obstacle_active = True
        elif pred_front >= self.clear_distance_m:
            self.front_obstacle_active = False

        if self.front_obstacle_active and not was_active:
            side = (
                'LEFT' if closest_bearing > 0.05
                else 'RIGHT' if closest_bearing < -0.05
                else 'CENTRE'
            )
            action = 'steering away' if self.avoidance_mode == 'steer' else 'STOPPING'
            self.get_logger().warn(
                f'[OBSTACLE FILTER] Obstacle {front_min:.3f}m at '
                f'{math.degrees(closest_bearing):+.1f}° ({side}) — {action}.'
            )
            if self.avoidance_mode == 'stop':
                self.publish_stop()
        elif was_active and not self.front_obstacle_active:
            self.get_logger().info(
                f'[OBSTACLE FILTER] Front sector clear ({front_min:.3f}m); resuming.'
            )

    def _front_sector_stats(self, msg):
        """Return (front_min, left_min, right_min, closest_bearing_rad, side_left_min, side_right_min).

        Bearing convention (ROS standard, sensor frame):
          positive = left of robot (CCW from forward)
          negative = right of robot (CW from forward)

        front sector : |bearing| <= front_sector_half_angle
        side sector  : front_sector_half_angle < |bearing| <= side_sector_half_angle
        """
        if msg.angle_increment == 0.0:
            return math.inf, math.inf, math.inf, 0.0, math.inf, math.inf

        front_min = math.inf
        left_min = math.inf
        right_min = math.inf
        side_left_min = math.inf
        side_right_min = math.inf
        closest_bearing = 0.0
        range_max = msg.range_max if msg.range_max > 0.0 else math.inf

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance <= 0.0 or distance > range_max:
                continue

            scan_angle = self.scan_angle_multiplier * (
                msg.angle_min + index * msg.angle_increment
            )
            # This LiDAR is mounted with its scan +x axis facing robot-backward.
            sensor_x = -math.cos(scan_angle)
            sensor_y = math.sin(scan_angle)
            base_x = (
                math.cos(self.lidar_in_base_yaw) * sensor_x
                - math.sin(self.lidar_in_base_yaw) * sensor_y
            )
            base_y = (
                math.sin(self.lidar_in_base_yaw) * sensor_x
                + math.cos(self.lidar_in_base_yaw) * sensor_y
            )
            angle = math.atan2(base_y, base_x)
            # Bearing relative to front_sector_center, wrapped to (-pi, pi]
            bearing = math.atan2(
                math.sin(angle - self.front_sector_center_angle),
                math.cos(angle - self.front_sector_center_angle),
            )

            abs_bearing = abs(bearing)

            if abs_bearing <= self.front_sector_half_angle:
                if distance < front_min:
                    front_min = distance
                    closest_bearing = bearing
                if bearing >= 0.0:
                    left_min = min(left_min, distance)
                else:
                    right_min = min(right_min, distance)
            elif abs_bearing <= self.side_sector_half_angle:
                if bearing > 0.0:
                    side_left_min = min(side_left_min, distance)
                else:
                    side_right_min = min(side_right_min, distance)

        return front_min, left_min, right_min, closest_bearing, side_left_min, side_right_min

    # ------------------------------------------------------------------
    # Steer avoidance
    # ------------------------------------------------------------------

    def _steer_correction(self):
        """Return (left_delta, right_delta) duty correction.

        Obstacle on the right (right_min < stop_distance_m) → steer left:
          left wheel gets negative delta (slower), right wheel positive (faster).
        Obstacle on the left → steer right: opposite signs.

        Front and side sectors are handled independently and their net corrections
        are summed, so a wall drifting in from the side is caught even when the
        forward path is still clear.
        """
        advance = self.forward_speed_m_s * self.latency_compensation_sec

        net = 0.0

        # --- front sector ---
        if self.stop_distance_m > 0.0:
            pred_left = max(0.0, self.latest_left_min_m - advance)
            pred_right = max(0.0, self.latest_right_min_m - advance)
            left_encroach = max(0.0, self.stop_distance_m - pred_left)
            right_encroach = max(0.0, self.stop_distance_m - pred_right)
            net += (right_encroach - left_encroach) / self.stop_distance_m

        # --- side sector ---
        if self.side_stop_distance_m > 0.0:
            pred_side_left = max(0.0, self.latest_side_left_min_m - advance)
            pred_side_right = max(0.0, self.latest_side_right_min_m - advance)
            side_left_encroach = max(0.0, self.side_stop_distance_m - pred_side_left)
            side_right_encroach = max(0.0, self.side_stop_distance_m - pred_side_right)
            net += (side_right_encroach - side_left_encroach) / self.side_stop_distance_m

        if net == 0.0:
            return 0.0, 0.0

        # Clamp net to [-2, 2] (two sectors can each contribute up to ±1)
        net = max(-2.0, min(2.0, net))
        correction = (net / 2.0) * self.avoidance_max_steer_duty

        # net > 0: right side closer → steer left → left_delta < 0, right_delta > 0
        return -correction, correction

    def _predicted_front_distance(self):
        advance = self.forward_speed_m_s * self.latency_compensation_sec
        return max(0.0, self.latest_front_min_m - advance)

    def _apply_pre_stop_decel(self, left, right):
        if (
            not self.pre_stop_decel_enabled
            or self.pre_stop_decel_distance_m <= 0.0
            or self.pre_stop_decel_scale >= 1.0
            or self._predicted_front_distance() > self.pre_stop_decel_distance_m
        ):
            return left, right, False

        forward = 0.5 * (float(left) + float(right))
        turn = 0.5 * (float(right) - float(left))
        if forward <= 0.0:
            return left, right, False

        forward *= self.pre_stop_decel_scale
        decel_left = max(-100.0, min(100.0, forward - turn))
        decel_right = max(-100.0, min(100.0, forward + turn))
        return decel_left, decel_right, True

    def _front_recovery_should_start(self):
        if (
            not self.front_recovery_enabled
            or self.front_recovery_trigger_distance_m <= 0.0
            or self.recovery_phase is not None
        ):
            return False
        if abs(self.latest_closest_bearing_rad) > self.front_recovery_center_half_angle:
            return False
        if self._predicted_front_distance() > self.front_recovery_trigger_distance_m:
            return False
        now = self.now_sec()
        return (
            self.last_recovery_end_sec is None
            or now - self.last_recovery_end_sec >= self.front_recovery_cooldown_sec
        )

    def _start_front_recovery(self):
        self.recovery_phase = 'reverse'
        self.recovery_start_time_sec = self.now_sec()
        self.recovery_start_xy = self.latest_pose_xy
        self.recovery_steer_direction = self._choose_recovery_steer_direction()
        self._log_front_recovery_start()

    def _choose_recovery_steer_direction(self):
        if self.latest_closest_bearing_rad > 0.05:
            return -1.0
        if self.latest_closest_bearing_rad < -0.05:
            return 1.0

        left_clearance = min(
            self._finite_clearance(self.latest_left_min_m),
            self._finite_clearance(self.latest_side_left_min_m),
        )
        right_clearance = min(
            self._finite_clearance(self.latest_right_min_m),
            self._finite_clearance(self.latest_side_right_min_m),
        )
        return 1.0 if left_clearance >= right_clearance else -1.0

    def _finite_clearance(self, value):
        if math.isfinite(value):
            return max(0.0, float(value))
        return 10.0

    def _front_recovery_reverse_complete(self):
        if self.recovery_phase != 'reverse':
            return False

        if self.recovery_start_time_sec is None:
            return True
        if self.now_sec() - self.recovery_start_time_sec >= self.front_recovery_timeout_sec:
            return True

        if self.recovery_start_xy is None or self.latest_pose_xy is None:
            return False
        dx = self.latest_pose_xy[0] - self.recovery_start_xy[0]
        dy = self.latest_pose_xy[1] - self.recovery_start_xy[1]
        return math.hypot(dx, dy) >= self.front_recovery_distance_m

    def _finish_front_recovery_reverse(self):
        self.recovery_phase = 'steer'
        now = self.now_sec()
        self.recovery_steer_until_sec = now + self.front_recovery_steer_duration_sec
        self.last_recovery_end_sec = now
        self._log_front_recovery_done()

    def _front_recovery_steer_active(self):
        return (
            self.recovery_phase == 'steer'
            and self.recovery_steer_until_sec is not None
            and self.now_sec() < self.recovery_steer_until_sec
        )

    def _finish_front_recovery_steer_if_done(self):
        if self.recovery_phase != 'steer':
            return
        if not self._front_recovery_steer_active():
            self.recovery_phase = None
            self.recovery_start_time_sec = None
            self.recovery_start_xy = None
            self.recovery_steer_until_sec = None

    def _recovery_reverse_command(self):
        duty = -self.front_recovery_reverse_duty
        return duty, duty

    def _apply_recovery_steer_bias(self, left, right):
        if not self._front_recovery_steer_active():
            return left, right, False
        bias = self.avoidance_max_steer_duty
        direction = self.recovery_steer_direction
        biased_left = max(-100.0, min(100.0, float(left) - direction * bias))
        biased_right = max(-100.0, min(100.0, float(right) + direction * bias))
        return biased_left, biased_right, True

    # ------------------------------------------------------------------
    # Command callbacks
    # ------------------------------------------------------------------

    def cmd_callback(self, msg):
        if not self.enabled:
            self.publish_command(msg.left, msg.right)
            return

        if self.avoidance_mode == 'steer':
            if self.recovery_phase == 'reverse':
                if self._front_recovery_reverse_complete():
                    self._finish_front_recovery_reverse()
                else:
                    self.publish_command(*self._recovery_reverse_command())
                    return
            self._finish_front_recovery_steer_if_done()
            if self._front_recovery_should_start():
                self._start_front_recovery()
                self.publish_command(*self._recovery_reverse_command())
                return
            if self._steer_emergency_stop_active():
                self._log_stop(
                    'steer emergency front obstacle '
                    f'{self.latest_front_min_m:.3f} m'
                )
                self.publish_stop()
                return
            left_delta, right_delta = self._steer_correction()
            left = max(-100.0, min(100.0, msg.left + left_delta))
            right = max(-100.0, min(100.0, msg.right + right_delta))
            left, right, recovery_steer_active = self._apply_recovery_steer_bias(left, right)
            left, right, decel_active = self._apply_pre_stop_decel(left, right)
            if abs(left_delta) > 0.1 or abs(right_delta) > 0.1:
                self._log_steer(left_delta, right_delta)
            elif recovery_steer_active:
                self._log_recovery_steer()
            elif decel_active:
                self._log_pre_stop_decel()
            self.publish_command(left, right)
            return

        # stop mode
        reason = self._stop_reason()
        if reason is None:
            self.publish_command(msg.left, msg.right)
        else:
            self._log_stop(reason)
            self.publish_stop()

    def stop_timer_callback(self):
        reason = self._stop_reason()
        if reason is not None:
            self._log_stop(reason)
            self.publish_stop()

    def recovery_timer_callback(self):
        if self.recovery_phase != 'reverse':
            return
        if self._front_recovery_reverse_complete():
            self._finish_front_recovery_reverse()
            return
        self.publish_command(*self._recovery_reverse_command())

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------

    def _log_steer(self, left_delta, right_delta):
        now = self.now_sec()
        if (
            self.last_status_log_sec is not None
            and now - self.last_status_log_sec < self.status_log_period_sec
        ):
            return
        self.last_status_log_sec = now
        direction = 'LEFT' if left_delta < 0 else 'RIGHT'
        self.get_logger().info(
            f'[OBSTACLE STEER] L={self.latest_left_min_m:.3f}m '
            f'R={self.latest_right_min_m:.3f}m  '
            f'→ nudge {direction}  Δ={abs(left_delta):.1f} duty'
        )

    def _log_pre_stop_decel(self):
        now = self.now_sec()
        if (
            self.last_status_log_sec is not None
            and now - self.last_status_log_sec < self.status_log_period_sec
        ):
            return
        self.last_status_log_sec = now
        self.get_logger().info(
            f'[OBSTACLE DECEL] front={self.latest_front_min_m:.3f}m '
            f'(pred={self._predicted_front_distance():.3f}m) '
            f'→ forward scale {self.pre_stop_decel_scale:.2f}'
        )

    def _log_front_recovery_start(self):
        now = self.now_sec()
        self.last_status_log_sec = now
        direction = 'LEFT' if self.recovery_steer_direction > 0.0 else 'RIGHT'
        self.get_logger().warn(
            f'[OBSTACLE RECOVERY] front={self.latest_front_min_m:.3f}m '
            f'(pred={self._predicted_front_distance():.3f}m), reversing '
            f'{self.front_recovery_distance_m:.2f}m then steering {direction}.'
        )

    def _log_front_recovery_done(self):
        now = self.now_sec()
        self.last_status_log_sec = now
        distance = 0.0
        if self.recovery_start_xy is not None and self.latest_pose_xy is not None:
            dx = self.latest_pose_xy[0] - self.recovery_start_xy[0]
            dy = self.latest_pose_xy[1] - self.recovery_start_xy[1]
            distance = math.hypot(dx, dy)
        self.get_logger().info(
            f'[OBSTACLE RECOVERY] reverse complete after {distance:.2f}m; '
            'resuming steer avoidance.'
        )

    def _log_recovery_steer(self):
        now = self.now_sec()
        if (
            self.last_status_log_sec is not None
            and now - self.last_status_log_sec < self.status_log_period_sec
        ):
            return
        self.last_status_log_sec = now
        direction = 'LEFT' if self.recovery_steer_direction > 0.0 else 'RIGHT'
        self.get_logger().info(
            f'[OBSTACLE RECOVERY] post-backup steer {direction} '
            f'Δ={self.avoidance_max_steer_duty:.1f} duty'
        )

    def _stop_reason(self):
        if not self.enabled:
            return None
        if self.front_obstacle_active:
            return f'front obstacle {self.latest_front_min_m:.3f} m'
        if not self.stop_on_stale_scan:
            return None
        if self.latest_scan_time_sec is None:
            return 'waiting for first LaserScan'
        scan_age = self.now_sec() - self.latest_scan_time_sec
        if scan_age > self.scan_timeout_sec:
            return f'LaserScan stale {scan_age:.2f} s'
        return None

    def _steer_emergency_stop_active(self):
        if self.front_recovery_enabled:
            return False
        return (
            self.steer_emergency_stop_distance_m > 0.0
            and self._predicted_front_distance()
            <= self.steer_emergency_stop_distance_m
        )

    def _log_stop(self, reason):
        now = self.now_sec()
        if (
            self.last_status_log_sec is not None
            and now - self.last_status_log_sec < self.status_log_period_sec
        ):
            return
        self.last_status_log_sec = now
        self.get_logger().warn(f'[OBSTACLE STOP] Holding zero command: {reason}.')

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def publish_stop(self):
        self.publish_command(0.0, 0.0)

    def publish_command(self, left_cmd, right_cmd):
        msg = LeftRightFloat32()
        msg.left = float(left_cmd)
        msg.right = float(right_cmd)
        msg.seq_num = self.output_seq_num
        self.cmd_pub.publish(msg)
        self.output_seq_num += 1


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopMotorFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
