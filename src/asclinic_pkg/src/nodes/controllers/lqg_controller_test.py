#!/usr/bin/env python3

import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node

_NODES_DIR = Path(__file__).resolve().parents[1]
if str(_NODES_DIR) not in sys.path:
    sys.path.insert(0, str(_NODES_DIR))

from asclinic_pkg.msg import LeftRightFloat32          # noqa: E402
from controllers.lqg_controller import LQGController   # noqa: E402


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LQGControllerTestNode(Node):
    """
    Full pipeline: reference path + odometry  ->  LQG  ->  motor duty

    Subscribes
    ----------
    <path_topic>            nav_msgs/Path          geometric path from global_planner
    <odom_topic>            nav_msgs/Odometry      fused or wheel odometry

    Publishes
    ---------
    set_motor_duty_cycle    asclinic_pkg/LeftRightFloat32   duty cycle % sent to roboclaw
    """

    def __init__(self):
        super().__init__('lqg_controller_test')

        # ── Robot geometry (match liu_odometry_from_encoders.py defaults) ──
        self.declare_parameter('wheel_radius',    0.072)   # m
        self.declare_parameter('half_wheel_base', 0.109)   # m

        # ── Velocity → duty cycle scaling ──────────────────────────────────
        # Tune v_max_mps to the forward speed (m/s) at 100 % duty cycle.
        # At 20 % duty the robot moves roughly 0.25–0.35 m/s on a flat floor,
        # so 100 % ≈ 1.5 m/s is a reasonable starting point.
        self.declare_parameter('v_max_mps',       1.50)
        self.declare_parameter('duty_cycle_limit', 60.0)   # hard cap % per wheel
        self.declare_parameter('min_moving_duty', 0.0)
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 1.0)
        self.declare_parameter('duty_slew_rate_percent_per_sec', 0.0)
        self.declare_parameter('command_filter_alpha', 0.0)

        # ── Path tracking ──────────────────────────────────────────────────
        self.declare_parameter('v_ref',            0.20)   # m/s cruise speed
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 1.8)
        self.declare_parameter('lookahead_distance', 0.40) # m pure-pursuit lookahead
        self.declare_parameter('goal_tolerance',   0.15)   # m stop radius

        # ── Control timing ─────────────────────────────────────────────────
        self.declare_parameter('control_period',   0.05)   # s  → 20 Hz

        # ── Topics ─────────────────────────────────────────────────────────
        self.declare_parameter('odom_topic', 'pitt_fused_odometry')
        self.declare_parameter('path_topic', 'reference_path')
        self.declare_parameter('motor_cmd_topic', 'set_motor_duty_cycle')
        self.declare_parameter('verbose', False)

        self.wheel_radius    = float(self.get_parameter('wheel_radius').value)
        self.half_wheel_base = float(self.get_parameter('half_wheel_base').value)
        self.v_max_mps       = float(self.get_parameter('v_max_mps').value)
        self.duty_limit      = float(self.get_parameter('duty_cycle_limit').value)
        self.min_moving_duty = abs(float(self.get_parameter('min_moving_duty').value))
        self.left_trim       = float(self.get_parameter('left_trim').value)
        self.right_trim      = float(self.get_parameter('right_trim').value)
        self.duty_slew_rate  = max(
            0.0, float(self.get_parameter('duty_slew_rate_percent_per_sec').value)
        )
        self.command_filter_alpha = min(
            0.98,
            max(0.0, float(self.get_parameter('command_filter_alpha').value)),
        )
        self.v_ref           = float(self.get_parameter('v_ref').value)
        self.max_linear_speed = abs(float(self.get_parameter('max_linear_speed').value))
        self.max_angular_speed = abs(float(self.get_parameter('max_angular_speed').value))
        self.lookahead       = float(self.get_parameter('lookahead_distance').value)
        self.goal_tol        = float(self.get_parameter('goal_tolerance').value)
        self.dt              = float(self.get_parameter('control_period').value)
        odom_topic           = str(self.get_parameter('odom_topic').value)
        path_topic           = str(self.get_parameter('path_topic').value)
        motor_cmd_topic      = str(self.get_parameter('motor_cmd_topic').value)
        self.verbose         = self._as_bool(self.get_parameter('verbose').value)
        self.min_moving_duty = min(self.min_moving_duty, self.duty_limit)

        # ── LQG ────────────────────────────────────────────────────────────
        # State: error in robot body frame [x_e, y_e, theta_e]
        # Input: velocity correction       [delta_v, delta_omega]
        # Q_lqr weights penalise error; R_lqr weights penalise control effort.
        # Qn / Rn are Kalman filter process / measurement noise covariances.
        Q_lqr = np.diag([5.0,  5.0,  1.0])   # punish x, y, heading error
        R_lqr = np.diag([1.0,  0.5])          # penalise speed / turn corrections
        Qn    = np.diag([1e-2, 1e-2, 1e-2])   # process noise
        Rn    = np.diag([5e-2, 5e-2, 2e-2])   # measurement noise

        self.lqg = LQGController(
            dt=self.dt,
            Q_lqr=Q_lqr,
            R_lqr=R_lqr,
            Qn=Qn,
            Rn=Rn,
            u_min=[-0.3, -1.5],
            u_max=[ 0.3,  1.5],
        )

        # ── Internal state ─────────────────────────────────────────────────
        self.path: list[tuple[float, float, float]] = []
        self.pose: tuple[float, float, float] | None = None
        self.path_index   = 0
        self.goal_reached = False
        self.last_left_cmd = 0.0
        self.last_right_cmd = 0.0

        # ── ROS ────────────────────────────────────────────────────────────
        self.path_sub = self.create_subscription(
            RosPath, path_topic, self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.motor_pub = self.create_publisher(
            LeftRightFloat32, motor_cmd_topic, 10
        )

        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'LQG controller test started | odom={odom_topic} | path={path_topic} | '
            f'cmd={motor_cmd_topic} | '
            f'v_ref={self.v_ref:.2f} m/s | lookahead={self.lookahead:.2f} m | '
            f'v_max={self.v_max_mps:.2f} m/s -> 100 % duty'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def path_callback(self, msg: RosPath) -> None:
        self.path = []
        for ps in msg.poses:
            x   = ps.pose.position.x
            y   = ps.pose.position.y
            yaw = quaternion_to_yaw(ps.pose.orientation)
            self.path.append((x, y, yaw))

        self.path_index   = 0
        self.goal_reached = False
        self.lqg.reset()
        self.get_logger().info(f'New path received: {len(self.path)} waypoints')

    def odom_callback(self, msg: Odometry) -> None:
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pose = (x, y, yaw)

    # ── Control loop ───────────────────────────────────────────────────────

    def control_loop(self) -> None:
        if self.pose is None or not self.path or self.goal_reached:
            return

        rx, ry, ryaw = self.pose

        # Stop when within goal_tolerance of the last waypoint
        gx, gy, _ = self.path[-1]
        if math.hypot(gx - rx, gy - ry) < self.goal_tol:
            self.goal_reached = True
            self._publish_stop()
            self.get_logger().info('Goal reached — stopping')
            return

        # ── Step 1: advance past waypoints already behind us ───────────────
        while self.path_index < len(self.path) - 1:
            wx, wy, _ = self.path[self.path_index]
            if math.hypot(wx - rx, wy - ry) < self.lookahead * 0.5:
                self.path_index += 1
            else:
                break

        # ── Step 2: pure-pursuit — find lookahead target waypoint ──────────
        target_idx = len(self.path) - 1
        for i in range(self.path_index, len(self.path)):
            wx, wy, _ = self.path[i]
            if math.hypot(wx - rx, wy - ry) >= self.lookahead:
                target_idx = i
                break

        ref_x, ref_y, ref_yaw = self.path[target_idx]

        # ── Step 3: reference velocities from pure-pursuit geometry ────────
        # alpha = angle from robot heading to the lookahead point
        dx    = ref_x - rx
        dy    = ref_y - ry
        dist  = math.hypot(dx, dy)
        alpha = wrap_angle(math.atan2(dy, dx) - ryaw)

        v_ref     = self.v_ref
        # curvature κ = 2·sin(α)/L  →  ω = v·κ
        omega_ref = 2.0 * v_ref * math.sin(alpha) / max(dist, 0.01)

        # ── Step 4: error state in robot body frame ─────────────────────────
        # Rotate world-frame displacement into robot frame
        x_e     =  math.cos(ryaw) * dx + math.sin(ryaw) * dy
        y_e     = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        theta_e =  wrap_angle(ref_yaw - ryaw)

        # ── Step 5: LQG correction ─────────────────────────────────────────
        # The lookahead errors above point from the robot to the reference.
        # The linear error model uses actual-minus-reference, so flip signs.
        delta_u, _ = self.lqg.step([-x_e, -y_e, -theta_e], v_ref, omega_ref)

        # ── Step 6: final velocity command ─────────────────────────────────
        v_cmd = float(np.clip(v_ref + delta_u[0], -self.max_linear_speed, self.max_linear_speed))
        omega_cmd = float(
            np.clip(omega_ref + delta_u[1], -self.max_angular_speed, self.max_angular_speed)
        )

        # ── Step 7: differential drive → individual wheel velocities ───────
        #   v_left  = v - half_base * omega
        #   v_right = v + half_base * omega
        v_left  = v_cmd - self.half_wheel_base * omega_cmd
        v_right = v_cmd + self.half_wheel_base * omega_cmd

        # ── Step 8: wheel velocity → duty cycle % ──────────────────────────
        #   duty = (v_wheel / v_max) * 100 %
        duty_left = self._wheel_speed_to_duty(v_left) * self.left_trim
        duty_right = self._wheel_speed_to_duty(v_right) * self.right_trim

        duty_left  = float(np.clip(duty_left,  -self.duty_limit, self.duty_limit))
        duty_right = float(np.clip(duty_right, -self.duty_limit, self.duty_limit))
        duty_left, duty_right = self._apply_command_filter(duty_left, duty_right)
        duty_left, duty_right = self._apply_slew_limit(duty_left, duty_right)

        # ── Step 9: publish to motor driver ────────────────────────────────
        self._publish_motor_command(duty_left, duty_right)

        if self.verbose:
            self.get_logger().info(
                f'target=({ref_x:.2f},{ref_y:.2f}) '
                f'e=[{x_e:.3f} {y_e:.3f} {math.degrees(theta_e):.1f}deg] '
                f'v={v_cmd:.3f} omega={omega_cmd:.3f} '
                f'duty L={duty_left:.1f}% R={duty_right:.1f}%'
            )

    def _publish_stop(self) -> None:
        self._publish_motor_command(0.0, 0.0)
        self.last_left_cmd = 0.0
        self.last_right_cmd = 0.0

    def _publish_motor_command(self, left: float, right: float) -> None:
        cmd = LeftRightFloat32()
        cmd.left = float(left)
        cmd.right = float(right)
        self.motor_pub.publish(cmd)
        self.last_left_cmd = float(left)
        self.last_right_cmd = float(right)

    def _wheel_speed_to_duty(self, speed_mps: float) -> float:
        if abs(speed_mps) < 1.0e-4:
            return 0.0

        duty = (speed_mps / max(self.v_max_mps, 1.0e-4)) * 100.0
        if self.min_moving_duty > 0.0 and abs(duty) < self.min_moving_duty:
            duty = math.copysign(self.min_moving_duty, duty)
        return duty

    def _apply_command_filter(self, left: float, right: float) -> tuple[float, float]:
        if self.command_filter_alpha <= 0.0:
            return left, right

        alpha = self.command_filter_alpha
        return (
            alpha * self.last_left_cmd + (1.0 - alpha) * left,
            alpha * self.last_right_cmd + (1.0 - alpha) * right,
        )

    def _apply_slew_limit(self, left: float, right: float) -> tuple[float, float]:
        if self.duty_slew_rate <= 0.0:
            return left, right

        max_delta = self.duty_slew_rate * self.dt
        return (
            self.last_left_cmd + float(np.clip(left - self.last_left_cmd, -max_delta, max_delta)),
            self.last_right_cmd + float(np.clip(right - self.last_right_cmd, -max_delta, max_delta)),
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = LQGControllerTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
