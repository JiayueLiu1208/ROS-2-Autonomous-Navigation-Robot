#!/usr/bin/env python3
"""
test_trajectory.py  —  Pure-pursuit path follower

Converts a geometric path from global_planner directly into motor duty cycles.
No LQG, no Kalman filter — just geometry.

Nodes required to run the robot alongside this one
---------------------------------------------------
  ros2 run asclinic_pkg map.py                           (publishes /map)
  ros2 run asclinic_pkg global_planner.py                (publishes /planned_path)
  ros2 run asclinic_pkg liu_odometry_from_encoders.py    (publishes wheel_odometry)
  ros2 run asclinic_pkg roboclaw_for_motors.py           (drives motors + reads encoders)

Run this node
-------------
  ros2 run asclinic_pkg test_trajectory.py

Topic I/O
---------
  Subscribes:  /planned_path       nav_msgs/Path
  Subscribes:  wheel_odometry      nav_msgs/Odometry
  Publishes:   set_motor_duty_cycle  asclinic_pkg/LeftRightFloat32
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path as RosPath, Odometry
from asclinic_pkg.msg import LeftRightFloat32


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class TestTrajectoryNode(Node):

    def __init__(self):
        super().__init__('test_trajectory')

        # ── Robot geometry ─────────────────────────────────────────────────
        # Matches liu_odometry_from_encoders.py defaults
        self.declare_parameter('half_wheel_base', 0.109)   # m

        # ── Velocity / duty scaling ────────────────────────────────────────
        # v_ref     : cruise speed in m/s
        # v_max_mps : robot speed at 100 % duty cycle — calibrate on your floor
        # duty_limit: hard cap on duty cycle % sent to each wheel
        self.declare_parameter('v_ref',           0.20)
        self.declare_parameter('v_max_mps',       1.50)
        self.declare_parameter('duty_limit',      40.0)

        # ── Pure-pursuit tuning ────────────────────────────────────────────
        self.declare_parameter('lookahead_distance', 0.40)  # m
        self.declare_parameter('goal_tolerance',     0.15)  # m

        # ── Topics ─────────────────────────────────────────────────────────
        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('path_topic', 'reference_path')
        self.declare_parameter('motor_cmd_topic', 'set_motor_duty_cycle')

        self.half_wheel_base = float(self.get_parameter('half_wheel_base').value)
        self.v_ref           = float(self.get_parameter('v_ref').value)
        self.v_max_mps       = float(self.get_parameter('v_max_mps').value)
        self.duty_limit      = float(self.get_parameter('duty_limit').value)
        self.lookahead       = float(self.get_parameter('lookahead_distance').value)
        self.goal_tol        = float(self.get_parameter('goal_tolerance').value)
        odom_topic           = str(self.get_parameter('odom_topic').value)
        path_topic           = str(self.get_parameter('path_topic').value)
        motor_cmd_topic      = str(self.get_parameter('motor_cmd_topic').value)

        # ── State ──────────────────────────────────────────────────────────
        self.path: list[tuple[float, float]] = []   # (x, y) waypoints
        self.pose: tuple[float, float, float] | None = None
        self.path_index   = 0
        self.goal_reached = False

        # ── ROS ────────────────────────────────────────────────────────────
        self.create_subscription(RosPath,   path_topic, self.path_callback, 10)
        self.create_subscription(Odometry,  odom_topic, self.odom_callback, 10)
        self.motor_pub = self.create_publisher(LeftRightFloat32, motor_cmd_topic, 10)

        self.create_timer(0.05, self.control_loop)   # 20 Hz

        self.get_logger().info(
            f'test_trajectory started | odom={odom_topic} | path={path_topic} | '
            f'cmd={motor_cmd_topic} | v_ref={self.v_ref} m/s | '
            f'lookahead={self.lookahead} m'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def path_callback(self, msg: RosPath) -> None:
        self.path = [
            (ps.pose.position.x, ps.pose.position.y)
            for ps in msg.poses
        ]
        self.path_index   = 0
        self.goal_reached = False
        self.get_logger().info(f'New path received: {len(self.path)} waypoints')

    def odom_callback(self, msg: Odometry) -> None:
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(msg.pose.pose.orientation),
        )

    # ── Control loop (20 Hz) ───────────────────────────────────────────────

    def control_loop(self) -> None:
        if self.pose is None or not self.path or self.goal_reached:
            return

        rx, ry, ryaw = self.pose

        # ── 1. Stop when close enough to the last waypoint ─────────────────
        gx, gy = self.path[-1]
        if math.hypot(gx - rx, gy - ry) < self.goal_tol:
            self.goal_reached = True
            self._stop()
            self.get_logger().info('Goal reached — stopping')
            return

        # ── 2. Advance past waypoints already behind us ────────────────────
        while self.path_index < len(self.path) - 1:
            wx, wy = self.path[self.path_index]
            if math.hypot(wx - rx, wy - ry) < self.lookahead * 0.5:
                self.path_index += 1
            else:
                break

        # ── 3. Find lookahead waypoint ─────────────────────────────────────
        target_idx = len(self.path) - 1
        for i in range(self.path_index, len(self.path)):
            wx, wy = self.path[i]
            if math.hypot(wx - rx, wy - ry) >= self.lookahead:
                target_idx = i
                break

        tx, ty = self.path[target_idx]

        # ── 4. Pure-pursuit steering ────────────────────────────────────────
        # alpha = angle from robot heading to the lookahead point
        dx    = tx - rx
        dy    = ty - ry
        dist  = math.hypot(dx, dy)
        alpha = wrap_angle(math.atan2(dy, dx) - ryaw)

        # Curvature κ = 2·sin(α) / lookahead_dist  →  ω = v · κ
        omega = 2.0 * self.v_ref * math.sin(alpha) / max(dist, 0.01)

        # ── 5. Differential drive → wheel velocities ────────────────────────
        #   v_left  = v - half_base · ω
        #   v_right = v + half_base · ω
        v_left  = self.v_ref - self.half_wheel_base * omega
        v_right = self.v_ref + self.half_wheel_base * omega

        # ── 6. Wheel velocity → duty cycle % ───────────────────────────────
        #   duty = (v_wheel / v_max) * 100
        duty_left  = max(-self.duty_limit,
                         min(self.duty_limit, (v_left  / self.v_max_mps) * 100.0))
        duty_right = max(-self.duty_limit,
                         min(self.duty_limit, (v_right / self.v_max_mps) * 100.0))

        # ── 7. Send to motor driver ─────────────────────────────────────────
        cmd = LeftRightFloat32()
        cmd.left  = duty_left
        cmd.right = duty_right
        self.motor_pub.publish(cmd)

        self.get_logger().debug(
            f'target=({tx:.2f},{ty:.2f}) alpha={math.degrees(alpha):.1f}° '
            f'ω={omega:.3f} L={duty_left:.1f}% R={duty_right:.1f}%'
        )

    def _stop(self) -> None:
        self.motor_pub.publish(LeftRightFloat32())


def main(args=None):
    rclpy.init(args=args)
    node = TestTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
