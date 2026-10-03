#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def set_yaw(pose, yaw: float) -> None:
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(0.5 * yaw)
    pose.orientation.w = math.cos(0.5 * yaw)


class ArcPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('arc_path_publisher')

        self.declare_parameter('path_topic', 'reference_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 2.0)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('start_angle', -math.pi / 2.0)
        self.declare_parameter('end_angle', math.pi / 2.0)
        self.declare_parameter('clockwise', False)
        self.declare_parameter('point_spacing', 0.10)
        self.declare_parameter('publish_period_sec', 1.0)

        self.path_topic = str(self.get_parameter('path_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.radius = max(0.05, abs(float(self.get_parameter('radius').value)))
        self.start_angle = float(self.get_parameter('start_angle').value)
        self.end_angle = float(self.get_parameter('end_angle').value)
        self.clockwise = as_bool(self.get_parameter('clockwise').value)
        self.point_spacing = max(0.02, float(self.get_parameter('point_spacing').value))
        publish_period = max(0.2, float(self.get_parameter('publish_period_sec').value))

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.timer = self.create_timer(publish_period, self.publish_path)
        self.published_once = False

        self.get_logger().info(
            'Arc path publisher ready: center=(%.2f, %.2f) radius=%.2f '
            'angles=(%.2f -> %.2f) clockwise=%s on %s'
            % (
                self.center_x,
                self.center_y,
                self.radius,
                self.start_angle,
                self.end_angle,
                self.clockwise,
                self.path_topic,
            )
        )

    def publish_path(self) -> None:
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        start_angle = self.start_angle
        end_angle = self.end_angle
        if self.clockwise:
            while end_angle > start_angle:
                end_angle -= 2.0 * math.pi
            tangent_offset = -math.pi / 2.0
        else:
            while end_angle < start_angle:
                end_angle += 2.0 * math.pi
            tangent_offset = math.pi / 2.0

        angle_span = end_angle - start_angle
        arc_length = abs(angle_span) * self.radius
        steps = max(1, int(math.ceil(arc_length / self.point_spacing)))

        for index in range(steps + 1):
            ratio = index / steps
            theta = start_angle + ratio * angle_span
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = self.center_x + self.radius * math.cos(theta)
            pose.pose.position.y = self.center_y + self.radius * math.sin(theta)
            set_yaw(pose.pose, theta + tangent_offset)
            path.poses.append(pose)

        self.path_pub.publish(path)
        if not self.published_once:
            start = path.poses[0].pose.position
            goal = path.poses[-1].pose.position
            self.get_logger().info(
                'Published arc reference path with %d poses: '
                '(%.2f, %.2f) -> (%.2f, %.2f).'
                % (len(path.poses), start.x, start.y, goal.x, goal.y)
            )
            self.published_once = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArcPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
