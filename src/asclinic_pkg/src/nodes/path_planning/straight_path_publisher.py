#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


def set_yaw(pose, yaw: float) -> None:
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(0.5 * yaw)
    pose.orientation.w = math.cos(0.5 * yaw)


class StraightPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('straight_path_publisher')

        self.declare_parameter('path_topic', 'reference_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('point_spacing', 0.10)
        self.declare_parameter('publish_period_sec', 1.0)

        self.path_topic = str(self.get_parameter('path_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.point_spacing = max(0.02, float(self.get_parameter('point_spacing').value))
        publish_period = max(0.2, float(self.get_parameter('publish_period_sec').value))

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.timer = self.create_timer(publish_period, self.publish_path)
        self.published_once = False

        self.get_logger().info(
            'Straight path publisher ready: (%.2f, %.2f) -> (%.2f, %.2f) on %s'
            % (self.start_x, self.start_y, self.goal_x, self.goal_y, self.path_topic)
        )

    def publish_path(self) -> None:
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        distance = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(distance / self.point_spacing)))
        yaw = math.atan2(dy, dx) if distance > 1.0e-6 else 0.0

        for index in range(steps + 1):
            ratio = index / steps
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = self.start_x + ratio * dx
            pose.pose.position.y = self.start_y + ratio * dy
            set_yaw(pose.pose, yaw)
            path.poses.append(pose)

        self.path_pub.publish(path)
        if not self.published_once:
            self.get_logger().info('Published straight reference path with %d poses.' % len(path.poses))
            self.published_once = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
