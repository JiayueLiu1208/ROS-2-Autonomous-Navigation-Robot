#!/usr/bin/env python3

"""
Mock wheel odometry publisher for testing fusion localization
without real hardware (motor controller/encoders).

Publishes simulated wheel odometry to /asc/wheel_odometry
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TwistWithCovariance
from std_msgs.msg import Header
import math
import numpy as np


class MockOdometryPublisher(Node):
    def __init__(self):
        super().__init__('mock_odometry_publisher')
        
        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            'wheel_odometry',
            10
        )
        
        # Timer to publish at ~10 Hz
        self.timer = self.create_timer(0.1, self.publish_odometry)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.seq = 0
        
        # Simulation parameters
        self.linear_vel = 0.3  # m/s (forward)
        self.angular_vel = 0.0  # rad/s
        
        self.get_logger().info('[MOCK ODOMETRY] Node started')
        self.get_logger().info(f'[MOCK ODOMETRY] Publishing to wheel_odometry at 10 Hz')
        self.get_logger().info(f'[MOCK ODOMETRY] Initial linear velocity: {self.linear_vel} m/s')
        self.get_logger().info(f'[MOCK ODOMETRY] Initial angular velocity: {self.angular_vel} rad/s')
    
    def publish_odometry(self):
        """Publish mock odometry data with simple kinematic model"""
        
        # Simple kinematic update: x, y, theta
        dt = 0.1
        
        # Update position
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt
        self.theta += self.angular_vel * dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create quaternion from yaw
        quat = self.yaw_to_quaternion(self.theta)
        
        # Create Odometry message
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quat
        
        # Covariance (3x3 for pose, but stored as 6x6)
        msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]
        
        # Velocity
        msg.twist.twist.linear.x = self.linear_vel
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.angular_vel
        
        # Twist covariance
        msg.twist.covariance = [0.0] * 36
        
        # Publish
        self.odom_pub.publish(msg)
        
        # Log occasionally
        if self.seq % 20 == 0:  # Every 2 seconds
            self.get_logger().info(
                f'[MOCK ODOMETRY] x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f} rad'
            )
        
        self.seq += 1
    
    @staticmethod
    def yaw_to_quaternion(yaw):
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat


def main(args=None):
    rclpy.init(args=args)
    node = MockOdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
