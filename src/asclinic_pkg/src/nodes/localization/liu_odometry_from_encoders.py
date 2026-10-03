#!/usr/bin/env python3

import os
import csv
import math
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

from asclinic_pkg.msg import LeftRightInt32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class LiuOdometryFromEncoders(Node):
    def __init__(self):
        super().__init__('liu_odometry_from_encoders')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('wheel_radius_left', 0.072)
        self.declare_parameter('wheel_radius_right', 0.072)
        self.declare_parameter('half_wheel_base', 0.109)
        self.declare_parameter('counts_per_revolution', 4480.0)

        self.declare_parameter('k_l', 5.15e-5)
        self.declare_parameter('k_r', 2.39e-4)

        self.declare_parameter('x_initial', 0.0)
        self.declare_parameter('y_initial', 0.0)
        self.declare_parameter('phi_initial', 0.0)

        self.declare_parameter('publish_odom_topic', 'wheel_odometry')
        self.declare_parameter('save_directory', '~/asclinic-ros2/ros2_ws/results/odom_logs')
        self.declare_parameter('verbose', True)

        self.wheel_radius_left = float(self.get_parameter('wheel_radius_left').value)
        self.wheel_radius_right = float(self.get_parameter('wheel_radius_right').value)
        self.half_wheel_base = float(self.get_parameter('half_wheel_base').value)
        self.counts_per_revolution = float(self.get_parameter('counts_per_revolution').value)

        self.k_l = float(self.get_parameter('k_l').value)
        self.k_r = float(self.get_parameter('k_r').value)

        self.x_p = float(self.get_parameter('x_initial').value)
        self.y_p = float(self.get_parameter('y_initial').value)
        self.phi = float(self.get_parameter('phi_initial').value)

        self.verbose = bool(self.get_parameter('verbose').value)

        # -----------------------------
        # Initial pose covariance
        # -----------------------------
        self.pose_covariance = np.zeros((3, 3), dtype=float)
        self.pose_covariance[0, 0] = (0.01) ** 2
        self.pose_covariance[1, 1] = (0.01) ** 2
        self.pose_covariance[2, 2] = (1.0 * math.pi / 180.0) ** 2

        # Time tracking
        self.prev_time = None
        self.start_time_sec = None
        self.elapsed_time_sec = 0.0

        # -----------------------------
        # CSV setup
        # -----------------------------
        self.save_directory = os.path.expanduser(
            str(self.get_parameter('save_directory').value)
        )
        os.makedirs(self.save_directory, exist_ok=True)

        run_tag = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.trace_csv_path = os.path.join(self.save_directory, f'odom_trace_{run_tag}.csv')
        self.summary_csv_path = os.path.join(self.save_directory, f'odom_final_summary_{run_tag}.csv')

        self.trace_csv_file = open(self.trace_csv_path, 'w', newline='', encoding='utf-8-sig')
        self.trace_writer = csv.writer(self.trace_csv_file, delimiter=';')

        self.trace_writer.writerow([
            'time_sec',
            'x_odom',
            'y_odom',
            'yaw_odom',
            'pose_cov_xx',
            'pose_cov_yy',
            'pose_cov_yawyaw',
            'delta_theta_l',
            'delta_theta_r',
            'delta_s',
            'delta_phi'
        ])
        self.trace_csv_file.flush()

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.encoder_sub = self.create_subscription(
            LeftRightInt32,
            'encoder_counts',
            self.encoder_callback,
            10
        )

        odom_topic = str(self.get_parameter('publish_odom_topic').value)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('========================================')
        self.get_logger().info('[ODOM] Node started')
        self.get_logger().info(f'[ODOM] wheel_radius_left   = {self.wheel_radius_left}')
        self.get_logger().info(f'[ODOM] wheel_radius_right  = {self.wheel_radius_right}')
        self.get_logger().info(f'[ODOM] half_wheel_base     = {self.half_wheel_base}')
        self.get_logger().info(f'[ODOM] counts_per_rev      = {self.counts_per_revolution}')
        self.get_logger().info(f'[ODOM] k_l                 = {self.k_l}')
        self.get_logger().info(f'[ODOM] k_r                 = {self.k_r}')
        self.get_logger().info(f'[ODOM] Publishing odometry on topic: {odom_topic}')
        self.get_logger().info(f'[ODOM] Saving trace CSV to: {self.trace_csv_path}')
        self.get_logger().info(f'[ODOM] Saving summary CSV to: {self.summary_csv_path}')
        self.get_logger().info('========================================')

    def wrap_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def counts_to_radians(self, delta_counts):
        return float(delta_counts) * 2.0 * math.pi / self.counts_per_revolution

    def build_ros_pose_covariance(self):
        cov6 = np.zeros((6, 6), dtype=float)
        idx = [0, 1, 5]  # x, y, yaw
        for i in range(3):
            for j in range(3):
                cov6[idx[i], idx[j]] = self.pose_covariance[i, j]
        return cov6.reshape(-1).tolist()

    def save_trace_row(self, delta_theta_l, delta_theta_r, delta_s, delta_phi):
        self.trace_writer.writerow([
            self.elapsed_time_sec,
            self.x_p,
            self.y_p,
            self.phi,
            self.pose_covariance[0, 0],
            self.pose_covariance[1, 1],
            self.pose_covariance[2, 2],
            delta_theta_l,
            delta_theta_r,
            delta_s,
            delta_phi
        ])
        self.trace_csv_file.flush()

    def save_summary_csv(self):
        with open(self.summary_csv_path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f, delimiter=';')
            writer.writerow([
                'time_final_sec',
                'x_odom_final',
                'y_odom_final',
                'yaw_odom_final',
                'pose_cov_xx',
                'pose_cov_yy',
                'pose_cov_yawyaw'
            ])
            writer.writerow([
                self.elapsed_time_sec,
                self.x_p,
                self.y_p,
                self.phi,
                self.pose_covariance[0, 0],
                self.pose_covariance[1, 1],
                self.pose_covariance[2, 2]
            ])

    def encoder_callback(self, msg):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        if self.start_time_sec is None:
            self.start_time_sec = now_sec

        self.elapsed_time_sec = now_sec - self.start_time_sec

        # Convert encoder delta counts to radians
        delta_theta_l = self.counts_to_radians(msg.left)
        delta_theta_r = self.counts_to_radians(msg.right)

        # Real robot: use encoder measurements directly
        delta_theta_l_meas = delta_theta_l
        delta_theta_r_meas = delta_theta_r

        # Encoder measurement covariance
        theta_covariance = np.zeros((2, 2), dtype=float)
        theta_covariance[0, 0] = self.k_l * abs(delta_theta_l_meas)
        theta_covariance[1, 1] = self.k_r * abs(delta_theta_r_meas)

        # Odometry increments
        delta_s_l = self.wheel_radius_left * delta_theta_l_meas
        delta_s_r = self.wheel_radius_right * delta_theta_r_meas

        delta_s = 0.5 * (delta_s_l + delta_s_r)
        delta_phi = (delta_s_r - delta_s_l) / (2.0 * self.half_wheel_base)

        psi = self.phi + 0.5 * delta_phi

        # Pose update
        x_new = self.x_p + delta_s * math.cos(psi)
        y_new = self.y_p + delta_s * math.sin(psi)
        phi_new = self.wrap_angle(self.phi + delta_phi)

        # Jacobian wrt previous pose
        F = np.array([
            [1.0, 0.0, -delta_s * math.sin(psi)],
            [0.0, 1.0,  delta_s * math.cos(psi)],
            [0.0, 0.0,  1.0]
        ], dtype=float)

        # Jacobian wrt wheel angle increments
        dDs_dthl = self.wheel_radius_left / 2.0
        dDs_dthr = self.wheel_radius_right / 2.0

        dDphi_dthl = -self.wheel_radius_left / (2.0 * self.half_wheel_base)
        dDphi_dthr =  self.wheel_radius_right / (2.0 * self.half_wheel_base)

        dpsi_dthl = 0.5 * dDphi_dthl
        dpsi_dthr = 0.5 * dDphi_dthr

        dx_dthl = dDs_dthl * math.cos(psi) - delta_s * math.sin(psi) * dpsi_dthl
        dx_dthr = dDs_dthr * math.cos(psi) - delta_s * math.sin(psi) * dpsi_dthr

        dy_dthl = dDs_dthl * math.sin(psi) + delta_s * math.cos(psi) * dpsi_dthl
        dy_dthr = dDs_dthr * math.sin(psi) + delta_s * math.cos(psi) * dpsi_dthr

        dphi_dthl = dDphi_dthl
        dphi_dthr = dDphi_dthr

        G = np.array([
            [dx_dthl,   dx_dthr],
            [dy_dthl,   dy_dthr],
            [dphi_dthl, dphi_dthr]
        ], dtype=float)

        # Covariance propagation
        pose_covariance_new = F @ self.pose_covariance @ F.T + G @ theta_covariance @ G.T

        # Save new state
        self.x_p = x_new
        self.y_p = y_new
        self.phi = phi_new
        self.pose_covariance = pose_covariance_new

        # Save one row per encoder callback
        self.save_trace_row(
            delta_theta_l_meas,
            delta_theta_r_meas,
            delta_s,
            delta_phi
        )

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x_p
        odom_msg.pose.pose.position.y = self.y_p
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.phi)
        odom_msg.pose.covariance = self.build_ros_pose_covariance()

        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt > 1e-6:
                v = delta_s / dt
                w = delta_phi / dt
            else:
                v = 0.0
                w = 0.0
        else:
            v = 0.0
            w = 0.0

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = w

        self.odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x_p
        tf_msg.transform.translation.y = self.y_p
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = yaw_to_quaternion(self.phi)
        self.tf_broadcaster.sendTransform(tf_msg)

        self.prev_time = now

        if self.verbose:
            self.get_logger().info(
                f'[ODOM] t={self.elapsed_time_sec:.3f}s, '
                f'x={self.x_p:.4f}, y={self.y_p:.4f}, phi={self.phi:.4f}, '
                f'cov_xx={self.pose_covariance[0,0]:.6e}, '
                f'cov_yy={self.pose_covariance[1,1]:.6e}, '
                f'cov_yawyaw={self.pose_covariance[2,2]:.6e}'
            )

    def destroy_node(self):
        try:
            self.save_summary_csv()
        except Exception as e:
            self.get_logger().warn(f'[ODOM] Failed to save summary CSV: {e}')

        try:
            if not self.trace_csv_file.closed:
                self.trace_csv_file.close()
        except Exception:
            pass

        self.get_logger().info(
            f'[ODOM] Final saved: x={self.x_p:.4f}, y={self.y_p:.4f}, yaw={self.phi:.4f}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiuOdometryFromEncoders()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[ODOM] Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
