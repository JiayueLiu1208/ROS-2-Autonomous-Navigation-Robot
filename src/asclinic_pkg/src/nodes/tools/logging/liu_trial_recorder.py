#!/usr/bin/env python3

import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class LiuTrialRecorder(Node):
    def __init__(self):
        super().__init__('liu_trial_recorder')

        # Parameters
        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('save_root', '~/asclinic-ros2/ros2_ws/results')
        self.declare_parameter('experiment_name', 'straight_0p5m')
        self.declare_parameter('trial_id', 1)
        self.declare_parameter('verbose', True)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.save_root = os.path.expanduser(str(self.get_parameter('save_root').value))
        self.experiment_name = str(self.get_parameter('experiment_name').value)
        self.trial_id = int(self.get_parameter('trial_id').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        # Runtime state
        self.start_time_sec = None
        self.last_time_sec = 0.0
        self.received_any = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.cov_xx = 0.0
        self.cov_yy = 0.0
        self.cov_yawyaw = 0.0

        # Prepare directory
        self.experiment_dir = os.path.join(self.save_root, self.experiment_name)
        os.makedirs(self.experiment_dir, exist_ok=True)

        # Unique file names so nothing is overwritten
        run_tag = datetime.now().strftime('%Y%m%d_%H%M%S')
        trial_tag = f'trial_{self.trial_id:02d}'

        self.trace_csv_path = os.path.join(
            self.experiment_dir,
            f'{trial_tag}_trace_{run_tag}.csv'
        )
        self.summary_csv_path = os.path.join(
            self.experiment_dir,
            f'{trial_tag}_summary_{run_tag}.csv'
        )

        # Open trace CSV
        self.trace_csv_file = open(self.trace_csv_path, 'w', newline='', encoding='utf-8-sig')
        self.trace_writer = csv.writer(self.trace_csv_file)
        self.trace_writer.writerow([
            'time_sec',
            'x_odom',
            'y_odom',
            'yaw_odom',
            'pose_cov_xx',
            'pose_cov_yy',
            'pose_cov_yawyaw'
        ])
        self.trace_csv_file.flush()

        # ROS subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info('========================================')
        self.get_logger().info('[RECORDER] Node started')
        self.get_logger().info(f'[RECORDER] odom_topic      = {self.odom_topic}')
        self.get_logger().info(f'[RECORDER] experiment_name = {self.experiment_name}')
        self.get_logger().info(f'[RECORDER] trial_id        = {self.trial_id}')
        self.get_logger().info(f'[RECORDER] trace file      = {self.trace_csv_path}')
        self.get_logger().info(f'[RECORDER] summary file    = {self.summary_csv_path}')
        self.get_logger().info('========================================')

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.start_time_sec is None:
            self.start_time_sec = now_sec

        t = now_sec - self.start_time_sec
        self.last_time_sec = t
        self.received_any = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        cov = msg.pose.covariance
        self.cov_xx = cov[0]
        self.cov_yy = cov[7]
        self.cov_yawyaw = cov[35]

        self.trace_writer.writerow([
            t,
            self.x,
            self.y,
            self.yaw,
            self.cov_xx,
            self.cov_yy,
            self.cov_yawyaw
        ])
        self.trace_csv_file.flush()

        if self.verbose:
            self.get_logger().info(
                f'[RECORDER] t={t:.3f}s, '
                f'x={self.x:.4f}, y={self.y:.4f}, yaw={self.yaw:.4f}, '
                f'cov_xx={self.cov_xx:.6e}, cov_yy={self.cov_yy:.6e}, '
                f'cov_yawyaw={self.cov_yawyaw:.6e}'
            )

    def save_summary_csv(self):
        with open(self.summary_csv_path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
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
                self.last_time_sec,
                self.x,
                self.y,
                self.yaw,
                self.cov_xx,
                self.cov_yy,
                self.cov_yawyaw
            ])

    def destroy_node(self):
        try:
            if self.received_any:
                self.save_summary_csv()
        except Exception as e:
            self.get_logger().warn(f'[RECORDER] Failed to save summary CSV: {e}')

        try:
            if not self.trace_csv_file.closed:
                self.trace_csv_file.close()
        except Exception:
            pass

        self.get_logger().info(
            f'[RECORDER] Final saved: x={self.x:.4f}, y={self.y:.4f}, yaw={self.yaw:.4f}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiuTrialRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[RECORDER] Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()