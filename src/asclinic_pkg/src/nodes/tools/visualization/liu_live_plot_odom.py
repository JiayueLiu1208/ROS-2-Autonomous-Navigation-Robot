#!/usr/bin/env python3

import os
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class LiuLivePlotOdom(Node):
    def __init__(self):
        super().__init__('liu_live_plot_odom')

        self.declare_parameter('odom_topic', 'wheel_odometry')
        self.declare_parameter('save_directory', '~/asclinic-ros2/ros2_ws/results')
        self.declare_parameter('save_period', 0.5)
        self.declare_parameter('max_points', 5000)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.save_directory = os.path.expanduser(
            str(self.get_parameter('save_directory').value)
        )
        self.save_period = float(self.get_parameter('save_period').value)
        self.max_points = int(self.get_parameter('max_points').value)

        os.makedirs(self.save_directory, exist_ok=True)

        self.latest_png_path = os.path.join(self.save_directory, 'odom_live_dashboard.png')

        self.t_data = []
        self.x_data = []
        self.y_data = []
        self.yaw_data = []
        self.cov_xx_data = []
        self.cov_yy_data = []
        self.cov_yawyaw_data = []

        self.start_time_sec = None
        self.received_any = False

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.save_timer = self.create_timer(self.save_period, self.save_dashboard)

        self.get_logger().info('========================================')
        self.get_logger().info('[LIVE PLOT] Node started')
        self.get_logger().info(f'[LIVE PLOT] Subscribing to: {self.odom_topic}')
        self.get_logger().info(f'[LIVE PLOT] Saving dashboard to: {self.latest_png_path}')
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

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        cov = msg.pose.covariance
        cov_xx = cov[0]
        cov_yy = cov[7]
        cov_yawyaw = cov[35]

        self.t_data.append(t)
        self.x_data.append(x)
        self.y_data.append(y)
        self.yaw_data.append(yaw)
        self.cov_xx_data.append(cov_xx)
        self.cov_yy_data.append(cov_yy)
        self.cov_yawyaw_data.append(cov_yawyaw)

        if len(self.t_data) > self.max_points:
            self.t_data.pop(0)
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.yaw_data.pop(0)
            self.cov_xx_data.pop(0)
            self.cov_yy_data.pop(0)
            self.cov_yawyaw_data.pop(0)

        self.received_any = True

    def save_dashboard(self):
        if not self.received_any:
            return

        fig, axes = plt.subplots(3, 2, figsize=(12, 8))

        # Trajectory
        axes[0, 0].plot(self.x_data, self.y_data, linewidth=1.8)
        axes[0, 0].plot(self.x_data[0], self.y_data[0], 'go', markersize=7)
        axes[0, 0].plot(self.x_data[-1], self.y_data[-1], 'ro', markersize=7)
        axes[0, 0].set_title('Odometry Trajectory')
        axes[0, 0].set_xlabel('x_odom (m)')
        axes[0, 0].set_ylabel('y_odom (m)')
        axes[0, 0].grid(True)
        axes[0, 0].axis('equal')

        # x(t)
        axes[0, 1].plot(self.t_data, self.x_data, linewidth=1.5)
        axes[0, 1].set_title('x_odom vs Time')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('x_odom (m)')
        axes[0, 1].grid(True)

        # y(t)
        axes[1, 0].plot(self.t_data, self.y_data, linewidth=1.5)
        axes[1, 0].set_title('y_odom vs Time')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('y_odom (m)')
        axes[1, 0].grid(True)

        # yaw(t)
        axes[1, 1].plot(self.t_data, self.yaw_data, linewidth=1.5)
        axes[1, 1].set_title('yaw_odom vs Time')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('yaw_odom (rad)')
        axes[1, 1].grid(True)

        # covariance xx/yy
        axes[2, 0].plot(self.t_data, self.cov_xx_data, linewidth=1.5, label='cov_xx')
        axes[2, 0].plot(self.t_data, self.cov_yy_data, linewidth=1.5, label='cov_yy')
        axes[2, 0].set_title('Position Covariance vs Time')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_ylabel('Covariance')
        axes[2, 0].grid(True)
        axes[2, 0].legend(loc='best')

        # covariance yawyaw
        axes[2, 1].plot(self.t_data, self.cov_yawyaw_data, linewidth=1.5)
        axes[2, 1].set_title('Yaw Covariance vs Time')
        axes[2, 1].set_xlabel('Time (s)')
        axes[2, 1].set_ylabel('pose_cov_yawyaw')
        axes[2, 1].grid(True)

        fig.tight_layout()
        fig.savefig(self.latest_png_path, dpi=150)
        plt.close(fig)

    def destroy_node(self):
        try:
            if self.received_any:
                final_name = datetime.now().strftime('odom_live_dashboard_final_%Y%m%d_%H%M%S.png')
                final_path = os.path.join(self.save_directory, final_name)
                self.save_dashboard()
                # 再复制一份最终带时间戳的
                fig, axes = plt.subplots(3, 2, figsize=(12, 8))

                axes[0, 0].plot(self.x_data, self.y_data, linewidth=1.8)
                axes[0, 0].plot(self.x_data[0], self.y_data[0], 'go', markersize=7)
                axes[0, 0].plot(self.x_data[-1], self.y_data[-1], 'ro', markersize=7)
                axes[0, 0].set_title('Odometry Trajectory')
                axes[0, 0].set_xlabel('x_odom (m)')
                axes[0, 0].set_ylabel('y_odom (m)')
                axes[0, 0].grid(True)
                axes[0, 0].axis('equal')

                axes[0, 1].plot(self.t_data, self.x_data, linewidth=1.5)
                axes[0, 1].set_title('x_odom vs Time')
                axes[0, 1].set_xlabel('Time (s)')
                axes[0, 1].set_ylabel('x_odom (m)')
                axes[0, 1].grid(True)

                axes[1, 0].plot(self.t_data, self.y_data, linewidth=1.5)
                axes[1, 0].set_title('y_odom vs Time')
                axes[1, 0].set_xlabel('Time (s)')
                axes[1, 0].set_ylabel('y_odom (m)')
                axes[1, 0].grid(True)

                axes[1, 1].plot(self.t_data, self.yaw_data, linewidth=1.5)
                axes[1, 1].set_title('yaw_odom vs Time')
                axes[1, 1].set_xlabel('Time (s)')
                axes[1, 1].set_ylabel('yaw_odom (rad)')
                axes[1, 1].grid(True)

                axes[2, 0].plot(self.t_data, self.cov_xx_data, linewidth=1.5, label='cov_xx')
                axes[2, 0].plot(self.t_data, self.cov_yy_data, linewidth=1.5, label='cov_yy')
                axes[2, 0].set_title('Position Covariance vs Time')
                axes[2, 0].set_xlabel('Time (s)')
                axes[2, 0].set_ylabel('Covariance')
                axes[2, 0].grid(True)
                axes[2, 0].legend(loc='best')

                axes[2, 1].plot(self.t_data, self.cov_yawyaw_data, linewidth=1.5)
                axes[2, 1].set_title('Yaw Covariance vs Time')
                axes[2, 1].set_xlabel('Time (s)')
                axes[2, 1].set_ylabel('pose_cov_yawyaw')
                axes[2, 1].grid(True)

                fig.tight_layout()
                fig.savefig(final_path, dpi=150)
                plt.close(fig)

                self.get_logger().info(f'[LIVE PLOT] Final dashboard saved to: {final_path}')
        except Exception as e:
            self.get_logger().warn(f'[LIVE PLOT] Failed to save final dashboard: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiuLivePlotOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[LIVE PLOT] Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()