#!/usr/bin/env python3

# ----------------------------------------------------------------------------
# ArUco 位姿估计精度验证 — 数据采集节点
#
# 使用方法：
#   终端 1: ros2 launch asclinic_pkg aruco_detector_launch.py
#   终端 2: ros2 run asclinic_pkg aruco_calibration_collector.py
#
# 工作流程：
#   1. 把小车放在离 ArUco 标记一个已知距离的位置
#   2. 等节点检测到标记并显示估计值
#   3. 在终端输入实际测量的距离（米）和角度（度，可选）
#   4. 数据自动保存到 CSV 文件
#   5. 移动小车到下一个位置，重复
#   6. 按 Ctrl+C 结束，自动保存所有数据
# ----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from asclinic_pkg.msg import FiducialMarkerArray

import numpy as np
import csv
import os
import threading
from datetime import datetime


# 数据保存路径
DEFAULT_CSV_PATH = "saved_camera_images/aruco_calibration_data.csv"


class ArucoCalibrationCollector(Node):

    def __init__(self):
        super().__init__('aruco_calibration_collector')

        # 参数
        self.declare_parameter('csv_path', DEFAULT_CSV_PATH)
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅 ArUco 检测结果
        self.aruco_sub = self.create_subscription(
            FiducialMarkerArray,
            'aruco_detections',
            self.aruco_callback,
            qos_profile
        )

        # 最新检测数据（线程安全）
        self.lock = threading.Lock()
        self.latest_detections = None

        # 已采集的数据
        self.collected_data = []
        self.sample_count = 0

        # 初始化 CSV 文件（写入表头，如果文件已存在则追加）
        file_exists = os.path.isfile(self.csv_path)
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if not file_exists:
            self.csv_writer.writerow([
                'sample_id', 'timestamp', 'marker_id',
                'est_x', 'est_y', 'est_z', 'est_distance',
                'est_rvec_x', 'est_rvec_y', 'est_rvec_z',
                'est_yaw_deg',
                'actual_distance', 'actual_angle',
                'distance_error', 'distance_error_pct'
            ])
            self.csv_file.flush()

        self.get_logger().info("=" * 60)
        self.get_logger().info("ArUco 标定数据采集节点已启动")
        self.get_logger().info(f"数据保存到: {self.csv_path}")
        self.get_logger().info("=" * 60)

        # 启动用户输入线程
        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.input_thread.start()

    def aruco_callback(self, msg):
        """接收 ArUco 检测结果"""
        with self.lock:
            self.latest_detections = msg

    def compute_yaw_from_rvec(self, rvec):
        """从 rvec 计算标记相对于摄像头的偏航角（度）"""
        import cv2
        rvec_np = np.array(rvec, dtype=np.float64).reshape(3, 1)
        rmat, _ = cv2.Rodrigues(rvec_np)
        # 偏航角 = atan2(rmat[1,0], rmat[0,0])  (绕 z 轴旋转)
        yaw_rad = np.arctan2(rmat[1, 0], rmat[0, 0])
        return np.degrees(yaw_rad)

    def user_input_loop(self):
        """在终端交互式输入实际测量值"""
        import time
        time.sleep(1.0)  # 等节点初始化完成

        print("\n" + "=" * 60)
        print("操作指南：")
        print("  1. 确保 aruco_detector 节点正在运行")
        print("  2. 将小车放在已知距离处，面向 ArUco 标记")
        print("  3. 按 Enter 采集当前帧的检测数据")
        print("  4. 输入实际测量的距离（米）")
        print("  5. 可选：输入实际角度（度），直接回车跳过")
        print("  6. 重复步骤 2-5 约 100 次")
        print("  7. 按 Ctrl+C 结束并保存")
        print("=" * 60)

        while rclpy.ok():
            try:
                print(f"\n--- 第 {self.sample_count + 1} 个数据点 ---")

                # 检查是否有检测数据
                with self.lock:
                    detections = self.latest_detections

                if detections is None or detections.num_markers == 0:
                    print("⚠ 当前未检测到 ArUco 标记，请调整位置")
                    input("按 Enter 重试...")
                    continue

                # 显示所有检测到的标记
                print(f"检测到 {detections.num_markers} 个标记：")
                for i, marker in enumerate(detections.markers):
                    est_distance = np.sqrt(
                        marker.tvec[0]**2 + marker.tvec[1]**2 + marker.tvec[2]**2
                    )
                    est_z = marker.tvec[2]  # z 轴距离（光轴方向）
                    yaw = self.compute_yaw_from_rvec(marker.rvec)
                    print(f"  [{i}] ID={marker.id:3d}  "
                          f"tvec=({marker.tvec[0]:+.3f}, {marker.tvec[1]:+.3f}, {marker.tvec[2]:+.3f})  "
                          f"直线距离={est_distance:.3f}m  "
                          f"z轴距离={est_z:.3f}m  "
                          f"偏航角={yaw:+.1f}°")

                # 选择标记（如果有多个）
                marker_idx = 0
                if detections.num_markers > 1:
                    idx_str = input(f"选择标记序号 [0-{detections.num_markers-1}]（默认0）: ").strip()
                    if idx_str:
                        marker_idx = int(idx_str)
                        if marker_idx < 0 or marker_idx >= detections.num_markers:
                            print("无效序号，跳过")
                            continue

                selected = detections.markers[marker_idx]

                # 确认要采集这个数据点
                confirm = input("按 Enter 记录此数据点（输入 s 跳过）: ").strip().lower()
                if confirm == 's':
                    continue

                # 输入实际距离
                actual_dist_str = input("输入实际测量距离（米）: ").strip()
                if not actual_dist_str:
                    print("未输入距离，跳过")
                    continue
                actual_distance = float(actual_dist_str)

                # 输入实际角度（可选）  
                actual_angle_str = input("输入实际角度（度，直接回车跳过）: ").strip()
                actual_angle = float(actual_angle_str) if actual_angle_str else float('nan')

                # 计算估计值
                est_x = selected.tvec[0]
                est_y = selected.tvec[1]
                est_z = selected.tvec[2]
                est_distance = np.sqrt(est_x**2 + est_y**2 + est_z**2)
                est_yaw = self.compute_yaw_from_rvec(selected.rvec)

                # 计算误差
                distance_error = est_distance - actual_distance
                distance_error_pct = (distance_error / actual_distance * 100) if actual_distance != 0 else float('nan')

                # 保存到 CSV
                self.sample_count += 1
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                row = [
                    self.sample_count, timestamp, selected.id,
                    f"{est_x:.4f}", f"{est_y:.4f}", f"{est_z:.4f}", f"{est_distance:.4f}",
                    f"{selected.rvec[0]:.4f}", f"{selected.rvec[1]:.4f}", f"{selected.rvec[2]:.4f}",
                    f"{est_yaw:.2f}",
                    f"{actual_distance:.4f}", f"{actual_angle:.2f}" if not np.isnan(actual_angle) else "",
                    f"{distance_error:.4f}", f"{distance_error_pct:.2f}"
                ]
                self.csv_writer.writerow(row)
                self.csv_file.flush()

                print(f"✓ 已保存第 {self.sample_count} 个数据点")
                print(f"  估计距离: {est_distance:.3f}m  |  实际距离: {actual_distance:.3f}m  |  误差: {distance_error:+.3f}m ({distance_error_pct:+.1f}%)")

            except ValueError as e:
                print(f"输入格式错误: {e}，请重试")
            except EOFError:
                break
            except KeyboardInterrupt:
                break

        print(f"\n共采集 {self.sample_count} 个数据点，已保存到 {self.csv_path}")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCalibrationCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
