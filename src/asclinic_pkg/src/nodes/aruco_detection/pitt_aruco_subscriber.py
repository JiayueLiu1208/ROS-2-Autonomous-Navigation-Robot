#!/usr/bin/env python3

# DESCRIPTION:
# ROS2 node that subscribes to ArUco marker detections,
# logs marker data, and saves results to a CSV file.

import rclpy
from rclpy.node import Node
from asclinic_pkg.msg import FiducialMarkerArray
import csv
import os
import time
import math
import numpy as np


class ArucoSubscriber(Node):

    def __init__(self):
        super().__init__('pitt_aruco_subscriber')

        # CSV output path
        self.csv_path = os.path.expanduser('~/saved_camera_images/aruco_detection_log.csv')
        self.csv_file = None
        self.csv_writer = None
        self._init_csv()

        # Subscribe to ArUco detections
        self.subscription = self.create_subscription(
            FiducialMarkerArray,
            '/asc/aruco_detections',
            self.aruco_callback,
            10
        )

        self.msg_count = 0
        self.get_logger().info(f'[ARUCO SUB] Node started. Logging to: {self.csv_path}')

    def _init_csv(self):
        """Initialize CSV file with header."""
        write_header = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if write_header:
            self.csv_writer.writerow([
                'timestamp', 'seq_num', 'num_markers',
                'marker_id', 'tvec_x', 'tvec_y', 'tvec_z',
                'rvec_x', 'rvec_y', 'rvec_z', 'distance'
            ])
            self.csv_file.flush()

    def aruco_callback(self, msg):
        self.msg_count += 1
        timestamp = time.time()

        if msg.num_markers == 0:
            if self.msg_count % 50 == 0:
                self.get_logger().info('[ARUCO SUB] No markers detected (periodic)')
            return

        self.get_logger().info(
            f'[ARUCO SUB] seq={msg.seq_num}, detected {msg.num_markers} marker(s):'
        )

        for marker in msg.markers:
            tx, ty, tz = marker.tvec[0], marker.tvec[1], marker.tvec[2]
            rx, ry, rz = marker.rvec[0], marker.rvec[1], marker.rvec[2]
            distance = math.sqrt(tx * tx + ty * ty + tz * tz)

            self.get_logger().info(
                f'  ID={marker.id:2d}  tvec=({tx:+.3f}, {ty:+.3f}, {tz:+.3f})  '
                f'z={tz:.3f}m  dist={distance:.3f}m  '
                f'rvec=({rx:+.3f}, {ry:+.3f}, {rz:+.3f})'
            )

            # Write to CSV
            self.csv_writer.writerow([
                f'{timestamp:.3f}', msg.seq_num, msg.num_markers,
                marker.id, f'{tx:.6f}', f'{ty:.6f}', f'{tz:.6f}',
                f'{rx:.6f}', f'{ry:.6f}', f'{rz:.6f}', f'{distance:.6f}'
            ])

        self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'[ARUCO SUB] CSV saved: {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
