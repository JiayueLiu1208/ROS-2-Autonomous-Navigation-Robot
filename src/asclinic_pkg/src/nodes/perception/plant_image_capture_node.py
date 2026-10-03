#!/usr/bin/env python3

import json
import os
import threading
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger


class PlantImageCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__('plant_image_capture_node')

        self.declare_parameter('image_topic', '/asc/camera_image')
        self.declare_parameter('output_dir', 'data/plant_dataset/raw')
        self.declare_parameter('capture_interval_sec', 2.0)
        self.declare_parameter('save_metadata', True)
        self.declare_parameter('max_images', 0)
        self.declare_parameter('image_prefix', 'plant')
        self.declare_parameter('capture_mode', 'manual')
        self.declare_parameter('trigger_topic', '/asc/plant_detector/capture_request')
        self.declare_parameter('capture_service', '/asc/plant_detector/capture_image')
        self.declare_parameter('robot_pose_topic', '/asc/pitt_fused_odometry')
        self.declare_parameter('plant_target_id', '')
        self.declare_parameter('discard_low_quality', False)
        self.declare_parameter('min_laplacian_variance', 30.0)
        self.declare_parameter('min_mean_brightness', 25.0)
        self.declare_parameter('max_mean_brightness', 245.0)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.output_dir = Path(os.path.expanduser(str(self.get_parameter('output_dir').value)))
        self.capture_interval_sec = float(self.get_parameter('capture_interval_sec').value)
        self.save_metadata = bool(self.get_parameter('save_metadata').value)
        self.max_images = int(self.get_parameter('max_images').value)
        self.image_prefix = str(self.get_parameter('image_prefix').value)
        self.capture_mode = str(self.get_parameter('capture_mode').value).lower()
        self.trigger_topic = str(self.get_parameter('trigger_topic').value)
        self.capture_service = str(self.get_parameter('capture_service').value)
        self.robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)
        self.plant_target_id = str(self.get_parameter('plant_target_id').value)
        self.discard_low_quality = bool(self.get_parameter('discard_low_quality').value)
        self.min_laplacian_variance = float(
            self.get_parameter('min_laplacian_variance').value
        )
        self.min_mean_brightness = float(self.get_parameter('min_mean_brightness').value)
        self.max_mean_brightness = float(self.get_parameter('max_mean_brightness').value)

        self.cv_bridge = CvBridge()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.latest_image_msg: Image | None = None
        self.latest_pose_msg: Odometry | None = None
        self.latest_lock = threading.Lock()
        self.saved_count = 0

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.pose_sub = self.create_subscription(
            Odometry,
            self.robot_pose_topic,
            self.pose_callback,
            10,
        )
        self.trigger_sub = self.create_subscription(
            UInt32,
            self.trigger_topic,
            self.trigger_callback,
            10,
        )
        self.capture_srv = self.create_service(
            Trigger,
            self.capture_service,
            self.capture_service_callback,
        )

        if self.capture_mode == 'interval':
            period = max(0.1, self.capture_interval_sec)
            self.interval_timer = self.create_timer(period, self.interval_callback)

        self.get_logger().info(
            '[PLANT CAPTURE] image_topic=%s output_dir=%s mode=%s max_images=%d'
            % (self.image_topic, self.output_dir, self.capture_mode, self.max_images)
        )
        self.get_logger().info(
            '[PLANT CAPTURE] manual trigger topic=%s service=%s'
            % (self.trigger_topic, self.capture_service)
        )

    def image_callback(self, msg: Image) -> None:
        with self.latest_lock:
            self.latest_image_msg = msg

    def pose_callback(self, msg: Odometry) -> None:
        with self.latest_lock:
            self.latest_pose_msg = msg

    def trigger_callback(self, msg: UInt32) -> None:
        if msg.data == 0:
            return
        self.capture_latest('manual_topic')

    def capture_service_callback(self, request, response):
        del request
        success, detail = self.capture_latest('service')
        response.success = success
        response.message = detail
        return response

    def interval_callback(self) -> None:
        self.capture_latest('interval')

    def capture_latest(self, capture_reason: str) -> tuple[bool, str]:
        if self.max_images > 0 and self.saved_count >= self.max_images:
            detail = 'max_images reached (%d)' % self.max_images
            self.get_logger().warn('[PLANT CAPTURE] %s' % detail)
            return False, detail

        with self.latest_lock:
            image_msg = self.latest_image_msg
            pose_msg = self.latest_pose_msg

        if image_msg is None:
            detail = 'no image received yet on %s' % self.image_topic
            self.get_logger().warn('[PLANT CAPTURE] %s' % detail)
            return False, detail

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            detail = 'image conversion failed: %s' % exc
            self.get_logger().warn('[PLANT CAPTURE] %s' % detail)
            return False, detail

        quality = self.measure_quality(cv_image)
        if self.discard_low_quality and not quality['quality_ok']:
            detail = (
                'discarded low-quality image blur=%.1f brightness=%.1f'
                % (quality['laplacian_variance'], quality['mean_brightness'])
            )
            self.get_logger().warn('[PLANT CAPTURE] %s' % detail)
            return False, detail

        self.saved_count += 1
        image_path = self.next_image_path()
        ok = cv2.imwrite(str(image_path), cv_image)
        if not ok:
            detail = 'cv2.imwrite failed for %s' % image_path
            self.get_logger().error('[PLANT CAPTURE] %s' % detail)
            return False, detail

        metadata_path = image_path.with_suffix('.json')
        if self.save_metadata:
            metadata = self.build_metadata(
                image_msg=image_msg,
                pose_msg=pose_msg,
                image_path=image_path,
                capture_reason=capture_reason,
                quality=quality,
                cv_image=cv_image,
            )
            with open(metadata_path, 'w', encoding='utf-8') as metadata_file:
                json.dump(metadata, metadata_file, indent=2)

        detail = 'saved %s' % image_path
        if self.save_metadata:
            detail += ' with metadata %s' % metadata_path
        self.get_logger().info('[PLANT CAPTURE] %s' % detail)
        return True, detail

    def next_image_path(self) -> Path:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        candidate = self.output_dir / (
            '%s_%s_%04d.jpg' % (self.image_prefix, timestamp, self.saved_count)
        )
        suffix = 1
        while candidate.exists():
            candidate = self.output_dir / (
                '%s_%s_%04d_%02d.jpg'
                % (self.image_prefix, timestamp, self.saved_count, suffix)
            )
            suffix += 1
        return candidate

    def measure_quality(self, cv_image) -> dict:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        mean_brightness = float(gray.mean())
        laplacian_variance = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        quality_ok = (
            self.min_mean_brightness <= mean_brightness <= self.max_mean_brightness
            and laplacian_variance >= self.min_laplacian_variance
        )
        return {
            'mean_brightness': mean_brightness,
            'laplacian_variance': laplacian_variance,
            'quality_ok': quality_ok,
        }

    def build_metadata(
        self,
        image_msg: Image,
        pose_msg: Odometry | None,
        image_path: Path,
        capture_reason: str,
        quality: dict,
        cv_image,
    ) -> dict:
        metadata = {
            'image': image_path.name,
            'timestamp': datetime.now().isoformat(),
            'ros_stamp': {
                'sec': int(image_msg.header.stamp.sec),
                'nanosec': int(image_msg.header.stamp.nanosec),
            },
            'frame_id': image_msg.header.frame_id,
            'camera_topic': self.image_topic,
            'capture_mode': self.capture_mode,
            'capture_reason': capture_reason,
            'plant_target_id': self.plant_target_id,
            'image_shape': {
                'height': int(cv_image.shape[0]),
                'width': int(cv_image.shape[1]),
                'channels': int(cv_image.shape[2]) if len(cv_image.shape) > 2 else 1,
            },
            'quality': quality,
        }
        if pose_msg is not None:
            metadata['robot_pose'] = {
                'topic': self.robot_pose_topic,
                'stamp': {
                    'sec': int(pose_msg.header.stamp.sec),
                    'nanosec': int(pose_msg.header.stamp.nanosec),
                },
                'frame_id': pose_msg.header.frame_id,
                'child_frame_id': pose_msg.child_frame_id,
                'position': {
                    'x': float(pose_msg.pose.pose.position.x),
                    'y': float(pose_msg.pose.pose.position.y),
                    'z': float(pose_msg.pose.pose.position.z),
                },
                'orientation': {
                    'x': float(pose_msg.pose.pose.orientation.x),
                    'y': float(pose_msg.pose.pose.orientation.y),
                    'z': float(pose_msg.pose.pose.orientation.z),
                    'w': float(pose_msg.pose.pose.orientation.w),
                },
            }
        return metadata


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlantImageCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
