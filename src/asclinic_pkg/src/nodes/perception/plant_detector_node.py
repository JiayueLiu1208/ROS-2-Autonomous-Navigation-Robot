#!/usr/bin/env python3

import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from asclinic_pkg.msg import PlantDetection, PlantDetections


DEFAULT_MODEL_RELATIVE_PATH = 'models/plant_detector/best.pt'


def _as_string_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [item.strip() for item in value.split(',') if item.strip()]
    if isinstance(value, Iterable):
        return [str(item).strip() for item in value if str(item).strip()]
    return []


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class PlantDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('plant_detector_node')

        self.declare_parameter('image_topic', '/asc/camera_image')
        self.declare_parameter('detections_topic', '/asc/plant_detections')
        self.declare_parameter('debug_image_topic', '/asc/plant_detector/debug_image')
        self.declare_parameter('enabled_topic', '/asc/plant_detector/enabled')
        self.declare_parameter('start_enabled', True)
        self.declare_parameter('model_path', DEFAULT_MODEL_RELATIVE_PATH)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('inference_rate_hz', 2.0)
        self.declare_parameter('target_classes', [])
        self.declare_parameter('save_positive_images', False)
        self.declare_parameter('save_positive_requires_valid_capture', False)
        self.declare_parameter('positive_image_dir', 'data/plant_dataset/positive')
        self.declare_parameter('positive_save_gate_topic', '')
        self.declare_parameter('min_bbox_area_ratio', 0.025)
        self.declare_parameter('check_image_quality', True)
        self.declare_parameter('min_laplacian_variance', 30.0)
        self.declare_parameter('min_mean_brightness', 25.0)
        self.declare_parameter('max_mean_brightness', 245.0)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.enabled_topic = str(self.get_parameter('enabled_topic').value)
        self.enabled = _as_bool(self.get_parameter('start_enabled').value)
        self.model_path = self._resolve_model_path(
            str(self.get_parameter('model_path').value)
        )
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.requested_device = str(self.get_parameter('device').value).lower()
        self.publish_debug_image = _as_bool(self.get_parameter('publish_debug_image').value)
        self.inference_rate_hz = float(self.get_parameter('inference_rate_hz').value)
        self.target_classes = set(_as_string_list(self.get_parameter('target_classes').value))
        self.save_positive_images = _as_bool(self.get_parameter('save_positive_images').value)
        self.save_positive_requires_valid_capture = _as_bool(
            self.get_parameter('save_positive_requires_valid_capture').value
        )
        self.positive_image_dir = Path(
            os.path.expanduser(str(self.get_parameter('positive_image_dir').value))
        )
        self.positive_save_gate_topic = str(
            self.get_parameter('positive_save_gate_topic').value
        ).strip()
        self.positive_save_enabled = not self.positive_save_gate_topic
        self.min_bbox_area_ratio = float(self.get_parameter('min_bbox_area_ratio').value)
        self.check_image_quality = _as_bool(self.get_parameter('check_image_quality').value)
        self.min_laplacian_variance = float(
            self.get_parameter('min_laplacian_variance').value
        )
        self.min_mean_brightness = float(self.get_parameter('min_mean_brightness').value)
        self.max_mean_brightness = float(self.get_parameter('max_mean_brightness').value)

        self.cv_bridge = CvBridge()
        self.model = None
        self.predict_device = 'cpu'
        self._latest_image_msg: Image | None = None
        self._latest_lock = threading.Lock()
        self._enabled_lock = threading.Lock()
        self._positive_save_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._stop_event = threading.Event()
        self._save_seq = 0
        self._last_quality_warning_time = 0.0

        self.detections_pub = self.create_publisher(
            PlantDetections,
            self.detections_topic,
            QoSProfile(depth=10),
        )
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                self.debug_image_topic,
                QoSProfile(depth=2),
            )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.enabled_sub = self.create_subscription(
            Bool,
            self.enabled_topic,
            self.enabled_callback,
            QoSProfile(depth=10),
        )
        self.positive_save_gate_sub = None
        if self.positive_save_gate_topic:
            self.positive_save_gate_sub = self.create_subscription(
                Bool,
                self.positive_save_gate_topic,
                self.positive_save_gate_callback,
                QoSProfile(depth=10),
            )

        if self.save_positive_images:
            self.positive_image_dir.mkdir(parents=True, exist_ok=True)

        self._load_model()

        self.worker_thread = threading.Thread(
            target=self._inference_loop,
            name='plant_detector_inference',
            daemon=True,
        )
        self.worker_thread.start()

        self.get_logger().info(
            '[PLANT DETECTOR] image_topic=%s detections_topic=%s debug_image=%s enabled=%s'
            % (
                self.image_topic,
                self.detections_topic,
                self.publish_debug_image,
                self.enabled,
            )
        )
        self.get_logger().info(
            '[PLANT DETECTOR] enable gate topic=%s' % self.enabled_topic
        )
        if self.positive_save_gate_topic:
            self.get_logger().info(
                '[PLANT DETECTOR] positive image save gate topic=%s'
                % self.positive_save_gate_topic
            )
        self.get_logger().info(
            '[PLANT DETECTOR] model_path=%s device=%s conf=%.3f iou=%.3f rate=%.2f Hz'
            % (
                self.model_path if self.model_path else '<not set>',
                self.predict_device,
                self.confidence_threshold,
                self.iou_threshold,
                self.inference_rate_hz,
            )
        )
        if self.target_classes:
            self.get_logger().info(
                '[PLANT DETECTOR] target_classes=%s' % ','.join(sorted(self.target_classes))
            )

    def _resolve_model_path(self, raw_path: str) -> str:
        model_path_text = os.path.expanduser(raw_path.strip() or DEFAULT_MODEL_RELATIVE_PATH)
        model_path = Path(model_path_text)
        if model_path.is_absolute():
            return str(model_path)

        candidates: list[Path] = []
        try:
            from ament_index_python.packages import get_package_share_directory

            candidates.append(
                Path(get_package_share_directory('asclinic_pkg')) / model_path
            )
        except Exception:
            pass

        candidates.append(Path.cwd() / model_path)
        candidates.append(Path(__file__).resolve().parents[3] / model_path)

        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
        return str(candidates[0] if candidates else model_path)

    def _load_model(self) -> None:
        if not os.path.isfile(self.model_path):
            self.get_logger().error(
                '[PLANT DETECTOR] YOLO model not found: %s' % self.model_path
            )
            return

        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error(
                '[PLANT DETECTOR] ultralytics is not installed. '
                'Install it outside ROS runtime setup before launching inference.'
            )
            return

        self.predict_device = self._select_device()
        try:
            self.model = YOLO(self.model_path)
        except Exception as exc:
            self.get_logger().error('[PLANT DETECTOR] Failed to load YOLO model: %s' % exc)
            self.model = None

    def _select_device(self) -> str:
        requested = self.requested_device
        if requested in ('cpu', 'none'):
            return 'cpu'

        cuda_available = False
        try:
            import torch

            cuda_available = bool(torch.cuda.is_available())
        except Exception:
            cuda_available = False

        if requested == 'auto':
            return 'cuda:0' if cuda_available else 'cpu'

        if requested.startswith('cuda') and not cuda_available:
            self.get_logger().warn(
                '[PLANT DETECTOR] CUDA requested but unavailable. Falling back to CPU.'
            )
            return 'cpu'

        if requested == 'cuda':
            return 'cuda:0'
        return requested

    def image_callback(self, msg: Image) -> None:
        if not self._is_enabled():
            return
        with self._latest_lock:
            self._latest_image_msg = msg
        self._frame_event.set()

    def enabled_callback(self, msg: Bool) -> None:
        new_enabled = bool(msg.data)
        old_enabled = self._is_enabled()
        with self._enabled_lock:
            self.enabled = new_enabled

        if not new_enabled:
            with self._latest_lock:
                self._latest_image_msg = None
                self._frame_event.clear()

        if new_enabled != old_enabled:
            state_text = 'enabled' if new_enabled else 'disabled'
            self.get_logger().info('[PLANT DETECTOR] Inference %s.' % state_text)

    def _is_enabled(self) -> bool:
        with self._enabled_lock:
            return self.enabled

    def positive_save_gate_callback(self, msg: Bool) -> None:
        new_enabled = bool(msg.data)
        old_enabled = self._positive_save_is_enabled()
        with self._positive_save_lock:
            self.positive_save_enabled = new_enabled
        if new_enabled != old_enabled:
            state_text = 'enabled' if new_enabled else 'disabled'
            self.get_logger().info(
                '[PLANT DETECTOR] Positive image save %s.' % state_text
            )

    def _positive_save_is_enabled(self) -> bool:
        with self._positive_save_lock:
            return self.positive_save_enabled

    def _inference_loop(self) -> None:
        min_period = 0.0
        if self.inference_rate_hz > 0.0:
            min_period = 1.0 / self.inference_rate_hz
        next_allowed_time = 0.0

        while not self._stop_event.is_set():
            self._frame_event.wait(timeout=0.1)
            if self._stop_event.is_set():
                break

            with self._latest_lock:
                image_msg = self._latest_image_msg
                self._latest_image_msg = None
                self._frame_event.clear()

            if image_msg is None:
                continue

            if not self._is_enabled():
                continue

            now = time.monotonic()
            if min_period > 0.0 and now < next_allowed_time:
                continue
            next_allowed_time = now + min_period

            if self.model is None:
                continue

            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            except CvBridgeError as exc:
                self.get_logger().warn('[PLANT DETECTOR] Image conversion failed: %s' % exc)
                continue

            start = time.monotonic()
            try:
                detections, annotated_image, quality = self._run_yolo(cv_image)
            except Exception as exc:
                self.get_logger().error('[PLANT DETECTOR] Inference failed: %s' % exc)
                continue
            elapsed_ms = (time.monotonic() - start) * 1000.0

            out_msg = self._build_detection_msg(image_msg, detections, cv_image.shape, quality)
            self.detections_pub.publish(out_msg)

            if (
                out_msg.detection_count > 0
                and self.save_positive_images
                and self._positive_save_is_enabled()
                and (
                    not self.save_positive_requires_valid_capture
                    or str(out_msg.decision_hint).strip().lower() == 'valid_capture'
                )
            ):
                self._save_positive_image(annotated_image, out_msg, quality)

            if self.debug_image_pub is not None:
                try:
                    debug_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                    debug_msg.header = image_msg.header
                    self.debug_image_pub.publish(debug_msg)
                except CvBridgeError as exc:
                    self.get_logger().warn('[PLANT DETECTOR] Debug image conversion failed: %s' % exc)

            if out_msg.detection_count > 0 or self._should_log_quality_warning(quality):
                self.get_logger().info(
                    '[PLANT DETECTOR] detections=%d best=%.3f decision=%s inference=%.1f ms blur=%.1f brightness=%.1f'
                    % (
                        out_msg.detection_count,
                        out_msg.best_confidence,
                        out_msg.decision_hint,
                        elapsed_ms,
                        quality['laplacian_variance'],
                        quality['mean_brightness'],
                    )
                )

    def _run_yolo(self, cv_image) -> tuple[list[dict], Any, dict]:
        result_list = self.model.predict(
            source=cv_image,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.predict_device,
            verbose=False,
        )
        result = result_list[0]
        names = getattr(result, 'names', None) or getattr(self.model, 'names', {})

        detections: list[dict] = []
        if result.boxes is not None and len(result.boxes) > 0:
            xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)

            for box, confidence, class_id in zip(xyxy, confs, class_ids):
                class_name = str(names.get(int(class_id), int(class_id)))
                if self.target_classes and class_name not in self.target_classes:
                    continue

                xmin, ymin, xmax, ymax = box.tolist()
                width = max(0.0, xmax - xmin)
                height = max(0.0, ymax - ymin)
                detections.append(
                    {
                        'class_name': class_name,
                        'class_id': int(class_id),
                        'confidence': float(confidence),
                        'xmin': int(round(xmin)),
                        'ymin': int(round(ymin)),
                        'xmax': int(round(xmax)),
                        'ymax': int(round(ymax)),
                        'centre_x': float(xmin + 0.5 * width),
                        'centre_y': float(ymin + 0.5 * height),
                        'width': float(width),
                        'height': float(height),
                    }
                )

        quality = self._measure_image_quality(cv_image)
        annotated = self._draw_detections(cv_image, detections, quality)
        return detections, annotated, quality

    def _measure_image_quality(self, cv_image) -> dict:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        mean_brightness = float(gray.mean())
        laplacian_variance = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        quality_ok = (
            not self.check_image_quality
            or (
                self.min_mean_brightness <= mean_brightness <= self.max_mean_brightness
                and laplacian_variance >= self.min_laplacian_variance
            )
        )
        return {
            'mean_brightness': mean_brightness,
            'laplacian_variance': laplacian_variance,
            'quality_ok': quality_ok,
        }

    def _draw_detections(self, cv_image, detections: list[dict], quality: dict):
        annotated = cv_image.copy()
        for det in detections:
            color = (0, 220, 0)
            cv2.rectangle(
                annotated,
                (det['xmin'], det['ymin']),
                (det['xmax'], det['ymax']),
                color,
                2,
            )
            label = '%s %.2f' % (det['class_name'], det['confidence'])
            cv2.putText(
                annotated,
                label,
                (det['xmin'], max(18, det['ymin'] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        if self.check_image_quality and not quality['quality_ok']:
            cv2.putText(
                annotated,
                'quality check failed',
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )
        return annotated

    def _build_detection_msg(
        self,
        image_msg: Image,
        detections: list[dict],
        image_shape,
        quality: dict,
    ) -> PlantDetections:
        out_msg = PlantDetections()
        out_msg.header = image_msg.header

        best_confidence = 0.0
        largest_area_ratio = 0.0
        image_area = float(max(1, image_shape[0] * image_shape[1]))

        for det in detections:
            item = PlantDetection()
            item.class_name = det['class_name']
            item.class_id = det['class_id']
            item.confidence = det['confidence']
            item.xmin = det['xmin']
            item.ymin = det['ymin']
            item.xmax = det['xmax']
            item.ymax = det['ymax']
            item.centre_x = det['centre_x']
            item.centre_y = det['centre_y']
            item.width = det['width']
            item.height = det['height']
            out_msg.detections.append(item)

            best_confidence = max(best_confidence, det['confidence'])
            largest_area_ratio = max(largest_area_ratio, (det['width'] * det['height']) / image_area)

        out_msg.detection_count = len(out_msg.detections)
        out_msg.plant_visible = out_msg.detection_count > 0
        out_msg.best_confidence = float(best_confidence)
        out_msg.decision_hint = self._decision_hint(
            out_msg.detection_count,
            best_confidence,
            largest_area_ratio,
            quality,
        )
        return out_msg

    def _decision_hint(
        self,
        detection_count: int,
        best_confidence: float,
        largest_area_ratio: float,
        quality: dict,
    ) -> str:
        if self.check_image_quality and not quality['quality_ok']:
            return 'retry_view'
        if detection_count == 0:
            return 'retry_view'
        if best_confidence < self.confidence_threshold:
            return 'low_confidence'
        if largest_area_ratio < self.min_bbox_area_ratio:
            return 'extra_closeup'
        return 'valid_capture'

    def _should_log_quality_warning(self, quality: dict) -> bool:
        if quality['quality_ok']:
            return False
        now = time.monotonic()
        if now - self._last_quality_warning_time < 5.0:
            return False
        self._last_quality_warning_time = now
        return True

    def _save_positive_image(
        self,
        cv_image,
        detections_msg: PlantDetections,
        quality: dict,
    ) -> None:
        self._save_seq += 1
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        stem = 'plant_positive_%s_%04d' % (timestamp, self._save_seq)
        image_path = self.positive_image_dir / (stem + '.jpg')
        metadata_path = self.positive_image_dir / (stem + '.json')

        cv2.imwrite(str(image_path), cv_image)
        metadata = {
            'image': image_path.name,
            'stamp': {
                'sec': int(detections_msg.header.stamp.sec),
                'nanosec': int(detections_msg.header.stamp.nanosec),
            },
            'frame_id': detections_msg.header.frame_id,
            'decision_hint': detections_msg.decision_hint,
            'detection_count': int(detections_msg.detection_count),
            'best_confidence': float(detections_msg.best_confidence),
            'annotated_image': True,
            'quality': quality,
            'detections': [
                {
                    'class_name': det.class_name,
                    'class_id': int(det.class_id),
                    'confidence': float(det.confidence),
                    'bbox': [int(det.xmin), int(det.ymin), int(det.xmax), int(det.ymax)],
                    'centre': [float(det.centre_x), float(det.centre_y)],
                    'size': [float(det.width), float(det.height)],
                }
                for det in detections_msg.detections
            ],
        }
        with open(metadata_path, 'w', encoding='utf-8') as metadata_file:
            json.dump(metadata, metadata_file, indent=2)
        self.get_logger().info(
            '[PLANT DETECTOR] Saved positive image %s (decision=%s count=%d best=%.3f).'
            % (
                str(image_path),
                detections_msg.decision_hint,
                int(detections_msg.detection_count),
                float(detections_msg.best_confidence),
            )
        )

    def destroy_node(self) -> None:
        self._stop_event.set()
        self._frame_event.set()
        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlantDetectorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
