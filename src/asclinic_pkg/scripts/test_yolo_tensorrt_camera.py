#!/usr/bin/env python3

"""ROS 2 TensorRT YOLO camera test node for Jetson-class inference machines.

The node owns a camera, exports a YOLO .pt model to a TensorRT .engine when
needed, runs inference through the engine, and overwrites one annotated image at
a fixed interval. It is intentionally separate from the mission detector.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image


os.environ.setdefault('YOLO_AUTOINSTALL', 'false')

SOURCE_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CACHE_DIR = Path.home() / '.cache' / 'asclinic_pkg' / 'tensorrt'


def read_text_if_present(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8', errors='ignore').replace('\x00', '').strip()
    except OSError:
        return ''


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def string_set_from_param(value: Any) -> set[str]:
    if value is None:
        return set()
    if isinstance(value, str):
        return {item.strip() for item in value.split(',') if item.strip()}
    if isinstance(value, (list, tuple)):
        return {str(item).strip() for item in value if str(item).strip()}
    return set()


def default_model_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        package_share = Path(get_package_share_directory('asclinic_pkg'))
        installed_model = package_share / 'models' / 'plant_detector' / 'best.pt'
        if installed_model.is_file():
            return installed_model
    except Exception:
        pass
    return SOURCE_PACKAGE_ROOT / 'models' / 'plant_detector' / 'best.pt'


def default_output_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        package_share = Path(get_package_share_directory('asclinic_pkg'))
        ros2_ws_root = package_share.parents[3]
        return ros2_ws_root / 'results' / 'yolo_tensorrt' / 'latest.jpg'
    except Exception:
        return SOURCE_PACKAGE_ROOT.parents[1] / 'results' / 'yolo_tensorrt' / 'latest.jpg'


def import_ultralytics_yolo():
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            'ultralytics is not installed in this Python environment.'
        ) from exc
    return YOLO


def check_tensorrt_export_dependencies() -> None:
    missing: list[str] = []
    try:
        import onnx  # noqa: F401
    except Exception:
        missing.append('onnx')
    try:
        import tensorrt  # noqa: F401
    except Exception:
        missing.append('tensorrt')

    if missing:
        install_hint = (
            'Install missing TensorRT export dependencies before exporting. '
            'For this Jetson run, start with: '
            'python3 -m pip install --user "onnx>=1.12.0,<2.0.0". '
            'Do not install onnxruntime-gpu from pip on Jetson; no matching '
            'aarch64 wheel is normally available, and this script exports with '
            'simplify=False so onnxruntime-gpu is not needed.'
        )
        raise RuntimeError('%s Missing: %s' % (install_hint, ', '.join(missing)))


def get_gpu_report() -> dict[str, Any]:
    report: dict[str, Any] = {
        'jetson_model': read_text_if_present(Path('/proc/device-tree/model')),
        'nvhost_gpu': Path('/dev/nvhost-gpu').exists(),
        'torch_imported': False,
        'torch_cuda': False,
        'cuda_device_count': 0,
        'cuda_device_name': '',
    }

    try:
        import torch

        report['torch_imported'] = True
        report['torch_cuda'] = bool(torch.cuda.is_available())
        if report['torch_cuda']:
            report['cuda_device_count'] = int(torch.cuda.device_count())
            report['cuda_device_name'] = str(torch.cuda.get_device_name(0))
    except Exception as exc:
        report['torch_error'] = str(exc)

    return report


def file_sha256(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            hasher.update(chunk)
    return hasher.hexdigest()


def default_engine_path(model_path: Path, imgsz: int, fp16: bool) -> Path:
    digest = file_sha256(model_path)[:12]
    precision = 'fp16' if fp16 else 'fp32'
    name = '%s_%s_imgsz%d_%s.engine' % (model_path.stem, digest, imgsz, precision)
    return DEFAULT_CACHE_DIR / name


def atomic_write_image(path: Path, image) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix or '.jpg'
    tmp_path = path.with_name('.%s.tmp%s' % (path.stem, suffix))
    if not cv2.imwrite(str(tmp_path), image):
        raise RuntimeError('cv2.imwrite failed for %s' % tmp_path)
    os.replace(tmp_path, path)


class TensorRTCameraTestNode(Node):
    def __init__(self) -> None:
        super().__init__('test_yolo_tensorrt_camera')

        self.declare_parameter('model_path', str(default_model_path()))
        self.declare_parameter('engine_path', '')
        self.declare_parameter('force_export', False)
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('camera_path', '')
        self.declare_parameter(
            'camera',
            '',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('camera_width', 0)
        self.declare_parameter('camera_height', 0)
        self.declare_parameter('camera_fps', 0.0)
        self.declare_parameter('camera_flush_reads', 3)
        self.declare_parameter('output_path', str(default_output_path()))
        self.declare_parameter('interval_sec', 1.0)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('fp16', True)
        self.declare_parameter('target_classes', '')
        self.declare_parameter('max_camera_failures', 30)
        self.declare_parameter('fail_if_no_cuda', True)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', 'yolo_tensorrt/debug_image')

        self.model_path = Path(
            os.path.expanduser(str(self.get_parameter('model_path').value))
        ).resolve()
        self.engine_path_text = str(self.get_parameter('engine_path').value).strip()
        self.force_export = as_bool(self.get_parameter('force_export').value)
        self.camera_device = int(self.get_parameter('camera_device').value)
        self.camera_path = str(self.get_parameter('camera_path').value).strip()
        legacy_camera = self.get_parameter('camera').value
        self.camera_setup = self._camera_setup_from_params(legacy_camera)
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.camera_fps = float(self.get_parameter('camera_fps').value)
        self.camera_flush_reads = max(1, int(self.get_parameter('camera_flush_reads').value))
        self.output_path = Path(
            os.path.expanduser(str(self.get_parameter('output_path').value))
        ).resolve()
        self.interval_sec = max(0.05, float(self.get_parameter('interval_sec').value))
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.fp16 = as_bool(self.get_parameter('fp16').value)
        self.target_classes = string_set_from_param(
            self.get_parameter('target_classes').value
        )
        self.max_camera_failures = max(
            1, int(self.get_parameter('max_camera_failures').value)
        )
        self.fail_if_no_cuda = as_bool(self.get_parameter('fail_if_no_cuda').value)
        self.publish_debug_image = as_bool(
            self.get_parameter('publish_debug_image').value
        )
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)

        self.camera_failures = 0
        self.frame_id = 'camera'
        self.cv_bridge = CvBridge()
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 2)

        self._log_gpu_report()
        self.YOLO = import_ultralytics_yolo()
        self.engine_path = self._resolve_engine_path()
        self._export_engine_if_needed()
        self.model = self.YOLO(str(self.engine_path))
        self.cap = self._open_camera()

        self.timer = self.create_timer(self.interval_sec, self._timer_callback)
        self.get_logger().info(
            'TensorRT YOLO camera test running: camera=%s engine=%s output=%s interval=%.2fs'
            % (self.camera_setup, self.engine_path, self.output_path, self.interval_sec)
        )

    def _camera_setup_from_params(self, legacy_camera: Any) -> int | str:
        if legacy_camera not in ('', None):
            return legacy_camera
        if self.camera_path:
            return os.path.expanduser(self.camera_path)
        return self.camera_device

    def _log_gpu_report(self) -> None:
        report = get_gpu_report()
        logger = self.get_logger()
        logger.info('GPU check:')
        logger.info('  Jetson model       : %s' % (report['jetson_model'] or '<unknown>'))
        logger.info(
            '  /dev/nvhost-gpu    : %s'
            % ('present' if report['nvhost_gpu'] else 'missing')
        )
        logger.info('  torch imported     : %s' % report['torch_imported'])
        logger.info('  torch CUDA visible : %s' % report['torch_cuda'])
        if report['cuda_device_name']:
            logger.info('  CUDA device        : %s' % report['cuda_device_name'])

        hardware_looks_like_jetson = bool(report['nvhost_gpu']) or (
            'nvidia' in str(report['jetson_model']).lower()
        )
        if not hardware_looks_like_jetson:
            raise RuntimeError('No Jetson/NVIDIA GPU hardware was detected.')
        if self.fail_if_no_cuda and not report['torch_cuda']:
            raise RuntimeError(
                'NVIDIA hardware is present, but PyTorch cannot see CUDA. '
                'Install CUDA-enabled torch, TensorRT, and Ultralytics for this '
                'Python environment.'
            )

    def _resolve_engine_path(self) -> Path:
        if self.model_path.suffix == '.engine' and not self.engine_path_text:
            return self.model_path
        if self.engine_path_text:
            return Path(os.path.expanduser(self.engine_path_text)).resolve()
        return default_engine_path(self.model_path, self.imgsz, self.fp16)

    def _export_engine_if_needed(self) -> None:
        if self.model_path.suffix == '.engine' and self.model_path == self.engine_path:
            self.get_logger().info('Using existing TensorRT engine: %s' % self.engine_path)
            return

        if self.engine_path.is_file() and not self.force_export:
            self.get_logger().info('TensorRT engine already exists: %s' % self.engine_path)
            return

        if not self.model_path.is_file():
            raise FileNotFoundError('YOLO model not found: %s' % self.model_path)
        check_tensorrt_export_dependencies()

        self.engine_path.parent.mkdir(parents=True, exist_ok=True)
        cached_pt = self.engine_path.with_suffix('.pt')
        if cached_pt.resolve() != self.model_path.resolve():
            shutil.copy2(self.model_path, cached_pt)
            export_source = cached_pt
        else:
            export_source = self.model_path

        logger = self.get_logger()
        logger.info('Exporting TensorRT engine from %s' % self.model_path)
        logger.info('  export source : %s' % export_source)
        logger.info('  output        : %s' % self.engine_path)
        logger.info('  precision     : %s' % ('FP16' if self.fp16 else 'FP32'))

        model = self.YOLO(str(export_source))
        exported = model.export(
            format='engine',
            imgsz=self.imgsz,
            half=self.fp16,
            device=0,
            batch=1,
            dynamic=False,
            simplify=False,
            verbose=False,
        )
        exported_path = Path(str(exported)).expanduser()
        if not exported_path.is_absolute():
            exported_path = (Path.cwd() / exported_path).resolve()

        if not exported_path.is_file():
            fallback = export_source.with_suffix('.engine')
            if fallback.is_file():
                exported_path = fallback
            else:
                raise RuntimeError('TensorRT export did not produce an engine file.')

        if exported_path.resolve() != self.engine_path.resolve():
            shutil.move(str(exported_path), str(self.engine_path))

        logger.info('Export complete: %s' % self.engine_path)

    def _open_camera(self) -> cv2.VideoCapture:
        try:
            camera_device_index = int(self.camera_setup)
            cap = cv2.VideoCapture(camera_device_index, cv2.CAP_V4L2)
            if not cap.isOpened():
                self.get_logger().warn(
                    'Failed to open camera with CAP_V4L2 for device index %d. '
                    'Falling back to default backend.' % camera_device_index
                )
                cap = cv2.VideoCapture(self.camera_setup)
        except (TypeError, ValueError):
            cap = cv2.VideoCapture(str(self.camera_setup))

        if not cap.isOpened():
            raise RuntimeError('Failed to open camera device/pipeline: %s' % self.camera_setup)

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.camera_width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        if self.camera_height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        if self.camera_fps > 0.0:
            cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            'Camera opened: %s (%.0fx%.0f @ %.1f FPS reported)'
            % (self.camera_setup, actual_width, actual_height, actual_fps)
        )
        return cap

    def _read_latest_frame(self):
        frame = None
        for _ in range(self.camera_flush_reads):
            ok, candidate = self.cap.read()
            if ok and candidate is not None:
                frame = candidate
        if frame is None:
            self.camera_failures += 1
            if self.camera_failures >= self.max_camera_failures:
                raise RuntimeError('Too many consecutive camera read failures.')
            return None
        self.camera_failures = 0
        return frame

    def _timer_callback(self) -> None:
        frame = self._read_latest_frame()
        if frame is None:
            return

        start = time.monotonic()
        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=0,
            verbose=False,
        )
        inference_ms = (time.monotonic() - start) * 1000.0
        detections = self._filtered_detections(results[0])
        annotated = self._draw_detections(frame, detections, inference_ms)
        atomic_write_image(self.output_path, annotated)
        self._publish_debug_image(annotated)

        best = max((det['confidence'] for det in detections), default=0.0)
        self.get_logger().info(
            'wrote %s | detections=%d best=%.3f inference=%.1f ms'
            % (self.output_path, len(detections), best, inference_ms)
        )

    def _filtered_detections(self, result) -> list[dict[str, Any]]:
        names = getattr(result, 'names', None) or getattr(self.model, 'names', {})
        detections: list[dict[str, Any]] = []
        if result.boxes is None or len(result.boxes) == 0:
            return detections

        xyxy = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(int)
        for box, confidence, class_id in zip(xyxy, confs, class_ids):
            class_name = str(names.get(int(class_id), int(class_id)))
            if self.target_classes and class_name not in self.target_classes:
                continue
            xmin, ymin, xmax, ymax = [int(round(value)) for value in box.tolist()]
            detections.append({
                'class_name': class_name,
                'class_id': int(class_id),
                'confidence': float(confidence),
                'bbox': (xmin, ymin, xmax, ymax),
            })
        return detections

    def _draw_detections(
        self,
        frame,
        detections: list[dict[str, Any]],
        inference_ms: float,
    ):
        annotated = frame.copy()
        for det in detections:
            xmin, ymin, xmax, ymax = det['bbox']
            color = (0, 220, 0)
            cv2.rectangle(annotated, (xmin, ymin), (xmax, ymax), color, 2)
            label = '%s %.2f' % (det['class_name'], det['confidence'])
            cv2.putText(
                annotated,
                label,
                (xmin, max(18, ymin - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            annotated,
            'TensorRT %.1f ms  detections=%d' % (inference_ms, len(detections)),
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 220, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated

    def _publish_debug_image(self, annotated) -> None:
        if self.debug_pub is None:
            return
        try:
            msg = self.cv_bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn('Debug image conversion failed: %s' % exc)
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.debug_pub.publish(msg)

    def destroy_node(self) -> None:
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = TensorRTCameraTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print('ERROR: %s' % exc, file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
