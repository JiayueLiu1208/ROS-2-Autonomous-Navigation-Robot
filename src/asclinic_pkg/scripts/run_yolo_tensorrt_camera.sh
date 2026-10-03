#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run the standalone TensorRT YOLO camera test node.

This wrapper builds/sources the workspace, checks GPU/CUDA inside the node,
exports models/plant_detector/best.pt to a TensorRT .engine if needed, then
overwrites one annotated image once per second.

Usage:
  ./src/asclinic_pkg/scripts/run_yolo_tensorrt_camera.sh [options]

Options:
  --no-build              Do not run colcon build before ros2 run.
  --model PATH            YOLO .pt or .engine path. Default: installed best.pt.
  --engine PATH           TensorRT engine output/cache path.
  --force-export          Rebuild the TensorRT engine.
  --camera N|PATH         Camera index or device path. Default: 0.
  --width PX              Optional camera width.
  --height PX             Optional camera height.
  --camera-fps FPS        Optional camera FPS request.
  --output PATH           Annotated image to overwrite.
                           Default: <ros2_ws>/results/yolo_tensorrt/latest.jpg
  --interval SEC          Seconds between inference/write cycles. Default: 1.0
  --imgsz N               TensorRT/export image size. Default: 640
  --confidence X          YOLO confidence threshold. Default: 0.25
  --iou X                 YOLO IoU threshold. Default: 0.45
  --fp32                  Export/run FP32 instead of default FP16.
  --classes LIST          Comma-separated class names to keep.
  --publish-debug-image   Publish annotated frames on a ROS Image topic too.
  --debug-topic TOPIC     Debug Image topic. Default: yolo_tensorrt/debug_image
  --allow-no-cuda         Do not fail if torch cannot see CUDA.
  --dry-run               Print the ros2 run command without executing it.
  --stop                  Stop the TensorRT camera test node.
  -h, --help              Show this help text.

Examples:
  ./src/asclinic_pkg/scripts/run_yolo_tensorrt_camera.sh --camera 0

  ./src/asclinic_pkg/scripts/run_yolo_tensorrt_camera.sh \
    --force-export \
    --output ./results/yolo_tensorrt/latest.jpg \
    --publish-debug-image
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

BUILD_FIRST=true
MODEL_PATH=""
ENGINE_PATH=""
FORCE_EXPORT=false
CAMERA="0"
CAMERA_WIDTH="0"
CAMERA_HEIGHT="0"
CAMERA_FPS="0.0"
OUTPUT_PATH="/tmp/asclinic_yolo_tensorrt_latest.jpg"
OUTPUT_PATH_SET=false
INTERVAL_SEC="1.0"
IMGSZ="640"
CONFIDENCE="0.25"
IOU="0.45"
FP16=true
TARGET_CLASSES=""
PUBLISH_DEBUG_IMAGE=false
DEBUG_TOPIC="yolo_tensorrt/debug_image"
FAIL_IF_NO_CUDA=true
DRY_RUN=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stop)
      echo "[STOP] Killing TensorRT YOLO camera test node..."
      pkill -f "test_yolo_tensorrt_camera" 2>/dev/null || true
      echo "[STOP] Done."
      exit 0
      ;;
    --no-build)
      BUILD_FIRST=false
      shift
      ;;
    --model)
      MODEL_PATH="${2:?missing value for --model}"
      shift 2
      ;;
    --engine)
      ENGINE_PATH="${2:?missing value for --engine}"
      shift 2
      ;;
    --force-export)
      FORCE_EXPORT=true
      shift
      ;;
    --camera)
      CAMERA="${2:?missing value for --camera}"
      shift 2
      ;;
    --width)
      CAMERA_WIDTH="${2:?missing value for --width}"
      shift 2
      ;;
    --height)
      CAMERA_HEIGHT="${2:?missing value for --height}"
      shift 2
      ;;
    --camera-fps)
      CAMERA_FPS="${2:?missing value for --camera-fps}"
      shift 2
      ;;
    --output)
      OUTPUT_PATH="${2:?missing value for --output}"
      OUTPUT_PATH_SET=true
      shift 2
      ;;
    --interval)
      INTERVAL_SEC="${2:?missing value for --interval}"
      shift 2
      ;;
    --imgsz)
      IMGSZ="${2:?missing value for --imgsz}"
      shift 2
      ;;
    --confidence)
      CONFIDENCE="${2:?missing value for --confidence}"
      shift 2
      ;;
    --iou)
      IOU="${2:?missing value for --iou}"
      shift 2
      ;;
    --fp32)
      FP16=false
      shift
      ;;
    --classes)
      TARGET_CLASSES="${2:?missing value for --classes}"
      shift 2
      ;;
    --publish-debug-image)
      PUBLISH_DEBUG_IMAGE=true
      shift
      ;;
    --debug-topic)
      DEBUG_TOPIC="${2:?missing value for --debug-topic}"
      shift 2
      ;;
    --allow-no-cuda)
      FAIL_IF_NO_CUDA=false
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

source_ros() {
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    return
  fi

  for distro in humble iron jazzy; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "/opt/ros/${distro}/setup.bash"
      set -u
      return
    fi
  done

  echo "Could not find /opt/ros/<distro>/setup.bash. Source ROS 2 first." >&2
  exit 1
}

source_ros
cd "$WS_ROOT"

if [[ "$OUTPUT_PATH_SET" == false ]]; then
  OUTPUT_PATH="${WS_ROOT}/results/yolo_tensorrt/latest.jpg"
fi

if [[ "$BUILD_FIRST" == true ]]; then
  if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon is not available. Install/source ROS 2 tools, or use --no-build if already built." >&2
    exit 1
  fi
  colcon build --packages-select asclinic_pkg
fi

if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
  echo "Workspace install/setup.bash not found. Build first or remove --no-build." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"
set -u

ros_args=(
  -p "force_export:=${FORCE_EXPORT}"
  -p "camera_width:=${CAMERA_WIDTH}"
  -p "camera_height:=${CAMERA_HEIGHT}"
  -p "camera_fps:=${CAMERA_FPS}"
  -p "output_path:=${OUTPUT_PATH}"
  -p "interval_sec:=${INTERVAL_SEC}"
  -p "imgsz:=${IMGSZ}"
  -p "confidence_threshold:=${CONFIDENCE}"
  -p "iou_threshold:=${IOU}"
  -p "fp16:=${FP16}"
  -p "publish_debug_image:=${PUBLISH_DEBUG_IMAGE}"
  -p "debug_image_topic:=${DEBUG_TOPIC}"
  -p "fail_if_no_cuda:=${FAIL_IF_NO_CUDA}"
)

if [[ "$CAMERA" =~ ^[0-9]+$ ]]; then
  ros_args+=(-p "camera_device:=${CAMERA}")
else
  ros_args+=(-p "camera_path:=${CAMERA}")
fi

if [[ -n "$MODEL_PATH" ]]; then
  ros_args+=(-p "model_path:=${MODEL_PATH}")
fi

if [[ -n "$ENGINE_PATH" ]]; then
  ros_args+=(-p "engine_path:=${ENGINE_PATH}")
fi

if [[ -n "$TARGET_CLASSES" ]]; then
  ros_args+=(-p "target_classes:=${TARGET_CLASSES}")
fi

cmd=(ros2 run asclinic_pkg test_yolo_tensorrt_camera --ros-args "${ros_args[@]}")

echo "Launching TensorRT YOLO camera test from: $WS_ROOT"
echo "Camera: ${CAMERA} (${CAMERA_WIDTH}x${CAMERA_HEIGHT}, fps=${CAMERA_FPS})"
echo "Output image: ${OUTPUT_PATH} every ${INTERVAL_SEC}s"
echo "TensorRT: fp16=${FP16}, imgsz=${IMGSZ}, force_export=${FORCE_EXPORT}, engine=${ENGINE_PATH:-auto-cache}"
echo "Debug image topic: ${PUBLISH_DEBUG_IMAGE} (${DEBUG_TOPIC})"

if [[ "$DRY_RUN" == true ]]; then
  printf 'Command:'
  printf ' %q' "${cmd[@]}"
  printf '\n'
  exit 0
fi

exec "${cmd[@]}"
