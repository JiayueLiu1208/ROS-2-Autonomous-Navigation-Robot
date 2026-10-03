#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run ros2_ws final path following with real CV localization and LQG.

Pipeline:
  RoboClaw + wheel odom + ArUco detector + Pitt fused odometry +
  final map/planner + LQG path controller

Usage:
  ./src/asclinic_pkg/scripts/run_final_lqg_cv.sh [options]

Common options:
  --no-build                 Do not build before launch.
  --edit-markers             Open the simple marker-map UI and exit.
  --print-marker-map         Print the ROS marker map string and exit.
  --marker-map-file PATH     CSV marker file. Default: config/marker_map_final.csv
  --wheel-odom               Control on wheel_odometry for comparison.
  --plant-detector           Also launch YOLO plant detector.
  --viewer                   Also launch the live global map viewer.
  --no-camera-upright        Do not move the camera servo before launch.
  --camera-pan-us US         Camera pan pulse. Default: 1548
  --camera-tilt-us US        Camera upright tilt pulse. Default: 1500
  --camera-device N          Camera device. Default: 0
  --camera-fps FPS           ArUco camera FPS. Default: 10
  --roboclaw-port PATH       RoboClaw serial port. Default: /dev/ttyACM0
  --initial-x X              Initial odom x. Default: 0.50
  --initial-y Y              Initial odom y. Default: 0.50
  --initial-yaw RAD          Initial odom yaw. Default: 1.5708
  --speed MPS                LQG cruise speed. Default: 0.12
  --lookahead M              LQG lookahead. Default: 0.55
  --duty-limit PERCENT       Motor duty cap. Default: 32.0
  --min-duty PERCENT         Minimum moving duty. Default: 18.0
  --right-trim R             Right motor trim. Default: 0.95
  --left-trim L              Left motor trim. Default: 1.0
  --filter A                 Command low-pass alpha. Default: 0.45
  --slew PCT_PER_SEC         Duty slew limit. Default: 45.0
  --control-verbose          Print LQG controller debug logs.
  --dry-run                  Print the launch command without executing.
  --stop                     Stop likely nodes from this stack.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
PKG_DIR="${WS_ROOT}/src/asclinic_pkg"
MARKER_EDITOR="${PKG_DIR}/src/nodes/tools/marker_map_editor.py"
LOG_DIR="${PKG_DIR}/logs"

BUILD_FIRST=true
EDIT_MARKERS=false
PRINT_MARKER_MAP=false
DRY_RUN=false
LAUNCH_PLANT_DETECTOR=false
LAUNCH_MAP_VIEWER=false
CAMERA_UPRIGHT=true
CONTROL_VERBOSE=false
NAMESPACE="asc"
CAMERA_DEVICE=0
CAMERA_FPS=10
ROBOCLAW_PORT="/dev/ttyACM0"
MARKER_MAP_FILE="${PKG_DIR}/config/marker_map_final.csv"
ODOM_TOPIC="pitt_fused_odometry"
INITIAL_X="0.50"
INITIAL_Y="0.50"
INITIAL_YAW="1.5708"
V_REF="0.12"
LOOKAHEAD="0.55"
DUTY_LIMIT="32.0"
MIN_DUTY="18.0"
LEFT_TRIM="1.0"
RIGHT_TRIM="0.95"
COMMAND_FILTER_ALPHA="0.45"
DUTY_SLEW="45.0"
SERVO_PAN_CHANNEL=14
SERVO_TILT_CHANNEL=13
SERVO_PAN_UPRIGHT_US=1548
SERVO_TILT_UPRIGHT_US=1500
SERVO_PID=""

stop_nodes() {
  pkill -f "i2c_for_servos" 2>/dev/null || true
  pkill -f "roboclaw_for_motors" 2>/dev/null || true
  pkill -f "liu_odometry_from_encoders" 2>/dev/null || true
  pkill -f "aruco_detector" 2>/dev/null || true
  pkill -f "pitt_fused_odemetry" 2>/dev/null || true
  pkill -f "plant_detector_node" 2>/dev/null || true
  pkill -f "system_status_publisher" 2>/dev/null || true
  pkill -f "path_planning/map.py" 2>/dev/null || true
  pkill -f "path_planning/global_planner.py" 2>/dev/null || true
  pkill -f "lqg_controller_test" 2>/dev/null || true
  pkill -f "live_global_map_viewer" 2>/dev/null || true
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --stop)
      echo "[STOP] Killing final LQG/CV nodes..."
      stop_nodes
      echo "[STOP] Done."
      exit 0
      ;;
    --no-build)
      BUILD_FIRST=false
      shift
      ;;
    --edit-markers)
      EDIT_MARKERS=true
      shift
      ;;
    --print-marker-map)
      PRINT_MARKER_MAP=true
      shift
      ;;
    --marker-map-file)
      MARKER_MAP_FILE="${2:?missing value for --marker-map-file}"
      shift 2
      ;;
    --wheel-odom)
      ODOM_TOPIC="wheel_odometry"
      shift
      ;;
    --plant-detector)
      LAUNCH_PLANT_DETECTOR=true
      shift
      ;;
    --viewer)
      LAUNCH_MAP_VIEWER=true
      shift
      ;;
    --no-camera-upright)
      CAMERA_UPRIGHT=false
      shift
      ;;
    --camera-pan-us)
      SERVO_PAN_UPRIGHT_US="${2:?missing value for --camera-pan-us}"
      shift 2
      ;;
    --camera-tilt-us)
      SERVO_TILT_UPRIGHT_US="${2:?missing value for --camera-tilt-us}"
      shift 2
      ;;
    --camera-device)
      CAMERA_DEVICE="${2:?missing value for --camera-device}"
      shift 2
      ;;
    --camera-fps)
      CAMERA_FPS="${2:?missing value for --camera-fps}"
      shift 2
      ;;
    --roboclaw-port|--port)
      ROBOCLAW_PORT="${2:?missing value for --roboclaw-port}"
      shift 2
      ;;
    --namespace)
      NAMESPACE="${2:?missing value for --namespace}"
      shift 2
      ;;
    --initial-x)
      INITIAL_X="${2:?missing value for --initial-x}"
      shift 2
      ;;
    --initial-y)
      INITIAL_Y="${2:?missing value for --initial-y}"
      shift 2
      ;;
    --initial-yaw)
      INITIAL_YAW="${2:?missing value for --initial-yaw}"
      shift 2
      ;;
    --speed)
      V_REF="${2:?missing value for --speed}"
      shift 2
      ;;
    --lookahead)
      LOOKAHEAD="${2:?missing value for --lookahead}"
      shift 2
      ;;
    --duty-limit)
      DUTY_LIMIT="${2:?missing value for --duty-limit}"
      shift 2
      ;;
    --min-duty)
      MIN_DUTY="${2:?missing value for --min-duty}"
      shift 2
      ;;
    --right-trim)
      RIGHT_TRIM="${2:?missing value for --right-trim}"
      shift 2
      ;;
    --left-trim)
      LEFT_TRIM="${2:?missing value for --left-trim}"
      shift 2
      ;;
    --filter)
      COMMAND_FILTER_ALPHA="${2:?missing value for --filter}"
      shift 2
      ;;
    --slew)
      DUTY_SLEW="${2:?missing value for --slew}"
      shift 2
      ;;
    --control-verbose)
      CONTROL_VERBOSE=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

MARKER_MAP_FILE="$(realpath -m "$MARKER_MAP_FILE")"

if [[ "$EDIT_MARKERS" == true ]]; then
  python3 "$MARKER_EDITOR" --file "$MARKER_MAP_FILE"
  exit $?
fi

MARKER_WORLD_MAP="$(python3 "$MARKER_EDITOR" --file "$MARKER_MAP_FILE" --print-string)"
if [[ -z "$MARKER_WORLD_MAP" ]]; then
  echo "Marker map is empty: $MARKER_MAP_FILE" >&2
  exit 1
fi

if [[ "$PRINT_MARKER_MAP" == true ]]; then
  printf '%s\n' "$MARKER_WORLD_MAP"
  exit 0
fi

source_ros() {
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    return
  fi

  for distro in humble iron jazzy; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      # shellcheck disable=SC1090
      set +u
      source "/opt/ros/${distro}/setup.bash"
      set -u
      return
    fi
  done

  echo "Could not find /opt/ros/<distro>/setup.bash." >&2
  exit 1
}

servo_topic() {
  local ns="${NAMESPACE#/}"
  if [[ -n "$ns" && "$ns" != "/" ]]; then
    printf '/%s/set_servo_pulse_width' "$ns"
  else
    printf '/set_servo_pulse_width'
  fi
}

set_camera_upright() {
  if [[ "$CAMERA_UPRIGHT" != true ]]; then
    echo "Camera servo: skipped (--no-camera-upright)."
    return
  fi

  mkdir -p "$LOG_DIR"
  local servo_log="${LOG_DIR}/servo_final_lqg_cv.log"
  : > "$servo_log"

  echo "Camera servo: pan ch${SERVO_PAN_CHANNEL} -> ${SERVO_PAN_UPRIGHT_US}us, tilt ch${SERVO_TILT_CHANNEL} -> ${SERVO_TILT_UPRIGHT_US}us"
  pkill -f "i2c_for_servos" 2>/dev/null || true
  sleep 0.3

  ros2 launch asclinic_pkg i2c_for_servos_launch.py \
    servo_driver_verbosity:=1 \
    >> "$servo_log" 2>&1 &
  SERVO_PID=$!

  sleep 2

  local topic
  topic="$(servo_topic)"
  if ! timeout 4s ros2 topic pub --once "$topic" asclinic_pkg/msg/ServoPulseWidth "{channel: ${SERVO_PAN_CHANNEL}, pulse_width_in_microseconds: ${SERVO_PAN_UPRIGHT_US}}" >> "$servo_log" 2>&1; then
    echo "Camera servo warning: pan command failed. See $servo_log"
  fi
  sleep 0.3
  if ! timeout 4s ros2 topic pub --once "$topic" asclinic_pkg/msg/ServoPulseWidth "{channel: ${SERVO_TILT_CHANNEL}, pulse_width_in_microseconds: ${SERVO_TILT_UPRIGHT_US}}" >> "$servo_log" 2>&1; then
    echo "Camera servo warning: tilt command failed. See $servo_log"
  fi
  echo "Camera servo: upright command sent. servo_pid=$SERVO_PID log=$servo_log"
}

source_ros
cd "$WS_ROOT"

if [[ "$BUILD_FIRST" == true ]]; then
  colcon build --packages-select asclinic_pkg
fi

if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
  echo "Workspace install/setup.bash not found. Build first or remove --no-build." >&2
  exit 1
fi

# shellcheck disable=SC1091
set +u
source "$WS_ROOT/install/setup.bash"
set -u

launch_args=(
  "namespace:=${NAMESPACE}"
  "launch_planning:=true"
  "launch_lqg:=true"
  "launch_plant_detector:=${LAUNCH_PLANT_DETECTOR}"
  "launch_map_viewer:=${LAUNCH_MAP_VIEWER}"
  "camera_device:=${CAMERA_DEVICE}"
  "camera_fps:=${CAMERA_FPS}"
  "roboclaw_usb_port:=${ROBOCLAW_PORT}"
  "initial_x:=${INITIAL_X}"
  "initial_y:=${INITIAL_Y}"
  "initial_yaw:=${INITIAL_YAW}"
  "marker_world_map:=${MARKER_WORLD_MAP}"
  "odom_topic:=${ODOM_TOPIC}"
  "v_ref:=${V_REF}"
  "lookahead_distance:=${LOOKAHEAD}"
  "duty_cycle_limit:=${DUTY_LIMIT}"
  "min_moving_duty:=${MIN_DUTY}"
  "left_trim:=${LEFT_TRIM}"
  "right_trim:=${RIGHT_TRIM}"
  "command_filter_alpha:=${COMMAND_FILTER_ALPHA}"
  "duty_slew_rate_percent_per_sec:=${DUTY_SLEW}"
  "control_verbose:=${CONTROL_VERBOSE}"
)

cmd=(ros2 launch asclinic_pkg final_lqg_cv.launch.py "${launch_args[@]}")

echo "Launching final LQG + real CV from: $WS_ROOT"
echo "Marker map file: $MARKER_MAP_FILE"
echo "Control odom: /${NAMESPACE}/${ODOM_TOPIC}"
echo "Camera upright: $CAMERA_UPRIGHT (pan=${SERVO_PAN_UPRIGHT_US}us, tilt=${SERVO_TILT_UPRIGHT_US}us)"
printf 'Command:'
printf ' %q' "${cmd[@]}"
printf '\n'

if [[ "$DRY_RUN" == true ]]; then
  exit 0
fi

set_camera_upright

exec "${cmd[@]}"
