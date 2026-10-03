#!/bin/bash
# =============================================================
# Tuned arc / semicircle path-following test for ros2_ws.
#
# Default path:
#   center=(0, 2), radius=2 m, start=-90 deg, end=+90 deg
#   start=(0, 0), initial_yaw=0, goal=(0, 4)
#
# Usage:
#   ./src/asclinic_pkg/scripts/run_arc_path_tuned.sh
#   ./src/asclinic_pkg/scripts/run_arc_path_tuned.sh --right-turn
#   ./src/asclinic_pkg/scripts/run_arc_path_tuned.sh --right-trim 0.96
#   ./src/asclinic_pkg/scripts/run_arc_path_tuned.sh --stop
# =============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS=${WS:-$(cd "$SCRIPT_DIR/../../.." && pwd)}
PKG_DIR="$WS/src/asclinic_pkg"
MARKER_EDITOR="$PKG_DIR/src/nodes/tools/marker_map_editor.py"
MARKER_MAP_FILE="$PKG_DIR/config/marker_map_final.csv"
ROBOCLAW_USB_PORT=/dev/ttyACM0
LAUNCH_CV_LOCALIZATION=false
ODOM_TOPIC=wheel_odometry
CAMERA_DEVICE=0
CAMERA_FPS=5
LAUNCH_TRACKING_CONTROLLER=true
LAUNCH_LQG_CONTROLLER=false
EDIT_MARKERS=false
PRINT_MARKER_MAP=false
BUILD_FIRST=true

INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=0.0

CENTER_X=0.0
CENTER_Y=2.0
RADIUS=2.0
START_ANGLE=-1.57079632679
END_ANGLE=1.57079632679
CLOCKWISE=false

NOMINAL_SPEED=0.10
DUTY_CYCLE_LIMIT=28.0
MIN_MOVING_DUTY=18.0
MAX_WHEEL_SPEED_AT_FULL_DUTY=1.50
DUTY_SLEW_RATE=35.0
COMMAND_FILTER_ALPHA=0.55
LOOKAHEAD_DISTANCE=0.35
CURVATURE_GAIN=1.25
GOAL_TOLERANCE=0.12
SLOWDOWN_DISTANCE=1.0
LEFT_TRIM=1.0
RIGHT_TRIM=0.95

SHOW_LIVE_PLOT=true
CONTROL_VERBOSE=true

usage() {
    cat <<EOF
Usage:
  $0 [options]
  $0 --stop

Path options:
  --left-turn             Default semicircle: center=(0,2), start=(0,0), goal=(0,4).
  --right-turn            Mirrored semicircle: center=(0,-2), start=(0,0), goal=(0,-4).
  --radius R              Arc radius in meters. Default: $RADIUS
  --center-x X            Arc center x. Default: $CENTER_X
  --center-y Y            Arc center y. Default: $CENTER_Y
  --start-angle RAD       Arc start angle. Default: $START_ANGLE
  --end-angle RAD         Arc end angle. Default: $END_ANGLE
  --clockwise BOOL        true/false. Default: $CLOCKWISE

Initial pose:
  --initial-x X           Default: $INITIAL_X
  --initial-y Y           Default: $INITIAL_Y
  --initial-yaw RAD       Default: $INITIAL_YAW

Controller tuning:
  --speed V               nominal_speed. Default: $NOMINAL_SPEED
  --lookahead L           lookahead_distance. Default: $LOOKAHEAD_DISTANCE
  --curvature-gain G      curvature_gain. Default: $CURVATURE_GAIN
  --filter A              command_filter_alpha. Default: $COMMAND_FILTER_ALPHA
  --slew S                duty slew rate percent/sec. Default: $DUTY_SLEW_RATE
  --goal-tol D            goal_tolerance. Default: $GOAL_TOLERANCE
  --slowdown D            slowdown_distance. Default: $SLOWDOWN_DISTANCE
  --right-trim R          right_trim. Default: $RIGHT_TRIM
  --left-trim L           left_trim. Default: $LEFT_TRIM
  --min-duty D            min_moving_duty. Default: $MIN_MOVING_DUTY
  --duty-limit D          duty_cycle_limit. Default: $DUTY_CYCLE_LIMIT

Hardware/logging:
  --no-build              Do not build before launch.
  --port DEV              RoboClaw port. Default: $ROBOCLAW_USB_PORT
  --cv                    Enable ArUco + pitt_fused_odometry and control on fused odom.
  --lqg                   Use LQG controller instead of tracking_controller_node.
  --marker-map-file PATH  CSV marker map for Pitt CV. Default: $MARKER_MAP_FILE
  --edit-markers          Open simple marker-map UI and exit.
  --print-marker-map      Print id:x,y,z,yaw;... marker string and exit.
  --camera-device N       Camera device for ArUco. Default: $CAMERA_DEVICE
  --camera-fps N          Camera FPS for ArUco. Default: $CAMERA_FPS
  --no-live               Disable live matplotlib window; still saves plots.
  --quiet-control         Disable verbose controller logs.
EOF
}

stop_nodes() {
    pkill -f "roboclaw_for_motors" 2>/dev/null
    pkill -f "liu_odometry_from_encoders" 2>/dev/null
    pkill -f "tracking_controller_node" 2>/dev/null
    pkill -f "lqg_controller_test" 2>/dev/null
    pkill -f "arc_path_publisher" 2>/dev/null
    pkill -f "path_tracking_diagnostics" 2>/dev/null
}

if [[ "${1:-}" == "--stop" ]]; then
    echo "[STOP] Killing arc path test nodes..."
    stop_nodes
    echo "[STOP] Done."
    exit 0
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help|-h)
            usage
            exit 0
            ;;
        --left-turn)
            CENTER_X=0.0
            CENTER_Y=2.0
            START_ANGLE=-1.57079632679
            END_ANGLE=1.57079632679
            CLOCKWISE=false
            shift
            ;;
        --right-turn)
            CENTER_X=0.0
            CENTER_Y=-2.0
            START_ANGLE=1.57079632679
            END_ANGLE=-1.57079632679
            CLOCKWISE=true
            shift
            ;;
        --radius)
            RADIUS="$2"
            shift 2
            ;;
        --center-x)
            CENTER_X="$2"
            shift 2
            ;;
        --center-y)
            CENTER_Y="$2"
            shift 2
            ;;
        --start-angle)
            START_ANGLE="$2"
            shift 2
            ;;
        --end-angle)
            END_ANGLE="$2"
            shift 2
            ;;
        --clockwise)
            CLOCKWISE="$2"
            shift 2
            ;;
        --initial-x)
            INITIAL_X="$2"
            shift 2
            ;;
        --initial-y)
            INITIAL_Y="$2"
            shift 2
            ;;
        --initial-yaw)
            INITIAL_YAW="$2"
            shift 2
            ;;
        --speed)
            NOMINAL_SPEED="$2"
            shift 2
            ;;
        --lookahead)
            LOOKAHEAD_DISTANCE="$2"
            shift 2
            ;;
        --curvature-gain)
            CURVATURE_GAIN="$2"
            shift 2
            ;;
        --filter)
            COMMAND_FILTER_ALPHA="$2"
            shift 2
            ;;
        --slew)
            DUTY_SLEW_RATE="$2"
            shift 2
            ;;
        --goal-tol)
            GOAL_TOLERANCE="$2"
            shift 2
            ;;
        --slowdown)
            SLOWDOWN_DISTANCE="$2"
            shift 2
            ;;
        --right-trim)
            RIGHT_TRIM="$2"
            shift 2
            ;;
        --left-trim)
            LEFT_TRIM="$2"
            shift 2
            ;;
        --min-duty)
            MIN_MOVING_DUTY="$2"
            shift 2
            ;;
        --duty-limit)
            DUTY_CYCLE_LIMIT="$2"
            shift 2
            ;;
        --port)
            ROBOCLAW_USB_PORT="$2"
            shift 2
            ;;
        --cv)
            LAUNCH_CV_LOCALIZATION=true
            ODOM_TOPIC=pitt_fused_odometry
            shift
            ;;
        --lqg)
            LAUNCH_TRACKING_CONTROLLER=false
            LAUNCH_LQG_CONTROLLER=true
            shift
            ;;
        --marker-map-file)
            MARKER_MAP_FILE="$2"
            shift 2
            ;;
        --edit-markers)
            EDIT_MARKERS=true
            shift
            ;;
        --print-marker-map)
            PRINT_MARKER_MAP=true
            shift
            ;;
        --camera-device)
            CAMERA_DEVICE="$2"
            shift 2
            ;;
        --camera-fps)
            CAMERA_FPS="$2"
            shift 2
            ;;
        --no-live)
            SHOW_LIVE_PLOT=false
            shift
            ;;
        --quiet-control)
            CONTROL_VERBOSE=false
            shift
            ;;
        --no-build)
            BUILD_FIRST=false
            shift
            ;;
        *)
            echo "[ERROR] Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

MARKER_MAP_FILE="$(realpath -m "$MARKER_MAP_FILE")"

if [[ "$EDIT_MARKERS" == true ]]; then
    python3 "$MARKER_EDITOR" --file "$MARKER_MAP_FILE"
    exit $?
fi

MARKER_WORLD_MAP="$(python3 "$MARKER_EDITOR" --file "$MARKER_MAP_FILE" --print-string)"

if [[ "$PRINT_MARKER_MAP" == true ]]; then
    printf '%s\n' "$MARKER_WORLD_MAP"
    exit 0
fi

echo "============================================"
echo " Tuned arc path test"
echo " Workspace       : $WS"
echo " RoboClaw port   : $ROBOCLAW_USB_PORT"
echo " Controller      : $([ "$LAUNCH_LQG_CONTROLLER" == true ] && echo LQG || echo tracking)"
echo " CV localization : $LAUNCH_CV_LOCALIZATION (odom_topic=$ODOM_TOPIC, camera=$CAMERA_DEVICE @ ${CAMERA_FPS}fps)"
echo " Marker map      : $MARKER_MAP_FILE"
echo " Build first     : $BUILD_FIRST"
echo " Initial pose    : ($INITIAL_X, $INITIAL_Y, $INITIAL_YAW)"
echo " Arc             : center=($CENTER_X, $CENTER_Y), radius=$RADIUS, angles=($START_ANGLE -> $END_ANGLE), clockwise=$CLOCKWISE"
echo " Control         : speed=$NOMINAL_SPEED, lookahead=$LOOKAHEAD_DISTANCE, curvature_gain=$CURVATURE_GAIN"
echo " Smoothing       : filter=$COMMAND_FILTER_ALPHA, slew=$DUTY_SLEW_RATE, slowdown=$SLOWDOWN_DISTANCE"
echo " Wheel trims     : left=$LEFT_TRIM, right=$RIGHT_TRIM"
echo " Results         : $WS/results/arc_path_test/<timestamp>"
echo "============================================"

cd "$WS"
source /opt/ros/humble/setup.bash

if [[ "$BUILD_FIRST" == true ]]; then
    colcon build --packages-select asclinic_pkg
fi

source install/setup.bash

ros2 launch asclinic_pkg arc_path_test.launch.py \
  roboclaw_usb_port:="$ROBOCLAW_USB_PORT" \
  launch_cv_localization:="$LAUNCH_CV_LOCALIZATION" \
  marker_world_map:="$MARKER_WORLD_MAP" \
  camera_device:="$CAMERA_DEVICE" \
  camera_fps:="$CAMERA_FPS" \
  initial_x:="$INITIAL_X" \
  initial_y:="$INITIAL_Y" \
  initial_yaw:="$INITIAL_YAW" \
  center_x:="$CENTER_X" \
  center_y:="$CENTER_Y" \
  radius:="$RADIUS" \
  start_angle:="$START_ANGLE" \
  end_angle:="$END_ANGLE" \
  clockwise:="$CLOCKWISE" \
  odom_topic:="$ODOM_TOPIC" \
  launch_tracking_controller:="$LAUNCH_TRACKING_CONTROLLER" \
  launch_lqg_controller:="$LAUNCH_LQG_CONTROLLER" \
  nominal_speed:="$NOMINAL_SPEED" \
  duty_cycle_limit:="$DUTY_CYCLE_LIMIT" \
  min_moving_duty:="$MIN_MOVING_DUTY" \
  max_wheel_speed_at_full_duty:="$MAX_WHEEL_SPEED_AT_FULL_DUTY" \
  duty_slew_rate_percent_per_sec:="$DUTY_SLEW_RATE" \
  command_filter_alpha:="$COMMAND_FILTER_ALPHA" \
  lookahead_distance:="$LOOKAHEAD_DISTANCE" \
  curvature_gain:="$CURVATURE_GAIN" \
  goal_tolerance:="$GOAL_TOLERANCE" \
  slowdown_distance:="$SLOWDOWN_DISTANCE" \
  left_trim:="$LEFT_TRIM" \
  right_trim:="$RIGHT_TRIM" \
  show_live_plot:="$SHOW_LIVE_PLOT" \
  control_verbose:="$CONTROL_VERBOSE"
