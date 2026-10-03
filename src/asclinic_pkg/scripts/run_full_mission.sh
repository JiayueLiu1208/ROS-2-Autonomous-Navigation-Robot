#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run the ASClinic full mission stack.

Default stack:
  motors + ArUco camera + encoder/fused odometry + map/planner +
  tracking controller + gated plant detector

Usage:
  ./src/asclinic_pkg/scripts/run_full_mission.sh [options]

Options:
  --no-build                 Do not run colcon build before launch.
  --no-detector              Do not launch the plant detector node.
  --viewer                   Launch the live global map viewer.
  --map-gui                  Try to open the Matplotlib live map window.
                             Without this, --viewer saves live_global_map.png
                             only, which avoids SSH/X11 backend stalls.
  --map-viewer-odom TOPIC    Odometry topic used as the main robot pose in the
                             live map. Default: pitt_fused_odometry
  --map-plot-period SEC      Viewer redraw period. Default: 0.25
  --map-save-period SEC      live_global_map.png save period. Default: 0.75
  --map-no-png               Do not write live_global_map.png while running.
  --wheel-only               Use wheel_odometry only; disable ArUco/fused CV
                             localization and plant detector.
  --no-camera-upright        Do not move the camera servo before launch.
  --camera-pan-us US         Camera pan pulse. Default: 1548
  --camera-tilt-us US        Camera upright tilt pulse. Default: 1400
  --no-camera-auto-pan       Disable localisation-based plant camera pan.
  --camera-pan-min-us US     Auto-pan minimum pulse. Default: 500
  --camera-pan-max-us US     Auto-pan maximum pulse. Default: 2500
  --camera-pan-max-angle-deg D
                             Auto-pan angle represented by min/max pulses.
                             Default: 180
  --camera-pan-idle-center-period SEC
                             Re-send center while no plant target is active.
                             0 means center once only. Default: 0
  --camera-reveal-half-angle-deg D
                             Fog-of-war camera reveal half angle. Default: 30
  --fog-scan-max-range M     Max LiDAR range used to clear fog through the
                             camera FOV. Default: 5.0
  --plant-capture-radius M   Stop/search/center YOLO bbox within this map-plant
                             radius. Default: 0.60
  --no-plant-bbox-servo      Disable bbox-based camera centering/search.
  --detector-start-enabled   Start YOLO enabled immediately. This is now the
                             default because the controller keeps YOLO on.
  --save-plant-images        Save YOLO-positive camera frames with boxes into
                             results/live_global_map/<run_id>/plant_detections.
                             This is enabled by default.
  --no-save-plant-images     Do not save YOLO-positive plant images.
  --plant-debug-image        Publish annotated YOLO image topic for live viewing.
  --no-plant-debug-image     Disable annotated YOLO image topic. Default.
  --plant-image-dir DIR      Override the positive detection image directory.
  --plant-match-mode MODE    YOLO mission-completion matching: any or stop_id.
                             Default: any
  --plant-completion-conf X  Minimum confidence to complete a stop. Default: 0.60
  --plant-completion-dist M  Max robot-to-stop distance for completion.
                             Default: 1.60
  --plant-allow-hints        Allow completion without decision_hint=valid_capture.
  --camera-device N          Camera device number passed to launch. Default: 0
  --camera-fps FPS           Camera FPS passed to launch. Default: 10
  --roboclaw-port PATH       RoboClaw serial device. Default: /dev/ttyACM0
  --namespace NAME           ROS namespace. Default: asc
  --model-path PATH          Override plant detector YOLO model path.
  --initial-x X              Initial global x. Default: 0.50
  --initial-y Y              Initial global y. Default: 0.40
  --initial-yaw RAD          Initial global yaw. Default: 0.0
  --nominal-speed MPS        Controller cruise speed. Default: 0.20
  --max-linear-speed MPS     Controller maximum linear speed. Default: 0.35
  --min-moving-duty PERCENT  Minimum nonzero wheel duty for speed conversion.
                             Default: 18.0
  --duty-limit PERCENT       Controller per-wheel duty cap. Default: 40.0
  --left-trim SCALE          Left wheel duty multiplier. Default: 1.0
  --right-trim SCALE         Right wheel duty multiplier. Default: 0.932
  --lookahead M              Controller lookahead distance. Default: 0.45
  --controller MODE          Path controller backend: direct, lqg, mpc, pid.
                             Default: direct
  --lqg / --mpc / --pid      Convenience aliases for --controller.
  --backend-delta-v-limit MPS
                             LQG/MPC/PID max speed correction. Default: 0.30
  --backend-delta-omega-limit RADPS
                             LQG/MPC/PID max turn-rate correction. Default: 1.50
  --lqg-smooth-turn-angle DEG
                             LQG keeps full smooth tracking below this heading
                             error. Default: 30.0
  --lqg-stop-turn-angle DEG  LQG only stop-turns above this heading error.
                             Default: 60.0
  --lqg-min-turn-speed-scale K
                             LQG speed scale near stop-turn angle. Default: 0.45
  --no-lqg-smooth-turn       Make LQG use point-to-point stop/rotate segments.
  --safe-speed               Convenience: nominal speed 0.12, duty limit 25.
  --fast-speed               Convenience: faster point-to-point mission motion:
                             higher straight duty, duty cap, and slew rate.
  --strong-control           Stronger point-to-point straight-line correction:
                             tighter turns, higher yaw correction, and lateral
                             path-error feedback.
  --segment-yaw-kp K         Straight segment yaw correction gain. Default: 14.0
  --segment-max-correction P Max per-wheel correction duty. Default: 4.0
  --segment-straight-duty P  Straight segment cruise duty. Default: 20.0
  --segment-min-duty P       Straight segment minimum duty. Default: 15.0
  --segment-decel-distance M Distance before corners/stops to ramp down.
                             Default: 0.70
  --velocity-profile         Use direct straight profile: cruise 35, approach
                             25, accel/decel 50 duty-percent/sec by default.
  --profile-cruise-duty P    Velocity profile straight cruise duty. Default: 35.0
  --profile-approach-duty P  Velocity profile duty near turns/stops. Default: 25.0
  --profile-accel-duty-per-sec P
                             Velocity profile ramp rate. Default: 50.0
  --segment-lateral-kp K     Lateral path correction gain. Default: 0.0
  --segment-lateral-max-deg D
                             Max heading offset from lateral correction. Default: 18.0
  --corner-turn-max-duty P   Max in-place turn duty. Default: 12.0
  --duty-slew PCT_PER_SEC    Duty slew limit. Default: 55.0
  --no-graceful-stop         Publish zero immediately when the controller stops.
                             Default uses a gentle stop ramp.
  --graceful-stop-decel P    Stop forward-duty ramp-down rate. Default: 35.0
  --graceful-stop-turn P     Stop turn-duty ramp-down rate. Default: 120.0
  --path-tracking            Use continuous path tracking instead of
                             point-to-point straight/turn segments.
  --no-preserve-replan-motion
                             Let point-to-point control fully reset on every
                             replanned path. Default preserves aligned motion.
  --preserve-replan-heading-deg D
                             Max initial heading mismatch for preserving motion.
                             Default: 12.0
  --preserve-replan-start-tolerance M
                             Max pose-to-new-path distance for preserving motion.
                             Default: 0.45
  --lidar-stop               Enable LiDAR obstacle stop: halt motors when any
                             object is within 0.2 m of the front of the robot.
                             Resumes when the path clears beyond 0.25 m.
                             Auto-launches the RPLIDAR driver for /scan and
                             fuses scan-to-map LiDAR localization.
  --no-lidar-scanmatch       Disable scan-to-map LiDAR localization/fusion.
  --rplidar-driver           Launch the RPLIDAR A1 driver even without
                             --lidar-stop.
  --no-rplidar-driver        Do not launch the RPLIDAR driver automatically.
  --lidar-verbose            Print front-sector LiDAR distance every 0.5 s
                             (only active with --lidar-stop).
  --lidar-mirror             Mirror LaserScan angles left/right. Use this when
                             LiDAR obstacles appear on the wrong side of the map.
  --lidar-yaw-deg D          LiDAR yaw relative to robot base. Positive is CCW.
                             Default: 0.0
  --lidar-x M                LiDAR x offset in base frame. Default: 0.0
  --lidar-y M                LiDAR y offset in base frame. Default: 0.0
  --lidar-avoid-distance M   Front LiDAR distance where avoidance steering
                             starts. Default: 0.55
  --lidar-clear-distance M   Front LiDAR distance where avoidance clears.
                             Default: 0.75
  --lidar-pre-stop-decel-scale K
                             Scale forward duty near front LiDAR obstacles.
                             Default: 0.70
  --lidar-pre-stop-decel-distance M
                             Front LiDAR distance where pre-stop decel starts.
                             Default: 0.60
  --no-lidar-pre-stop-decel  Disable LiDAR pre-stop forward deceleration.
  --lidar-steer-emergency-stop M
                             In steer mode, publish hard zero if predicted front
                             LiDAR distance is below M. Default: 0.0 disabled
  --lidar-front-backup       In steer mode, reverse briefly before steering when
                             a close obstacle is directly in front.
  --lidar-front-backup-trigger M
                             Predicted front LiDAR distance that starts backup.
                             Default: 0.0 disabled
  --lidar-front-backup-distance M
                             Odometry distance to reverse before steering.
                             Default: 0.20
  --lidar-front-backup-duty P
                             Reverse duty during front backup. Default: 22.0
  --lidar-front-backup-timeout SEC
                             Backup fallback timeout if odometry is unavailable.
                             Default: 1.5
  --lidar-front-backup-steer-duration SEC
                             Duration for forced steer after backup. Default: 0.8
  --no-lidar-front-backup    Disable front backup recovery.
  --lidar-obstacle-ttl SEC   Planner LiDAR obstacle lifetime. Default: 1.0
  --lidar-static-filter M    Ignore LiDAR hits within this distance of known
                             map obstacles/walls. Default: 0.0 (disabled)
  --lidar-beam-stride N      Planner uses every Nth LiDAR beam. Default: 4
  --lidar-max-points N       Max active planner LiDAR points. Default: 1500
  --no-lidar-wall-localization
                             Disable LiDAR wall SVD localization correction.
  --replan-cooldown SEC      Minimum time between local dynamic replans.
                             Default: 2.0
  --plant-goal-standoff M    Plan to the plant location, then stop this far
                             before it along the path. Default: 1.0
  --start-heading-bias M     Extra A* cost against immediately turning away
                             from current heading. Default: 0.15
  --start-heading-bias-distance M
                             Distance over which initial heading bias fades.
                             Default: 0.70
  --turn-start-cost M        Extra A* cost each time a path starts a new turn.
                             Default: 0.25
  --clearance-radius M       A* penalizes cells closer than this to obstacles,
                             including walls. Default: 0.30
  --clearance-weight W       Strength of the near-obstacle A* penalty.
                             Default: 3.0
  --hard-clearance M         Treat cells within this distance of obstacles as
                             blocked. Default: 0.0
  --fog-path-preference R    A* preference for traversable cells still unknown
                             in explored_map. 1.0 disables; 1.5 makes fogged
                             free cells cost about 1/1.5 of normal. Default: 1.0
  --fog-cleanup              Add temporary residual-fog waypoints once fog left
                             drops below --fog-cleanup-trigger-ratio.
  --fog-cleanup-trigger-ratio R
                             Enable cleanup only when fog left is below this
                             fraction of traversable map. Default: 0.50
  --fog-cleanup-min-cluster-ratio R
                             Ignore fog clusters at or below this map fraction.
                             Default: 0.01
  --fog-cleanup-max-cluster-ratio R
                             Ignore fog clusters at or above this map fraction.
                             Default: 0.10
  --fog-cleanup-max-goals N  Maximum residual-fog temp goals. Default: 1
  --exact-global-ordering    Use heading-state A* for initial mission ordering
                             instead of the faster static 2D order estimate.
  --global-order-stride N    Coarsening factor for fast initial mission
                             ordering. Default: 1
  --planned-coverage         Add path-based residual-fog exploration goals:
                             plant-only order first, sweep actual planned
                             paths, then re-order plant + exploration goals.
  --planned-coverage-radius M
                             Radius swept around planned paths. Default: 2.0
                             gives a 4 m corridor.
  --planned-coverage-no-los  Use plain corridor coverage without static
                             line-of-sight ray checks.
  --planned-coverage-sample-step M
                             Spacing between path samples used for coverage.
                             Default: 0.25
  --planned-coverage-min-cluster-area M2
                             Ignore residual fog clusters smaller than this.
                             Default: 0.60
  --planned-coverage-max-goals N
                             Maximum residual-fog exploration goals. Default: 2
  --planned-coverage-goal-tolerance M
                             Distance for planner-side completion of generated
                             exploration goals. Default: 0.25
  --fog-inspection           Enable low-priority slow/hold + camera pan toward
                             nearby unseen free-space frontier.
  --fog-slow-scale K         Motor duty scale while inspecting fog. Default: 0.30
  --fog-attention-distance M Max distance for fog inspection targets.
                             Default: 2.50
  --fog-hold-distance M      Hold once fog frontier is this close. Default: 1.20
  --fog-pre-hold-decel-scale K
                             Extra scale for brief pre-hold decel. Default: 0.70
  --fog-pre-hold-decel-duration SEC
                             Duration of pre-hold decel before zero hold.
                             Default: 0.40
  --no-fog-pre-hold-decel    Disable fog pre-hold deceleration.
  --fog-hold-duration SEC    Hold duration for a fog pan. Default: 1.80
  --fog-cooldown SEC         Cooldown after a fog hold. Default: 1.00
  --fog-clearance-cooldown SEC
                             Alias for --fog-cooldown. The fog-inspection
                             shortcut currently uses 6.00.
  --fog-min-frontier-area M2 Ignore smaller fog frontier clusters. Default: 0.04
  --fog-camera-pan-max-angle-deg D
                             Fog-only camera pan cap. 0 uses full camera range.
                             Default: 0.0
  --fog-camera-command-period SEC
                             Minimum seconds between fog camera pan commands.
                             Increase this to pan less frequently. Default: 0.10
  --fog-camera-rotate-period SEC
                             Alias for --fog-camera-command-period. The
                             fog-inspection shortcut currently uses 5.00.
  --no-static-prior-seen     Do not mark known static walls/tables/lecterns as
                             explored at mapper startup.
  --control-verbose          Print controller debug logs.
  --dry-run                  Print the launch command without executing it.
  --stop                     Stop likely full-mission nodes.
  -h, --help                 Show this help text.

Examples:
  ./src/asclinic_pkg/scripts/run_full_mission.sh --safe-speed
  ./src/asclinic_pkg/scripts/run_full_mission.sh --camera-device 2 --roboclaw-port /dev/ttyACM1
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
LOG_DIR="${WS_ROOT}/src/asclinic_pkg/logs"

BUILD_FIRST=true
LAUNCH_DETECTOR=true
LAUNCH_MAP_VIEWER=false
MAP_VIEWER_SHOW_GUI=false
MAP_VIEWER_BACKEND="Agg"
MAP_VIEWER_PLOT_PERIOD_SEC="0.25"
MAP_VIEWER_SAVE_PERIOD_SEC="0.75"
MAP_VIEWER_SAVE_LIVE_PNG=true
LAUNCH_ARUCO_DETECTOR=true
LAUNCH_FUSED_ESTIMATOR=true
CAMERA_UPRIGHT=true
DETECTOR_START_ENABLED=true
CAMERA_DEVICE=0
CAMERA_FPS=10
ROBOCLAW_PORT="/dev/ttyACM0"
NAMESPACE="asc"
MODEL_PATH=""
INITIAL_X="0.50"
INITIAL_Y="0.40"
INITIAL_YAW="0.0"
NOMINAL_SPEED="0.20"
MAX_LINEAR_SPEED="0.35"
DUTY_LIMIT="40.0"
MIN_MOVING_DUTY="18.0"
LEFT_TRIM="1.0"
RIGHT_TRIM="0.932"
LOOKAHEAD="0.45"
DUTY_SLEW="55.0"
PATH_CONTROLLER_BACKEND="direct"
BACKEND_DELTA_V_LIMIT="0.30"
BACKEND_DELTA_OMEGA_LIMIT="1.50"
LQG_SMOOTH_TURN_ENABLED=true
LQG_SMOOTH_TURN_ANGLE_DEG="30.0"
LQG_STOP_TURN_ANGLE_DEG="60.0"
LQG_MIN_SMOOTH_TURN_SPEED_SCALE="0.45"
POINT_TO_POINT_ROTATE_TOLERANCE="0.06"
POINT_TO_POINT_ARRIVAL_TOLERANCE="0.04"
CORNER_TURN_MAX_DUTY="12.0"
SEGMENT_STRAIGHT_DUTY="20.0"
SEGMENT_MIN_STRAIGHT_DUTY="15.0"
SEGMENT_YAW_KP="14.0"
SEGMENT_MAX_CORRECTION="4.0"
SEGMENT_LATERAL_KP="0.0"
SEGMENT_LATERAL_DEADBAND_M="0.03"
SEGMENT_LATERAL_MAX_HEADING_DEG="18.0"
SEGMENT_DECELERATION_DISTANCE="0.70"
SEGMENT_VELOCITY_PROFILE_ENABLED=false
SEGMENT_PROFILE_CRUISE_DUTY="35.0"
SEGMENT_PROFILE_APPROACH_DUTY="25.0"
SEGMENT_PROFILE_ACCEL_DUTY_PER_SEC="50.0"
GRACEFUL_STOP_ENABLED=true
GRACEFUL_STOP_DECEL_DUTY_PER_SEC="35.0"
GRACEFUL_STOP_TURN_DUTY_PER_SEC="120.0"
LOCALIZATION_MODE="cv+wheel"
CONTROL_ODOM_TOPIC="pitt_fused_odometry"
MAP_VIEWER_ODOM_TOPIC="pitt_fused_odometry"
POINT_TO_POINT_MODE=true
PRESERVE_REPLAN_MOTION=true
PRESERVE_REPLAN_HEADING_TOLERANCE_DEG="12.0"
PRESERVE_REPLAN_START_TOLERANCE_M="0.45"
CONTROL_VERBOSE=false
RPLIDAR_DRIVER_MODE="auto"
LAUNCH_RPLIDAR_DRIVER=false
LAUNCH_LIDAR_STOP=false
LAUNCH_LIDAR_WALL_LOCALIZER=true
USE_LIDAR_WALL_UPDATE=true
LAUNCH_LIDAR_SCAN_MATCH_LOCALIZER=false
USE_LIDAR_SCANMATCH_UPDATE=false
LIDAR_SCANMATCH_ODOM_TOPIC="pitt_lidar_scanmatch_odometry"
LIDAR_VERBOSE=false
LIDAR_SCAN_ANGLE_MULTIPLIER="1.0"
LIDAR_IN_BASE_X="0.0"
LIDAR_IN_BASE_Y="0.0"
LIDAR_IN_BASE_YAW_DEG="0.0"
LIDAR_AVOID_DISTANCE_M="0.55"
LIDAR_CLEAR_DISTANCE_M="0.75"
LIDAR_PRE_STOP_DECEL_ENABLED=true
LIDAR_PRE_STOP_DECEL_SCALE="0.70"
LIDAR_PRE_STOP_DECEL_DISTANCE_M="0.60"
LIDAR_STEER_EMERGENCY_STOP_DISTANCE_M="0.0"
LIDAR_FRONT_RECOVERY_ENABLED=false
LIDAR_FRONT_RECOVERY_TRIGGER_DISTANCE_M="0.0"
LIDAR_FRONT_RECOVERY_DISTANCE_M="0.20"
LIDAR_FRONT_RECOVERY_REVERSE_DUTY="22.0"
LIDAR_FRONT_RECOVERY_TIMEOUT_SEC="1.5"
LIDAR_FRONT_RECOVERY_STEER_DURATION_SEC="0.8"
DYNAMIC_OBSTACLE_TTL_SEC="1.0"
DYNAMIC_OBSTACLE_STATIC_FILTER_M="0.0"
DYNAMIC_OBSTACLE_BEAM_STRIDE="4"
DYNAMIC_OBSTACLE_MAX_POINTS="1500"
DYNAMIC_OBSTACLE_MARKER_LATEST_ONLY=true
REPLAN_COOLDOWN_SEC="2.0"
PLANT_GOAL_STANDOFF_M="1.0"
START_HEADING_BIAS_M="0.15"
START_HEADING_BIAS_DISTANCE_M="0.70"
TURN_START_COST_M="0.25"
OBSTACLE_CLEARANCE_RADIUS_M="0.30"
OBSTACLE_CLEARANCE_WEIGHT_M="3.0"
HARD_OBSTACLE_CLEARANCE_M="0.0"
FOG_PATH_PREFERENCE="1.00"
FAST_GLOBAL_ORDERING=true
GLOBAL_ORDER_GRID_STRIDE="1"
FOG_CLEANUP_ENABLED=false
FOG_CLEANUP_TRIGGER_RATIO="0.50"
FOG_CLEANUP_MIN_CLUSTER_RATIO="0.01"
FOG_CLEANUP_MAX_CLUSTER_RATIO="0.10"
FOG_CLEANUP_MAX_GOALS="1"
PLANNED_COVERAGE_ENABLED=false
PLANNED_COVERAGE_RADIUS_M="2.0"
PLANNED_COVERAGE_USE_LINE_OF_SIGHT=true
PLANNED_COVERAGE_SAMPLE_STEP_M="0.25"
PLANNED_COVERAGE_MIN_CLUSTER_AREA_M2="0.60"
PLANNED_COVERAGE_MAX_GOALS="2"
PLANNED_COVERAGE_GOAL_TOLERANCE_M="0.25"
FOG_INSPECTION_ENABLED=false
FOG_SLOW_SCALE="0.30"
FOG_ATTENTION_MAX_DISTANCE_M="2.50"
FOG_HOLD_TRIGGER_DISTANCE_M="1.20"
FOG_PRE_HOLD_DECEL_ENABLED=true
FOG_PRE_HOLD_DECEL_SCALE="0.70"
FOG_PRE_HOLD_DECEL_DURATION_SEC="0.40"
FOG_HOLD_DURATION_SEC="1.80"
FOG_COOLDOWN_SEC="1.00"
FOG_MIN_FRONTIER_AREA_M2="0.04"
FOG_CAMERA_PAN_MAX_ANGLE_DEG="0.0"
FOG_CAMERA_COMMAND_PERIOD_SEC="0.10"
MARK_STATIC_PRIOR_SEEN_ON_START=true
PLANT_SAVE_POSITIVE_IMAGES=true
PLANT_PUBLISH_DEBUG_IMAGE=false
PLANT_POSITIVE_IMAGE_DIR=""
PLANT_COMPLETION_MATCH_MODE="any"
PLANT_COMPLETION_CONFIDENCE_THRESHOLD="0.60"
PLANT_COMPLETION_MAX_STOP_DISTANCE_M="1.60"
PLANT_COMPLETION_REQUIRE_VALID_CAPTURE=true
PLANT_BBOX_SERVO=true
PLANT_CAPTURE_RADIUS_M="1.20"
DRY_RUN=false
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
SERVO_PAN_CHANNEL=14
SERVO_TILT_CHANNEL=13
SERVO_PAN_UPRIGHT_US=1515
SERVO_TILT_UPRIGHT_US=1400
CAMERA_AUTO_PAN=true
CAMERA_PAN_MIN_US=500
CAMERA_PAN_MAX_US=2500
CAMERA_PAN_MAX_ANGLE_DEG="180.0"
CAMERA_PAN_IDLE_CENTER_PERIOD_SEC="0.0"
CAMERA_REVEAL_HALF_ANGLE_DEG="30.0"
FOG_SCAN_MAX_RANGE_M="5.0"
CAMERA_PAN_ACTIVE_RADIUS_M="1.60"
SERVO_PID=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stop)
      echo "[STOP] Killing likely full-mission nodes..."
      pkill -f "i2c_for_servos" 2>/dev/null || true
      pkill -f "roboclaw_for_motors" 2>/dev/null || true
      pkill -f "liu_odometry_from_encoders" 2>/dev/null || true
      pkill -f "aruco_detector" 2>/dev/null || true
      pkill -f "pitt_fused_odemetry" 2>/dev/null || true
      pkill -f "lidar_wall_localizer" 2>/dev/null || true
      pkill -f "plant_detector_node" 2>/dev/null || true
      pkill -f "tracking_controller_node" 2>/dev/null || true
      pkill -f "obstacle_stop_motor_filter" 2>/dev/null || true
      pkill -f "fog_inspection_motor_filter" 2>/dev/null || true
      pkill -f "semantic_exploration_mapper" 2>/dev/null || true
      pkill -f "ros2 launch rplidar_ros rplidar_a1_launch.py" 2>/dev/null || true
      pkill -f "rplidar_node" 2>/dev/null || true
      pkill -f "system_status_publisher" 2>/dev/null || true
      pkill -f "lidar_map_saver" 2>/dev/null || true
      pkill -f "live_global_map_viewer" 2>/dev/null || true
      pkill -f "path_planning/map.py" 2>/dev/null || true
      pkill -f "path_planning/global_planner.py" 2>/dev/null || true
      pkill -f "install/asclinic_pkg/lib/asclinic_pkg/map.py" 2>/dev/null || true
      pkill -f "install/asclinic_pkg/lib/asclinic_pkg/global_planner.py" 2>/dev/null || true
      pkill -f "final_demo_map_server" 2>/dev/null || true
      pkill -f "known_map_path_planner" 2>/dev/null || true
      echo "[STOP] Done."
      exit 0
      ;;
    --no-build)
      BUILD_FIRST=false
      shift
      ;;
    --no-detector)
      LAUNCH_DETECTOR=false
      shift
      ;;
    --viewer)
      LAUNCH_MAP_VIEWER=true
      shift
      ;;
    --map-gui)
      LAUNCH_MAP_VIEWER=true
      MAP_VIEWER_SHOW_GUI=true
      MAP_VIEWER_BACKEND="TkAgg"
      shift
      ;;
    --no-map-gui)
      MAP_VIEWER_SHOW_GUI=false
      MAP_VIEWER_BACKEND="Agg"
      shift
      ;;
    --map-viewer-odom|--map-viewer-odom-topic)
      MAP_VIEWER_ODOM_TOPIC="${2:?missing value for --map-viewer-odom}"
      shift 2
      ;;
    --map-plot-period)
      MAP_VIEWER_PLOT_PERIOD_SEC="${2:?missing value for --map-plot-period}"
      shift 2
      ;;
    --map-save-period)
      MAP_VIEWER_SAVE_PERIOD_SEC="${2:?missing value for --map-save-period}"
      shift 2
      ;;
    --map-no-png)
      MAP_VIEWER_SAVE_LIVE_PNG=false
      shift
      ;;
    --wheel-only)
      LOCALIZATION_MODE="wheel-only"
      LAUNCH_ARUCO_DETECTOR=false
      LAUNCH_FUSED_ESTIMATOR=false
      LAUNCH_LIDAR_WALL_LOCALIZER=false
      USE_LIDAR_WALL_UPDATE=false
      LAUNCH_LIDAR_SCAN_MATCH_LOCALIZER=false
      USE_LIDAR_SCANMATCH_UPDATE=false
      LAUNCH_DETECTOR=false
      CONTROL_ODOM_TOPIC="wheel_odometry"
      MAP_VIEWER_ODOM_TOPIC="wheel_odometry"
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
    --no-camera-auto-pan)
      CAMERA_AUTO_PAN=false
      shift
      ;;
    --camera-auto-pan)
      CAMERA_AUTO_PAN=true
      shift
      ;;
    --camera-pan-min-us)
      CAMERA_PAN_MIN_US="${2:?missing value for --camera-pan-min-us}"
      shift 2
      ;;
    --camera-pan-max-us)
      CAMERA_PAN_MAX_US="${2:?missing value for --camera-pan-max-us}"
      shift 2
      ;;
    --camera-pan-max-angle-deg)
      CAMERA_PAN_MAX_ANGLE_DEG="${2:?missing value for --camera-pan-max-angle-deg}"
      shift 2
      ;;
    --camera-pan-idle-center-period|--camera-idle-center-period)
      CAMERA_PAN_IDLE_CENTER_PERIOD_SEC="${2:?missing value for --camera-pan-idle-center-period}"
      shift 2
      ;;
    --camera-reveal-half-angle-deg)
      CAMERA_REVEAL_HALF_ANGLE_DEG="${2:?missing value for --camera-reveal-half-angle-deg}"
      shift 2
      ;;
    --fog-scan-max-range)
      FOG_SCAN_MAX_RANGE_M="${2:?missing value for --fog-scan-max-range}"
      shift 2
      ;;
    --plant-capture-radius)
      PLANT_CAPTURE_RADIUS_M="${2:?missing value for --plant-capture-radius}"
      shift 2
      ;;
    --plant-goal-standoff)
      PLANT_GOAL_STANDOFF_M="${2:?missing value for --plant-goal-standoff}"
      shift 2
      ;;
    --no-plant-bbox-servo)
      PLANT_BBOX_SERVO=false
      shift
      ;;
    --detector-start-enabled)
      DETECTOR_START_ENABLED=true
      shift
      ;;
    --save-plant-images)
      PLANT_SAVE_POSITIVE_IMAGES=true
      shift
      ;;
    --no-save-plant-images)
      PLANT_SAVE_POSITIVE_IMAGES=false
      shift
      ;;
    --plant-debug-image|--plant-debug-images)
      PLANT_PUBLISH_DEBUG_IMAGE=true
      shift
      ;;
    --no-plant-debug-image|--no-plant-debug-images)
      PLANT_PUBLISH_DEBUG_IMAGE=false
      shift
      ;;
    --plant-image-dir)
      PLANT_POSITIVE_IMAGE_DIR="${2:?missing value for --plant-image-dir}"
      PLANT_SAVE_POSITIVE_IMAGES=true
      shift 2
      ;;
    --plant-match-mode)
      PLANT_COMPLETION_MATCH_MODE="${2:?missing value for --plant-match-mode}"
      shift 2
      ;;
    --plant-completion-conf)
      PLANT_COMPLETION_CONFIDENCE_THRESHOLD="${2:?missing value for --plant-completion-conf}"
      shift 2
      ;;
    --plant-completion-distance|--plant-completion-dist)
      PLANT_COMPLETION_MAX_STOP_DISTANCE_M="${2:?missing value for --plant-completion-distance}"
      shift 2
      ;;
    --plant-allow-hints)
      PLANT_COMPLETION_REQUIRE_VALID_CAPTURE=false
      shift
      ;;
    --camera-device)
      CAMERA_DEVICE="${2:?missing value for --camera-device}"
      shift 2
      ;;
    --camera-fps)
      CAMERA_FPS="${2:?missing value for --camera-fps}"
      shift 2
      ;;
    --roboclaw-port)
      ROBOCLAW_PORT="${2:?missing value for --roboclaw-port}"
      shift 2
      ;;
    --namespace)
      NAMESPACE="${2:?missing value for --namespace}"
      shift 2
      ;;
    --model-path)
      MODEL_PATH="${2:?missing value for --model-path}"
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
    --nominal-speed)
      NOMINAL_SPEED="${2:?missing value for --nominal-speed}"
      shift 2
      ;;
    --max-linear-speed)
      MAX_LINEAR_SPEED="${2:?missing value for --max-linear-speed}"
      shift 2
      ;;
    --min-moving-duty)
      MIN_MOVING_DUTY="${2:?missing value for --min-moving-duty}"
      shift 2
      ;;
    --duty-limit)
      DUTY_LIMIT="${2:?missing value for --duty-limit}"
      shift 2
      ;;
    --left-trim)
      LEFT_TRIM="${2:?missing value for --left-trim}"
      shift 2
      ;;
    --right-trim)
      RIGHT_TRIM="${2:?missing value for --right-trim}"
      shift 2
      ;;
    --lookahead)
      LOOKAHEAD="${2:?missing value for --lookahead}"
      shift 2
      ;;
    --controller|--path-controller)
      PATH_CONTROLLER_BACKEND="${2:?missing value for --controller}"
      shift 2
      ;;
    --lqg)
      PATH_CONTROLLER_BACKEND="lqg"
      shift
      ;;
    --backend-delta-v-limit)
      BACKEND_DELTA_V_LIMIT="${2:?missing value for --backend-delta-v-limit}"
      shift 2
      ;;
    --backend-delta-omega-limit|--backend-delta-w-limit)
      BACKEND_DELTA_OMEGA_LIMIT="${2:?missing value for --backend-delta-omega-limit}"
      shift 2
      ;;
    --no-lqg-smooth-turn)
      LQG_SMOOTH_TURN_ENABLED=false
      shift
      ;;
    --lqg-smooth-turn-angle)
      LQG_SMOOTH_TURN_ANGLE_DEG="${2:?missing value for --lqg-smooth-turn-angle}"
      shift 2
      ;;
    --lqg-stop-turn-angle)
      LQG_STOP_TURN_ANGLE_DEG="${2:?missing value for --lqg-stop-turn-angle}"
      shift 2
      ;;
    --lqg-min-turn-speed-scale)
      LQG_MIN_SMOOTH_TURN_SPEED_SCALE="${2:?missing value for --lqg-min-turn-speed-scale}"
      shift 2
      ;;
    --mpc)
      PATH_CONTROLLER_BACKEND="mpc"
      shift
      ;;
    --pid)
      PATH_CONTROLLER_BACKEND="pid"
      shift
      ;;
    --safe-speed)
      NOMINAL_SPEED="0.12"
      DUTY_LIMIT="25.0"
      shift
      ;;
    --fast-speed)
      NOMINAL_SPEED="0.25"
      DUTY_LIMIT="45.0"
      SEGMENT_STRAIGHT_DUTY="24.0"
      SEGMENT_MIN_STRAIGHT_DUTY="18.0"
      DUTY_SLEW="85.0"
      shift
      ;;
    --strong-control)
      POINT_TO_POINT_ROTATE_TOLERANCE="0.04"
      POINT_TO_POINT_ARRIVAL_TOLERANCE="0.035"
      CORNER_TURN_MAX_DUTY="14.0"
      SEGMENT_YAW_KP="24.0"
      SEGMENT_MAX_CORRECTION="8.0"
      SEGMENT_LATERAL_KP="0.85"
      SEGMENT_LATERAL_DEADBAND_M="0.03"
      SEGMENT_LATERAL_MAX_HEADING_DEG="18.0"
      DUTY_SLEW="75.0"
      shift
      ;;
    --segment-yaw-kp)
      SEGMENT_YAW_KP="${2:?missing value for --segment-yaw-kp}"
      shift 2
      ;;
    --segment-max-correction)
      SEGMENT_MAX_CORRECTION="${2:?missing value for --segment-max-correction}"
      shift 2
      ;;
    --segment-straight-duty)
      SEGMENT_STRAIGHT_DUTY="${2:?missing value for --segment-straight-duty}"
      shift 2
      ;;
    --segment-min-duty)
      SEGMENT_MIN_STRAIGHT_DUTY="${2:?missing value for --segment-min-duty}"
      shift 2
      ;;
    --segment-decel-distance)
      SEGMENT_DECELERATION_DISTANCE="${2:?missing value for --segment-decel-distance}"
      shift 2
      ;;
    --velocity-profile)
      SEGMENT_VELOCITY_PROFILE_ENABLED=true
      shift
      ;;
    --profile-cruise-duty)
      SEGMENT_PROFILE_CRUISE_DUTY="${2:?missing value for --profile-cruise-duty}"
      shift 2
      ;;
    --profile-approach-duty)
      SEGMENT_PROFILE_APPROACH_DUTY="${2:?missing value for --profile-approach-duty}"
      shift 2
      ;;
    --profile-accel-duty-per-sec|--profile-accel-duty|--profile-accel)
      SEGMENT_PROFILE_ACCEL_DUTY_PER_SEC="${2:?missing value for --profile-accel-duty-per-sec}"
      shift 2
      ;;
    --segment-lateral-kp)
      SEGMENT_LATERAL_KP="${2:?missing value for --segment-lateral-kp}"
      shift 2
      ;;
    --segment-lateral-max-deg)
      SEGMENT_LATERAL_MAX_HEADING_DEG="${2:?missing value for --segment-lateral-max-deg}"
      shift 2
      ;;
    --corner-turn-max-duty)
      CORNER_TURN_MAX_DUTY="${2:?missing value for --corner-turn-max-duty}"
      shift 2
      ;;
    --duty-slew)
      DUTY_SLEW="${2:?missing value for --duty-slew}"
      shift 2
      ;;
    --no-graceful-stop)
      GRACEFUL_STOP_ENABLED=false
      shift
      ;;
    --graceful-stop)
      GRACEFUL_STOP_ENABLED=true
      shift
      ;;
    --graceful-stop-decel|--stop-decel-duty-per-sec)
      GRACEFUL_STOP_DECEL_DUTY_PER_SEC="${2:?missing value for --graceful-stop-decel}"
      shift 2
      ;;
    --graceful-stop-turn|--stop-turn-duty-per-sec)
      GRACEFUL_STOP_TURN_DUTY_PER_SEC="${2:?missing value for --graceful-stop-turn}"
      shift 2
      ;;
    --path-tracking)
      POINT_TO_POINT_MODE=false
      shift
      ;;
    --no-preserve-replan-motion)
      PRESERVE_REPLAN_MOTION=false
      shift
      ;;
    --preserve-replan-motion)
      PRESERVE_REPLAN_MOTION=true
      shift
      ;;
    --preserve-replan-heading-deg)
      PRESERVE_REPLAN_HEADING_TOLERANCE_DEG="${2:?missing value for --preserve-replan-heading-deg}"
      shift 2
      ;;
    --preserve-replan-start-tolerance)
      PRESERVE_REPLAN_START_TOLERANCE_M="${2:?missing value for --preserve-replan-start-tolerance}"
      shift 2
      ;;
    --lidar-stop)
      LAUNCH_LIDAR_STOP=true
      LAUNCH_LIDAR_SCAN_MATCH_LOCALIZER=true
      USE_LIDAR_SCANMATCH_UPDATE=true
      shift
      ;;
    --lidar-scanmatch-shadow)
      LAUNCH_LIDAR_SCAN_MATCH_LOCALIZER=true
      USE_LIDAR_SCANMATCH_UPDATE=true
      shift
      ;;
    --no-lidar-scanmatch)
      LAUNCH_LIDAR_SCAN_MATCH_LOCALIZER=false
      USE_LIDAR_SCANMATCH_UPDATE=false
      shift
      ;;
    --rplidar-driver)
      RPLIDAR_DRIVER_MODE=true
      shift
      ;;
    --no-rplidar-driver)
      RPLIDAR_DRIVER_MODE=false
      shift
      ;;
    --lidar-verbose)
      LIDAR_VERBOSE=true
      shift
      ;;
    --lidar-mirror)
      LIDAR_SCAN_ANGLE_MULTIPLIER="-1.0"
      shift
      ;;
    --lidar-yaw-deg)
      LIDAR_IN_BASE_YAW_DEG="${2:?missing value for --lidar-yaw-deg}"
      shift 2
      ;;
    --lidar-x)
      LIDAR_IN_BASE_X="${2:?missing value for --lidar-x}"
      shift 2
      ;;
    --lidar-y)
      LIDAR_IN_BASE_Y="${2:?missing value for --lidar-y}"
      shift 2
      ;;
    --lidar-avoid-distance|--lidar-stop-distance)
      LIDAR_AVOID_DISTANCE_M="${2:?missing value for --lidar-avoid-distance}"
      shift 2
      ;;
    --lidar-clear-distance)
      LIDAR_CLEAR_DISTANCE_M="${2:?missing value for --lidar-clear-distance}"
      shift 2
      ;;
    --lidar-pre-stop-decel-scale)
      LIDAR_PRE_STOP_DECEL_SCALE="${2:?missing value for --lidar-pre-stop-decel-scale}"
      LIDAR_PRE_STOP_DECEL_ENABLED=true
      shift 2
      ;;
    --lidar-pre-stop-decel-distance)
      LIDAR_PRE_STOP_DECEL_DISTANCE_M="${2:?missing value for --lidar-pre-stop-decel-distance}"
      LIDAR_PRE_STOP_DECEL_ENABLED=true
      shift 2
      ;;
    --lidar-pre-stop-decel)
      LIDAR_PRE_STOP_DECEL_ENABLED=true
      shift
      ;;
    --no-lidar-pre-stop-decel)
      LIDAR_PRE_STOP_DECEL_ENABLED=false
      shift
      ;;
    --lidar-steer-emergency-stop)
      LIDAR_STEER_EMERGENCY_STOP_DISTANCE_M="${2:?missing value for --lidar-steer-emergency-stop}"
      shift 2
      ;;
    --lidar-front-backup)
      LIDAR_FRONT_RECOVERY_ENABLED=true
      shift
      ;;
    --lidar-front-backup-trigger)
      LIDAR_FRONT_RECOVERY_TRIGGER_DISTANCE_M="${2:?missing value for --lidar-front-backup-trigger}"
      LIDAR_FRONT_RECOVERY_ENABLED=true
      shift 2
      ;;
    --lidar-front-backup-distance)
      LIDAR_FRONT_RECOVERY_DISTANCE_M="${2:?missing value for --lidar-front-backup-distance}"
      LIDAR_FRONT_RECOVERY_ENABLED=true
      shift 2
      ;;
    --lidar-front-backup-duty)
      LIDAR_FRONT_RECOVERY_REVERSE_DUTY="${2:?missing value for --lidar-front-backup-duty}"
      LIDAR_FRONT_RECOVERY_ENABLED=true
      shift 2
      ;;
    --lidar-front-backup-timeout)
      LIDAR_FRONT_RECOVERY_TIMEOUT_SEC="${2:?missing value for --lidar-front-backup-timeout}"
      LIDAR_FRONT_RECOVERY_ENABLED=true
      shift 2
      ;;
    --lidar-front-backup-steer-duration)
      LIDAR_FRONT_RECOVERY_STEER_DURATION_SEC="${2:?missing value for --lidar-front-backup-steer-duration}"
      LIDAR_FRONT_RECOVERY_ENABLED=true
      shift 2
      ;;
    --no-lidar-front-backup)
      LIDAR_FRONT_RECOVERY_ENABLED=false
      shift
      ;;
    --lidar-obstacle-ttl)
      DYNAMIC_OBSTACLE_TTL_SEC="${2:?missing value for --lidar-obstacle-ttl}"
      shift 2
      ;;
    --lidar-static-filter)
      DYNAMIC_OBSTACLE_STATIC_FILTER_M="${2:?missing value for --lidar-static-filter}"
      shift 2
      ;;
    --lidar-beam-stride)
      DYNAMIC_OBSTACLE_BEAM_STRIDE="${2:?missing value for --lidar-beam-stride}"
      shift 2
      ;;
    --lidar-max-points)
      DYNAMIC_OBSTACLE_MAX_POINTS="${2:?missing value for --lidar-max-points}"
      shift 2
      ;;
    --no-lidar-wall-localization)
      LAUNCH_LIDAR_WALL_LOCALIZER=false
      USE_LIDAR_WALL_UPDATE=false
      shift
      ;;
    --replan-cooldown)
      REPLAN_COOLDOWN_SEC="${2:?missing value for --replan-cooldown}"
      shift 2
      ;;
    --start-heading-bias)
      START_HEADING_BIAS_M="${2:?missing value for --start-heading-bias}"
      shift 2
      ;;
    --start-heading-bias-distance)
      START_HEADING_BIAS_DISTANCE_M="${2:?missing value for --start-heading-bias-distance}"
      shift 2
      ;;
    --turn-start-cost)
      TURN_START_COST_M="${2:?missing value for --turn-start-cost}"
      shift 2
      ;;
    --clearance-radius)
      OBSTACLE_CLEARANCE_RADIUS_M="${2:?missing value for --clearance-radius}"
      shift 2
      ;;
    --clearance-weight)
      OBSTACLE_CLEARANCE_WEIGHT_M="${2:?missing value for --clearance-weight}"
      shift 2
      ;;
    --hard-clearance)
      HARD_OBSTACLE_CLEARANCE_M="${2:?missing value for --hard-clearance}"
      shift 2
      ;;
    --fog-path-preference)
      FOG_PATH_PREFERENCE="${2:?missing value for --fog-path-preference}"
      shift 2
      ;;
    --fog-cleanup)
      FOG_CLEANUP_ENABLED=true
      shift
      ;;
    --no-fog-cleanup)
      FOG_CLEANUP_ENABLED=false
      shift
      ;;
    --fog-cleanup-trigger-ratio)
      FOG_CLEANUP_TRIGGER_RATIO="${2:?missing value for --fog-cleanup-trigger-ratio}"
      shift 2
      ;;
    --fog-cleanup-min-cluster-ratio)
      FOG_CLEANUP_MIN_CLUSTER_RATIO="${2:?missing value for --fog-cleanup-min-cluster-ratio}"
      shift 2
      ;;
    --fog-cleanup-max-cluster-ratio)
      FOG_CLEANUP_MAX_CLUSTER_RATIO="${2:?missing value for --fog-cleanup-max-cluster-ratio}"
      shift 2
      ;;
    --fog-cleanup-max-goals)
      FOG_CLEANUP_MAX_GOALS="${2:?missing value for --fog-cleanup-max-goals}"
      shift 2
      ;;
    --exact-global-ordering)
      FAST_GLOBAL_ORDERING=false
      shift
      ;;
    --global-order-stride)
      GLOBAL_ORDER_GRID_STRIDE="${2:?missing value for --global-order-stride}"
      shift 2
      ;;
    --planned-coverage|--path-coverage-exploration)
      PLANNED_COVERAGE_ENABLED=true
      shift
      ;;
    --planned-coverage-radius)
      PLANNED_COVERAGE_RADIUS_M="${2:?missing value for --planned-coverage-radius}"
      shift 2
      ;;
    --planned-coverage-no-los)
      PLANNED_COVERAGE_USE_LINE_OF_SIGHT=false
      shift
      ;;
    --planned-coverage-sample-step)
      PLANNED_COVERAGE_SAMPLE_STEP_M="${2:?missing value for --planned-coverage-sample-step}"
      shift 2
      ;;
    --planned-coverage-min-cluster-area)
      PLANNED_COVERAGE_MIN_CLUSTER_AREA_M2="${2:?missing value for --planned-coverage-min-cluster-area}"
      shift 2
      ;;
    --planned-coverage-max-goals)
      PLANNED_COVERAGE_MAX_GOALS="${2:?missing value for --planned-coverage-max-goals}"
      shift 2
      ;;
    --planned-coverage-goal-tolerance)
      PLANNED_COVERAGE_GOAL_TOLERANCE_M="${2:?missing value for --planned-coverage-goal-tolerance}"
      shift 2
      ;;
    --fog-inspection)
      FOG_INSPECTION_ENABLED=true
      shift
      ;;
    --fog-slow-scale)
      FOG_SLOW_SCALE="${2:?missing value for --fog-slow-scale}"
      shift 2
      ;;
    --fog-attention-distance|--fog-attention-max-distance)
      FOG_ATTENTION_MAX_DISTANCE_M="${2:?missing value for --fog-attention-distance}"
      shift 2
      ;;
    --fog-hold-distance|--fog-hold-trigger-distance)
      FOG_HOLD_TRIGGER_DISTANCE_M="${2:?missing value for --fog-hold-distance}"
      shift 2
      ;;
    --fog-pre-hold-decel-scale)
      FOG_PRE_HOLD_DECEL_SCALE="${2:?missing value for --fog-pre-hold-decel-scale}"
      FOG_PRE_HOLD_DECEL_ENABLED=true
      shift 2
      ;;
    --fog-pre-hold-decel-duration)
      FOG_PRE_HOLD_DECEL_DURATION_SEC="${2:?missing value for --fog-pre-hold-decel-duration}"
      FOG_PRE_HOLD_DECEL_ENABLED=true
      shift 2
      ;;
    --fog-pre-hold-decel)
      FOG_PRE_HOLD_DECEL_ENABLED=true
      shift
      ;;
    --no-fog-pre-hold-decel)
      FOG_PRE_HOLD_DECEL_ENABLED=false
      shift
      ;;
    --fog-hold-duration)
      FOG_HOLD_DURATION_SEC="${2:?missing value for --fog-hold-duration}"
      shift 2
      ;;
    --fog-cooldown|--fog-clearance-cooldown|--fog-camera-clearance-cooldown)
      FOG_COOLDOWN_SEC="${2:?missing value for --fog-cooldown}"
      shift 2
      ;;
    --fog-min-frontier-area)
      FOG_MIN_FRONTIER_AREA_M2="${2:?missing value for --fog-min-frontier-area}"
      shift 2
      ;;
    --fog-camera-pan-max-angle-deg|--fog-camera-pan-max-angle)
      FOG_CAMERA_PAN_MAX_ANGLE_DEG="${2:?missing value for --fog-camera-pan-max-angle-deg}"
      shift 2
      ;;
    --fog-camera-command-period|--fog-camera-pan-period|--fog-pan-period|--fog-camera-rotate-period|--fog-camera-rotation-period|--fog-rotate-period)
      FOG_CAMERA_COMMAND_PERIOD_SEC="${2:?missing value for --fog-camera-command-period}"
      shift 2
      ;;
    --no-static-prior-seen)
      MARK_STATIC_PRIOR_SEEN_ON_START=false
      shift
      ;;
    --control-verbose)
      CONTROL_VERBOSE=true
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

if [[ "$RPLIDAR_DRIVER_MODE" == "auto" ]]; then
  if [[ "$LAUNCH_LIDAR_STOP" == true || "$LAUNCH_LIDAR_SCAN_MATCH_LOCALIZER" == true ]]; then
    LAUNCH_RPLIDAR_DRIVER=true
  else
    LAUNCH_RPLIDAR_DRIVER=false
  fi
else
  LAUNCH_RPLIDAR_DRIVER="$RPLIDAR_DRIVER_MODE"
fi

if [[ "$LAUNCH_FUSED_ESTIMATOR" == true && "$USE_LIDAR_SCANMATCH_UPDATE" == true ]]; then
  LOCALIZATION_MODE="cv+wheel+lidar-scanmap"
elif [[ "$LAUNCH_FUSED_ESTIMATOR" == true && "$USE_LIDAR_WALL_UPDATE" == true ]]; then
  LOCALIZATION_MODE="cv+wheel+lidar-wall"
fi

PATH_CONTROLLER_BACKEND="${PATH_CONTROLLER_BACKEND,,}"
case "$PATH_CONTROLLER_BACKEND" in
  direct|lqg|mpc|pid)
    ;;
  *)
    echo "Invalid --controller: ${PATH_CONTROLLER_BACKEND}. Use direct, lqg, mpc, or pid." >&2
    exit 2
    ;;
esac

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

  echo "Could not find /opt/ros/<distro>/setup.bash. Source ROS 2 first." >&2
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
  local servo_log="${LOG_DIR}/servo_full_mission.log"
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

# shellcheck disable=SC1091
set +u
source "$WS_ROOT/install/setup.bash"
set -u

if [[ "$PLANT_SAVE_POSITIVE_IMAGES" == true && -z "$PLANT_POSITIVE_IMAGE_DIR" ]]; then
  PLANT_POSITIVE_IMAGE_DIR="${WS_ROOT}/results/live_global_map/${RUN_TAG}/plant_detections"
fi

launch_args=(
  "namespace:=${NAMESPACE}"
  "run_tag:=${RUN_TAG}"
  "launch_planning:=true"
  "launch_control:=true"
  "launch_plant_detector:=${LAUNCH_DETECTOR}"
  "launch_map_viewer:=${LAUNCH_MAP_VIEWER}"
  "map_viewer_show_gui:=${MAP_VIEWER_SHOW_GUI}"
  "map_viewer_backend:=${MAP_VIEWER_BACKEND}"
  "map_viewer_plot_period_sec:=${MAP_VIEWER_PLOT_PERIOD_SEC}"
  "map_viewer_save_period_sec:=${MAP_VIEWER_SAVE_PERIOD_SEC}"
  "map_viewer_save_live_png:=${MAP_VIEWER_SAVE_LIVE_PNG}"
  "launch_aruco_detector:=${LAUNCH_ARUCO_DETECTOR}"
  "launch_fused_estimator:=${LAUNCH_FUSED_ESTIMATOR}"
  "launch_rplidar_driver:=${LAUNCH_RPLIDAR_DRIVER}"
  "launch_lidar_wall_localizer:=${LAUNCH_LIDAR_WALL_LOCALIZER}"
  "launch_lidar_scan_match_localizer:=${LAUNCH_LIDAR_SCAN_MATCH_LOCALIZER}"
  "use_lidar_wall_update:=${USE_LIDAR_WALL_UPDATE}"
  "use_lidar_scanmatch_update:=${USE_LIDAR_SCANMATCH_UPDATE}"
  "plant_detector_start_enabled:=${DETECTOR_START_ENABLED}"
  "plant_save_positive_images:=${PLANT_SAVE_POSITIVE_IMAGES}"
  "plant_publish_debug_image:=${PLANT_PUBLISH_DEBUG_IMAGE}"
  "plant_positive_image_dir:=${PLANT_POSITIVE_IMAGE_DIR:-data/plant_dataset/positive}"
  "plant_positive_save_gate_topic:=/${NAMESPACE}/plant_detector/save_positive_enabled"
  "plant_completion_match_mode:=${PLANT_COMPLETION_MATCH_MODE}"
  "plant_completion_confidence_threshold:=${PLANT_COMPLETION_CONFIDENCE_THRESHOLD}"
  "plant_completion_max_stop_distance_m:=${PLANT_COMPLETION_MAX_STOP_DISTANCE_M}"
  "plant_completion_require_valid_capture:=${PLANT_COMPLETION_REQUIRE_VALID_CAPTURE}"
  "camera_device:=${CAMERA_DEVICE}"
  "camera_fps:=${CAMERA_FPS}"
  "roboclaw_usb_port:=${ROBOCLAW_PORT}"
  "initial_x:=${INITIAL_X}"
  "initial_y:=${INITIAL_Y}"
  "initial_yaw:=${INITIAL_YAW}"
  "path_controller_backend:=${PATH_CONTROLLER_BACKEND}"
  "backend_delta_v_limit:=${BACKEND_DELTA_V_LIMIT}"
  "backend_delta_omega_limit:=${BACKEND_DELTA_OMEGA_LIMIT}"
  "lqg_smooth_turn_enabled:=${LQG_SMOOTH_TURN_ENABLED}"
  "lqg_smooth_turn_angle_deg:=${LQG_SMOOTH_TURN_ANGLE_DEG}"
  "lqg_stop_turn_angle_deg:=${LQG_STOP_TURN_ANGLE_DEG}"
  "lqg_min_smooth_turn_speed_scale:=${LQG_MIN_SMOOTH_TURN_SPEED_SCALE}"
  "nominal_speed:=${NOMINAL_SPEED}"
  "max_linear_speed:=${MAX_LINEAR_SPEED}"
  "duty_cycle_limit:=${DUTY_LIMIT}"
  "min_moving_duty:=${MIN_MOVING_DUTY}"
  "left_trim:=${LEFT_TRIM}"
  "right_trim:=${RIGHT_TRIM}"
  "duty_slew_rate_percent_per_sec:=${DUTY_SLEW}"
  "graceful_stop_enabled:=${GRACEFUL_STOP_ENABLED}"
  "graceful_stop_decel_duty_per_sec:=${GRACEFUL_STOP_DECEL_DUTY_PER_SEC}"
  "graceful_stop_turn_duty_per_sec:=${GRACEFUL_STOP_TURN_DUTY_PER_SEC}"
  "lookahead_distance:=${LOOKAHEAD}"
  "control_odom_topic:=${CONTROL_ODOM_TOPIC}"
  "map_viewer_odom_topic:=${MAP_VIEWER_ODOM_TOPIC}"
  "lidar_scanmatch_odom_topic:=${LIDAR_SCANMATCH_ODOM_TOPIC}"
  "point_to_point_mode:=${POINT_TO_POINT_MODE}"
  "point_to_point_rotate_tolerance:=${POINT_TO_POINT_ROTATE_TOLERANCE}"
  "point_to_point_arrival_tolerance:=${POINT_TO_POINT_ARRIVAL_TOLERANCE}"
  "preserve_replan_motion_enabled:=${PRESERVE_REPLAN_MOTION}"
  "preserve_replan_heading_tolerance_deg:=${PRESERVE_REPLAN_HEADING_TOLERANCE_DEG}"
  "preserve_replan_start_tolerance_m:=${PRESERVE_REPLAN_START_TOLERANCE_M}"
  "corner_turn_max_duty:=${CORNER_TURN_MAX_DUTY}"
  "segment_straight_duty:=${SEGMENT_STRAIGHT_DUTY}"
  "segment_min_straight_duty:=${SEGMENT_MIN_STRAIGHT_DUTY}"
  "segment_yaw_kp:=${SEGMENT_YAW_KP}"
  "segment_max_correction:=${SEGMENT_MAX_CORRECTION}"
  "segment_lateral_kp:=${SEGMENT_LATERAL_KP}"
  "segment_lateral_deadband_m:=${SEGMENT_LATERAL_DEADBAND_M}"
  "segment_lateral_max_heading_deg:=${SEGMENT_LATERAL_MAX_HEADING_DEG}"
  "segment_deceleration_distance:=${SEGMENT_DECELERATION_DISTANCE}"
  "segment_velocity_profile_enabled:=${SEGMENT_VELOCITY_PROFILE_ENABLED}"
  "segment_profile_cruise_duty:=${SEGMENT_PROFILE_CRUISE_DUTY}"
  "segment_profile_approach_duty:=${SEGMENT_PROFILE_APPROACH_DUTY}"
  "segment_profile_accel_duty_per_sec:=${SEGMENT_PROFILE_ACCEL_DUTY_PER_SEC}"
  "control_verbose:=${CONTROL_VERBOSE}"
  "launch_obstacle_filter:=${LAUNCH_LIDAR_STOP}"
  "launch_fog_inspection_filter:=${FOG_INSPECTION_ENABLED}"
  "lidar_verbose:=${LIDAR_VERBOSE}"
  "lidar_scan_angle_multiplier:=${LIDAR_SCAN_ANGLE_MULTIPLIER}"
  "lidar_avoid_distance_m:=${LIDAR_AVOID_DISTANCE_M}"
  "lidar_clear_distance_m:=${LIDAR_CLEAR_DISTANCE_M}"
  "lidar_pre_stop_decel_enabled:=${LIDAR_PRE_STOP_DECEL_ENABLED}"
  "lidar_pre_stop_decel_scale:=${LIDAR_PRE_STOP_DECEL_SCALE}"
  "lidar_pre_stop_decel_distance_m:=${LIDAR_PRE_STOP_DECEL_DISTANCE_M}"
  "lidar_steer_emergency_stop_distance_m:=${LIDAR_STEER_EMERGENCY_STOP_DISTANCE_M}"
  "lidar_front_recovery_enabled:=${LIDAR_FRONT_RECOVERY_ENABLED}"
  "lidar_front_recovery_trigger_distance_m:=${LIDAR_FRONT_RECOVERY_TRIGGER_DISTANCE_M}"
  "lidar_front_recovery_distance_m:=${LIDAR_FRONT_RECOVERY_DISTANCE_M}"
  "lidar_front_recovery_reverse_duty:=${LIDAR_FRONT_RECOVERY_REVERSE_DUTY}"
  "lidar_front_recovery_timeout_sec:=${LIDAR_FRONT_RECOVERY_TIMEOUT_SEC}"
  "lidar_front_recovery_steer_duration_sec:=${LIDAR_FRONT_RECOVERY_STEER_DURATION_SEC}"
  "plant_goal_standoff_m:=${PLANT_GOAL_STANDOFF_M}"
  "replan_cooldown_sec:=${REPLAN_COOLDOWN_SEC}"
  "obstacle_clearance_radius_m:=${OBSTACLE_CLEARANCE_RADIUS_M}"
  "obstacle_clearance_weight_m:=${OBSTACLE_CLEARANCE_WEIGHT_M}"
  "hard_obstacle_clearance_m:=${HARD_OBSTACLE_CLEARANCE_M}"
  "fog_path_preference:=${FOG_PATH_PREFERENCE}"
  "dynamic_obstacle_ttl_sec:=${DYNAMIC_OBSTACLE_TTL_SEC}"
  "dynamic_obstacle_static_filter_m:=${DYNAMIC_OBSTACLE_STATIC_FILTER_M}"
  "dynamic_obstacle_beam_stride:=${DYNAMIC_OBSTACLE_BEAM_STRIDE}"
  "dynamic_obstacle_max_points:=${DYNAMIC_OBSTACLE_MAX_POINTS}"
  "dynamic_obstacle_marker_latest_only:=${DYNAMIC_OBSTACLE_MARKER_LATEST_ONLY}"
  "start_heading_bias_m:=${START_HEADING_BIAS_M}"
  "start_heading_bias_distance_m:=${START_HEADING_BIAS_DISTANCE_M}"
  "turn_start_cost_m:=${TURN_START_COST_M}"
  "lidar_in_base_x:=${LIDAR_IN_BASE_X}"
  "lidar_in_base_y:=${LIDAR_IN_BASE_Y}"
  "lidar_in_base_yaw_deg:=${LIDAR_IN_BASE_YAW_DEG}"
  "fast_global_ordering:=${FAST_GLOBAL_ORDERING}"
  "global_order_grid_stride:=${GLOBAL_ORDER_GRID_STRIDE}"
  "fog_cleanup_enabled:=${FOG_CLEANUP_ENABLED}"
  "fog_cleanup_trigger_ratio:=${FOG_CLEANUP_TRIGGER_RATIO}"
  "fog_cleanup_min_cluster_ratio:=${FOG_CLEANUP_MIN_CLUSTER_RATIO}"
  "fog_cleanup_max_cluster_ratio:=${FOG_CLEANUP_MAX_CLUSTER_RATIO}"
  "fog_cleanup_max_goals:=${FOG_CLEANUP_MAX_GOALS}"
  "planned_coverage_enabled:=${PLANNED_COVERAGE_ENABLED}"
  "planned_coverage_radius_m:=${PLANNED_COVERAGE_RADIUS_M}"
  "planned_coverage_use_line_of_sight:=${PLANNED_COVERAGE_USE_LINE_OF_SIGHT}"
  "planned_coverage_sample_step_m:=${PLANNED_COVERAGE_SAMPLE_STEP_M}"
  "planned_coverage_min_cluster_area_m2:=${PLANNED_COVERAGE_MIN_CLUSTER_AREA_M2}"
  "planned_coverage_max_goals:=${PLANNED_COVERAGE_MAX_GOALS}"
  "planned_coverage_goal_tolerance_m:=${PLANNED_COVERAGE_GOAL_TOLERANCE_M}"
  "mark_static_prior_seen_on_start:=${MARK_STATIC_PRIOR_SEEN_ON_START}"
  "fog_slow_scale:=${FOG_SLOW_SCALE}"
  "fog_attention_max_distance_m:=${FOG_ATTENTION_MAX_DISTANCE_M}"
  "fog_hold_trigger_distance_m:=${FOG_HOLD_TRIGGER_DISTANCE_M}"
  "fog_pre_hold_decel_enabled:=${FOG_PRE_HOLD_DECEL_ENABLED}"
  "fog_pre_hold_decel_scale:=${FOG_PRE_HOLD_DECEL_SCALE}"
  "fog_pre_hold_decel_duration_sec:=${FOG_PRE_HOLD_DECEL_DURATION_SEC}"
  "fog_hold_duration_sec:=${FOG_HOLD_DURATION_SEC}"
  "fog_cooldown_sec:=${FOG_COOLDOWN_SEC}"
  "fog_min_frontier_area_m2:=${FOG_MIN_FRONTIER_AREA_M2}"
  "fog_camera_pan_max_angle_deg:=${FOG_CAMERA_PAN_MAX_ANGLE_DEG}"
  "fog_camera_command_period_sec:=${FOG_CAMERA_COMMAND_PERIOD_SEC}"
  "camera_auto_pan_enabled:=${CAMERA_AUTO_PAN}"
  "camera_pan_center_us:=${SERVO_PAN_UPRIGHT_US}"
  "camera_pan_min_us:=${CAMERA_PAN_MIN_US}"
  "camera_pan_max_us:=${CAMERA_PAN_MAX_US}"
  "camera_pan_max_angle_deg:=${CAMERA_PAN_MAX_ANGLE_DEG}"
  "camera_reveal_half_angle_deg:=${CAMERA_REVEAL_HALF_ANGLE_DEG}"
  "fog_scan_max_range_m:=${FOG_SCAN_MAX_RANGE_M}"
  "camera_pan_active_radius_m:=${CAMERA_PAN_ACTIVE_RADIUS_M}"
  "camera_pan_idle_center_period_sec:=${CAMERA_PAN_IDLE_CENTER_PERIOD_SEC}"
  "camera_tilt_center_us:=${SERVO_TILT_UPRIGHT_US}"
  "plant_bbox_servo_enabled:=${PLANT_BBOX_SERVO}"
  "plant_capture_radius_m:=${PLANT_CAPTURE_RADIUS_M}"
  "plant_detection_enable_radius:=${PLANT_CAPTURE_RADIUS_M}"
)

if [[ -n "$MODEL_PATH" ]]; then
  launch_args+=("model_path:=${MODEL_PATH}")
fi

cmd=(ros2 launch asclinic_pkg full_mission.launch.py "${launch_args[@]}")

echo "Launching ASClinic full mission from: $WS_ROOT"
echo "Initial pose: x=${INITIAL_X}, y=${INITIAL_Y}, yaw=${INITIAL_YAW}"
echo "Localization: ${LOCALIZATION_MODE} (control_odom=${CONTROL_ODOM_TOPIC}, viewer_odom=${MAP_VIEWER_ODOM_TOPIC})"
echo "RPLIDAR driver: launch=${LAUNCH_RPLIDAR_DRIVER} (publishes /scan)"
echo "LiDAR wall localization: launch=${LAUNCH_LIDAR_WALL_LOCALIZER}, fuse=${USE_LIDAR_WALL_UPDATE}"
echo "LiDAR scan-map localization: launch=${LAUNCH_LIDAR_SCAN_MATCH_LOCALIZER}, fuse=${USE_LIDAR_SCANMATCH_UPDATE}, odom=${LIDAR_SCANMATCH_ODOM_TOPIC}"
echo "Map viewer: launch=${LAUNCH_MAP_VIEWER}, gui=${MAP_VIEWER_SHOW_GUI}, backend=${MAP_VIEWER_BACKEND}, plot=${MAP_VIEWER_PLOT_PERIOD_SEC}s, save_png=${MAP_VIEWER_SAVE_LIVE_PNG}@${MAP_VIEWER_SAVE_PERIOD_SEC}s"
echo "Control mode: point_to_point=${POINT_TO_POINT_MODE}, backend=${PATH_CONTROLLER_BACKEND}, preserve_replan=${PRESERVE_REPLAN_MOTION}@${PRESERVE_REPLAN_HEADING_TOLERANCE_DEG}deg/${PRESERVE_REPLAN_START_TOLERANCE_M}m"
echo "Backend limits: delta_v=${BACKEND_DELTA_V_LIMIT}m/s, delta_omega=${BACKEND_DELTA_OMEGA_LIMIT}rad/s"
echo "LQG smooth turns (ignored unless backend=lqg): enabled=${LQG_SMOOTH_TURN_ENABLED}, smooth<=${LQG_SMOOTH_TURN_ANGLE_DEG}deg, stop>${LQG_STOP_TURN_ANGLE_DEG}deg, min_speed_scale=${LQG_MIN_SMOOTH_TURN_SPEED_SCALE}"
echo "Control tuning: straight=${SEGMENT_STRAIGHT_DUTY}/${SEGMENT_MIN_STRAIGHT_DUTY}, yaw_kp=${SEGMENT_YAW_KP}, max_corr=${SEGMENT_MAX_CORRECTION}, lateral_kp=${SEGMENT_LATERAL_KP}, slew=${DUTY_SLEW}, graceful_stop=${GRACEFUL_STOP_ENABLED}@${GRACEFUL_STOP_DECEL_DUTY_PER_SEC}/${GRACEFUL_STOP_TURN_DUTY_PER_SEC} duty/s"
echo "Velocity profile: enabled=${SEGMENT_VELOCITY_PROFILE_ENABLED}, cruise=${SEGMENT_PROFILE_CRUISE_DUTY}, approach=${SEGMENT_PROFILE_APPROACH_DUTY}, accel=${SEGMENT_PROFILE_ACCEL_DUTY_PER_SEC}/s, decel_distance=${SEGMENT_DECELERATION_DISTANCE}m"
echo "Dynamic replanning: cooldown=${REPLAN_COOLDOWN_SEC}s, start_heading_bias=${START_HEADING_BIAS_M}m/${START_HEADING_BIAS_DISTANCE_M}m, turn_start_cost=${TURN_START_COST_M}m, clearance_radius=${OBSTACLE_CLEARANCE_RADIUS_M}m, clearance_weight=${OBSTACLE_CLEARANCE_WEIGHT_M}, hard_clearance=${HARD_OBSTACLE_CLEARANCE_M}m, fog_path_preference=${FOG_PATH_PREFERENCE}, fast_global_ordering=${FAST_GLOBAL_ORDERING}, global_order_stride=${GLOBAL_ORDER_GRID_STRIDE}, lidar_angle_multiplier=${LIDAR_SCAN_ANGLE_MULTIPLIER}, lidar_yaw=${LIDAR_IN_BASE_YAW_DEG}deg, lidar_offset=(${LIDAR_IN_BASE_X},${LIDAR_IN_BASE_Y})m, lidar_avoid=${LIDAR_AVOID_DISTANCE_M}/${LIDAR_CLEAR_DISTANCE_M}m, lidar_pre_decel=${LIDAR_PRE_STOP_DECEL_ENABLED}@${LIDAR_PRE_STOP_DECEL_SCALE}/${LIDAR_PRE_STOP_DECEL_DISTANCE_M}m, lidar_steer_emergency_stop=${LIDAR_STEER_EMERGENCY_STOP_DISTANCE_M}m, lidar_front_backup=${LIDAR_FRONT_RECOVERY_ENABLED}@${LIDAR_FRONT_RECOVERY_TRIGGER_DISTANCE_M}m/${LIDAR_FRONT_RECOVERY_DISTANCE_M}m duty=${LIDAR_FRONT_RECOVERY_REVERSE_DUTY}, lidar_ttl=${DYNAMIC_OBSTACLE_TTL_SEC}s, lidar_static_filter=${DYNAMIC_OBSTACLE_STATIC_FILTER_M}m, lidar_beam_stride=${DYNAMIC_OBSTACLE_BEAM_STRIDE}, lidar_max_points=${DYNAMIC_OBSTACLE_MAX_POINTS}"
echo "Planned coverage: enabled=${PLANNED_COVERAGE_ENABLED}, radius=${PLANNED_COVERAGE_RADIUS_M}m, los=${PLANNED_COVERAGE_USE_LINE_OF_SIGHT}, sample_step=${PLANNED_COVERAGE_SAMPLE_STEP_M}m, min_cluster=${PLANNED_COVERAGE_MIN_CLUSTER_AREA_M2}m^2, max_goals=${PLANNED_COVERAGE_MAX_GOALS}, goal_tol=${PLANNED_COVERAGE_GOAL_TOLERANCE_M}m"
echo "Fog cleanup waypoints: enabled=${FOG_CLEANUP_ENABLED}, trigger<${FOG_CLEANUP_TRIGGER_RATIO}, cluster=${FOG_CLEANUP_MIN_CLUSTER_RATIO}..${FOG_CLEANUP_MAX_CLUSTER_RATIO}, max_goals=${FOG_CLEANUP_MAX_GOALS}"
echo "Fog inspection: enabled=${FOG_INSPECTION_ENABLED}, slow_scale=${FOG_SLOW_SCALE}, pre_hold=${FOG_PRE_HOLD_DECEL_ENABLED}@${FOG_PRE_HOLD_DECEL_SCALE}/${FOG_PRE_HOLD_DECEL_DURATION_SEC}s, max_dist=${FOG_ATTENTION_MAX_DISTANCE_M}m, hold=${FOG_HOLD_TRIGGER_DISTANCE_M}m/${FOG_HOLD_DURATION_SEC}s, cooldown=${FOG_COOLDOWN_SEC}s, min_frontier=${FOG_MIN_FRONTIER_AREA_M2}m^2, fog_pan_cap=${FOG_CAMERA_PAN_MAX_ANGLE_DEG}deg, fog_pan_period=${FOG_CAMERA_COMMAND_PERIOD_SEC}s"
echo "Static prior seen on start: ${MARK_STATIC_PRIOR_SEEN_ON_START}"
echo "Camera upright: $CAMERA_UPRIGHT (pan=${SERVO_PAN_UPRIGHT_US}us, tilt=${SERVO_TILT_UPRIGHT_US}us)"
echo "Camera auto-pan: $CAMERA_AUTO_PAN (center=${SERVO_PAN_UPRIGHT_US}us, range=${CAMERA_PAN_MIN_US}-${CAMERA_PAN_MAX_US}us, max_angle=${CAMERA_PAN_MAX_ANGLE_DEG}deg, idle_center_period=${CAMERA_PAN_IDLE_CENTER_PERIOD_SEC}s)"
echo "Exploration fog reveal: camera half-angle=${CAMERA_REVEAL_HALF_ANGLE_DEG}deg, scan_max=${FOG_SCAN_MAX_RANGE_M}m"
echo "Plant bbox servo: $PLANT_BBOX_SERVO (capture_radius=${PLANT_CAPTURE_RADIUS_M}m)"
echo "Plant path stand-off: ${PLANT_GOAL_STANDOFF_M}m"
echo "Plant completion: match=${PLANT_COMPLETION_MATCH_MODE}, confidence>=${PLANT_COMPLETION_CONFIDENCE_THRESHOLD}, max_stop_dist=${PLANT_COMPLETION_MAX_STOP_DISTANCE_M}m, require_valid_capture=${PLANT_COMPLETION_REQUIRE_VALID_CAPTURE}"
echo "Mission run id: ${RUN_TAG}"
if [[ "$PLANT_SAVE_POSITIVE_IMAGES" == true ]]; then
  echo "Plant positive images: ${PLANT_POSITIVE_IMAGE_DIR}"
  echo "Plant image save gate: /${NAMESPACE}/plant_detector/save_positive_enabled"
fi
if [[ "$PLANT_PUBLISH_DEBUG_IMAGE" == true ]]; then
  echo "Plant debug image topic: /${NAMESPACE}/plant_detector/debug_image"
else
  echo "Plant debug image topic: disabled"
fi
printf 'Command:'
printf ' %q' "${cmd[@]}"
printf '\n'

if [[ "$DRY_RUN" == true ]]; then
  exit 0
fi

set_camera_upright

exec "${cmd[@]}"
