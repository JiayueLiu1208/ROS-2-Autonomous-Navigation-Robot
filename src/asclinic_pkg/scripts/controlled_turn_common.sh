#!/usr/bin/env bash

controlled_turn_usage() {
  local angle="$1"
  local script_name
  script_name="$(basename "$0")"
  cat <<EOF
Run a controlled in-place ${angle} degree rotation using wheel odometry feedback.

Usage:
  ${script_name} [options]

Options:
  --direction 1|-1       Rotation direction passed to the controller.
                         Default: 1
  --cw                   Shortcut for --direction -1.
  --ccw                  Shortcut for --direction 1.
  --duty PERCENT         Motor duty used during rotation. Default: 8.0
  --namespace NAME       ROS namespace. Default: asc
  --odom-topic TOPIC     Odometry topic relative to namespace unless absolute.
                         Default: wheel_odometry
  --cmd-topic TOPIC      Motor command topic relative to namespace unless absolute.
                         Default: set_motor_duty_cycle
  --odom-wait SEC        Require one odometry message before starting.
                         Default: 5
  --skip-odom-check      Launch without the preflight odometry message check.
  --timeout SEC          Safety timeout. Default: 45
  --no-timeout           Disable safety timeout.
  --quiet                Reduce rotate-node logs.
  --dry-run              Print the command without executing it.
  -h, --help             Show this help text.

Before running, start the minimal motor + encoder odometry stack, and make sure
the mission controller is not also publishing motor commands.
EOF
}

run_controlled_turn() {
  local angle="$1"
  shift

  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)"
  local ws_root
  ws_root="$(cd "${script_dir}/../../.." && pwd)"

  local namespace="asc"
  local odom_topic="wheel_odometry"
  local cmd_topic="set_motor_duty_cycle"
  local duty="8.0"
  local direction="1"
  local timeout_sec="45"
  local odom_wait_sec="5"
  local skip_odom_check=false
  local verbose="true"
  local dry_run=false

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --direction)
        direction="${2:?missing value for --direction}"
        shift 2
        ;;
      --cw)
        direction="-1"
        shift
        ;;
      --ccw)
        direction="1"
        shift
        ;;
      --duty)
        duty="${2:?missing value for --duty}"
        shift 2
        ;;
      --namespace)
        namespace="${2:?missing value for --namespace}"
        shift 2
        ;;
      --odom-topic)
        odom_topic="${2:?missing value for --odom-topic}"
        shift 2
        ;;
      --cmd-topic)
        cmd_topic="${2:?missing value for --cmd-topic}"
        shift 2
        ;;
      --odom-wait)
        odom_wait_sec="${2:?missing value for --odom-wait}"
        shift 2
        ;;
      --skip-odom-check)
        skip_odom_check=true
        shift
        ;;
      --timeout)
        timeout_sec="${2:?missing value for --timeout}"
        shift 2
        ;;
      --no-timeout)
        timeout_sec="0"
        shift
        ;;
      --quiet)
        verbose="false"
        shift
        ;;
      --dry-run)
        dry_run=true
        shift
        ;;
      -h|--help)
        controlled_turn_usage "$angle"
        return 0
        ;;
      *)
        echo "Unknown option: $1" >&2
        controlled_turn_usage "$angle"
        return 2
        ;;
    esac
  done

  if [[ "$direction" != "1" && "$direction" != "-1" ]]; then
    echo "--direction must be 1 or -1" >&2
    return 2
  fi

  source_ros() {
    if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "/opt/ros/${ROS_DISTRO}/setup.bash"
      set -u
      return
    fi

    local distro
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
    return 1
  }

  qualify_topic() {
    local topic="$1"
    local ns="${namespace#/}"
    if [[ "$topic" == /* ]]; then
      printf '%s' "$topic"
    elif [[ -n "$ns" && "$ns" != "/" ]]; then
      printf '/%s/%s' "$ns" "$topic"
    else
      printf '/%s' "$topic"
    fi
  }

  ros_double() {
    local value="$1"
    if [[ "$value" =~ ^[-+]?[0-9]+$ ]]; then
      printf '%s.0' "$value"
    else
      printf '%s' "$value"
    fi
  }

  source_ros
  cd "$ws_root"
  if [[ ! -f "$ws_root/install/setup.bash" ]]; then
    echo "Workspace install/setup.bash not found. Build first with:" >&2
    echo "  colcon build --packages-select asclinic_pkg" >&2
    return 1
  fi

  set +u
  # shellcheck disable=SC1091
  source "$ws_root/install/setup.bash"
  set -u

  local namespace_arg="/${namespace#/}"
  if [[ -z "${namespace#/}" || "$namespace" == "/" ]]; then
    namespace_arg="/"
  fi

  local qualified_cmd_topic
  qualified_cmd_topic="$(qualify_topic "$cmd_topic")"
  local qualified_odom_topic
  qualified_odom_topic="$(qualify_topic "$odom_topic")"

  publish_stop() {
    ros2 topic pub --once "$qualified_cmd_topic" asclinic_pkg/msg/LeftRightFloat32 \
      "{left: 0.0, right: 0.0, seq_num: 0}" >/dev/null 2>&1 || true
  }

  stop_and_exit() {
    local status="$1"
    publish_stop
    exit "$status"
  }

  trap 'stop_and_exit 130' INT
  trap 'stop_and_exit 143' TERM

  local angle_param
  local duty_param
  angle_param="$(ros_double "$angle")"
  duty_param="$(ros_double "$duty")"

  local cmd=(
    ros2 run asclinic_pkg liu_rotate_in_place_angle.py
    --ros-args
    -r "__ns:=${namespace_arg}"
    -p "target_angle_deg:=${angle_param}"
    -p "rotation_direction:=${direction}"
    -p "duty_cycle_percent:=${duty_param}"
    -p "odom_topic:=${odom_topic}"
    -p "cmd_topic:=${cmd_topic}"
    -p "verbose:=${verbose}"
  )

  echo "Controlled turn: ${angle_param} deg, direction=${direction}, duty=${duty_param}%"
  echo "Topics: odom=${qualified_odom_topic}, cmd=${qualified_cmd_topic}"
  echo "Safety: timeout=${timeout_sec}s; stop command will be sent on exit."
  echo "Make sure no mission/controller node is also publishing motor commands."
  printf 'Command:'
  printf ' %q' "${cmd[@]}"
  printf '\n'

  if [[ "$dry_run" == true ]]; then
    return 0
  fi

  if [[ "$skip_odom_check" != true && "$odom_wait_sec" != "0" ]]; then
    echo "Checking for one odometry message on ${qualified_odom_topic}..."
    if ! timeout "$odom_wait_sec" ros2 topic echo --once "$qualified_odom_topic" >/dev/null 2>&1; then
      echo "No odometry message received on ${qualified_odom_topic} within ${odom_wait_sec}s." >&2
      echo "Start motor encoder odometry first, for example:" >&2
      echo "  ros2 launch asclinic_pkg motors.launch.py namespace:=${namespace} launch_roboclaw:=true launch_servos:=false" >&2
      echo "  ros2 launch asclinic_pkg localisation.launch.py namespace:=${namespace} launch_encoder_odometry:=true launch_fused_estimator:=false launch_mock_odometry:=false" >&2
      return 1
    fi
  fi

  local status
  set +e
  if [[ "$timeout_sec" == "0" ]]; then
    "${cmd[@]}"
    status=$?
  else
    timeout --foreground "$timeout_sec" "${cmd[@]}"
    status=$?
  fi
  set -e

  publish_stop
  trap - INT TERM

  if [[ "$status" -eq 124 ]]; then
    echo "Controlled turn timed out after ${timeout_sec}s; stop command sent." >&2
  fi
  return "$status"
}
