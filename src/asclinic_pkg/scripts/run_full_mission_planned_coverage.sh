#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/run_full_mission.sh" \
  --viewer \
  --controller direct \
  --control-verbose \
  --strong-control \
  --fast-speed \
  --velocity-profile \
  --profile-cruise-duty 35.0 \
  --profile-approach-duty 25.0 \
  --profile-accel-duty-per-sec 50.0 \
  --segment-decel-distance 0.85 \
  --duty-slew 50.0 \
  --duty-limit 50.0 \
  --lidar-stop \
  --lidar-verbose \
  --lidar-mirror \
  --save-plant-images \
  --plant-match-mode any \
  --plant-completion-conf 0.50 \
  --plant-completion-dist 1.60 \
  --replan-cooldown 2.5 \
  --clearance-radius 0.30 \
  --clearance-weight 1.0 \
  --planned-coverage \
  --planned-coverage-radius 2.0 \
  --planned-coverage-sample-step 0.40 \
  --planned-coverage-min-cluster-area 0.60 \
  --planned-coverage-max-goals 2 \
  --planned-coverage-goal-tolerance 0.25 \
  "$@"
