#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/run_full_mission.sh" \
  --viewer \
  --controller lqg \
  --lqg-smooth-turn-angle 18.0 \
  --lqg-stop-turn-angle 38.0 \
  --lqg-min-turn-speed-scale 0.28 \
  --control-verbose \
  --strong-control \
  --fast-speed \
  --nominal-speed 0.38 \
  --max-linear-speed 0.55 \
  --velocity-profile \
  --profile-cruise-duty 58.0 \
  --profile-approach-duty 30.0 \
  --profile-accel-duty-per-sec 70.0 \
  --segment-decel-distance 1.30 \
  --duty-slew 75.0 \
  --duty-limit 65.0 \
  --lidar-stop \
  --lidar-verbose \
  --lidar-mirror \
  --lidar-avoid-distance 0.65 \
  --lidar-clear-distance 0.85 \
  --lidar-pre-stop-decel-scale 0.70 \
  --lidar-pre-stop-decel-distance 0.60 \
  --lidar-front-backup-trigger 0.42 \
  --lidar-front-backup-distance 0.20 \
  --lidar-front-backup-duty 24.0 \
  --lidar-front-backup-timeout 1.5 \
  --lidar-front-backup-steer-duration 0.80 \
  --save-plant-images \
  --plant-match-mode any \
  --plant-completion-conf 0.50 \
  --plant-completion-dist 1.60 \
  --replan-cooldown 5.0 \
  --clearance-radius 0.30 \
  --clearance-weight 1.0 \
  --fog-path-preference 1.00 \
  --fog-cleanup \
  --fog-cleanup-min-cluster-ratio 0.008 \
  --fog-scan-max-range 5.0 \
  --fog-inspection \
  --fog-camera-pan-max-angle 45.0 \
  --fog-camera-command-period 5.00 \
  --fog-slow-scale 0.30 \
  --fog-attention-distance 2.50 \
  --fog-hold-distance 1.20 \
  --fog-pre-hold-decel-scale 0.70 \
  --fog-pre-hold-decel-duration 0.40 \
  --fog-hold-duration 1.80 \
  --fog-cooldown 6.00 \
  --fog-min-frontier-area 0.04 \
  --camera-pan-idle-center-period 0.0 \
  "$@"
