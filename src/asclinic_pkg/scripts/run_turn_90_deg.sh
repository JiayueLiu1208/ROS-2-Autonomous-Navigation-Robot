#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=controlled_turn_common.sh
source "${SCRIPT_DIR}/controlled_turn_common.sh"

run_controlled_turn 90 "$@"
