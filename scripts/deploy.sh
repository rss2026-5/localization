#!/usr/bin/env bash
#
# Push localization files to the robot via rsync.
#
# Usage:
#   ./scripts/deploy.sh           # defaults to car 102
#   ./scripts/deploy.sh 104       # specify car number

set -euo pipefail

CAR="${1:-102}"
ROBOT="racecar@192.168.1.${CAR}"
REMOTE_WS="~/racecar_ws/src"
LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "==> Deploying to ${ROBOT}"
echo ""

ssh "${ROBOT}" "mkdir -p ${REMOTE_WS}/localization"

rsync -avz --delete \
    --exclude '.git' \
    --exclude '__pycache__' \
    --exclude '.pytest_cache' \
    --exclude 'bag_files' \
    --exclude '*.db3' \
    "${LOCAL_ROOT}/" "${ROBOT}:${REMOTE_WS}/localization/"

echo ""
echo "==> Done. On the robot:"
echo "    export SIM_WS=/root/sim_ws"
echo "    cd ~/racecar_ws && colcon build --packages-select localization --symlink-install && source install/setup.bash"
echo "    ros2 launch localization localize_real.launch.xml"
echo ""
