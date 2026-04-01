#!/usr/bin/env bash
#
# Push all localization files to the robot via scp.
#
# Usage:
#   ./scripts/deploy.sh           # defaults to car 102
#   ./scripts/deploy.sh 104       # specify car number
#
# After this finishes, SSH into the robot, connect to Docker, and run:
#   cd ~/racecar_ws && colcon build --symlink-install && source install/setup.bash

set -euo pipefail

CAR="${1:-102}"
ROBOT="racecar@192.168.1.${CAR}"
REMOTE_WS="~/racecar_ws/src"
LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "==> Deploying to ${ROBOT}"
echo ""

# Ensure remote directory structure exists
echo "  Creating remote directories..."
ssh "${ROBOT}" "mkdir -p ${REMOTE_WS}/localization/localization ${REMOTE_WS}/localization/launch ${REMOTE_WS}/localization/maps ${REMOTE_WS}/localization/scripts ${REMOTE_WS}/localization/resource"

# localization ROS2 package (Python nodes + params + launch files)
echo "  localization/"
scp -r "${LOCAL_ROOT}/localization/" "${ROBOT}:${REMOTE_WS}/localization/localization/"

# launch files
echo "  launch/"
scp -r "${LOCAL_ROOT}/launch/" "${ROBOT}:${REMOTE_WS}/localization/launch/"

# maps
echo "  maps/"
scp -r "${LOCAL_ROOT}/maps/" "${ROBOT}:${REMOTE_WS}/localization/maps/"

# helper scripts (record_test.sh, etc.)
echo "  scripts/"
scp -r "${LOCAL_ROOT}/scripts/" "${ROBOT}:${REMOTE_WS}/localization/scripts/"

# build files (needed for colcon build)
echo "  build files (setup.py, package.xml, setup.cfg, resource/)"
scp "${LOCAL_ROOT}/setup.py" "${ROBOT}:${REMOTE_WS}/localization/"
scp "${LOCAL_ROOT}/package.xml" "${ROBOT}:${REMOTE_WS}/localization/"
scp "${LOCAL_ROOT}/setup.cfg" "${ROBOT}:${REMOTE_WS}/localization/"
scp -r "${LOCAL_ROOT}/resource/" "${ROBOT}:${REMOTE_WS}/localization/resource/"

echo ""
echo "==> Files deployed. Now on the robot (inside Docker):"
echo ""
echo "    cd ~/racecar_ws && colcon build --symlink-install && source install/setup.bash"
echo ""
echo "==> To run the particle filter on the real car:"
echo ""
echo "    ros2 launch localization localize_real.launch.xml"
echo ""
echo "==> Then set the initial pose in Foxglove or RViz using the 2D Pose Estimate tool."
echo "==> Drive the car with the joystick — hold RB as the deadman switch."
echo ""
