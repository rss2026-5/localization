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

# localization ROS2 package (Python nodes + params + launch files)
echo "  localization/"
scp -r "${LOCAL_ROOT}/localization/" "${ROBOT}:${REMOTE_WS}/localization/localization/"

# launch files
echo "  launch/"
scp -r "${LOCAL_ROOT}/launch/" "${ROBOT}:${REMOTE_WS}/localization/launch/"

# maps
echo "  maps/"
scp -r "${LOCAL_ROOT}/maps/" "${ROBOT}:${REMOTE_WS}/localization/maps/"

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
