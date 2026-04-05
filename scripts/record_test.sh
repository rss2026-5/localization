#!/usr/bin/env bash
#
# Record all topics needed for the Lab 5 report in a single rosbag.
#
# Run this ON THE CAR (inside Docker) AFTER launching:
#   ros2 launch localization localize_real.launch.xml
#
# Usage:
#   ./scripts/record_test.sh              # default bag name with timestamp
#   ./scripts/record_test.sh my_run_name  # custom name
#
# Bags are saved inside the workspace so they persist on the host.
# Copy them to your laptop with:
#   scp -r racecar@192.168.1.102:~/racecar_ws/src/localization/bag_files/ ./bag_files/

set -euo pipefail

NAME="${1:-test_run_$(date +%Y%m%d_%H%M%S)}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BAG_DIR="${SCRIPT_DIR}/../bag_files/${NAME}"

echo "==> Recording to ${BAG_DIR}"
echo "    Press Ctrl-C to stop."
echo ""

ros2 bag record \
    /vesc/odom \
    /scan \
    /pf/pose/odom \
    /pf/particles \
    /map \
    /initialpose \
    /tf \
    /tf_static \
    -o "${BAG_DIR}"
