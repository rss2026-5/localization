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
# What gets recorded:
#   /vesc/odom          - raw odometry (motion model input)
#   /scan               - raw lidar scans (sensor model input)
#   /pf/pose/odom       - estimated pose from particle filter
#   /pf/particles       - particle cloud for visualization
#   /map                - occupancy grid (for replay without map server)
#   /initialpose        - pose initializations (to replay the same init)
#   /tf                 - all transforms including map->base_link
#
# Stop recording with Ctrl-C.
# Replay later with: ros2 bag play <bag_dir>

set -euo pipefail

NAME="${1:-test_run_$(date +%Y%m%d_%H%M%S)}"
BAG_DIR="$HOME/rosbags/${NAME}"

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
