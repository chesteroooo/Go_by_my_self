#!/usr/bin/env bash
# =============================================================================
# record_aurora.sh — capture Aurora S SLAM data to a rosbag for OFFLINE work:
#   * measure visual-SLAM pose drift / tracking loss on the real route
#   * replay the run without the car or the Aurora (rosbag play)
#   * extract camera frames for the red-road segmentation dataset
#   * develop & test segmentation-assisted localization against real data
#
# Usage:
#   ./record_aurora.sh [light|full] [label]
#     light (default) : pose + tf + IMU + SLAM health/status + map + scan
#                       small (~MB/min) — enough for drift/tracking analysis
#     full            : light + point_cloud + stereo/depth images + semantic
#                       large (~GB/min) — for dense reconstruction + seg dataset
#
# Prereqs:
#   1) Aurora node running:  roslaunch ~/Go_by_my_self/Detect/new_detect/aurora_slam.launch
#   2) This terminal points at the SAME ROS master as that node
#      (standalone test: export ROS_MASTER_URI=http://localhost:11311)
#
# Bags are written to $AURORA_BAG_DIR (default ~/aurora_bags), OUTSIDE the repo
# so multi-GB files never get committed.
# =============================================================================
set -euo pipefail

PROFILE="${1:-light}"
LABEL="${2:-run}"
NS="/slamware_ros_sdk_server_node"
OUTDIR="${AURORA_BAG_DIR:-$HOME/aurora_bags}"
mkdir -p "$OUTDIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$OUTDIR/aurora_${LABEL}_${PROFILE}_${STAMP}"

# small, always-recorded topics: the trajectory + transform tree + SLAM health
LIGHT_TOPICS="$NS/robot_pose $NS/odom /tf /tf_static $NS/imu_raw_data \
$NS/system_status $NS/state $NS/relocalization_status \
$NS/map $NS/map_metadata $NS/scan"

# heavy perception topics: dense cloud, stereo/depth images, onboard semantics
FULL_EXTRA="$NS/point_cloud $NS/left_image_raw $NS/right_image_raw \
$NS/depth_image_raw $NS/depth_image_colorized $NS/semantic_segmentation \
$NS/stereo_keypoints"

case "$PROFILE" in
  full)  TOPICS="$LIGHT_TOPICS $FULL_EXTRA" ;;
  light) TOPICS="$LIGHT_TOPICS" ;;
  *) echo "unknown profile '$PROFILE' (use: light | full)"; exit 1 ;;
esac

echo "[record_aurora] profile = $PROFILE"
echo "[record_aurora] output  = ${OUT}.bag"
echo "[record_aurora] topics  = $TOPICS"
echo "[record_aurora] recording... press Ctrl+C to stop."
exec rosbag record -O "$OUT" $TOPICS
