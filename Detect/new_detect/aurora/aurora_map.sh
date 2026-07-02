#!/usr/bin/env bash
# =============================================================================
# aurora_map.sh — headless Aurora map control (no Aurora Remote app).
# Requires the node running and this shell on the SAME ROS master:
#   source ~/aurora_ros/devel/setup.bash
#   roslaunch ~/Go_by_my_self/Detect/new_detect/aurora_slam.launch
#
# Usage:
#   aurora_map.sh save  ~/maps/site.stcm    # download current map -> file (on PC)
#   aurora_map.sh load  ~/maps/site.stcm    # upload a saved map -> device
#   aurora_map.sh reloc                     # ask device to relocalize in loaded map
#   aurora_map.sh reset [kind]              # clear onboard map (kind 0=explorer, default)
#                                           #   if reset has no effect, power-cycle the Aurora
# =============================================================================
# self-source ROS + the Aurora workspace so this works from any terminal
# (before `set -u`, since the setup scripts reference unset vars)
source /opt/ros/noetic/setup.bash 2>/dev/null || true
[ -f "$HOME/aurora_ros/devel/setup.bash" ] && source "$HOME/aurora_ros/devel/setup.bash" 2>/dev/null || true

set -euo pipefail
NS=/slamware_ros_sdk_server_node
cmd="${1:-}"; arg="${2:-}"
case "$cmd" in
  save)  rosservice call "$NS/sync_get_stcm" "mapfile: '${arg:?usage: save <path.stcm>}'" ;;
  load)  rosservice call "$NS/sync_set_stcm" "mapfile: '${arg:?usage: load <path.stcm>}'" ;;
  reloc) rosservice call "$NS/relocalization" "{}" ;;
  reset) rostopic pub -1 "$NS/clear_map" slamware_ros_sdk/ClearMapRequest "{kind: {kind: ${arg:-0}}}" ;;
  *) echo "usage: $0 {save <f.stcm>|load <f.stcm>|reloc|reset [kind]}"; exit 1 ;;
esac
