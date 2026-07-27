#!/bin/bash
# =============================================================================
# autopilot_test.sh — ONE-CLICK autopilot test. Does EVERYTHING:
#   pick a route -> pick a map -> start robot base -> start Aurora SLAM ->
#   load the map -> wait until it localizes -> run the autopilot.
# You only choose the route (and map, if more than one). Ctrl-C stops it all.
# =============================================================================
source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/cartographer_noetic/devel_isolated/setup.bash --extend 2>/dev/null
source ~/wheeltec_robot/devel/setup.bash --extend 2>/dev/null
source ~/wheeltec_lidar/devel/setup.bash --extend 2>/dev/null
source ~/wheeltec_arm/devel/setup.bash --extend 2>/dev/null
source ~/aurora_ros/devel/setup.bash --extend 2>/dev/null   # --extend: keep wheeltec pkgs
source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null; conda activate wheeltec 2>/dev/null
ND=~/Go_by_my_self/Detect/new_detect

BG=()
cleanup(){ echo; echo "[autopilot] stopping everything..."; for p in "${BG[@]}"; do kill "$p" 2>/dev/null; done
  pkill -f slamware_ros_sdk_server_node 2>/dev/null; pkill -f ros_move_follow_route 2>/dev/null; }
trap cleanup EXIT                                   # normal/any exit -> cleanup
trap 'echo; echo "[autopilot] Ctrl-C -> back to menu"; exit 130' INT TERM   # Ctrl-C exits cleanly

# ---- 1. choose ROUTE ----
mapfile -t PLANS < <(ls $ND/route/routes_*/plan_*.csv 2>/dev/null)
[ ${#PLANS[@]} -eq 0 ] && { echo "no route plans found under route/routes_*/"; exit 1; }
echo "==== choose a ROUTE to test ===="
for i in "${!PLANS[@]}"; do echo "  $((i+1))) ${PLANS[$i]#$ND/route/}"; done
read -rp "route #: " ri
PLAN="${PLANS[$((ri-1))]}"
[ -z "$PLAN" ] && { echo "invalid choice"; exit 1; }
echo "  -> ${PLAN##*/}"

# ---- 2. choose MAP ----
mapfile -t MAPS < <(ls ~/maps/*.stcm 2>/dev/null)
if [ ${#MAPS[@]} -eq 0 ]; then
  echo "!! No map found in ~/maps/*.stcm — autopilot needs a saved SLAM map."
  echo "   Copy one over first (ask Claude to rsync your .stcm), then retry."
  exit 1
elif [ ${#MAPS[@]} -eq 1 ]; then
  MAP="${MAPS[0]}"; echo "map: ${MAP##*/}"
else
  echo "==== choose a MAP ===="
  for i in "${!MAPS[@]}"; do echo "  $((i+1))) ${MAPS[$i]##*/}"; done
  read -rp "map #: " mi; MAP="${MAPS[$((mi-1))]}"
  [ -z "$MAP" ] && { echo "invalid choice"; exit 1; }
fi

# ---- 3. roscore ----
rostopic list >/dev/null 2>&1 || { echo "[autopilot] starting roscore..."; roscore >~/roscore.log 2>&1 & BG+=($!); sleep 6; }

# ---- 4. robot base (motors) if not already running ----
if ! rosnode list 2>/dev/null | grep -q wheeltec_robot; then
  echo "[autopilot] starting robot base (motors)..."
  roslaunch turn_on_wheeltec_robot mapping.launch >~/base.log 2>&1 & BG+=($!); sleep 8
else
  echo "[autopilot] robot base already running."
fi

# ---- 5. Aurora visual SLAM ----
echo "[autopilot] starting Aurora visual SLAM..."
roslaunch $ND/aurora/aurora_slam.launch >~/aurora_launch.log 2>&1 & BG+=($!); sleep 8

# ---- 6. load the map + ask to relocalize ----
echo "[autopilot] loading map ${MAP##*/} ..."
$ND/aurora/aurora_map.sh load "$MAP"; sleep 2
$ND/aurora/aurora_map.sh reloc 2>/dev/null

# ---- 7. wait until Aurora is actually localized (pose streaming) ----
echo "[autopilot] >>> Place the car at the route START, facing the travel direction. <<<"
echo "[autopilot] waiting for Aurora to localize (up to 180 s)."
echo "[autopilot]   press  q  to abort back to the menu  (or Ctrl-C)."
OK=0; START=$(date +%s)
while true; do
  if timeout 2 rostopic echo -n1 /slamware_ros_sdk_server_node/robot_pose >/dev/null 2>&1; then
    echo; echo "[autopilot] Aurora pose is streaming — localized ✓"; OK=1; break
  fi
  el=$(( $(date +%s) - START ))
  [ "$el" -ge 180 ] && break
  printf "\r  waiting... %3ds   (press q to abort)   " "$el"
  read -t 1 -n 1 -r k 2>/dev/null && { [ "$k" = q ] || [ "$k" = Q ]; } && { echo; echo "[autopilot] aborted -> back to menu"; exit 1; }
done
echo
if [ "$OK" -ne 1 ]; then
  echo "!! Aurora never streamed a pose (not localized) — see the Aurora power/app note."
  echo "   Watch live state with menu item 6 (aurora_status.py)."
  echo -n "   run autopilot anyway? [y/N] (q/Enter = back to menu) "; read -r go
  { [ "$go" = y ] || [ "$go" = Y ]; } || { echo "[autopilot] back to menu"; exit 1; }
fi

# ---- 8. run the autopilot ----
echo "[autopilot] STARTING autopilot on ${PLAN##*/}"
echo "            SPACE = e-stop/resume   |   q = quit   |   walk beside the car"
python3 $ND/route/ros_move_follow_route.py --plan "$PLAN"
