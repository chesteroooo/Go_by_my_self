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
# 每一行順便標出它配對的地圖在不在 ~/maps —— 不在的話等一下要手動挑圖，
# 挑錯就是照別張圖的座標開出去，所以先在選單上就講清楚。
for i in "${!PLANS[@]}"; do
  # first comment line of the plan carries direction + length
  w=$(awk '/^source:/{print $2; exit}' "$(dirname "${PLANS[$i]}")/index.yaml" 2>/dev/null)
  w="${w##*[\\/]}"
  if [ -n "$w" ] && [ -f ~/maps/"$w" ]; then tag="$w"; else tag="!! ${w:-?} 不在 ~/maps，要手動挑圖"; fi
  printf "  %2d) %-30s %-20s %s\n" "$((i+1))" "${PLANS[$i]#$ND/route/}" \
         "$(head -1 "${PLANS[$i]}" | sed 's/^# *//; s/ *(x,y,yaw_rad).*//')" "$tag"
done
read -rp "route #: " ri
case "$ri" in ''|*[!0-9]*) echo "invalid choice"; exit 1 ;; esac
{ [ "$ri" -ge 1 ] && [ "$ri" -le "${#PLANS[@]}" ]; } || { echo "invalid choice"; exit 1; }
PLAN="${PLANS[$((ri-1))]}"
echo "  -> ${PLAN##*/}"

# ---- 2. MAP: use the map this route was extracted from ----
#每張 .stcm 是獨立座標系 —— 路線配錯地圖，車會照別張圖的座標開出去。
# routes_*/index.yaml 的 source: 記著出處，優先自動配對。
mapfile -t MAPS < <(ls ~/maps/*.stcm 2>/dev/null)
if [ ${#MAPS[@]} -eq 0 ]; then
  echo "!! No map found in ~/maps/*.stcm — autopilot needs a saved SLAM map."
  echo "   Copy one over first (ask Claude to rsync your .stcm), then retry."
  exit 1
fi
WANT=$(awk '/^source:/{print $2; exit}' "$(dirname "$PLAN")/index.yaml" 2>/dev/null)
WANT="${WANT##*[\\/]}"          # 有些舊 index.yaml 存的是 Windows 路徑
if [ -n "$WANT" ] && [ -f ~/maps/"$WANT" ]; then
  MAP=~/maps/"$WANT"; echo "map: $WANT   (auto-paired — this route was extracted from it)"
elif [ ${#MAPS[@]} -eq 1 ]; then
  MAP="${MAPS[0]}"; echo "map: ${MAP##*/}"
else
  [ -n "$WANT" ] && echo "!! this route came from '$WANT', which is NOT in ~/maps — pick carefully:"
  echo "==== choose a MAP ===="
  for i in "${!MAPS[@]}"; do echo "  $((i+1))) ${MAPS[$i]##*/}"; done
  read -rp "map #: " mi
  case "$mi" in ''|*[!0-9]*) echo "invalid choice"; exit 1 ;; esac
  { [ "$mi" -ge 1 ] && [ "$mi" -le "${#MAPS[@]}" ]; } || { echo "invalid choice"; exit 1; }
  MAP="${MAPS[$((mi-1))]}"
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

# ---- 6. load the map ----
echo "[autopilot] >>> Place the car at the route START, facing the travel direction. <<<"
echo "[autopilot] loading map ${MAP##*/} ($(du -h "$MAP" | cut -f1)) — the pose pausing during upload is normal"
$ND/aurora/aurora_map.sh load "$MAP" || { echo "!! map upload failed"; exit 1; }
echo -n "[autopilot] waiting for the pose to come back"
for _ in $(seq 1 60); do
  timeout 2 rostopic echo -n1 /slamware_ros_sdk_server_node/robot_pose >/dev/null 2>&1 && break
  echo -n .
done; echo

# ---- 7. RELOCALIZE ----
# A streaming pose does NOT mean localized: with the map loaded but never matched, Aurora
# still publishes a pose in a fresh session frame. The old check here only tested "is a pose
# arriving", so it passed while completely unlocalized and the car drove off along another
# map's coordinates (the 0706 failure). reloc_wait.py waits for the real thing.
if ! python3 $ND/aurora/reloc_wait.py --timeout 180; then
  echo "!! relocalization did not succeed — watch live state with menu item 6 (aurora_status.py)."
  echo -n "   run autopilot anyway? [y/N] (q/Enter = back to menu) "; read -r go
  { [ "$go" = y ] || [ "$go" = Y ]; } || { echo "[autopilot] back to menu"; exit 1; }
fi

# ---- 7b. the pose must actually sit on the chosen route, pointing the right way ----
if ! python3 $ND/route/preflight.py --plan "$PLAN"; then
  echo -n "   start anyway? [y/N] (q/Enter = back to menu) "; read -r go2
  { [ "$go2" = y ] || [ "$go2" = Y ]; } || { echo "[autopilot] back to menu"; exit 1; }
fi

# ---- 8. integrated live map ----
# Read-only (never publishes /cmd_vel), so it runs happily next to the driver node.
# --leg converts this leg's pose into the merged A/B/C/D frame, so whichever route you
# picked you get the SAME integrated map with the car drawn in the right place.
LEG=$(basename "$(dirname "$PLAN")"); LEG="${LEG#routes_}"
echo "[autopilot] opening the integrated site map (leg $LEG) ..."
python3 $ND/route/route_monitor.py --leg "$LEG" >~/route_monitor.log 2>&1 & BG+=($!)
sleep 2

# ---- 9. run the autopilot ----
echo "[autopilot] STARTING autopilot on ${PLAN##*/}"
echo "            SPACE = e-stop/resume   |   q = quit   |   walk beside the car"
python3 $ND/route/ros_move_follow_route.py --plan "$PLAN"
