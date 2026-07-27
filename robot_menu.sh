#!/bin/bash
# =============================================================================
# robot_menu.sh — one menu to launch every test on the Jetson.
# Run it ON THE JETSON (its own desktop for the GUI tests, or over SSH for the
# headless ones). Movement items (1,7) auto-start the robot base; autopilot (4)
# starts everything it needs.
# =============================================================================
# full ROS workspace chain (same as ~/.bashrc) so ALL packages resolve:
# turn_on_wheeltec_robot (base) + cartographer/lidar/arm + Aurora msgs
source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/cartographer_noetic/devel_isolated/setup.bash --extend 2>/dev/null
source ~/wheeltec_robot/devel/setup.bash --extend 2>/dev/null
source ~/wheeltec_lidar/devel/setup.bash --extend 2>/dev/null
source ~/wheeltec_arm/devel/setup.bash --extend 2>/dev/null
source ~/aurora_ros/devel/setup.bash --extend 2>/dev/null   # --extend: don't drop wheeltec pkgs
source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null
conda activate wheeltec 2>/dev/null
ND=~/Go_by_my_self/Detect/new_detect

# start the robot base (motor driver) if it isn't already up — needed to MOVE
ensure_base(){
  if rosnode list 2>/dev/null | grep -q wheeltec_robot; then echo "[base] already running."; return; fi
  echo "[base] starting robot base (motors) in the background..."
  nohup roslaunch turn_on_wheeltec_robot mapping.launch >~/base.log 2>&1 &
  echo "[base] waiting ~8s for it to come up..."; sleep 8
}

while true; do
  clear
  echo "=============================================="
  echo "   ROBOT TEST MENU  (Jetson $(hostname -I | awk '{print $1}'))"
  echo "   env: $(python3 -c 'import torch;print("torch",torch.__version__,"CUDA",torch.cuda.is_available())' 2>/dev/null)"
  [ -n "$DISPLAY" ] && echo "   display: $DISPLAY (GUI ok)" || echo "   display: NONE (only headless 3/6 work here)"
  echo "=============================================="
  echo "  PERCEPTION"
  echo "   1) Segmentation + lane-centering  (GPU, drives)  [GUI]"
  echo "   2) Segmentation — perception only (no motors)    [GUI]"
  echo "   3) Camera smoke test              (headless)"
  echo "  AUTOPILOT"
  echo "   4) *** AUTOPILOT TEST ***  pick a route, it does the rest"
  echo "  AURORA / DEBUG"
  echo "   5) Aurora built-in segmentation test             [GUI]"
  echo "   6) Aurora status monitor            (headless)"
  echo "  MANUAL"
  echo "   7) Teleop panel  (pop-up window)                 [GUI]"
  echo "   q) quit"
  echo "----------------------------------------------"
  read -rp "choose: " c
  c="${c//[[:space:]]/}"       # strip stray spaces/CR from the RDP keyboard
  case "$c" in
    1) ensure_base; python3 "$ND/ros_detect_dual.py" ;;
    2) python3 "$ND/ros_detect_dual.py" _drive:=false ;;
    3) python3 ~/seg_smoke.py ;;
    4) bash ~/Go_by_my_self/autopilot_test.sh ;;
    5) python3 "$ND/aurora/inspect_semantic_seg.py" ;;
    6) python3 "$ND/aurora/aurora_status.py" ;;
    7) ensure_base; python3 "$ND/ros_teleop_panel.py" ;;
    q|Q) exit 0 ;;
    *) echo "unknown choice: '$c'"; sleep 1 ;;
  esac
  echo; read -rp "[done] press Enter to return to menu..." _
done
