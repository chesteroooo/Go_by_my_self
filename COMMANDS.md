# COMMANDS — Aurora S SLAM + segmentation runbook

Quick reference for every workflow. **Every terminal that runs a ROS command
needs the env setup (§0) first** — a fresh un-sourced terminal is the #1 cause of
`ModuleNotFoundError: slamware_ros_sdk` and `Unable to load type ...` errors.

---

## 0. Env setup — run at the top of EVERY terminal

Aurora work (home / standalone):
```bash
source /opt/ros/noetic/setup.bash && source ~/aurora_ros/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311 && export ROS_IP=127.0.0.1
```
On the real robot: skip the two `export` lines (your `~/.bashrc` points at the
robot master `10.0.11.2`). Tip: put the two `source` lines in `~/.bashrc`.

---

## 1. Collect training data (RealSense D435i color)
```bash
python3 Detect/new_detect/segmentation/collect_floor_dataset.py --name redroad --interval 0.5
```
- Saves to `Detect/new_detect/train_data/session_<timestamp>_redroad/`
- Keys: `s` = save one, `p` = pause/resume auto-save, `q` = quit
- D435i = one program at a time (close other camera programs first).

---

## 2. Aurora — build a map (use the App)
Map building is done in **Aurora Remote** (visual feedback beats headless here).
**Close any ROS Aurora node first — only ONE client can hold the Aurora at a time.**
1. Open Aurora Remote → connect to `192.168.11.1`
2. Device Operations → **Reset Map**, hold still until initialization completes (green)
3. Drive/walk the route — smooth motion, mount height, close a loop
4. Map Manager → **Download to File** → save to `~/maps/<name>.stcm`
   (`.stcm` files are gitignored; keep them all in `~/maps/`)

## 3. Aurora — save the map from ROS (alternative to the App)
```bash
mkdir -p ~/maps
Detect/new_detect/aurora/aurora_map.sh save ~/maps/redroad.stcm      # expect: success: True
```

## 4. Aurora — test the map (load + relocalize)
```bash
Detect/new_detect/aurora/aurora_map.sh load  ~/maps/redroad.stcm
Detect/new_detect/aurora/aurora_map.sh reloc
# watch T2: pose searches, then locks into the map
```
Also: `aurora_map.sh reset` (clear the onboard map).

## 5. Aurora — analyze SLAM quality (offline)
```bash
Detect/new_detect/aurora/record_aurora.sh full site_run1     # light=small | full=+images/cloud
# ...move, Ctrl+C to stop...
python3 Detect/new_detect/aurora/analyze_aurora_bag.py ~/aurora_bags/aurora_site_run1_*.bag
```

## 6. Aurora — test its built-in semantic segmentation (on the road)
```bash
python3 Detect/new_detect/aurora/inspect_semantic_seg.py     # node must be running + facing the road
```

---

## 7. Segmentation model — one-time dependency install
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install transformers pillow opencv-python numpy
```

## 8. Segmentation — test pretrained SegFormer on images
```bash
python3 Detect/new_detect/segmentation/segformer_road.py <img1.jpg> <img2.jpg>
# overlays -> ~/aurora_bags/segformer_out/
```

## 9. Segmentation — auto-label -> YOLOv8-seg dataset
```bash
python3 Detect/new_detect/segmentation/autolabel_segformer.py \
  --data Detect/new_detect/train_data \
  --out ~/aurora_bags/autolabel_dataset --n 300 --viz
```
Then: upload `autolabel_dataset/{images,labels,data.yaml}` to Roboflow/CVAT →
correct (fix red-lane vs grey-sidewalk) → train YOLOv8-seg on Colab/Kaggle GPU →
put the trained model at `Detect/new_detect/best.pt` (auto-found by `ros_detect_dual.py`;
override with `_model:=/path/to/best.pt`).

---

## 10. Existing detection / control nodes
```bash
python3 Detect/new_detect/ros_detect_apriltag.py      # AprilTag perception -> /target_info
python3 Detect/new_detect/ros_move_pair_task.py       # outbound+return task controller
python3 Detect/new_detect/ros_detect_dual.py          # AprilTag + best.pt road seg + lane centering
#   drive mode is on by default but starts PAUSED — press SPACE in the OpenCV window to
#   start/stop lane-centered driving (publishes /cmd_vel; do NOT run ros_move_* alongside).
#   Perception-only (pair with a ros_move_* controller instead):
python3 Detect/new_detect/ros_detect_dual.py _drive:=false
#   Lane-following test on the 4-core N100: disable AprilTag/IR so seg runs ~7Hz not ~2Hz:
python3 Detect/new_detect/ros_detect_dual.py _tags:=false
python3 Detect/new_detect/ros_test_ground_bypass.py   # self-contained bypass test
python3 Detect/new_detect/ros_teleop_panel.py         # teleop -> /cmd_vel: pops up a control window
#   Hold-to-drive pad (arcs on diagonals) + linear/angular speed sliders; auto-stops
#   0.6s after the window closes / link drops. `--web` = browser panel at http://<pc-ip>:8765
#   (for Windows/phone remote control). `--sim` = test indoors without ROS.
#   Do NOT run alongside other /cmd_vel publishers (drive-mode dual, ros_move_*).
```

## 11. Push to GitHub (only when you decide to)
```bash
git add -A
git commit -m "your message"
git pull --rebase origin main && git push origin main
```

---

## Inspect anything live
```bash
rostopic list
rostopic echo /cmd_vel
rostopic hz /slamware_ros_sdk_server_node/robot_pose
rosservice list | grep stcm
```

## Aurora power
- DC input: **XT30, 9–24 V, ~10 W** (LiDAR not included) — a 19 V source is in range.
- Or **USB-C PD** (≥9 V PD source; a 5 V-only port will NOT power it).
- USB-C is power-only; **data is Ethernet** (`192.168.11.1`).
