# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Goal — Visual-SLAM Autonomous Driving

The end goal is **autonomous driving of the car**, using the **SLAMTEC Aurora S** visual SLAM as
the **primary localization for almost the entire route**. How to run it lives in
`Detect/new_detect/aurora/aurora_slam.launch` (see "Aurora S Visual SLAM" under Running the System).

**Aurora hardware:** connects to the PC over **Ethernet at `192.168.11.1`** (data); USB-C is
**power-only** (USB PD). Mounted at the **front of the car, laterally centred, ~0.45 m high,
0.15 m forward of `base_link`**.

**Deployment environment & the localization challenge:** the route includes one stretch that is
**open and straight** — paved with **red + grey brick**, flanked by **grass on both sides**, with
**trees on the left** and **buildings on the right**. This open, self-similar corridor is expected
to be **hard for pure visual SLAM to localize** in (few close, stable features).

**Plan — segmentation-assisted localization:** use **segmentation** (red/grey-brick road vs
grass/trees/buildings) to compensate where visual SLAM degrades on that open straight path — keep
the car centred on the segmented road and constrain heading so lateral/heading drift is corrected
even when the SLAM pose is weak. This builds on the existing floor-segmentation work
(`ros_detect_dual.py`, `collect_floor_dataset.py`).

## Current Focus — TANet Paper (deadline 2026-08-15)

**Read `paper/PLAN.md` before working on anything paper-related — it is the single source of
truth for the plan, schedule, and experiment design.** Summary: a lightweight VLM (4-bit GGUF,
llama.cpp) running on a **QCS6490 board (Ubuntu)** acts as an event-triggered scene-state
supervisor — it detects the visual-SLAM degradation zones (the open brick corridor above) from
single frames. Ground truth is derived from recorded rosbag SLAM telemetry via
`analyze_aurora_bag.py`, not hand labels. The QCS6490 is framed as the **target platform to
replace the onboard x86 PC** (Aurora computes SLAM on-device; the wheeltec base has its own
controller). Main results = accuracy × latency × power on QCS6490, vs the onboard PC (no GPU,
compare perf/watt) and cloud Gemini (the field 5G private network has no internet — cite as
edge motivation). Monitoring VQA (what blocks the path / alert?) is a qualitative demo only.

## Environment Setup

ROS Noetic must be sourced before running any ROS nodes:
```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://10.0.11.2:11311
export ROS_IP=10.0.11.3
```

Install Python dependencies:
```bash
pip3 install pupil-apriltags opencv-python PyYAML numpy
sudo apt install ros-noetic-cv-bridge ros-noetic-vision-opencv -y
```

## Repository Structure

```
Go_by_my_self/
├── calibration/                        # 攝影機校正工具 (僅 old_detect 需要)
│   ├── capture_chessboard_images.py    # 拍攝棋盤格校正影像
│   └── calibrate_from_images.py        # 計算內參，輸出 Detect/old_detect/calib_result.yaml
├── Detect/
│   ├── apriltag_setting/               # AprilTag pair 核心演算法 (共用模組)
│   │   ├── pair_detector_setting.py    # PairDetector：偵測 tag pair、方向、穩定度
│   │   └── pair_detector_balance.py    # 延伸：帶 depth_diff 的 BalancePairDetector
│   ├── old_detect/                     # 攝影機在車上 (ROS topic 傳影像)
│   │   ├── calib_result.yaml           # 攝影機內參 (old_detect 專用)
│   │   ├── ros_detect.py               # 接收 /usb_cam/image_raw，發布 /target_info
│   │   └── ros_move_follow_tag.py      # 依 /target_info 跟隨 tag (PID)
│   └── new_detect/                     # RealSense D435i 直接連電腦
│       ├── ros_detect_apriltag.py      # IR 串流 AprilTag 偵測，發布 /target_info
│       ├── ros_detect_dual.py          # IR(AprilTag)+Color(best.pt 路面分割) 雙串流＋車道置中
│       ├── ros_move_pair_task.py       # 去程+回程任務控制器 (鍵盤 e-stop，內含 RouteMap)
│       ├── ros_move_turn_right.py      # 右轉任務：0_2 正上方停車→右轉90°→停在 0_4 前
│       ├── ros_test_bypass.py          # 障礙繞行測試 (靜止障礙 S 形右繞，深度+光達雙重把關)
│       ├── ros_test_ground_bypass.py   # 地面 tag 門 + /odom 路徑記憶 + 閉迴路繞障回線
│       ├── ros_teleop_panel.py         # 遙控面板 → /cmd_vel (預設彈出視窗；--web 瀏覽器 :8765)
│       ├── route_map.yaml              # 路線地圖設定
│       ├── aurora/                     # Aurora S 視覺 SLAM 工具（皆獨立、無相依）
│       │   ├── aurora_slam.launch      # SLAM 包裝 launch (frames 改名 aurora_*，不與 wheeltec TF 衝突)
│       │   ├── aurora_status.py        # 無 App 的即時狀態列 (init/tracking/reloc/pose)
│       │   ├── aurora_map.sh           # 無 App 的地圖 save/load/reloc/reset (ROS services)
│       │   ├── record_aurora.sh        # rosbag 錄製 (light/full profile)
│       │   ├── analyze_aurora_bag.py   # 離線 SLAM 品質分析 (漂移/跳點/狀態/TF)
│       │   └── inspect_semantic_seg.py # 測試 Aurora 內建語意分割
│       ├── segmentation/               # 地板分割 / 資料集工具（皆獨立）
│       │   ├── collect_floor_dataset.py# YOLO 地板訓練資料收集 (Color 串流) → ../train_data/
│       │   ├── segformer_road.py       # 預訓練 SegFormer 路面分割測試
│       │   ├── autolabel_segformer.py  # SegFormer 自動標註 → YOLOv8-seg 資料集 (跨平台)
│       │   └── gemini_filter.py        # Gemini API 影像品質過濾 (clear/minor/bad)
│       ├── train_data/                 # 訓練影像 (session_* 資料夾)
│       └── sorted_data/                # 已分類訓練素材
├── COMMANDS.md                         # 所有工作流程的指令速查表
├── requirements.txt
└── README.md
```

## Camera Calibration (one-time, old_detect only)

RealSense (new_detect) uses the camera's built-in factory intrinsics, so calibration is **only** needed for the old USB-cam pipeline.

```bash
python3 calibration/capture_chessboard_images.py   # 按 s 儲存影像，q 離開
python3 calibration/calibrate_from_images.py        # 輸出 Detect/old_detect/calib_result.yaml
```

## Running the System

### On the Robot (SSH to wheeltec@10.0.11.2)
```bash
roslaunch turn_on_wheeltec_robot mapping.launch
roslaunch usb_cam usb_cam-test.launch
```

### Old Detect — Camera on Robot
```bash
python3 Detect/old_detect/ros_detect.py          # 感知節點，發布 /target_info
python3 Detect/old_detect/ros_move_follow_tag.py # 控制節點，訂閱 /target_info
```

### New Detect — RealSense D435i on PC

**IMPORTANT: the D435i can only be opened by ONE program at a time.**
Detect nodes (`ros_detect_*.py`), test nodes (`ros_test_*.py`), `collect_floor_dataset.py`,
and `realsense-viewer` all open the camera — never run two of them together.
The `ros_test_*.py` nodes are self-contained (camera + control in one file): run them
*instead of* `ros_detect_apriltag.py`, not alongside it. The `ros_move_*.py` controllers
do NOT open the camera (they only subscribe to `/target_info`) and pair with a detect node.

```bash
python3 Detect/new_detect/ros_detect_apriltag.py # IR AprilTag 感知節點，發布 /target_info
python3 Detect/new_detect/ros_move_pair_task.py  # 去程+回程任務控制節點

# 或：同時跑 AprilTag(IR) + best.pt 路面分割(Color) + 車道置中
# 模型放 Detect/new_detect/best.pt（自動尋找，_model:= 可覆寫）。drive 模式預設開啟但啟動為
# PAUSE，OpenCV 視窗按 SPACE 開始/暫停置中行駛（此時發 /cmd_vel，勿同時跑 ros_move_*）
python3 Detect/new_detect/ros_detect_dual.py                 # /target_info /floor_detected /floor_info /cmd_vel
python3 Detect/new_detect/ros_detect_dual.py _drive:=false   # 純感知，搭配 ros_move_* 控制器
python3 Detect/new_detect/ros_detect_dual.py _tags:=false    # 關 AprilTag/IR，CPU 全給分割（車道跟隨測試）

# 或（單獨跑，不可與上面同時）：自帶相機的測試節點
python3 Detect/new_detect/ros_test_bypass.py         # 障礙繞行測試
python3 Detect/new_detect/ros_test_ground_bypass.py  # 地面 tag 門 + /odom 路徑記憶 + 繞障回線
```

### Aurora S Visual SLAM (Ethernet — independent of the D435i)

The Aurora S connects over Ethernet at `192.168.11.1` (power via USB-C PD or DC 9–24V). Build the
vendor SDK once (`git clone https://github.com/Slamtec/aurora_ros ~/aurora_ros`, then `catkin_make`),
then launch the project wrapper:
```bash
source ~/aurora_ros/devel/setup.bash
roslaunch ~/Go_by_my_self/Detect/new_detect/aurora/aurora_slam.launch   # 視覺 SLAM，frames = aurora_*
```
Companion tools in `Detect/new_detect/aurora/`: `aurora_status.py` (live status, no app),
`aurora_map.sh save|load|reloc|reset` (headless map control), `record_aurora.sh` +
`analyze_aurora_bag.py` (record & analyze SLAM quality). **Only ONE client can hold the
Aurora at a time** — close the Aurora Remote app before launching the ROS node, and vice versa.
Publishes 6DOF pose `/slamware_ros_sdk_server_node/robot_pose` plus `point_cloud`/depth/stereo/IMU.
Runs as an **independent TF tree** (`aurora_map → aurora_odom → aurora_base_link`), so it does not
collide with the robot's `map → odom → base_link`. For a standalone test without the robot, first
`export ROS_MASTER_URI=http://localhost:11311` (roslaunch then starts its own roscore).

### Inspect Topics
```bash
rostopic echo /target_info
rostopic echo /cmd_vel
rostopic hz /slamware_ros_sdk_server_node/robot_pose   # Aurora visual-SLAM pose (~15 Hz)
```

## Architecture

Off-board processing: the robot streams camera images over a 5GHz USB network card to an Ubuntu PC, which handles all computation and sends velocity commands back.

### ROS Topic Flow (old_detect)
```
Robot Camera → /usb_cam/image_raw → ros_detect.py → /target_info → ros_move_follow_tag.py → /cmd_vel → Robot Motors
```

### ROS Topic Flow (new_detect)
```
D435i IR → ros_detect_apriltag.py → /target_info → ros_move_pair_task.py → /cmd_vel → Robot Motors
                                     /floor_detected ↑ (ros_detect_dual.py only)

ros_detect_dual.py (drive mode):
D435i IR    → AprilTag         → /target_info
D435i Color → best.pt road seg → /floor_detected + /floor_info → lane-centering P-control → /cmd_vel
```

### /floor_info Message Format (geometry_msgs/Pose — repurposed, ros_detect_dual.py)
| Field | Meaning |
|---|---|
| `orientation.w` | 1.0 = road detected, 0.0 = not |
| `position.x` | err_norm: road center vs image center, left=−, right=+, in [-1, 1] |
| `position.y` | heading_norm: far-band vs near-band road centroid (road direction) |
| `position.z` | Near-band road coverage ratio 0–1 |

### /target_info Message Format (geometry_msgs/Pose — repurposed fields)
| Field | Meaning |
|---|---|
| `orientation.x` | Left tag ID |
| `orientation.y` | Right tag ID |
| `orientation.w` | 1.0 = detected, 0.0 = not found |
| `position.x` | Horizontal pixel error (left=negative, right=positive) |
| `position.y` | depth_diff = t_right.z − t_left.z (metres) |
| `position.z` | 3D Euclidean distance to tag pair (metres) |

### Key Modules

- **apriltag_setting/pair_detector_setting.py** — `PairDetector`: detects AprilTag pairs, computes 3D pose via SVD-averaged rotation, tracks stability via history deque. Detect nodes add this folder to `sys.path` (`../apriltag_setting`) before importing.
- **apriltag_setting/pair_detector_balance.py** — Extends `PairDetector` with per-tag translation vectors and depth_diff.
- **old_detect/ros_detect.py** — Loads `calib_result.yaml`, undistorts images from `/usb_cam/image_raw`, publishes to `/target_info`.
- **old_detect/ros_move_follow_tag.py** — PID follower, maintains target distance (0.5 m).
- **new_detect/ros_detect_apriltag.py** — Reads from RealSense D435i IR stream (global shutter, no jello), publishes to `/target_info`.
- **new_detect/ros_detect_dual.py** — Dual stream: IR→AprilTag (`/target_info`) + Color→YOLOv8-seg `best.pt` road segmentation. Publishes `/floor_detected` (Bool) and `/floor_info` (Pose repurposed: `orientation.w`=detected, `position.x`=lane-center offset err_norm, `position.y`=heading_norm, `position.z`=road coverage). YOLO runs in a worker thread (CPU ~120-190 ms @ imgsz 320) so the 30 fps AprilTag loop never blocks; the road class id is auto-resolved from `model.names` (dataset classes: 0=grass, 1=road, 2=sidewalk). Drive mode (default on, `_drive:=false` to disable) does lane-centering P-control on `/cmd_vel`: starts PAUSED, SPACE toggles run/pause, auto-stops when road coverage < `MIN_ROAD_COVER` or the seg result is stale.
- **new_detect/ros_move_pair_task.py** — Outbound+return task controller (state machine, keyboard e-stop). Defines `RouteMap` (reads `route_map.yaml`, pair→location lookup + BFS), reused by `ros_move_turn_right.py`.
- **new_detect/ros_test_ground_bypass.py** — Self-contained (IR+Depth): ground-gate steering, `/odom` path-line memory across blind gaps, closed-loop depth+lidar obstacle bypass that returns to the remembered line.
- **new_detect/aurora/aurora_slam.launch** — Wrapper for the SLAMTEC Aurora S ROS SDK (`slamware_ros_sdk`, built in `~/aurora_ros`). Runs the vendor node with all frames renamed `aurora_*` so its SLAM TF tree stays independent of the wheeltec tree. Primary localization for the autonomous-driving goal; fusion of its point cloud into `base_link` (mount offset 0.15 m fwd / 0 / 0.45 m up) is a later step (see the commented block at the bottom of the launch).
- **new_detect/segmentation/** — Standalone dataset/segmentation tools: `collect_floor_dataset.py` (RealSense capture → `../train_data/`), `segformer_road.py` (pretrained SegFormer test), `autolabel_segformer.py` (SegFormer → YOLOv8-seg auto-labels, cross-platform, `--data` required), `gemini_filter.py` (Gemini API image-quality sorter; needs `GEMINI_API_KEY`). Saved SLAM maps (`.stcm`) live in `~/maps/` — they are gitignored (too big for GitHub).

## Key Tuning Parameters

| File | Parameter | Default | Effect |
|---|---|---|---|
| old_detect/ros_move_follow_tag.py | `TARGET_DIST` | 0.5 m | Desired tag-to-robot distance |
| new_detect/ros_move_pair_task.py | `MAX_SPEED_V` | 0.2 | Cruise forward speed |
| new_detect/ros_move_pair_task.py | `TAG_LOST_TIMEOUT` | 10.0 s | Seconds with no tag before stopping |
| new_detect/ros_test_ground_bypass.py | `CAM_TILT_DEG` | 30.0° | Camera down-tilt; wrong value makes the ground read as an obstacle |
| new_detect/ros_test_ground_bypass.py | `DODGE_OFFSET` | 0.45 m | Rightward shift when bypassing (keep inside lane) |
| new_detect/ros_detect_apriltag.py | `TAG_SIZE_M` | 0.11 m | Printed tag side length (must match reality) |
| new_detect/ros_detect_apriltag.py | `W` / `H` / `FPS` | 1280/720/30 | IR stream resolution & frame rate |
| new_detect/ros_detect_dual.py | `LANE_SPEED_V` | 0.15 | Lane-centering cruise speed |
| new_detect/ros_detect_dual.py | `KP_CENTER` / `KP_HEADING` | 0.35 / 0.20 | err_norm / heading_norm → angular.z gains |
| new_detect/ros_detect_dual.py | `MIN_ROAD_COVER` | 0.10 | Near-band road coverage below this = road lost → stop |
| new_detect/ros_detect_dual.py | `YOLO_IMGSZ` | 320 | Seg inference size; larger = slower on CPU |
