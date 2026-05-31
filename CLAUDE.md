# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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
│       ├── ros_detect_dual.py          # IR(AprilTag)+Color(YOLO 地板) 雙串流
│       ├── ros_move_map.py             # 地圖感知移動控制器 (狀態機)
│       └── route_map.yaml              # 路線地圖設定
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
```bash
python3 Detect/new_detect/ros_detect_apriltag.py # IR AprilTag 感知節點，發布 /target_info
python3 Detect/new_detect/ros_move_map.py        # 地圖感知控制節點

# 或：同時跑 AprilTag(IR) + YOLO 地板偵測(Color)
python3 Detect/new_detect/ros_detect_dual.py     # 發布 /target_info 與 /floor_detected
```

### Inspect Topics
```bash
rostopic echo /target_info
rostopic echo /cmd_vel
```

## Architecture

Off-board processing: the robot streams camera images over a 5GHz USB network card to an Ubuntu PC, which handles all computation and sends velocity commands back.

### ROS Topic Flow (old_detect)
```
Robot Camera → /usb_cam/image_raw → ros_detect.py → /target_info → ros_move_follow_tag.py → /cmd_vel → Robot Motors
```

### ROS Topic Flow (new_detect)
```
D435i IR → ros_detect_apriltag.py → /target_info → ros_move_map.py → /cmd_vel → Robot Motors
                                     /floor_detected ↑ (ros_detect_dual.py only)
```

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
- **new_detect/ros_detect_dual.py** — Dual stream: IR→AprilTag (`/target_info`) + Color→YOLO floor segmentation (`/floor_detected`).
- **new_detect/ros_move_map.py** — State machine (INIT_SEARCH → INIT_ALIGN → DRIVING → STOPPED) with route map awareness.

## Key Tuning Parameters

| File | Parameter | Default | Effect |
|---|---|---|---|
| old_detect/ros_move_follow_tag.py | `TARGET_DIST` | 0.5 m | Desired tag-to-robot distance |
| new_detect/ros_move_map.py | `INIT_SEARCH_W` | 0.3 | Rotation speed when searching for first tag |
| new_detect/ros_move_map.py | `MAX_SPEED_V` | 0.2 | Forward speed while driving |
| new_detect/ros_move_map.py | `TAG_TIMEOUT` | 10.0 s | Seconds with no tag before stopping |
| new_detect/ros_detect_apriltag.py | `TAG_SIZE_M` | 0.08 m | Printed tag side length (must match reality) |
| new_detect/ros_detect_apriltag.py | `W` / `H` / `FPS` | 848/480/60 | IR stream resolution & frame rate |
