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
├── calibration/                        # 攝影機校正工具 (共用)
│   ├── capture_chessboard_images.py    # 拍攝棋盤格校正影像
│   └── calibrate_from_images.py        # 計算內參，輸出 Detect/calib_result.yaml
├── Detect/
│   ├── calib_result.yaml               # 攝影機內參 (兩種 detect 共用)
│   ├── pair_detector_setting.py        # 核心演算法：PairDetector
│   ├── pair_detector_balance.py        # 延伸：帶深度差的 PairDetector
│   ├── requirements.txt
│   ├── old_detect/                     # 攝影機在車上 (ROS topic 傳影像)
│   │   ├── ros_detect.py              # 接收 /usb_cam/image_raw，發布 /target_info
│   │   └── ros_move_follow_tag.py     # 依 /target_info 跟隨 tag
│   └── new_detect/                    # 攝影機直接連電腦 (OpenCV 直讀)
│       ├── ros_detect_local.py        # 本機 USB 攝影機，發布 /target_info
│       ├── ros_move_map.py            # 地圖感知移動控制器
│       └── route_map.yaml             # 路線地圖設定
└── README.md
```

## Camera Calibration (one-time)

```bash
python3 calibration/capture_chessboard_images.py   # 按 s 儲存影像，q 離開
python3 calibration/calibrate_from_images.py        # 輸出 Detect/calib_result.yaml
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

### New Detect — Camera on PC
```bash
python3 Detect/new_detect/ros_detect_local.py    # 感知節點，發布 /target_info
python3 Detect/new_detect/ros_move_map.py        # 地圖感知控制節點
```

### Inspect Topics
```bash
rostopic echo /target_info
rostopic echo /cmd_vel
```

## Architecture

Off-board processing: the robot streams camera images over a 5GHz USB network card to a WSL2 PC, which handles all computation and sends velocity commands back.

### ROS Topic Flow (old_detect)
```
Robot Camera → /usb_cam/image_raw → ros_detect.py → /target_info → ros_move_follow_tag.py → /cmd_vel → Robot Motors
```

### ROS Topic Flow (new_detect)
```
PC Camera (OpenCV) → ros_detect_local.py → /target_info → ros_move_map.py → /cmd_vel → Robot Motors
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

- **pair_detector_setting.py** — `PairDetector`: detects AprilTag pairs, computes 3D pose via SVD-averaged rotation, tracks stability via history deque.
- **pair_detector_balance.py** — Extends `PairDetector` with per-tag translation vectors and depth_diff.
- **old_detect/ros_detect.py** — Loads `calib_result.yaml`, undistorts images from `/usb_cam/image_raw`, publishes to `/target_info`.
- **old_detect/ros_move_follow_tag.py** — PID follower, maintains target distance (0.5 m).
- **new_detect/ros_detect_local.py** — Reads from local USB camera via OpenCV, publishes to `/target_info`.
- **new_detect/ros_move_map.py** — State machine (INIT_SEARCH → INIT_ALIGN → DRIVING → STOPPED) with route map awareness.

## Key Tuning Parameters

| File | Parameter | Default | Effect |
|---|---|---|---|
| old_detect/ros_move_follow_tag.py | `TARGET_DIST` | 0.5 m | Desired tag-to-robot distance |
| new_detect/ros_move_map.py | `SEARCH_SPEED_W` | 0.37 | Rotation speed when searching |
