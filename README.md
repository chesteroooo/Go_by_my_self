# Go by Myself — AprilTag 自動導航車

場外運算 (Off-board Processing) 架構。車子負責採集影像與移動，透過 5GHz USB 網卡將影像回傳至電腦 (WSL2)，由電腦進行視覺運算並發送控制指令。

## 系統架構

| 裝置 | 說明 | IP |
|---|---|---|
| 車子 (Wheeltec, ROS Noetic) | 拍攝影像、接收速度指令驅動馬達 | 10.0.11.2 |
| 電腦 (Windows 11 + WSL2 Ubuntu 20.04) | AprilTag 偵測、發送控制指令 | 10.0.11.3 |

連線方式：USB 5GHz 網卡（低延遲、固定 IP）

## 資料夾結構

```
Go_by_my_self/
├── calibration/                        # 攝影機校正工具
│   ├── capture_chessboard_images.py    # 拍攝棋盤格影像
│   └── calibrate_from_images.py        # 計算內參，自動輸出到 Detect/calib_result.yaml
├── Detect/
│   ├── calib_result.yaml               # 攝影機校正結果 (兩種 detect 共用)
│   ├── pair_detector_setting.py        # 核心模組：AprilTag pair 偵測演算法
│   ├── pair_detector_balance.py        # 延伸模組：帶深度差資訊的 PairDetector
│   ├── requirements.txt                # Python 套件需求
│   ├── old_detect/                     # 攝影機裝在車上（透過 ROS topic 傳影像）
│   │   ├── ros_detect.py              # 訂閱 /usb_cam/image_raw，發布 /target_info
│   │   └── ros_move_follow_tag.py     # 跟隨 tag 移動（PID 控制）
│   └── new_detect/                    # 攝影機直接連電腦（OpenCV 直接讀取）
│       ├── ros_detect_local.py        # 本機 USB 攝影機，發布 /target_info
│       ├── ros_move_map.py            # 地圖感知移動控制（狀態機）
│       └── route_map.yaml             # 路線地圖設定
└── README.md
```

## 環境建置

### 1. 安裝 WSL2 (Ubuntu 20.04)

以管理員身分開啟 PowerShell：
```powershell
wsl --install -d Ubuntu-20.04
```

### 2. 開啟鏡像網路模式（Windows 11 必做）

在 `C:\Users\你的使用者名稱\` 建立 `.wslconfig`：
```ini
[wsl2]
networkingMode=mirrored
```
重啟 WSL：`wsl --shutdown`

### 3. 安裝 ROS Noetic

```bash
wget http://fishros.com/install -O fishros && . fishros
# 選擇：[1] 安裝 ROS -> [1] Noetic -> [1] Desktop-Full
```

### 4. 安裝 Python 套件

```bash
pip3 install pupil-apriltags opencv-python PyYAML numpy
sudo apt install ros-noetic-cv-bridge ros-noetic-vision-opencv -y
```

### 5. 設定 ROS 連線 IP

在 `~/.bashrc` 最下方加入：
```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://10.0.11.2:11311
export ROS_IP=10.0.11.3
```
```bash
source ~/.bashrc
```

## 攝影機校正（首次使用前執行一次）

```bash
python3 calibration/capture_chessboard_images.py   # 按 s 儲存影像，q 離開（建議 15~25 張）
python3 calibration/calibrate_from_images.py        # 自動輸出 Detect/calib_result.yaml
```

## 如何執行

### 步驟 1：啟動車子（SSH 進車子 `ssh wheeltec@10.0.11.2`）

```bash
roslaunch turn_on_wheeltec_robot mapping.launch
roslaunch usb_cam usb_cam-test.launch
```

### 步驟 2A：Old Detect — 攝影機在車上

在電腦 WSL 中執行（兩個 terminal 各開一個）：
```bash
python3 Detect/old_detect/ros_detect.py          # 感知節點
python3 Detect/old_detect/ros_move_follow_tag.py # 控制節點
```

### 步驟 2B：New Detect — 攝影機連電腦

確認攝影機接上電腦後（WSL2 需先用 `usbipd attach` 掛載），執行：
```bash
python3 Detect/new_detect/ros_detect_local.py    # 感知節點
python3 Detect/new_detect/ros_move_map.py        # 地圖感知控制節點
```

### 確認 Topic 輸出

```bash
rostopic echo /target_info
rostopic echo /cmd_vel
```

## /target_info 訊息格式（geometry_msgs/Pose 欄位對應）

| 欄位 | 意義 |
|---|---|
| `orientation.x` | 左 tag ID |
| `orientation.y` | 右 tag ID |
| `orientation.w` | 1.0 = 偵測到，0.0 = 未偵測到 |
| `position.x` | 水平像素誤差（負 = tag 在左，正 = tag 在右）|
| `position.y` | depth_diff = t_right.z − t_left.z（公尺）|
| `position.z` | 到 tag pair 中心的 3D 歐幾里得距離（公尺）|

## WSL2 USB 攝影機設定（New Detect 使用前）

在 Windows 端執行一次（需安裝 [usbipd-win](https://github.com/dorssel/usbipd-win)）：
```powershell
usbipd list                              # 列出 USB 裝置
usbipd attach --wsl --busid <BUSID>     # 掛載攝影機到 WSL2
```
WSL2 確認：`ls /dev/video*` 應看到 `/dev/video0`
