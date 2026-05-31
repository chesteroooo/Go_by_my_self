# Go by Myself — AprilTag 雙標籤自動導航車

> 機器人沿著地面上成對的 AprilTag 標籤自主行走。攝影機與運算放在電腦端（場外運算），
> 機器人只負責「拍影像」與「動」，所有視覺辨識與決策都在 Ubuntu 電腦上完成。

---

## 目錄

- [這個專案在做什麼](#這個專案在做什麼)
- [運作原理](#運作原理)
- [硬體需求](#硬體需求)
- [系統架構](#系統架構)
- [資料夾結構](#資料夾結構)
- [安裝步驟](#安裝步驟)
- [執行系統](#執行系統)
- [/target_info 訊息格式](#target_info-訊息格式)
- [參數調校](#參數調校)
- [如何開發與擴充](#如何開發與擴充)
- [疑難排解](#疑難排解)

---

## 這個專案在做什麼

地面上會貼上一對一對的 **AprilTag**（黑白方形標籤）。每一對標籤標記一個「地點」或「路徑節點」。
機器人用攝影機看到這對標籤，計算：

1. **方向** — 自己有沒有對準標籤（左右偏移、是否歪斜）
2. **距離** — 離標籤多遠
3. **位置** — 現在走到地圖上的哪一個節點

然後機器人據此調整方向、持續前進，沿著標籤鋪設的路線自動行走。

**為什麼用「成對」標籤而不是單一標籤？**
兩個並排的標籤可以算出**深度差**（`depth_diff`）——左右標籤誰離鏡頭比較遠。
這讓機器人知道自己是不是「斜著」面對牆面，可以即時轉正，比單一標籤穩定得多。

---

## 運作原理

```
        ┌─────────────────────────────────────────────────────┐
        │                  Ubuntu 電腦（場外運算）              │
        │                                                       │
   影像  │   D435i 紅外線串流 ──► AprilTag 偵測 ──► 配對演算法    │
  ◄──────┤                                              │        │
        │                                          /target_info  │
        │                                              │        │
        │                          移動控制器（狀態機）◄┘        │
        │                                  │                     │
        └──────────────────────────────────┼─────────────────────┘
                                  /cmd_vel  │  速度指令
                                            ▼
        ┌─────────────────────────────────────────────────────┐
        │              Wheeltec 機器人（ROS Noetic）            │
        │            收到速度指令 → 驅動馬達 → 移動              │
        └─────────────────────────────────────────────────────┘
```

**關鍵設計：用紅外線（IR）而不是彩色影像做 AprilTag**
D435i 的 IR 鏡頭是 **global shutter（全域快門）**，機器人在磚頭路上震動時不會產生「果凍效應」失真；
而且 IR 本來就是灰階，AprilTag 可以直接吃，不需轉換。彩色（RGB）鏡頭是 rolling shutter，震動時影像會扭曲。

---

## 硬體需求

| 項目 | 說明 |
|---|---|
| Wheeltec 機器人 | 搭載 ROS Noetic，提供 `/cmd_vel` 馬達控制介面 |
| Intel RealSense D435i | 深度攝影機，使用其 IR 串流做 AprilTag 偵測 |
| Ubuntu 20.04 電腦 | 執行所有視覺運算與控制邏輯 |
| 5GHz USB 網卡 | 機器人與電腦間的低延遲連線（固定 IP） |
| AprilTag 標籤（tag36h11） | 列印貼在地面，建議邊長 8cm 以上 |

---

## 系統架構

| 裝置 | 角色 | IP |
|---|---|---|
| 車子（Wheeltec, ROS Noetic） | ROS Master，接收 `/cmd_vel` 驅動馬達 | `10.0.11.2` |
| 電腦（Ubuntu 20.04） | ROS 節點，視覺偵測 + 發送控制指令 | `10.0.11.3` |

連線方式：USB 5GHz 網卡（低延遲、固定 IP）。電腦端**不需要**自己跑 `roscore`，
而是透過 `ROS_MASTER_URI` 連到車子的 `roscore`。

### ROS Topic 資料流

```
D435i IR ─► ros_detect_apriltag.py ─► /target_info ─► ros_move_map.py ─► /cmd_vel ─► 機器人馬達
```

---

## 資料夾結構

```
Go_by_my_self/
├── calibration/                       # 攝影機校正工具（僅 old_detect 需要）
│   ├── capture_chessboard_images.py   # 拍攝棋盤格校正影像
│   └── calibrate_from_images.py       # 計算內參，輸出 old_detect/calib_result.yaml
│
├── Detect/
│   ├── apriltag_setting/              # ★ AprilTag 配對核心演算法（共用模組）
│   │   ├── pair_detector_setting.py   #   PairDetector：偵測 tag pair、方向、穩定度
│   │   └── pair_detector_balance.py   #   BalancePairDetector：加上 depth_diff
│   │
│   ├── old_detect/                   # 舊版：攝影機裝在車上（透過 ROS topic 傳影像）
│   │   ├── calib_result.yaml          #   攝影機內參（old_detect 專用）
│   │   ├── ros_detect.py              #   訂閱 /usb_cam/image_raw，發布 /target_info
│   │   └── ros_move_follow_tag.py     #   跟隨 tag 移動（PID 控制）
│   │
│   └── new_detect/                   # ★ 現行：RealSense D435i 直接連電腦
│       ├── ros_detect_apriltag.py     #   IR 串流 AprilTag 偵測，發布 /target_info
│       ├── ros_detect_dual.py         #   IR(AprilTag) + Color(YOLO 地板) 雙串流
│       ├── ros_move_map.py            #   地圖感知移動控制器（狀態機）
│       └── route_map.yaml             #   路線地圖設定
│
├── requirements.txt                  # Python 套件需求
└── README.md
```

> **★ 標記為目前主要使用的部分。** `old_detect/` 是攝影機裝在車上的舊架構，保留作參考。

---

## 安裝步驟

### 步驟 1 — 安裝 ROS Noetic

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
sudo apt install ros-noetic-cv-bridge ros-noetic-vision-opencv -y
```

### 步驟 2 — 安裝 RealSense D435i 驅動

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCD
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update && sudo apt install librealsense2-dkms librealsense2-utils -y

realsense-viewer   # 插上相機，確認能看到畫面
```

### 步驟 3 — 安裝 Python 套件

```bash
pip3 install -r requirements.txt
```

> 內含 `numpy opencv-python PyYAML pupil-apriltags pyrealsense2 ultralytics`。
> `ultralytics` 僅 `ros_detect_dual.py` 的 YOLO 功能需要，其他節點不裝也能跑。

### 步驟 4 — 設定 ROS 網路連線

確認 USB 網卡 IP：

```bash
ip addr show          # 找出 10.0.11.x 子網段那張網卡的 IP
```

在 `~/.bashrc` 最下方加入（用 `nano ~/.bashrc`）：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://10.0.11.2:11311   # 連到車子的 roscore
export ROS_IP=10.0.11.3                         # ← 改成你網卡的實際 IP
```

套用設定：

```bash
source ~/.bashrc
```

---

## 執行系統

> 需要 **3 個終端機**。先確認車子已開機且與電腦同網段（`ping 10.0.11.2` 通）。

### 終端機 1 — 啟動車子（SSH 進車子）

```bash
ssh wheeltec@10.0.11.2
roslaunch turn_on_wheeltec_robot mapping.launch
```

保持這個視窗開著（它就是 ROS Master / roscore）。

驗證電腦端有連上：在電腦另開終端機執行 `rostopic list`，應看到車子的一堆 topic。

### 終端機 2 — AprilTag 偵測節點（電腦端）

```bash
python3 Detect/new_detect/ros_detect_apriltag.py
```

會跳出 IR 影像視窗，對準標籤時應看到綠框與距離標示。按 `q` 離開。

### 終端機 3 — 移動控制節點（電腦端）

```bash
python3 Detect/new_detect/ros_move_map.py
```

機器人會開始旋轉找標籤 → 對齊 → 沿路線前進。

### 確認資料流（除錯用）

```bash
rostopic echo /target_info     # 看偵測輸出
rostopic echo /cmd_vel         # 看送給馬達的速度指令
```

<details>
<summary>舊架構：Old Detect（攝影機裝在車上）</summary>

需要先在車子上多啟動 usb_cam：

```bash
# 終端機 1（車子）
roslaunch turn_on_wheeltec_robot mapping.launch
roslaunch usb_cam usb_cam-test.launch

# 終端機 2（電腦）
python3 Detect/old_detect/ros_detect.py

# 終端機 3（電腦）
python3 Detect/old_detect/ros_move_follow_tag.py
```

old_detect 使用 `old_detect/calib_result.yaml` 校正檔（需先跑校正，見下方）。
</details>

---

## /target_info 訊息格式

偵測節點與控制節點之間用 `geometry_msgs/Pose` 傳遞資訊，欄位被重新定義如下：

| 欄位 | 意義 |
|---|---|
| `orientation.x` | 左 tag ID |
| `orientation.y` | 右 tag ID |
| `orientation.w` | `1.0` = 偵測到，`0.0` = 未偵測到 |
| `position.x` | 水平像素誤差（負 = 偏左，正 = 偏右）|
| `position.y` | `depth_diff` = 右tag.z − 左tag.z（公尺）|
| `position.z` | 到 tag pair 中心的 3D 直線距離（公尺）|

**`depth_diff` 怎麼用：**
- `> 0` → 右標籤較遠 → 機器人偏左 → 需右轉
- `< 0` → 左標籤較遠 → 機器人偏右 → 需左轉
- `≈ 0` → 正對標籤

---

## 參數調校

| 檔案 | 參數 | 預設 | 作用 |
|---|---|---|---|
| `new_detect/ros_detect_apriltag.py` | `TAG_SIZE_M` | `0.08` | **列印標籤的實際邊長（公尺），必須量準否則距離會錯** |
| `new_detect/ros_detect_apriltag.py` | `W` / `H` / `FPS` | `848/480/60` | IR 串流解析度與幀率 |
| `new_detect/ros_detect_apriltag.py` | `quad_decimate` | `1` | 偵測縮圖倍率：`1`=最遠最慢，`1.5`=較快較近 |
| `new_detect/ros_move_map.py` | `INIT_SEARCH_W` | `0.3` | 開機搜尋標籤時的旋轉速度 |
| `new_detect/ros_move_map.py` | `MAX_SPEED_V` | `0.2` | 前進線速度 |
| `new_detect/ros_move_map.py` | `TAG_TIMEOUT` | `10.0` | 連續看不到標籤幾秒後停車 |
| `old_detect/ros_move_follow_tag.py` | `TARGET_DIST` | `0.5` | 跟隨模式想保持的距離（公尺）|

> **拉遠偵測距離**：標籤越大越好（15–20cm 可到 3–4m）；把鏡頭往下傾 10–15°；
> `quad_decimate` 設 `1`（會降 FPS）。詳見開發章節。

---

## 如何開發與擴充

### 核心模組：AprilTag 配對演算法

兩個偵測節點都共用 `Detect/apriltag_setting/` 裡的演算法（透過 `sys.path` 加入該資料夾後 import）：

- **`PairDetector`**（`pair_detector_setting.py`）
  把畫面中的標籤兩兩配對，判斷左右順序、行進方向（going/returning）、穩定度（連續 N 幀一致才算穩定），並用 SVD 平均兩個標籤的旋轉矩陣得到合併 pose。

- **`BalancePairDetector`**（`pair_detector_balance.py`）
  繼承上者，額外提供每個標籤各自的平移向量 `t_left` / `t_right`，用來算 `depth_diff`。

### 編輯路線地圖

`Detect/new_detect/route_map.yaml` 定義每對標籤對應的地點與彼此的連通關係：

```yaml
nodes:
  - id: "entrance"        # 節點代號
    pair: [0, 1]          # 這個地點貼的兩個 tag ID（順序無關）
    name: "入口"           # 顯示名稱

edges:
  - from: "entrance"
    to: "corridor"
    going_direction: "going"   # 從 entrance→corridor 時 pair_detector 應看到的方向
```

加新地點：在 `nodes` 加一筆（配一對新的 tag ID），並在 `edges` 描述它和哪個節點相連。
`RouteMap` 已內建 BFS 最短路徑（`bfs_path()`），可供未來做「指定目的地導航」使用。

> 目前 `ros_move_map.py` 只用地圖**顯示目前位置**，尚未用 BFS 做目的地決策——這是預留的擴充點。

### 加入 YOLO 地板/路徑偵測

`ros_detect_dual.py` 已搭好雙串流骨架：IR 跑 AprilTag、Color 跑 YOLO segmentation，
並發布 `/floor_detected`（`std_msgs/Bool`）。要啟用：

1. 收集訓練影像（用 Color 串流，640×480，與推論解析度一致）
2. 用 [Roboflow](https://roboflow.com) 標註地板區域，匯出 YOLOv8-seg 格式
3. 在 Colab 訓練 `yolov8n-seg.pt`，下載 `best.pt`
4. 修改 `ros_detect_dual.py`：
   ```python
   YOLO_MODEL_PATH = "/path/to/best.pt"   # 換成你的模型
   FLOOR_CLASS_ID  = 0                      # 地板在你模型中的 class ID
   ```

模型檔不存在時節點仍可正常跑 AprilTag，只會關閉 YOLO 並印警告。

### 新增標籤注意事項

- 用 **tag36h11** 家族（程式寫死這個家族）
- 改了標籤實際尺寸，要同步更新各偵測節點的 `TAG_SIZE_M`
- 距離不準時，先確認 `TAG_SIZE_M` 跟尺一致

---

## 攝影機校正（僅 old_detect 需要）

RealSense（new_detect）使用相機**出廠內參**，不需校正。
只有舊的 USB 攝影機架構（old_detect）需要手動校正：

```bash
python3 calibration/capture_chessboard_images.py   # 按 s 儲存影像（建議 15~25 張，多角度），q 離開
python3 calibration/calibrate_from_images.py        # 自動輸出 Detect/old_detect/calib_result.yaml
```

---

## 疑難排解

| 症狀 | 可能原因與解法 |
|---|---|
| `rostopic list` 顯示 `Unable to communicate with master` | 車子沒開機 / 沒跑 roslaunch；或 `ROS_MASTER_URI`、`ROS_IP` 設錯。先 `ping 10.0.11.2` |
| 偵測視窗打不開 / 抓不到相機 | `realsense-viewer` 確認相機正常；USB 要插 3.0 孔；重插 |
| 距離數值明顯不對 | `TAG_SIZE_M` 沒設成標籤實際邊長 |
| 標籤太遠就偵測不到 | 換大標籤、鏡頭往下傾、`quad_decimate` 設 1 |
| FPS 太低 | 調高 `quad_decimate`（如 1.5）、降低解析度 |
| 機器人一直原地轉找不到標籤 | 標籤不在初始視野內；確認 `INIT_SEARCH_W` 與超時設定 |
| `No module named 'pupil_apriltags'` | `pip3 install -r requirements.txt`；numpy 需 ≥1.20 |
