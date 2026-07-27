# TODO — Aurora 自駕測試與重新建圖

> 站點定義：**A**＝起點 (compus1.2 座標 0,0)、**M1**＝本次測試目標點 (97,-13)、
> **M2**＝斜路中段 (213,29)、**B**＝東北端建築群 (339,106)。
> 相關工具在 `Detect/new_detect/route/`，路線檔在 `Detect/new_detect/route/routes_compus1_2/`。
> compus1.2 已知體質（分析結論）：去程圖層完整；回程圖層只建到 M2；M2→M1 無回程覆蓋；
> 方向圖層間有 ~3–4 m 偏移；開闊直路段（M1–M2 之間）追蹤最弱。
>
> **為什麼首測選 A↔M1**：這一段「兩個方向都有真實建圖覆蓋」——去程走 session 1 圖層、
> 回程走 session 2 實際開過的西向圖層，不需要任何逆向行駛，是風險最低的完整往返測試。

---

## 任務 1：用 compus1.2 測試 A→M1 自動行駛 + 掉頭回程

### Phase 0 — 行前準備（出門前）
- [ ] 把 `compus1.2.stcm` 傳到 Ubuntu PC：`~/maps/compus1.2.stcm`
- [ ] PC 上更新 repo，確認有 `Detect/new_detect/route/routes_compus1_2/plan_A_M1.csv` 和 `plan_M1_A.csv`
- [ ] 確認跟線參數（`ros_move_follow_route.py` 開頭）：`MAX_SPEED_V=0.2`、`MAX_LATERAL=1.5`
- [ ] 帶皮尺（Phase 2 量直線用）、筆記（記錄失效位置）

### Phase 1 — 現場靜態檢查（輪子不動）
- [ ] 車上（SSH wheeltec@10.0.11.2）：`roslaunch turn_on_wheeltec_robot mapping.launch`（供底盤 /cmd_vel）
- [ ] PC：`source /opt/ros/noetic/setup.bash`、`export ROS_MASTER_URI=http://10.0.11.2:11311`、`export ROS_IP=10.0.11.3`
- [ ] **關閉 Aurora Remote App**（單一 client），接上乙太網（192.168.11.1）
- [ ] `source ~/aurora_ros/devel/setup.bash && roslaunch ~/Go_by_my_self/Detect/new_detect/aurora/aurora_slam.launch`
- [ ] 載圖：`Detect/new_detect/aurora/aurora_map.sh load ~/maps/compus1.2.stcm`
- [ ] 車放 A 點附近、**車頭朝 B 的行進方向**，開 `aurora_status.py` 等重定位成功，**記下花幾秒**
  - ⚠ 若 3 分鐘內重定位不成功：很可能是地圖過期（4 月建圖，光照/植被已變）。
    記錄現象後直接跳到「任務 2 重新建圖」，今天測試改為建圖日。
- [ ] `rostopic hz /slamware_ros_sdk_server_node/robot_pose` 確認 ~15 Hz

### Phase 2 — 遙控直線量測（20 m，先不上自動控制）
- [ ] 開 rosbag（**要錄，目的是事後除錯**；輕量 topic 整趟錄都沒負擔）：
  ```bash
  rosbag record -O ~/aurora_bags/test_$(date +%m%d_%H%M).bag \
    /slamware_ros_sdk_server_node/robot_pose \
    /slamware_ros_sdk_server_node/system_status \
    /slamware_ros_sdk_server_node/relocalization_status \
    /cmd_vel /route_plan
  ```
- [ ] 遙控（或手推）沿去程方向直線走 **皮尺量好的 20.0 m**
- [ ] 比對 pose 位移 vs 20.0 m；再回到起點看閉合誤差
- [ ] **通過標準：誤差 < 0.5 m、`aurora_status` 全程 tracking、無跳點** → 才進 Phase 3

### Phase 3 — 自動行駛 A→M1（144.3 m，經轉角 (13,-40)）
- [ ] 確認 rosbag 還在錄
- [ ] 車回 A、車頭朝行進方向，執行：
  ```bash
  python3 ~/Go_by_my_self/Detect/new_detect/route/ros_move_follow_route.py \
    --plan ~/Go_by_my_self/Detect/new_detect/route/routes_compus1_2/plan_A_M1.csv
  ```
  （空白鍵＝急停/解除，q＝結束；橫向誤差 >1.5 m 或位姿逾時會自動停車）
- [ ] 人全程跟車。重點觀察：起步後的直角轉角 (13,-40)（pure pursuit 會稍微切彎）、
      下段走廊直線的橫向穩定度
- [ ] 到 M1 節點自動停車。記錄：最大橫向偏差、有沒有自動停車事件、在哪裡

### Phase 4 — 掉頭回程 M1→A（136.7 m，走 session 2 西向圖層）
- [ ] 在 M1 按 q 結束節點；遙控原地掉頭 180°
- [ ] `aurora_status` 確認仍在 tracking——這一步本身就是重要數據：
      session 2 建圖時就是在這個點朝西出發的，理論上朝西的地標視角存在
- [ ] 執行回程：
  ```bash
  python3 .../ros_move_follow_route.py --plan .../routes_compus1_2/plan_M1_A.csv
  ```
- [ ] 唯一預期風險：掉頭瞬間位姿從去程圖層切到 session 2 圖層，可能有 <1 m 的小跳動
      （session 2 當時有錨定，偏移中位僅 0.78 m）；若橫向超限自動停車，重啟節點接續即可
- [ ] 若整段順利回到 A：**這就證明了「雙向都有覆蓋的路段可以完整自動往返」**，
      重新建圖後全程 A↔B 就會是同樣的體驗

### Phase 5 — 收尾與分析
- [ ] Ctrl-C 停 rosbag
- [ ] `python3 Detect/new_detect/aurora/analyze_aurora_bag.py <bag>` 跑既有分析
- [ ] 記錄四個數字：重定位秒數／20 m 直線誤差／往返最大橫向偏差／掉頭時的位姿跳動量
- [ ] 把 bag 或數字丟給 Claude 做定量分析

### （選配）Phase 6 — 挑戰全程 A→B→A（454.8 m）
只在 Phase 1–4 全部順利、時間電量充足時做。已知風險寫在路線檔警告裡：
- `plan_A_B.csv`：去程全程有覆蓋，主要考驗開闊直路 M1–M2 弱區
- `plan_B_A.csv`：M2 處換圖層（落差 3.2 m，預期自動停車一次）；
  **M2→M1 是逆向段**（無回程建圖）——車行為異常立刻空白鍵，遙控通過到 M1 再接續

---

## 任務 2：重新建圖（目標：單一 session、雙向互相迴環的乾淨地圖）

### 準備
- [ ] **時段選得和之後自駕運行相近**（外觀特徵對光照敏感；也避開行人多的時間）
- [ ] 電量確保連續 ~40 分鐘；Aurora App 保持關閉
- [ ] （建議）建圖同時在 PC 錄 pose bag，事後可比對「即時位姿 vs 最佳化後地圖」

### 建圖流程（一次做完、不關 node）
1. [ ] 起點 A（特徵豐富處）靜止 5–10 秒，前後小幅移動讓 IMU 初始化
2. [ ] **A→B 第一趟**：低速平穩；開闊直路（舊圖弱區，約 compus2 的 (170,48) 一帶）放慢、避免急轉
3. [ ] 到 B：**慢速原地 360°**（15–20 秒轉一圈）——把去/回兩個朝向的視角縫起來
4. [ ] **B→A 第一趟**
5. [ ] 回到 A：**慢速原地 360°**
6. [ ] **A→B 第二趟**（同方向第二次經過 → 產生同向迴環，鎖緊去程層）
7. [ ] 到 B：再 360° 一圈
8. [ ] **B→A 第二趟**
9. [ ] 回 A、車頭朝出發方向、靜止幾秒 → `aurora_map.sh save ~/maps/compus4.stcm`
10. [ ] 途中若 `aurora_status` 顯示追蹤丟失：原地停住、緩慢左右擺動待恢復，再繼續

### 驗收（回來後做）
- [ ] `python3 Detect/new_detect/route/extract_route.py ~/maps/compus4.stcm --out routes_compus4 --png`
      → 預覽圖應是乾淨的兩圈、無碎段
- [ ] 給 Claude 跑「回程地標重用率＋方向層間隙」驗證：
      **目標：回程 KF 重用率明顯 >0%、兩方向軌跡間隙 < 1 m**（compus1.2 是 0% / 3.9 m）
- [ ] 用新圖重新產生站點與路線：`route_graph.py stations-init` → 改站名 → `plan A B`、`plan B A`
      → **`plan B A` 不需要 `--allow-reverse` 即成立**，才算建圖成功
- [ ] 重跑任務 1 的 Phase 1–4 驗證實駕

---

### 檔案對照
| 東西 | 位置 |
|---|---|
| 抽路徑工具 | `Detect/new_detect/route/extract_route.py` |
| 站點/規劃工具 | `Detect/new_detect/route/route_graph.py` |
| 跟線節點 | `Detect/new_detect/route/ros_move_follow_route.py` |
| 首測路線（A↔M1） | `Detect/new_detect/route/routes_compus1_2/plan_A_M1.csv`、`plan_M1_A.csv` |
| 全程路線（A↔B，選配） | `Detect/new_detect/route/routes_compus1_2/plan_A_B.csv`、`plan_B_A.csv` |
| 站點定義 | `Detect/new_detect/route/routes_compus1_2/stations.yaml` |
| Aurora 狀態/地圖工具 | `Detect/new_detect/aurora/aurora_status.py`、`aurora_map.sh` |
