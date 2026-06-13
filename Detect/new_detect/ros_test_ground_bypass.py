#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地面 Tag + 深度繞障 + 里程計路徑記憶 (Ground-Gate Path Memory & Bypass Test)

實作的想法（你的設計）：
  1. 看到地面 tag pair（門）且還夠遠 → 用 pixel + depth_diff 對準「開過去」
  2. 對準的瞬間（dd≈0 且置中）→ 把當下 /odom 的 yaw 記成「路徑角」，
     當下位置記成「路徑線上的點」→ 路徑線 = 過該點、沿該角的直線
  3. 車離 tag 太近（< GATE_FREEZE_DIST）→ 「凍結」tag 轉向：
     不再朝 tag 修正（近距離像素誤差會爆炸、害車急轉），
     改用記憶的路徑線跟隨，直接壓過 tag
  4. 看不到 tag（門與門之間）→ 持續用 /odom 跟隨記憶的路徑線
     （不是單純直走 — 偏了會自己修回線上）
  5. 前方走廊出現障礙 → 停車觀察：走掉→續走；會動→等；靜止且右側淨空
     → 把「橫向目標」設成右偏 DODGE_OFFSET，沿平移後的路徑線繞過，
     用 /odom 的沿路里程判斷通過障礙後，把橫向目標歸零 → 自動回到原路徑線
  6. 下一個 tag 門出現 → 重新對準、重新記錄路徑角（里程計漂移歸零）

需要：車端 roslaunch 有發布 /odom（Wheeltec 預設有）。
  收不到 /odom 時：tag 轉向照常，但「路徑記憶 / 繞障」停用（只會停車等）。

鍵盤（OpenCV 視窗）：[空白鍵]=開始/暫停   [q]=結束（啟動時不動）
本程式自帶相機（IR+Depth）— 單獨執行，不可與其他相機程式同時跑。
"""

import math
import os
import sys
import time

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "apriltag_setting"))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from pupil_apriltags import Detector

from pair_detector_balance import BalancePairDetector
from pair_detector_setting import draw_pair_labels, select_best_pair

# ================= 參數設定 =================
W, H, FPS  = 1280, 720, 30
IR_INDEX   = 1
TAG_SIZE_M = 0.11
FRAME_TIMEOUT_MS = 5000
PAIR_MAX_GAP_M   = 1.0     # 同組 pair 兩 tag 最大 3D 間距（濾跨地點假 pair）

ODOM_TOPIC = "/odom"       # 車端里程計（rostopic list 確認名稱）

# ---- 門（地面 pair）對準 ----
GATE_FREEZE_DIST = 1.0     # 距 pair < 此值 → 凍結 tag 轉向，改走記憶路徑線壓過去
KD_DRIVE   = 1.5           # depth_diff → 角速度（對準頭向）
KP_DRIVE   = 0.0006        # pixel_error → 角速度（對準門中心）
ALIGN_DD   = 0.02          # |depth_diff| < 此值 …
ALIGN_PX   = 40            # …且 |pixel_error| < 此值 → 視為對準，記錄路徑角

# ---- 路徑線跟隨（/odom）----
K_YAW        = 1.5         # 航向誤差 → 角速度增益（若實車方向相反請改負值）
K_LAT        = 1.2         # 橫向誤差 (m) → 期望航向修正 (rad) 增益
MAX_CORR_RAD = math.radians(35.0)   # 橫向修正最多偏離路徑角幾度
MAX_W        = 0.4
CRUISE_V     = 0.15
LAT_TOL      = 0.08        # 視為「已在線上」的橫向誤差 (m)

# ---- 障礙（3D 走廊，同 ros_test_bypass.py）----
OB_TRIGGER_M    = 1.2
OB_STOP_M       = 0.5
OB_MIN_M        = 0.15
OB_MAX_M        = 4.0
CAM_TILT_DEG    = 30.0     # ★鏡頭下傾角（度）— 沒補償的話地面會被當成障礙！
                           #   用 Depth Corridor 視窗調：空曠路面被白/橘標亮 → 調大；
                           #   膝蓋高的真障礙不亮 → 調小。盡量量實際安裝角填入。
CORRIDOR_HALF_W = 0.25
CORRIDOR_Y_UP   = 0.30
CORRIDOR_Y_DOWN = 0.15     # 補償後的垂直下限：必須小於鏡頭離地高（留 ~10cm 餘裕）
RIGHT_X_MIN     = 0.25
RIGHT_X_MAX     = 0.70     # ≈ DODGE_OFFSET + 半車寬。1.5m 車道別超過路緣，
                           # 否則路旁草地永遠讓「右側不淨空」、永遠不能繞
RIGHT_FREE_M    = 2.0
OB_MIN_PIXELS   = 150
DEPTH_STRIDE    = 2

# ---- 評估 / 繞障 ----
EVAL_SEC       = 1.0       # 停車觀察秒數
MOVING_EPS     = 0.15      # 觀察期間距離變化 > 此值 → 會動的障礙，不繞
GONE_MARGIN    = 0.3
REEVAL_SEC     = 3.0
DODGE_OFFSET   = 0.45      # 右偏多少 (m) — 不可超過車道右側餘裕
DODGE_V        = 0.12
OBSTACLE_LEN_M = 1.0       # 沿路通過長度 = 觸發時的障礙距離 + 此值（蓋過障礙+車長）

# ---- 光達後方檢查（可選；收不到 /scan 就跳過並警告）----
SCAN_TOPIC      = "/scan"
REAR_CENTER_DEG = 180.0    # 「正後方」角度 — 依光達安裝方向調整
REAR_SECTOR_DEG = 60.0
REAR_CLEAR_M    = 3.0

# ---- 搜尋 ----
INIT_SEARCH_W      = 0.3
SEARCH_FULL_CIRCLE = 6.2832 / INIT_SEARCH_W * 1.25
# ===========================================


def angnorm(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ir_intrinsics(profile, index):
    sp   = profile.get_stream(rs.stream.infrared, index)
    intr = sp.as_video_stream_profile().get_intrinsics()
    return (intr.fx, intr.fy, intr.ppx, intr.ppy)


def disable_ir_emitter(profile):
    try:
        depth_sensor = profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 0)
            rospy.loginfo("IR emitter 已關閉（室內深度品質會變差，室外白天正常）")
    except Exception as e:
        rospy.logwarn(f"關閉 emitter 失敗: {e}")


class DepthCorridor:
    """深度影像反投影成 3D 點，依真實座標篩選「會撞到的範圍」。

    相機座標：x 向右、y 向下、z 向前。tilt_deg = 鏡頭下傾角：
    點雲會先轉回「水平」座標再過濾 — 不補償的話，下傾的鏡頭會把
    前方 1～2m 的地面當成走廊內的障礙（永遠觸發停車）。"""

    def __init__(self, intr, stride=2, tilt_deg=0.0):
        self.stride = stride
        t = math.radians(tilt_deg)
        self._ct, self._st = math.cos(t), math.sin(t)
        us = (np.arange(0, intr.width,  stride, dtype=np.float32) - intr.ppx) / intr.fx
        vs = (np.arange(0, intr.height, stride, dtype=np.float32) - intr.ppy) / intr.fy
        self.xfac = us[None, :]
        self.yfac = vs[:, None]
        self.z = self.x = self.y = None

    def update(self, depth_m):
        zc = depth_m[::self.stride, ::self.stride]
        yc = self.yfac * zc
        self.x = self.xfac * zc
        self.y = yc * self._ct + zc * self._st   # 水平座標的垂直（下正）
        self.z = zc * self._ct - yc * self._st   # 水平距離

    def region(self, x_min, x_max, y_up, y_down, z_min, z_max, min_pts):
        m = ((self.z > z_min) & (self.z < z_max) &
             (self.x >= x_min) & (self.x <= x_max) &
             (self.y >= -y_up) & (self.y <= y_down))
        zs = self.z[m]
        if zs.size < min_pts:
            return None, m
        return float(np.percentile(zs, 5)), m


def colorize_depth(corridor, z_max, masks_colors):
    z   = corridor.z
    v8  = np.clip(z / z_max * 255.0, 0, 255).astype(np.uint8)
    vis = cv2.applyColorMap(v8, cv2.COLORMAP_JET)
    vis[z <= 0] = 0
    for mask, color in masks_colors:
        vis[mask] = (vis[mask] * 0.3 + np.array(color, dtype=np.float32) * 0.7).astype(np.uint8)
    return vis


class ScanWatcher:
    """訂閱 /scan，提供後方扇區最近距離。沒收到訊息時回傳 None。"""

    def __init__(self):
        self.msg = None
        rospy.Subscriber(SCAN_TOPIC, LaserScan, self._cb, queue_size=1)

    def _cb(self, msg):
        self.msg = msg

    def rear_min(self):
        m = self.msg
        if m is None:
            return None
        center = math.radians(REAR_CENTER_DEG)
        half   = math.radians(REAR_SECTOR_DEG) / 2.0
        best   = None
        ang = m.angle_min
        for r in m.ranges:
            d = (ang - center + math.pi) % (2 * math.pi) - math.pi
            if abs(d) <= half and m.range_min < r < m.range_max:
                best = r if best is None else min(best, r)
            ang += m.angle_increment
        return best


class OdomWatcher:
    """訂閱 /odom，提供位置 (x, y) 與 yaw。"""

    def __init__(self, topic):
        self.ok  = False
        self.x   = self.y = self.yaw = 0.0
        rospy.Subscriber(topic, Odometry, self._cb, queue_size=1, tcp_nodelay=True)

    def _cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.ok = True


def main():
    rospy.init_node("ground_bypass_test_node", anonymous=True)

    detector = Detector(
        families="tag36h11", nthreads=4,
        quad_decimate=1.0, quad_sigma=0.5,
        refine_edges=True, decode_sharpening=0.5,
    )
    pd      = BalancePairDetector(history_len=6, stable_threshold=4,
                                  max_pair_gap_m=PAIR_MAX_GAP_M)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    ob_pub  = rospy.Publisher("/obstacle_ahead", Float32, queue_size=1)
    scan    = ScanWatcher()
    odom    = OdomWatcher(ODOM_TOPIC)

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.infrared, IR_INDEX, W, H, rs.format.y8,  FPS)
    config.enable_stream(rs.stream.depth,              W, H, rs.format.z16, FPS)
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        rospy.logerr(f"無法開啟 RealSense 相機：{e}")
        rospy.logerr("D435i 一次只能被『一個』程式開啟。本程式自帶相機+控制，"
                     "先關閉其他相機程式（ros_detect_*.py / ros_test_*.py / "
                     "collect_floor_dataset.py / realsense-viewer）再執行。")
        return
    disable_ir_emitter(profile)
    camera_params = ir_intrinsics(profile, IR_INDEX)
    depth_scale   = profile.get_device().first_depth_sensor().get_depth_scale()
    depth_intr    = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    corridor      = DepthCorridor(depth_intr, stride=DEPTH_STRIDE, tilt_deg=CAM_TILT_DEG)

    rospy.loginfo(f"IR+Depth 串流已開啟：{W}x{H} @ {FPS}fps")
    rospy.loginfo("⌨️  OpenCV 視窗：[空白鍵]=開始/暫停   [q]=結束（啟動時不動）")

    rate          = rospy.Rate(FPS)
    run_enabled   = False
    state         = "SEARCH"   # SEARCH / DRIVE / EVAL / WAIT / DODGE / DEAD
    last_pair_key = None
    search_start  = None
    odom_warned   = False

    # 路徑記憶（odom 座標系）
    path_yaw = None            # 路徑角（對準門時的 odom yaw）
    path_x = path_y = 0.0      # 路徑線上的一點
    lat_target = 0.0           # 橫向目標：0=在線上，-DODGE_OFFSET=右偏繞障

    # 評估 / 繞障
    eval_start = eval_first = None
    wait_since = None
    dodge_phase  = None        # "out" → "pass" → "back"
    pass_until_s = 0.0

    def line_metrics():
        """回傳 (lat, s)：相對路徑線的橫向偏移（左+）與沿線里程。"""
        if path_yaw is None or not odom.ok:
            return 0.0, 0.0
        dx, dy = odom.x - path_x, odom.y - path_y
        d  = (math.cos(path_yaw), math.sin(path_yaw))
        n  = (-math.sin(path_yaw), math.cos(path_yaw))   # 左法向
        return dx * n[0] + dy * n[1], dx * d[0] + dy * d[1]

    def line_follow_w(lat):
        """沿（可能被 lat_target 平移的）路徑線跟隨的角速度。"""
        err  = lat - lat_target
        corr = max(min(K_LAT * err, MAX_CORR_RAD), -MAX_CORR_RAD)
        desired = path_yaw - corr        # 偏左(err>0) → 期望航向偏右
        return max(min(K_YAW * angnorm(desired - odom.yaw), MAX_W), -MAX_W)

    try:
        while not rospy.is_shutdown():
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                rospy.logwarn_throttle(2.0, f"等待影像逾時: {e}")
                rate.sleep()
                continue

            ir_frame    = frames.get_infrared_frame(IR_INDEX)
            depth_frame = frames.get_depth_frame()
            if not ir_frame:
                rate.sleep()
                continue
            now  = time.time()
            gray = np.asanyarray(ir_frame.get_data())

            # ---------- AprilTag 門 ----------
            results = detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=camera_params, tag_size=TAG_SIZE_M)
            pairs = pd.update_and_detect(results)
            p     = select_best_pair(pairs, prefer_key=last_pair_key)
            last_pair_key = p["key"] if p is not None else None

            tag_visible = False
            dd = px = dist = 0.0
            if p is not None and p.get("t") is not None:
                tl, tr = p.get("t_left"), p.get("t_right")
                if tl is not None and tr is not None:
                    tag_visible = True
                    dd   = float(tr[2] - tl[2])
                    dist = float(np.linalg.norm(p["t"]))
                    px   = float(p["center"][0]) - W / 2.0

            # ---------- Depth 3D 走廊 ----------
            ob_front = ob_right = None
            depth_vis = None
            if depth_frame:
                depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
                corridor.update(depth_m)
                ob_front, front_mask = corridor.region(
                    -CORRIDOR_HALF_W, CORRIDOR_HALF_W, CORRIDOR_Y_UP, CORRIDOR_Y_DOWN,
                    OB_MIN_M, OB_MAX_M, OB_MIN_PIXELS)
                ob_right, right_mask = corridor.region(
                    RIGHT_X_MIN, RIGHT_X_MAX, CORRIDOR_Y_UP, CORRIDOR_Y_DOWN,
                    OB_MIN_M, OB_MAX_M, OB_MIN_PIXELS)
                depth_vis = colorize_depth(corridor, OB_MAX_M,
                                           [(front_mask, (255, 255, 255)),
                                            (right_mask, (255, 200, 0))])
            ob_pub.publish(Float32(data=ob_front if ob_front is not None else -1.0))

            front_blocked = ob_front is not None and ob_front < OB_STOP_M
            front_trigger = ob_front is not None and ob_front < OB_TRIGGER_M
            front_clear   = ob_front is None or ob_front > OB_TRIGGER_M + GONE_MARGIN

            lat, s_along = line_metrics()
            aligned = False

            # =================== 狀態機 ===================
            twist = Twist()
            if run_enabled:

                if not odom.ok and not odom_warned:
                    rospy.logwarn(f"收不到 {ODOM_TOPIC} — 路徑記憶/繞障停用，只剩 tag 轉向與停車等待")
                    odom_warned = True

                # ── SEARCH：旋轉找第一個門 ──
                if state == "SEARCH":
                    if tag_visible:
                        rospy.loginfo("🎯 找到 Tag 門，開始跟隨...")
                        state = "DRIVE"
                    else:
                        if search_start is None:
                            search_start = now
                        if now - search_start > SEARCH_FULL_CIRCLE:
                            rospy.logerr("❌ 旋轉一圈仍找不到 Tag，停止（空白鍵暫停後再開可重試）。")
                            state = "DEAD"
                        else:
                            twist.angular.z = INIT_SEARCH_W

                # ── DRIVE：門對準（遠）/ 凍結+路徑線跟隨（近或無 tag）──
                elif state == "DRIVE":
                    if front_trigger:
                        rospy.loginfo(f"🚧 前方 {ob_front:.2f}m 有障礙，停車觀察 {EVAL_SEC:.0f}s...")
                        eval_start, eval_first = now, ob_front
                        state = "EVAL"
                    elif tag_visible and dist > GATE_FREEZE_DIST:
                        # 遠：朝門開（置中 + 轉正）
                        w = -(KD_DRIVE * dd + KP_DRIVE * px)
                        twist.linear.x  = CRUISE_V
                        twist.angular.z = max(min(w, MAX_W), -MAX_W)
                        # 對準瞬間 → 記錄路徑角與線上點（持續刷新，門過了就停止更新）
                        if abs(dd) < ALIGN_DD and abs(px) < ALIGN_PX and odom.ok:
                            if path_yaw is None:
                                rospy.loginfo(f"📐 首次記錄路徑角 {math.degrees(odom.yaw):+.1f}°")
                            path_yaw, path_x, path_y = odom.yaw, odom.x, odom.y
                            aligned = True
                        rospy.loginfo_throttle(
                            1.0, f"  [GATE] dist={dist:.2f}m dDep={dd:+.3f} px={px:+.0f}"
                                 f"{'  ✓aligned' if aligned else ''}")
                    else:
                        # 近距離凍結 / 沒 tag：沿記憶路徑線走（壓過 tag、跨越盲區）
                        twist.linear.x = CRUISE_V
                        if path_yaw is not None and odom.ok:
                            twist.angular.z = line_follow_w(lat)
                            rospy.loginfo_throttle(
                                1.0, f"  [LINE] lat={lat:+.2f}m yawerr="
                                     f"{math.degrees(angnorm(path_yaw - odom.yaw)):+.1f}deg")
                        else:
                            twist.angular.z = 0.0   # 無路徑記憶 → 只能直走
                            rospy.loginfo_throttle(2.0, "  [BLIND] 尚無路徑記憶，直走")

                # ── EVAL：停車觀察障礙 ──
                elif state == "EVAL":
                    if now - eval_start >= EVAL_SEC:
                        rear       = scan.rear_min()
                        right_free = ob_right is None or ob_right >= RIGHT_FREE_M
                        rear_ok    = rear is None or rear >= REAR_CLEAR_M
                        can_dodge  = odom.ok and path_yaw is not None

                        if front_clear:
                            rospy.loginfo("✅ 障礙已離開，繼續行駛。")
                            state = "DRIVE"
                        elif ob_front is not None and eval_first is not None \
                                and abs(ob_front - eval_first) > MOVING_EPS:
                            rospy.loginfo("🚶 障礙在移動（可能是行人）→ 停車等待，不繞。")
                            wait_since, state = now, "WAIT"
                        elif right_free and rear_ok and can_dodge:
                            rospy.loginfo(f"↪️  靜止障礙，右側淨空 → 右偏 {DODGE_OFFSET:.2f}m 繞行"
                                          f"（odom 閉迴路，通過後自動回線）")
                            lat_target   = -DODGE_OFFSET          # 右 = 負（左法向為正）
                            dodge_phase  = "out"
                            pass_until_s = s_along + (ob_front or OB_TRIGGER_M) + OBSTACLE_LEN_M
                            state = "DODGE"
                        else:
                            why = ("無 odom/路徑記憶" if not can_dodge else
                                   ("右側不淨空" if not right_free else f"後方有來車 ({rear:.1f}m)"))
                            rospy.logwarn(f"⛔ 無法繞行（{why}）→ 停車等待。")
                            wait_since, state = now, "WAIT"

                # ── WAIT：停車等，定期重新評估 ──
                elif state == "WAIT":
                    if front_clear:
                        rospy.loginfo("✅ 前方淨空，繼續行駛。")
                        state = "DRIVE"
                    elif now - wait_since >= REEVAL_SEC:
                        eval_start, eval_first = now, ob_front
                        state = "EVAL"

                # ── DODGE：沿「平移後的路徑線」繞過，odom 判斷通過後回原線 ──
                elif state == "DODGE":
                    if front_blocked:
                        rospy.logwarn_throttle(1.0, f"⏸ 繞行凍結：前方 {ob_front:.2f}m 太近...")
                    else:
                        twist.linear.x  = DODGE_V
                        twist.angular.z = line_follow_w(lat)
                        if dodge_phase == "out" and abs(lat - lat_target) < LAT_TOL:
                            rospy.loginfo(f"↪️  已右偏到位（lat={lat:+.2f}m），平行通過障礙...")
                            dodge_phase = "pass"
                        elif dodge_phase == "pass" and s_along >= pass_until_s:
                            rospy.loginfo("⤴️  已通過障礙（沿路里程達標），返回原路徑線...")
                            lat_target  = 0.0
                            dodge_phase = "back"
                        elif dodge_phase == "back" and abs(lat) < LAT_TOL:
                            rospy.loginfo(f"✅ 回到原路徑線（lat={lat:+.2f}m），繼續行駛。")
                            dodge_phase = None
                            state = "DRIVE"

                # ── DEAD：停住 ──
                elif state == "DEAD":
                    pass

            cmd_pub.publish(twist)

            # ---------- 視覺化 ----------
            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            for r in results:
                corners = r.corners.astype(int)
                for k in range(4):
                    cv2.line(vis, tuple(corners[k]), tuple(corners[(k + 1) % 4]), (0, 255, 0), 2)
            draw_pair_labels(vis, pairs)

            status = state if run_enabled else "PAUSED (press SPACE)"
            cv2.putText(vis, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if run_enabled else (0, 0, 255), 2, cv2.LINE_AA)
            if tag_visible:
                cv2.putText(vis, f"GATE dist={dist:.2f}m dDep={dd:+.3f} px={px:+.0f}"
                                 f"{'  FREEZE' if dist <= GATE_FREEZE_DIST else ''}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2, cv2.LINE_AA)
            if path_yaw is not None and odom.ok:
                cv2.putText(vis, f"PATH yaw={math.degrees(path_yaw):+.1f}deg  lat={lat:+.2f}m"
                                 f"  target={lat_target:+.2f}m",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(vis, f"PATH: none  odom={'OK' if odom.ok else 'NO DATA'}",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            f_txt = f"front={ob_front:.2f}m" if ob_front is not None else "front: no data"
            r_txt = f"right={ob_right:.2f}m" if ob_right is not None else "right: no data(free)"
            cv2.putText(vis, f"{f_txt}  {r_txt}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255) if front_trigger else (255, 200, 0), 2, cv2.LINE_AA)

            cv2.imshow("Ground Gate + Bypass Test (IR + Depth + Odom)", cv2.resize(vis, (854, 480)))
            if depth_vis is not None:
                cv2.putText(depth_vis, f"{f_txt}  (white=front, orange=right)", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("Depth Corridor", depth_vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                run_enabled = not run_enabled
                if not run_enabled:
                    cmd_pub.publish(Twist())
                    lat_target, dodge_phase = 0.0, None     # 暫停同時重置繞行
                    state = "DRIVE" if path_yaw is not None else "SEARCH"
                    search_start = None
                rospy.loginfo(f"{'▶️  開始行駛' if run_enabled else '⏸  暫停（已送停車指令、重置繞行）'}")

            rate.sleep()
    finally:
        stop = Twist()
        for _ in range(5):
            cmd_pub.publish(stop)
            time.sleep(0.02)
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
