#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
障礙繞行測試節點 (Obstacle Bypass Test) — 側掛 Tag 平行駕駛 + S 形右繞

路側 pair 平行駕駛，並加上「繞過靜止障礙」：
  1. 沿路行駛（路側 pair 的 depth_diff 保持與道路平行；沒 tag 就直走）
  2. 前方走廊出現障礙 < OB_TRIGGER_M → 停車觀察 EVAL_SEC 秒：
       - 障礙自己走掉（行人）→ 繼續走
       - 距離在變（會動的東西）→ 不繞，停車等（WAIT）
       - 靜止 → 檢查「右側深度 ROI 是否淨空」+「光達後方是否無來車」
  3. 都通過 → S 形右繞（開迴路分段）：
       右轉 θ → 斜走 → 回正（此時已右偏 DODGE_OFFSET）→ 平行通過 PASS_DIST
       → 左轉 θ → 斜走 → 回正 → RECAPTURE：用繞行前記下的 pair 橫向位置
       (t.x) 把殘餘橫向誤差修掉，真正回到「原平行線」→ 繼續沿路走
  4. 繞行全程持續監看前方走廊：< OB_STOP_M 立即凍結（計時暫停），淨空才續走

「不出路面」的保證（現階段）：DODGE_OFFSET 上限由幾何決定 —
  1.5m 車道、車寬 ~0.35m、靠中行駛 → 右側餘裕約 0.55m，預設 0.45m。
  之後 YOLO 路面模型訓練好，應改用 mask 邊界做即時檢查（見 ros_detect_dual.py）。

光達後方檢查（防後方自行車）：訂閱 /scan，後方扇區最近點 < REAR_CLEAR_M
  就不繞。收不到 /scan 時印警告並跳過此檢查 — 正式上路前務必確認光達方向
  （REAR_CENTER_DEG 依光達安裝方向調整）。

鍵盤（OpenCV 視窗）：[空白鍵]=開始/暫停   [q]=結束（啟動時不動）
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

# 同一組 pair 兩 tag 的最大 3D 間距 (m)：大於板上兩 tag 間距、小於相鄰地點距離，
# 用來濾掉多地點同時入鏡時的跨地點假 pair
PAIR_MAX_GAP_M = 0.8

# 平行駕駛
CRUISE_V           = 0.15
KD_PARALLEL        = 2.0
MAX_W              = 0.3
MEMORY_W_DECAY_SEC = 1.0
LOST_STOP_SEC      = 5.0     # 連續沒 tag 也沒在繞行 → 停車
DD_TRIM_M          = 0.0     # depth_diff 安裝偏差校正：把車手動擺到與道路平行，讀 dDep 值填入

# 繞行後回到「原平行線」：以繞行前 pair 的橫向位置 (t.x) 當參考，
# S 形幾何回線後再用它把殘餘橫向誤差修掉
K_LAT             = 0.8      # 橫向誤差 → 角速度增益
RECAPTURE_TOL_M   = 0.08     # 橫向殘餘誤差容許值 (m)
RECAPTURE_TIMEOUT = 5.0      # 最多修幾秒；pair 不見就接受幾何回線結果

# 前方走廊（觸發/煞停）— 3D 走廊：深度像素反投影成真實座標再篩選，
# 比固定矩形 ROI 準（矩形會把路邊/地面掃進來）
OB_TRIGGER_M    = 1.2        # 前方 < 此值 → 停車進入評估
OB_STOP_M       = 0.5        # 繞行/行進中 < 此值 → 立即凍結
OB_MIN_M        = 0.15
OB_MAX_M        = 4.0
CAM_TILT_DEG    = 30.0       # ★鏡頭下傾角（度）— 沒補償的話地面會被當成障礙！
CORRIDOR_HALF_W = 0.25       # 前方走廊半寬 = 車寬/2 + 餘裕 (m)
CORRIDOR_Y_UP   = 0.30       # 走廊上緣：相機上方幾公尺內 (m)
CORRIDOR_Y_DOWN = 0.15       # 走廊下緣：相機下方幾公尺內 (m)，必須小於鏡頭離地高，
                             # 否則地面變障礙；鏡頭下傾時再調小
RIGHT_X_MIN     = 0.25       # 右側借道檢查的橫向範圍（相機 x 向右，m）
RIGHT_X_MAX     = 0.70       # ≈ DODGE_OFFSET + 半車寬。1.5m 車道別超過路緣，
                             # 否則路旁草地永遠讓「右側不淨空」
RIGHT_FREE_M    = 2.0        # 右側區 5% 分位數 ≥ 此值 → 視為可借道
OB_MIN_PIXELS   = 150        # 區內有效點少於此數 → 視為無資料
DEPTH_STRIDE    = 2          # 深度圖降採樣倍率（省 CPU）

# 評估（停車觀察）
EVAL_SEC    = 1.0            # 停車觀察秒數
MOVING_EPS  = 0.15           # 觀察期間距離變化 > 此值 → 會動的障礙，不繞
GONE_MARGIN = 0.3            # 障礙距離 > TRIGGER+此值 → 視為已離開
REEVAL_SEC  = 3.0            # WAIT 狀態每隔幾秒重新評估一次

# S 形繞行幾何（開迴路；之後可改用 /odom 閉迴路更準）
DODGE_OFFSET = 0.45          # 右偏多少 (m) — 不可超過車道右側餘裕！
DODGE_ANGLE  = math.radians(35.0)
DODGE_TURN_W = 0.4           # 繞行轉向角速度 (rad/s)
DODGE_V      = 0.12          # 繞行直行速度 (m/s)
PASS_DIST    = 1.2           # 右偏後平行通過的距離 (m)，蓋過障礙長度 + 餘量

# 光達後方檢查（可選；收不到 /scan 就跳過並警告）
SCAN_TOPIC      = "/scan"
REAR_CENTER_DEG = 180.0      # 「正後方」在光達座標的角度 — 依實際安裝方向調整！
REAR_SECTOR_DEG = 60.0       # 後方扇區寬度
REAR_CLEAR_M    = 3.0        # 扇區內最近點 < 此值 → 不繞（可能有來車）

CAM_HEIGHT      = 0.25       # 路線預覽線的地面高度
ROUTE_PREVIEW_M = 3.0
# ===========================================


def ir_intrinsics(profile, index):
    sp   = profile.get_stream(rs.stream.infrared, index)
    intr = sp.as_video_stream_profile().get_intrinsics()
    camera_params = (intr.fx, intr.fy, intr.ppx, intr.ppy)
    K = np.array([
        [intr.fx, 0.0,     intr.ppx],
        [0.0,     intr.fy, intr.ppy],
        [0.0,     0.0,     1.0     ],
    ], dtype=np.float32)
    return camera_params, K


def disable_ir_emitter(profile):
    try:
        depth_sensor = profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 0)
            rospy.loginfo("IR emitter 已關閉（室內深度品質會變差，室外白天正常）")
    except Exception as e:
        rospy.logwarn(f"關閉 emitter 失敗: {e}")


class DepthCorridor:
    """把深度影像反投影成 3D 點，依真實座標篩選「會撞到的範圍」。

    相機座標：x 向右、y 向下、z 向前。
    假設鏡頭水平安裝；有下傾請把 y_down 調小。
    """

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
        self.y = yc * self._ct + zc * self._st   # 下傾補償後的垂直（下正）
        self.z = zc * self._ct - yc * self._st   # 水平距離

    def region(self, x_min, x_max, y_up, y_down, z_min, z_max, min_pts):
        """回傳 (5% 分位距離 或 None, 篩選遮罩)。y_up/y_down = 相機上/下方公尺數。"""
        m = ((self.z > z_min) & (self.z < z_max) &
             (self.x >= x_min) & (self.x <= x_max) &
             (self.y >= -y_up) & (self.y <= y_down))
        zs = self.z[m]
        if zs.size < min_pts:
            return None, m
        return float(np.percentile(zs, 5)), m


def colorize_depth(corridor, z_max, masks_colors):
    """深度 → 彩色圖（近紅遠藍，黑=無資料），並把檢查區內的點標色。"""
    z   = corridor.z
    v8  = np.clip(z / z_max * 255.0, 0, 255).astype(np.uint8)
    vis = cv2.applyColorMap(v8, cv2.COLORMAP_JET)
    vis[z <= 0] = 0
    for mask, color in masks_colors:
        vis[mask] = (vis[mask] * 0.3 + np.array(color, dtype=np.float32) * 0.7).astype(np.uint8)
    return vis


def road_direction_from_pair(t_left, t_right):
    """板面向量在水平面旋轉 90° = 道路方向（取朝前 z>0 的一邊）。"""
    bx = float(t_right[0] - t_left[0])
    bz = float(t_right[2] - t_left[2])
    for dx, dz in ((bz, -bx), (-bz, bx)):
        if dz > 0:
            n = math.hypot(dx, dz)
            if n > 1e-6:
                return (dx / n, dz / n)
    return None


def draw_route_line(img, K, road_dir, color=(0, 255, 0)):
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    pts = []
    for s in np.linspace(0.4, ROUTE_PREVIEW_M, 14):
        x = road_dir[0] * s
        z = road_dir[1] * s
        if z <= 0.05:
            continue
        pts.append((int(fx * x / z + cx), int(fy * CAM_HEIGHT / z + cy)))
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, color, 3, cv2.LINE_AA)


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
            # 角度差正規化到 [-pi, pi]
            d = (ang - center + math.pi) % (2 * math.pi) - math.pi
            if abs(d) <= half and m.range_min < r < m.range_max:
                best = r if best is None else min(best, r)
            ang += m.angle_increment
        return best


def main():
    rospy.init_node("obstacle_bypass_test_node", anonymous=True)

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

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.infrared, IR_INDEX, W, H, rs.format.y8,  FPS)
    config.enable_stream(rs.stream.depth,              W, H, rs.format.z16, FPS)
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        rospy.logerr(f"無法開啟 RealSense 相機：{e}")
        rospy.logerr("D435i 一次只能被『一個』程式開啟。本程式自帶相機+控制，"
                     "請『取代』而非『搭配』ros_detect_apriltag.py 執行 — "
                     "先關閉其他相機程式（ros_detect_*.py / ros_test_*.py / "
                     "collect_floor_dataset.py / realsense-viewer）。")
        return
    disable_ir_emitter(profile)
    camera_params, K_ir = ir_intrinsics(profile, IR_INDEX)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    depth_intr  = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    corridor    = DepthCorridor(depth_intr, stride=DEPTH_STRIDE, tilt_deg=CAM_TILT_DEG)

    rospy.loginfo(f"IR+Depth 串流已開啟：{W}x{H} @ {FPS}fps")
    rospy.loginfo("⌨️  OpenCV 視窗：[空白鍵]=開始/暫停   [q]=結束（啟動時不動）")

    # S 形繞行的分段序列：(v, w, 秒數)
    t_turn = DODGE_ANGLE / DODGE_TURN_W
    t_diag = (DODGE_OFFSET / math.sin(DODGE_ANGLE)) / DODGE_V
    t_pass = PASS_DIST / DODGE_V
    SEG_OUT = [   # 右轉→斜走→回正（右偏 DODGE_OFFSET）
        (0.0,     -DODGE_TURN_W, t_turn),
        (DODGE_V,  0.0,          t_diag),
        (0.0,      DODGE_TURN_W, t_turn),
    ]
    SEG_PASS = [  # 平行通過障礙
        (DODGE_V,  0.0,          t_pass),
    ]
    SEG_BACK = [  # 左轉→斜走→回正（回到原線）
        (0.0,      DODGE_TURN_W, t_turn),
        (DODGE_V,  0.0,          t_diag),
        (0.0,     -DODGE_TURN_W, t_turn),
    ]

    rate        = rospy.Rate(FPS)
    run_enabled = False
    state       = "DRIVE"     # DRIVE / EVAL / WAIT / DODGE
    last_w      = 0.0
    lost_since  = None
    prev_t      = time.time()
    last_pair_key = None      # 黏滯選擇：鎖定上一幀的 pair，避免逐幀跳換
    ref_lat       = None      # 繞行前 pair 的橫向位置 (t.x) — 回線參考
    ref_key       = None
    recapture_until = 0.0

    # EVAL / WAIT / DODGE 狀態變數
    eval_start = eval_first_dist = None
    wait_since = None
    segments   = []           # 進行中的繞行分段（list of [v, w, 剩餘秒數]）
    scan_warned = False

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

            now = time.time()
            dt  = min(now - prev_t, 0.2)   # 上限防止視窗卡住時暴衝
            prev_t = now

            gray = np.asanyarray(ir_frame.get_data())

            # ---------- AprilTag → 平行航向 ----------
            results = detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=camera_params, tag_size=TAG_SIZE_M,
            )
            pairs = pd.update_and_detect(results)
            p     = select_best_pair(pairs, prefer_key=last_pair_key)
            last_pair_key = p["key"] if p is not None else None

            depth_diff = None
            road_dir   = None
            if p is not None:
                tl, tr = p.get("t_left"), p.get("t_right")
                if tl is not None and tr is not None:
                    depth_diff = float(tr[2] - tl[2]) - DD_TRIM_M
                    road_dir   = road_direction_from_pair(tl, tr)

            # ---------- Depth 3D 走廊 ----------
            ob_front = ob_right = None
            depth_vis = None
            if depth_frame:
                depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
                corridor.update(depth_m)
                ob_front, front_mask = corridor.region(
                    -CORRIDOR_HALF_W, CORRIDOR_HALF_W,
                    CORRIDOR_Y_UP, CORRIDOR_Y_DOWN,
                    OB_MIN_M, OB_MAX_M, OB_MIN_PIXELS)
                ob_right, right_mask = corridor.region(
                    RIGHT_X_MIN, RIGHT_X_MAX,
                    CORRIDOR_Y_UP, CORRIDOR_Y_DOWN,
                    OB_MIN_M, OB_MAX_M, OB_MIN_PIXELS)
                depth_vis = colorize_depth(corridor, OB_MAX_M,
                                           [(front_mask, (255, 255, 255)),
                                            (right_mask, (255, 200, 0))])
            ob_pub.publish(Float32(data=ob_front if ob_front is not None else -1.0))

            front_blocked = ob_front is not None and ob_front < OB_STOP_M
            front_trigger = ob_front is not None and ob_front < OB_TRIGGER_M
            front_clear   = ob_front is None or ob_front > OB_TRIGGER_M + GONE_MARGIN

            # =================== 狀態機 ===================
            twist = Twist()
            if run_enabled:

                # ── DRIVE：沿路走（tag 平行 / 記憶衰減 / 直走）──
                if state == "DRIVE":
                    if front_trigger:
                        rospy.loginfo(f"🚧 前方 {ob_front:.2f}m 有障礙，停車觀察 {EVAL_SEC:.0f}s...")
                        eval_start, eval_first_dist = now, ob_front
                        state = "EVAL"
                    else:
                        if depth_diff is not None:
                            lost_since = None
                            last_w = max(min(-KD_PARALLEL * depth_diff, MAX_W), -MAX_W)
                            twist.linear.x, twist.angular.z = CRUISE_V, last_w
                        else:
                            if lost_since is None:
                                lost_since = now
                            elapsed = now - lost_since
                            if elapsed >= LOST_STOP_SEC:
                                pass   # 停車（零速）
                            else:
                                twist.linear.x = CRUISE_V
                                decay = max(1.0 - elapsed / MEMORY_W_DECAY_SEC, 0.0)
                                twist.angular.z = last_w * decay

                # ── EVAL：停車觀察 — 走掉？會動？靜止可繞？ ──
                elif state == "EVAL":
                    if now - eval_start >= EVAL_SEC:
                        rear = scan.rear_min()
                        if scan.msg is None and not scan_warned:
                            rospy.logwarn(f"收不到 {SCAN_TOPIC} — 跳過後方來車檢查（測試環境務必淨空後方！）")
                            scan_warned = True
                        right_free = ob_right is None or ob_right >= RIGHT_FREE_M
                        rear_ok    = rear is None or rear >= REAR_CLEAR_M

                        if front_clear:
                            rospy.loginfo("✅ 障礙已離開，繼續行駛。")
                            state = "DRIVE"
                        elif ob_front is not None and abs(ob_front - eval_first_dist) > MOVING_EPS:
                            rospy.loginfo("🚶 障礙在移動（可能是行人）→ 停車等待，不繞。")
                            wait_since, state = now, "WAIT"
                        elif right_free and rear_ok:
                            rospy.loginfo(f"↪️  靜止障礙，右側淨空"
                                          f"{'' if rear is None else f'、後方 {rear:.1f}m 無來車'} → 開始 S 形右繞")
                            # 記下目前 pair 的橫向位置，繞完用它修回原平行線
                            if p is not None and p.get("t") is not None:
                                ref_lat, ref_key = float(p["t"][0]), p["key"]
                            else:
                                ref_lat = ref_key = None
                            segments = [list(s) for s in (SEG_OUT + SEG_PASS + SEG_BACK)]
                            state = "DODGE"
                        else:
                            why = "右側不淨空" if not right_free else f"後方有來車 ({rear:.1f}m)"
                            rospy.logwarn(f"⛔ 無法繞行（{why}）→ 停車等待。")
                            wait_since, state = now, "WAIT"

                # ── WAIT：停車等，定期重新評估 ──
                elif state == "WAIT":
                    if front_clear:
                        rospy.loginfo("✅ 前方淨空，繼續行駛。")
                        state = "DRIVE"
                    elif now - wait_since >= REEVAL_SEC:
                        eval_start, eval_first_dist = now, ob_front
                        state = "EVAL"

                # ── DODGE：依分段序列執行 S 形繞行；前方太近就凍結 ──
                elif state == "DODGE":
                    if front_blocked:
                        rospy.logwarn_throttle(1.0, f"⏸ 繞行凍結：前方 {ob_front:.2f}m 太近，等待淨空...")
                        # 凍結：零速、分段計時不前進
                    elif segments:
                        v, w, remain = segments[0]
                        twist.linear.x, twist.angular.z = v, w
                        segments[0][2] = remain - dt
                        if segments[0][2] <= 0:
                            segments.pop(0)
                    else:
                        rospy.loginfo("S 形幾何完成，用 pair 橫向位置修回原平行線...")
                        lost_since = None
                        recapture_until = now + RECAPTURE_TIMEOUT
                        state = "RECAPTURE"

                # ── RECAPTURE：繞行後用同一組 pair 的橫向位置修回原線 ──
                elif state == "RECAPTURE":
                    lat_now = None
                    if p is not None and p.get("key") == ref_key and p.get("t") is not None:
                        lat_now = float(p["t"][0])
                    if front_blocked:
                        rospy.logwarn_throttle(1.0, f"⏸ 回線中前方 {ob_front:.2f}m 太近，暫停...")
                    elif ref_lat is None or lat_now is None:
                        rospy.loginfo("✅ 繞行完成（無 pair 參考，採 S 形幾何回線），繼續行駛。")
                        state = "DRIVE"
                    else:
                        lat_err = lat_now - ref_lat
                        if abs(lat_err) < RECAPTURE_TOL_M or now >= recapture_until:
                            rospy.loginfo(f"✅ 已回到原平行線（殘餘 {lat_err:+.2f}m），繼續行駛。")
                            state = "DRIVE"
                        else:
                            # 車偏左 → 板子橫向變遠 (lat_err>0) → 向右修；偏右反之
                            dd = depth_diff if depth_diff is not None else 0.0
                            w  = -(KD_PARALLEL * dd + K_LAT * lat_err)
                            twist.linear.x  = DODGE_V
                            twist.angular.z = max(min(w, MAX_W), -MAX_W)

            cmd_pub.publish(twist)

            # ---------- 視覺化 ----------
            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            for r in results:
                corners = r.corners.astype(int)
                for k in range(4):
                    cv2.line(vis, tuple(corners[k]), tuple(corners[(k + 1) % 4]), (0, 255, 0), 2)
            draw_pair_labels(vis, pairs)
            # 路線預覽線：只在「當下有偵測到且穩定」的 pair 時才畫
            if road_dir is not None and p.get("stable"):
                draw_route_line(vis, K_ir, road_dir)

            f_col = (0, 0, 255) if front_trigger else (0, 200, 255)
            f_txt = f"front={ob_front:.2f}m" if ob_front is not None else "front: no data"
            cv2.putText(vis, f_txt, (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, f_col, 2, cv2.LINE_AA)
            r_txt = f"right={ob_right:.2f}m" if ob_right is not None else "right: no data(free)"
            cv2.putText(vis, r_txt, (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2, cv2.LINE_AA)

            rear = scan.rear_min()
            rear_txt = "scan: none" if scan.msg is None else (
                f"rear={rear:.1f}m" if rear is not None else "rear: clear")
            status = state if run_enabled else "PAUSED (press SPACE)"
            cv2.putText(vis, f"{status}   {rear_txt}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if run_enabled else (0, 0, 255), 2, cv2.LINE_AA)
            if depth_diff is not None:
                cv2.putText(vis, f"dDep={depth_diff:+.3f}m", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA)

            cv2.imshow("Obstacle Bypass Test (IR + Depth)", cv2.resize(vis, (854, 480)))
            if depth_vis is not None:
                cv2.putText(depth_vis, f"{f_txt}  {r_txt}  (white=front, orange=right)", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("Depth Corridor", depth_vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                run_enabled = not run_enabled
                if not run_enabled:
                    segments, state = [], "DRIVE"   # 暫停同時重置繞行
                    cmd_pub.publish(Twist())
                rospy.loginfo(f"{'▶️  開始行駛' if run_enabled else '⏸  暫停（已送停車指令、重置狀態）'}")

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
