#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense D435i 雙串流偵測節點 (Dual Stream Detection Node)

同一個 pipeline 同時跑兩路：
  - IR  串流 (global shutter) → AprilTag pair 偵測 → 發布 /target_info
  - Color 串流               → YOLO-seg (best.pt) 路面分割 → 車道置中
                                發布 /floor_detected (Bool) 與 /floor_info (Pose)
                                drive 模式下直接發布 /cmd_vel 讓車保持在路面中央

為何分開：
  - AprilTag 需要 global shutter（無果凍），只有 IR 有
  - YOLO segmentation 需要彩色（RGB），用 Color 串流
  - 各自用自己的內參，pose 才準確

車道置中 (lane centering)：
  - 取 road mask 下半部（近場）逐列質心 → 對影像中線的偏移 err_norm ∈ [-1, 1]
  - 再取較遠一段的質心估路面走向 heading_norm（前饋，直路提早修正）
  - w = -(KP_CENTER*err + KP_HEADING*heading)，符號同 ros_move_pair_task.py
  - 鍵盤（OpenCV 視窗須為焦點）：SPACE = 開始/暫停行駛，q = 離開
  - 啟動時為 PAUSE（不動），按 SPACE 才開始走；路面遺失/分割逾時自動停車
  - rosrun 參數 _drive:=false 可完全關閉 /cmd_vel（改與 ros_move_* 控制器搭配）
  - rosrun 參數 _tags:=false 可關閉 AprilTag/IR（4 核 N100 上 CPU 全給分割，seg 更快）
    ⚠ drive 模式下請勿同時跑任何 ros_move_*.py，兩者都發 /cmd_vel 會打架

/floor_info 欄位（geometry_msgs/Pose 挪用，同 /target_info 的做法）：
  orientation.w : 1.0 = 偵測到路面, 0.0 = 沒有
  position.x    : err_norm（路面中心相對影像中線，左負右正）
  position.y    : heading_norm（遠處路面相對近處的偏移，估路的走向）
  position.z    : 近場 road 覆蓋率 0~1

Ubuntu RealSense 設定（首次使用前）：
  sudo apt install librealsense2-dkms librealsense2-utils -y
  pip3 install pyrealsense2 ultralytics
  realsense-viewer   # 確認相機連接正常
"""

import glob
import os
import sys
import threading
import time

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "apriltag_setting"))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
from pupil_apriltags import Detector

from pair_detector_balance import BalancePairDetector
from pair_detector_setting import draw_axes, draw_pair_labels, select_best_pair

# ================= 參數設定 =================
IR_W     = 1280
IR_H     = 720
IR_INDEX = 1       # 左 IR = 1（AprilTag 用左眼即可）

COLOR_W  = 640
COLOR_H  = 480

FPS = 30

TAG_SIZE_M       = 0.11    # 實際印出 11cm
FRAME_TIMEOUT_MS = 5000

# 同一組 pair 兩 tag 的最大 3D 間距 (m)：濾掉多地點同時入鏡時的跨地點假 pair
PAIR_MAX_GAP_M   = 1.0

# YOLO-seg 設定
YOLO_CONF       = 0.5      # 信心門檻
YOLO_IMGSZ      = 320      # 推論尺寸（CPU 推論，越小越快；320 對車道置中已足夠）
YOLO_THREADS    = 2        # torch 執行緒數（4 核 N100：跟 AprilTag 分核，避免互搶）
TAG_THREADS     = 2        # AprilTag 執行緒數（原 4 會跟 YOLO 搶滿 CPU → seg 逾時閃爍）
ROAD_CLASS_NAME = "road"   # 模型中路面的類別名稱（自動從 model.names 找 id）
FLOOR_CLASS_ID  = 1        # 找不到名稱時的備援 class id（autolabel: 0=grass 1=road 2=sidewalk）

# 車道置中設定
NEAR_BAND_FRAC  = 0.45     # 近場帶：影像底部這個比例的高度，決定 err_norm
FAR_BAND_TOP    = 0.30     # 遠場帶頂端（影像高度比例），估 heading 用
MIN_ROAD_COVER  = 0.10     # 近場帶 road 覆蓋率低於此值視為沒路（停車）
SEG_STALE_SEC   = 2.0      # 分割結果超過這個秒數沒更新 → 視為遺失（停車）
ERR_EMA_ALPHA   = 0.5      # err/heading 的 EMA 平滑係數（新值權重）

LANE_SPEED_V    = 1.2    # 置中巡航線速度 (m/s)，低於 pair_task 的 0.2
KP_CENTER       = 0.35     # err_norm → 角速度增益
KP_HEADING      = 0.20     # heading_norm → 角速度前饋增益
MAX_CENTER_W    = 0.25     # 最大角速度 (rad/s)，同 pair_task 的 MAX_DRIVE_W
# ===========================================


def get_intrinsics(profile: rs.pipeline_profile, stream, index=-1):
    sp   = profile.get_stream(stream, index) if index >= 0 else profile.get_stream(stream)
    intr = sp.as_video_stream_profile().get_intrinsics()
    camera_params = (intr.fx, intr.fy, intr.ppx, intr.ppy)
    K = np.array([
        [intr.fx, 0.0,     intr.ppx],
        [0.0,     intr.fy, intr.ppy],
        [0.0,     0.0,     1.0     ],
    ], dtype=np.float32)
    return camera_params, K, intr.width


def disable_ir_emitter(profile: rs.pipeline_profile):
    """關閉 IR 投射器，讓 IR 影像乾淨（沒有點陣），利於 AprilTag。"""
    try:
        depth_sensor = profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 0)
            rospy.loginfo("IR emitter 已關閉")
        else:
            rospy.logwarn("此裝置不支援 emitter_enabled 選項")
    except Exception as e:
        rospy.logwarn(f"關閉 emitter 失敗: {e}")


def find_model_path():
    """尋找分割模型：優先用 ~model 參數，否則在 new_detect 底下找 best3.pt → best.pt。"""
    param = rospy.get_param("~model", "")
    if param:
        return os.path.expanduser(param)
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        # 有 TensorRT engine 就優先用（Orin GPU 上最快）；否則用 .pt
        os.path.join(here, "best3.engine"),
        os.path.join(here, "best.engine"),
        os.path.join(here, "best3.pt"),
        os.path.join(here, "best.pt"),
        os.path.join(here, "segmentation", "best3.pt"),
        os.path.join(here, "segmentation", "best.pt"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    hits = sorted(glob.glob(os.path.join(here, "**", "best*.engine"), recursive=True)) or \
           sorted(glob.glob(os.path.join(here, "**", "best*.pt"), recursive=True))
    return hits[0] if hits else candidates[0]


def load_yolo(model_path: str):
    """載入 YOLO segmentation 模型並找出所有 road class id，失敗時回傳 (None, None)。

    模型可能有一個以上名為 'road' 的類別
    （best3.pt: {0:'road', 1:'grass', 2:'road', 3:'sidewalk'}），全部都算路面，回傳 id 的 set。
    """
    try:
        from ultralytics import YOLO
    except ImportError:
        rospy.logwarn("ultralytics 未安裝，執行 pip3 install ultralytics。YOLO 功能停用")
        return None, None
    if not os.path.isfile(model_path):
        rospy.logwarn(f"找不到 YOLO 模型：{model_path}，YOLO 功能停用")
        return None, None
    model = YOLO(model_path)
    names = model.names if isinstance(model.names, dict) else dict(enumerate(model.names))
    road_ids = {int(cid) for cid, name in names.items()
                if str(name).lower() in (ROAD_CLASS_NAME, "floor")}
    if not road_ids:
        road_ids = {FLOOR_CLASS_ID}
        rospy.logwarn(f"模型類別 {names} 中沒有 '{ROAD_CLASS_NAME}'，改用 class id {FLOOR_CLASS_ID}")
    rospy.loginfo(f"YOLO 模型載入成功：{model_path}  classes={names}  road_ids={sorted(road_ids)}")
    return model, road_ids


class SegWorker(threading.Thread):
    """獨立執行緒跑 YOLO-seg（CPU 推論 100~300ms，不能卡住 30fps 主迴圈）。

    主迴圈用 submit() 丟最新彩色影格（latest-wins），用 latest() 取最近一次結果。
    """

    def __init__(self, model, road_ids, n_threads=YOLO_THREADS, device="cpu", imgsz=YOLO_IMGSZ):
        super().__init__(daemon=True)
        self.model     = model
        self.road_ids  = set(road_ids)   # 可能有多個 road 類別，全部算路面
        self.n_threads = n_threads
        self.device    = device          # "0"=GPU（Orin）, "cpu"=N100
        self.imgsz     = imgsz
        self._lock    = threading.Lock()
        self._event   = threading.Event()
        self._stop    = threading.Event()
        self._frame   = None
        self._result  = None   # dict(mask, ts, infer_ms)

    def submit(self, frame):
        with self._lock:
            self._frame = frame
        self._event.set()

    def latest(self):
        with self._lock:
            return self._result

    def stop(self):
        self._stop.set()
        self._event.set()

    def run(self):
        # 只有 CPU 推論才需要限制 torch 執行緒（跟 AprilTag 分核）；GPU 上 seg 不佔 CPU 核
        if str(self.device) in ("cpu", "-1", ""):
            try:
                import torch
                torch.set_num_threads(self.n_threads)   # 4 核 N100：留核心給 AprilTag/主迴圈
            except Exception:
                pass
        while not self._stop.is_set():
            self._event.wait()
            self._event.clear()
            if self._stop.is_set():
                break
            with self._lock:
                frame, self._frame = self._frame, None
            if frame is None:
                continue
            t0 = time.time()
            try:
                res = self.model(frame, verbose=False, conf=YOLO_CONF,
                                 imgsz=self.imgsz, device=self.device)[0]
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"YOLO 推論失敗: {e}")
                continue
            h, w = frame.shape[:2]
            mask = np.zeros((h, w), dtype=np.uint8)
            if res.masks is not None:
                for i, cls in enumerate(res.boxes.cls):
                    if int(cls) in self.road_ids:
                        m = res.masks.data[i].cpu().numpy().astype(np.uint8)
                        mask |= cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
            with self._lock:
                self._result = {"mask": mask, "ts": time.time(),
                                "infer_ms": (time.time() - t0) * 1000.0}


def lane_metrics(mask):
    """從 road mask 算車道置中資訊。

    回傳 (err_norm, heading_norm, cover)；近場覆蓋率不足時回傳 None。
      err_norm     : 近場路面質心相對影像中線（左負右正，除以半寬正規化）
      heading_norm : 遠場質心 − 近場質心（估路的走向）
      cover        : 近場帶 road 覆蓋率 0~1
    """
    h, w   = mask.shape
    y_near = int(h * (1.0 - NEAR_BAND_FRAC))
    near   = mask[y_near:, :]
    cover  = float(near.mean())
    if cover < MIN_ROAD_COVER:
        return None
    cols    = np.arange(w, dtype=np.float32)
    cx_near = float((near.sum(axis=0) * cols).sum() / near.sum())
    far     = mask[int(h * FAR_BAND_TOP):y_near, :]
    cx_far  = cx_near
    if far.sum() > 50:   # 遠場太少 road 像素就不估 heading
        cx_far = float((far.sum(axis=0) * cols).sum() / far.sum())
    half = w / 2.0
    return (cx_near - half) / half, (cx_far - cx_near) / half, cover


def smooth_fps(prev_fps: float, prev_t: float):
    now = time.time()
    dt  = now - prev_t
    if dt <= 0:
        return prev_fps, now
    inst = 1.0 / dt
    return (inst if prev_fps == 0.0 else 0.9 * prev_fps + 0.1 * inst), now


def main():
    rospy.init_node("dual_detect_node", anonymous=True)

    # _tags:=false → 完全跳過 AprilTag（不開 IR 串流），整台 CPU 給 YOLO（純車道跟隨測試用）
    use_tags = bool(rospy.get_param("~tags", True))

    # 無顯示器（SSH）自動走無視窗模式；_gui:=false 可強制關視窗。
    # headless 下不開 OpenCV 視窗、不讀鍵盤（drive 模式的 SPACE 需有視窗的桌面）。
    gui = bool(rospy.get_param("~gui", True)) and bool(os.environ.get("DISPLAY"))
    if not gui:
        rospy.loginfo("headless 模式（無 $DISPLAY 或 _gui:=false）：不開視窗、不讀鍵盤，只發布 topic。"
                      "要看畫面/用 SPACE 置中，請在 Jetson 桌面（螢幕或遠端桌面）執行。")

    detector = pd = None
    if use_tags:
        detector = Detector(
            families="tag36h11",
            nthreads=TAG_THREADS,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
        )
        pd = BalancePairDetector(history_len=6, stable_threshold=4,
                                 max_pair_gap_m=PAIR_MAX_GAP_M)
    else:
        rospy.loginfo("AprilTag 關閉（~tags=false）：不開 IR 串流，CPU 全給路面分割")

    model_path      = find_model_path()
    yolo, road_ids  = load_yolo(model_path)

    # 推論裝置：~device 可覆寫（"0"=GPU / "cpu"）；預設自動偵測（Orin 有 CUDA 用 GPU，N100 用 CPU）
    device = str(rospy.get_param("~device", "")).strip()
    if not device:
        try:
            import torch
            device = "0" if torch.cuda.is_available() else "cpu"
        except Exception:
            device = "cpu"
    imgsz  = int(rospy.get_param("~imgsz", YOLO_IMGSZ))
    on_gpu = str(device) not in ("cpu", "-1", "")

    seg_worker      = None
    if yolo is not None:
        # GPU 上 seg 不佔 CPU 核 → 不需 N100 的 2+2 分核；CPU 上才留核心給 AprilTag
        if on_gpu:
            n_thr = os.cpu_count() or 4
        else:
            n_thr = YOLO_THREADS if use_tags else max(YOLO_THREADS, (os.cpu_count() or 4) - 1)
        seg_worker = SegWorker(yolo, road_ids, n_thr, device=device, imgsz=imgsz)
        seg_worker.start()
        rospy.loginfo(f"YOLO 裝置={device}  imgsz={imgsz}  ({'GPU' if on_gpu else 'CPU'})")

    drive_enabled = bool(rospy.get_param("~drive", True))

    target_pub = rospy.Publisher("/target_info",    Pose, queue_size=1)
    floor_pub  = rospy.Publisher("/floor_detected", Bool, queue_size=1)
    info_pub   = rospy.Publisher("/floor_info",     Pose, queue_size=1)
    cmd_pub    = rospy.Publisher("/cmd_vel", Twist, queue_size=1) if drive_enabled else None
    if drive_enabled:
        rospy.loginfo("drive 模式開啟：啟動為 PAUSE，OpenCV 視窗按 SPACE 開始/暫停置中行駛"
                      "（勿同時跑 ros_move_*.py）")
    else:
        rospy.loginfo("drive 模式關閉（~drive=false）：只發 /floor_info，不發 /cmd_vel")

    pipeline = rs.pipeline()
    config   = rs.config()
    if use_tags:
        config.enable_stream(rs.stream.infrared, IR_INDEX, IR_W, IR_H, rs.format.y8, FPS)
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)

    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        rospy.logerr(f"無法開啟 RealSense 相機：{e}")
        rospy.logerr("D435i 一次只能被『一個』程式開啟。請先關閉其他相機程式"
                     "（ros_detect_apriltag.py / ros_detect_dual.py / ros_test_bypass.py / "
                     "ros_test_ground_bypass.py / collect_floor_dataset.py / realsense-viewer）再執行。")
        return
    disable_ir_emitter(profile)

    if use_tags:
        ir_params, K_ir, ir_w = get_intrinsics(profile, rs.stream.infrared, IR_INDEX)
        if gui:
            cv2.namedWindow("AprilTag (IR)", cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow("AprilTag (IR)", 50, 50)
        rospy.loginfo(f"IR {IR_W}x{IR_H}  Color {COLOR_W}x{COLOR_H}  @ {FPS}fps")
    else:
        rospy.loginfo(f"Color {COLOR_W}x{COLOR_H} @ {FPS}fps（IR 關閉）")

    if gui:
        cv2.namedWindow("Floor (Color)", cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow("Floor (Color)", 50 + 640 + 30, 50)

    rate       = rospy.Rate(FPS)
    fps_ir,  t_ir  = 0.0, time.time()
    fps_rgb, t_rgb = 0.0, time.time()
    last_pair_key  = None    # 黏滯選擇：鎖定上一幀的 pair，避免逐幀跳換
    driving        = False   # SPACE 切換；啟動時暫停
    err_f = head_f = None    # EMA 平滑後的置中誤差

    try:
        while not rospy.is_shutdown():
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                rospy.logwarn_throttle(2.0, f"等待影像逾時: {e}")
                rate.sleep()
                continue

            ir_frame    = frames.get_infrared_frame(IR_INDEX) if use_tags else None
            color_frame = frames.get_color_frame()

            # ==================== IR → AprilTag ====================
            if ir_frame:
                gray = np.asanyarray(ir_frame.get_data())

                results = detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=ir_params,
                    tag_size=TAG_SIZE_M,
                )

                vis_ir = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                for r in results:
                    corners = r.corners.astype(int)
                    for k in range(4):
                        cv2.line(vis_ir, tuple(corners[k]), tuple(corners[(k+1)%4]), (0, 255, 0), 2)
                    if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
                        rvec, _ = cv2.Rodrigues(r.pose_R)
                        tvec = r.pose_t.reshape(3, 1).astype(np.float32)
                        draw_axes(vis_ir, K_ir, rvec, tvec, length=TAG_SIZE_M * 0.5)

                pairs = pd.update_and_detect(results)
                draw_pair_labels(vis_ir, pairs)

                # ── /target_info ──
                msg = Pose()
                p   = select_best_pair(pairs, prefer_key=last_pair_key)
                last_pair_key = p["key"] if p is not None else None
                if p is not None:
                    msg.orientation.x = float(p["id_left"])
                    msg.orientation.y = float(p["id_right"])
                    msg.orientation.w = 1.0

                    id_to_result = {int(r.tag_id): r for r in results}
                    r_b = id_to_result.get(p["id_right"])
                    if r_b is not None:
                        msg.position.x = float(np.mean(r_b.corners[:, 0])) - (ir_w / 2.0)
                    else:
                        cx_p, _ = p["center"]
                        msg.position.x = float(cx_p - (ir_w / 2.0))

                    tl, tr = p.get("t_left"), p.get("t_right")
                    if tl is not None and tr is not None:
                        msg.position.y = float(tr[2] - tl[2])
                    t_comb = p.get("t")
                    if t_comb is not None:
                        msg.position.z = float(np.linalg.norm(t_comb))
                else:
                    msg.orientation.w = 0.0
                    msg.orientation.x = -1.0
                    msg.orientation.y = -1.0
                target_pub.publish(msg)

                fps_ir, t_ir = smooth_fps(fps_ir, t_ir)
                cv2.putText(vis_ir, f"FPS={fps_ir:4.1f}  tags={len(results)} pairs={len(pairs)}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)
                if gui:
                    cv2.imshow("AprilTag (IR)", cv2.resize(vis_ir, (640, 360)))

            # ============ Color → YOLO-seg → 車道置中 ============
            if color_frame:
                color = np.asanyarray(color_frame.get_data())

                seg = None
                if seg_worker is not None:
                    seg_worker.submit(color)         # latest-wins，worker 忙碌時自動丟舊幀
                    seg = seg_worker.latest()

                metrics = None
                stale   = True
                seg_age = 0.0
                if seg is not None:
                    seg_age = time.time() - seg["ts"]
                    stale   = seg_age > SEG_STALE_SEC
                    if not stale:
                        metrics = lane_metrics(seg["mask"])
                    # 疊上 road mask（綠色半透明）
                    m = seg["mask"] == 1
                    color[m] = (color[m] * 0.5 + np.array([0, 180, 0]) * 0.5).astype(np.uint8)

                detected = metrics is not None

                # ── EMA 平滑置中誤差 ──
                if detected:
                    err, head, cover = metrics
                    if err_f is None:
                        err_f, head_f = err, head
                    else:
                        err_f  = ERR_EMA_ALPHA * err  + (1 - ERR_EMA_ALPHA) * err_f
                        head_f = ERR_EMA_ALPHA * head + (1 - ERR_EMA_ALPHA) * head_f
                else:
                    err_f = head_f = None

                # ── /floor_detected + /floor_info ──
                floor_pub.publish(Bool(data=detected))
                info = Pose()
                info.orientation.w = 1.0 if detected else 0.0
                if detected:
                    info.position.x = float(err_f)
                    info.position.y = float(head_f)
                    info.position.z = float(cover)
                info_pub.publish(info)

                # ── 置中行駛 /cmd_vel ──
                if cmd_pub is not None:
                    twist = Twist()
                    if driving and detected:
                        twist.linear.x  = LANE_SPEED_V
                        w = -(KP_CENTER * err_f + KP_HEADING * head_f)
                        twist.angular.z = max(min(w, MAX_CENTER_W), -MAX_CENTER_W)
                        rospy.loginfo_throttle(
                            1.0, f"[CENTER] err={err_f:+.2f} head={head_f:+.2f} "
                                 f"cover={cover:.2f} w={twist.angular.z:+.3f}")
                    elif driving:
                        rospy.logwarn_throttle(2.0, "[CENTER] 路面遺失/分割逾時 → 停車")
                    cmd_pub.publish(twist)   # 暫停或遺失時持續發 0（停車）

                # ── 視覺化 ──
                hh, ww = color.shape[:2]
                y_near = int(hh * (1.0 - NEAR_BAND_FRAC))
                cv2.line(color, (ww // 2, y_near), (ww // 2, hh), (0, 255, 255), 1)
                if detected:
                    cx = int(ww / 2 + err_f * ww / 2)
                    cv2.line(color, (cx, y_near), (cx, hh), (0, 0, 255), 2)

                fps_rgb, t_rgb = smooth_fps(fps_rgb, t_rgb)
                if drive_enabled:
                    mode = "RUN" if driving else "PAUSE(SPACE)"
                else:
                    mode = "no-drive"
                if detected:
                    label, lcolor = f"road err={err_f:+.2f}", (0, 255, 0)
                elif seg is not None and stale:
                    # 分割太慢（CPU 不夠力）：顯示橘色 + 結果年齡，跟「真的沒路」區分開
                    label, lcolor = f"road STALE {seg_age:.1f}s", (0, 165, 255)
                else:
                    label, lcolor = "road: NO", (0, 0, 255)
                seg_ms = f" seg={seg['infer_ms']:.0f}ms" if seg else ""
                cv2.putText(color, f"FPS={fps_rgb:4.1f} {label} [{mode}]{seg_ms}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, lcolor, 2, cv2.LINE_AA)
                if gui:
                    cv2.imshow("Floor (Color)", color)

            if gui:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" ") and cmd_pub is not None:
                    driving = not driving
                    rospy.loginfo(f"[CENTER] {'開始行駛' if driving else '暫停（停車）'}")

            rate.sleep()
    finally:
        if cmd_pub is not None:
            cmd_pub.publish(Twist())   # 離開前確保停車
        if seg_worker is not None:
            seg_worker.stop()
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
