#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense D435i 雙串流偵測節點 (Dual Stream Detection Node)

同一個 pipeline 同時跑兩路：
  - IR  串流 (global shutter) → AprilTag pair 偵測 → 發布 /target_info
  - Color 串流               → YOLO 地板/路徑 segmentation → 發布 /floor_detected

為何分開：
  - AprilTag 需要 global shutter（無果凍），只有 IR 有
  - YOLO segmentation 需要彩色（RGB），用 Color 串流
  - 各自用自己的內參，pose 才準確

Ubuntu RealSense 設定（首次使用前）：
  sudo apt install librealsense2-dkms librealsense2-utils -y
  pip3 install pyrealsense2 ultralytics
  realsense-viewer   # 確認相機連接正常
"""

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
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from pupil_apriltags import Detector

from pair_detector_balance import BalancePairDetector
from pair_detector_setting import draw_axes, draw_pair_labels

# ================= 參數設定 =================
IR_W     = 1280
IR_H     = 720
IR_INDEX = 1       # 左 IR = 1（AprilTag 用左眼即可）

COLOR_W  = 640
COLOR_H  = 480

FPS = 30

TAG_SIZE_M       = 0.08
FRAME_TIMEOUT_MS = 5000

# YOLO 設定
YOLO_MODEL_PATH = "your_floor_model.pt"   # ← 換成你的模型路徑
YOLO_EVERY_N    = 3                        # 每 N 幀跑一次 YOLO（降低 CPU 負擔）
YOLO_CONF       = 0.5                      # 信心門檻
FLOOR_CLASS_ID  = 0                        # 你的模型中地板的 class ID
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


def load_yolo(model_path: str):
    """載入 YOLO segmentation 模型，失敗時回傳 None。"""
    try:
        from ultralytics import YOLO
        model = YOLO(model_path)
        rospy.loginfo(f"YOLO 模型載入成功：{model_path}")
        return model
    except FileNotFoundError:
        rospy.logwarn(f"找不到 YOLO 模型：{model_path}，YOLO 功能停用")
        return None
    except ImportError:
        rospy.logwarn("ultralytics 未安裝，執行 pip3 install ultralytics。YOLO 功能停用")
        return None


def smooth_fps(prev_fps: float, prev_t: float):
    now = time.time()
    dt  = now - prev_t
    if dt <= 0:
        return prev_fps, now
    inst = 1.0 / dt
    return (inst if prev_fps == 0.0 else 0.9 * prev_fps + 0.1 * inst), now


def main():
    rospy.init_node("dual_detect_node", anonymous=True)

    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.5,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
    )
    pd   = BalancePairDetector(history_len=6, stable_threshold=4)
    yolo = load_yolo(YOLO_MODEL_PATH)

    target_pub = rospy.Publisher("/target_info",    Pose, queue_size=1)
    floor_pub  = rospy.Publisher("/floor_detected", Bool, queue_size=1)

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.infrared, IR_INDEX, IR_W,    IR_H,    rs.format.y8,   FPS)
    config.enable_stream(rs.stream.color,              COLOR_W, COLOR_H, rs.format.bgr8, FPS)

    profile = pipeline.start(config)
    disable_ir_emitter(profile)

    ir_params, K_ir, ir_w = get_intrinsics(profile, rs.stream.infrared, IR_INDEX)

    rospy.loginfo(f"IR {IR_W}x{IR_H}  Color {COLOR_W}x{COLOR_H}  @ {FPS}fps")

    cv2.namedWindow("AprilTag (IR)", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Floor (Color)", cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("AprilTag (IR)", 50, 50)
    cv2.moveWindow("Floor (Color)", 50 + 640 + 30, 50)

    rate       = rospy.Rate(FPS)
    fps_ir,  t_ir  = 0.0, time.time()
    fps_rgb, t_rgb = 0.0, time.time()
    frame_idx      = 0
    last_yolo_vis  = None    # 快取上一次 YOLO 視覺化（節流用）
    floor_state    = False   # 保留上一次 YOLO 的地板判定，避免跳變閃爍

    try:
        while not rospy.is_shutdown():
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                rospy.logwarn_throttle(2.0, f"等待影像逾時: {e}")
                rate.sleep()
                continue

            ir_frame    = frames.get_infrared_frame(IR_INDEX)
            color_frame = frames.get_color_frame()
            frame_idx  += 1

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
                if pairs:
                    p          = pairs[0]
                    id_a, id_b = p["members"]
                    msg.orientation.x = float(id_a)
                    msg.orientation.y = float(id_b)
                    msg.orientation.w = 1.0

                    id_to_result = {int(r.tag_id): r for r in results}
                    r_b = id_to_result.get(id_b)
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
                cv2.imshow("AprilTag (IR)", cv2.resize(vis_ir, (640, 360)))

            # ==================== Color → YOLO floor ====================
            if color_frame:
                color = np.asanyarray(color_frame.get_data())

                if yolo is not None and frame_idx % YOLO_EVERY_N == 0:
                    floor_state = False   # 只在實際跑 YOLO 的幀重新判定
                    yolo_results = yolo(color, verbose=False, conf=YOLO_CONF)
                    masks = yolo_results[0].masks
                    if masks is not None:
                        for i, cls in enumerate(yolo_results[0].boxes.cls):
                            if int(cls) == FLOOR_CLASS_ID:
                                floor_state = True
                                mask = masks.data[i].cpu().numpy().astype(np.uint8)
                                mask = cv2.resize(mask, (COLOR_W, COLOR_H))
                                color[mask == 1] = (
                                    color[mask == 1] * 0.5 + np.array([0, 180, 0]) * 0.5
                                ).astype(np.uint8)
                    last_yolo_vis = color.copy()
                elif last_yolo_vis is not None:
                    color = last_yolo_vis.copy()   # 跳過的幀顯示上一次 YOLO 結果

                floor_pub.publish(Bool(data=floor_state))

                fps_rgb, t_rgb = smooth_fps(fps_rgb, t_rgb)
                label       = "Floor: YES" if floor_state else "Floor: NO"
                label_color = (0, 255, 0)  if floor_state else (0, 0, 255)
                cv2.putText(color, f"FPS={fps_rgb:4.1f}  {label}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, label_color, 2, cv2.LINE_AA)
                cv2.imshow("Floor (Color)", color)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            rate.sleep()
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
