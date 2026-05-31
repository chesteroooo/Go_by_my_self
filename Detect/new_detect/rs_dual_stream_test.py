#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense D435i 雙串流測試（不需要 ROS）

目的：
  - 用「同一個 pipeline」同時開兩路影像，各做各的：
      * IR（紅外線）串流  -> AprilTag 偵測
        D435i 的 IR 鏡頭是 global shutter（全域快門），在磚頭路震動下「不會果凍」，
        而且本來就是灰階，AprilTag 直接用、不需 cvtColor。
        會關閉 IR 投射器（emitter），避免點陣干擾 tag 偵測。
      * Color（RGB）串流 -> 給 YOLO segmentation 用
        YOLO 需要彩色，所以用 RGB（rolling shutter，果凍要靠機構減震處理）。

  - 兩路皆顯示視窗 + FPS，方便你比對 IR 是否真的比 RGB 穩。

注意：
  - IR 與 RGB 內參不同，AprilTag pose 必須用「IR 串流的內參」。
  - 相機需先 attach 進 WSL 才能跑。
"""

import os
import sys
import time

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector

from pair_detector_balance import BalancePairDetector
from pair_detector_setting import draw_axes, draw_pair_labels

# ================= 參數設定 =================
W = 640
H = 480
FPS = 30

IR_INDEX = 1          # 左 IR = 1，右 IR = 2（AprilTag 用左眼即可）
TAG_SIZE_M = 0.08     # Tag 邊長（公尺）
FRAME_TIMEOUT_MS = 5000

# YOLO 很重，N100 上不需要每幀都跑；每 N 幀跑一次（先當顯示節流用）
YOLO_EVERY_N = 1
# ===========================================


def stream_intrinsics(profile: rs.pipeline_profile, stream, index=-1):
    if index >= 0:
        sp = profile.get_stream(stream, index)
    else:
        sp = profile.get_stream(stream)
    intr = sp.as_video_stream_profile().get_intrinsics()
    camera_params = (intr.fx, intr.fy, intr.ppx, intr.ppy)
    K = np.array([
        [intr.fx, 0.0, intr.ppx],
        [0.0, intr.fy, intr.ppy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)
    return intr, camera_params, K


def disable_ir_emitter(profile: rs.pipeline_profile):
    """關閉 IR 投射器，讓 IR 影像乾淨（沒有點陣），利於 AprilTag。"""
    try:
        dev = profile.get_device()
        depth_sensor = dev.first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 0)
            print("[info] IR emitter 已關閉")
        else:
            print("[warn] 此裝置不支援 emitter_enabled 選項")
    except Exception as e:
        print(f"[warn] 關閉 emitter 失敗: {e}")


def smooth_fps(prev_fps, prev_t):
    now = time.time()
    dt = now - prev_t
    if dt <= 0:
        return prev_fps, now
    inst = 1.0 / dt
    fps = inst if prev_fps == 0.0 else 0.9 * prev_fps + 0.1 * inst
    return fps, now


def put_fps(img, fps, extra=""):
    cv2.putText(
        img, f"FPS={fps:4.1f}  {extra}", (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA,
    )


def main():
    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.5,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
    )
    pd = BalancePairDetector(history_len=6, stable_threshold=4)

    pipeline = rs.pipeline()
    config = rs.config()
    # 同一 pipeline 開兩路：IR（給 AprilTag）+ Color（給 YOLO）
    config.enable_stream(rs.stream.infrared, IR_INDEX, W, H, rs.format.y8, FPS)
    config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)

    profile = pipeline.start(config)
    disable_ir_emitter(profile)

    # AprilTag 用 IR 內參；色彩內參備用（畫 YOLO 框時若要投影可用）
    _, ir_params, K_ir = stream_intrinsics(profile, rs.stream.infrared, IR_INDEX)

    # ---- 在這裡載入你的 YOLO segmentation 模型 ----
    # from ultralytics import YOLO
    # yolo = YOLO("your_seg_model.pt")
    yolo = None
    # ------------------------------------------------

    # 先建立兩個視窗並分開擺位，避免在 WSLg 下重疊（彩色蓋住灰階）
    cv2.namedWindow("AprilTag (IR global shutter)", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Color (for YOLO)", cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("AprilTag (IR global shutter)", 50, 50)
    cv2.moveWindow("Color (for YOLO)", 50 + W + 30, 50)

    fps_ir, t_ir = 0.0, time.time()
    fps_rgb, t_rgb = 0.0, time.time()
    frame_idx = 0
    last_warn = 0.0
    warned_no_ir = False

    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                now = time.time()
                if now - last_warn > 2.0:
                    print(f"等待影像逾時: {e}")
                    last_warn = now
                continue

            ir_frame = frames.get_infrared_frame(IR_INDEX)
            color_frame = frames.get_color_frame()

            if not ir_frame and not warned_no_ir:
                print(f"[warn] 抓不到 IR(index={IR_INDEX})影像，請確認 D435i 有開 infrared 串流")
                warned_no_ir = True

            # ========== IR -> AprilTag（無果凍） ==========
            if ir_frame:
                gray = np.asanyarray(ir_frame.get_data())  # 已是灰階 y8
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
                        cv2.line(vis_ir, tuple(corners[k]), tuple(corners[(k + 1) % 4]), (0, 255, 0), 2)
                    if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
                        rvec, _ = cv2.Rodrigues(r.pose_R)
                        tvec = r.pose_t.reshape(3, 1).astype(np.float32)
                        draw_axes(vis_ir, K_ir, rvec, tvec, length=TAG_SIZE_M * 0.5)

                pairs = pd.update_and_detect(results)
                draw_pair_labels(vis_ir, pairs)

                fps_ir, t_ir = smooth_fps(fps_ir, t_ir)
                put_fps(vis_ir, fps_ir, f"tags={len(results)} pairs={len(pairs)}")
                cv2.imshow("AprilTag (IR global shutter)", vis_ir)

            # ========== Color -> YOLO segmentation ==========
            if color_frame:
                color = np.asanyarray(color_frame.get_data())

                frame_idx += 1
                if yolo is not None and frame_idx % YOLO_EVERY_N == 0:
                    # results_yolo = yolo(color, verbose=False)
                    # color = results_yolo[0].plot()   # 疊上 segmentation 結果
                    pass

                fps_rgb, t_rgb = smooth_fps(fps_rgb, t_rgb)
                put_fps(color, fps_rgb, "(for YOLO)")
                cv2.imshow("Color (for YOLO)", color)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
