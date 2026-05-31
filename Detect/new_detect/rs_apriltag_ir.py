#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense D435i — 純 AprilTag 偵測（IR 灰階，無 ROS）

特色：
  - 只用 IR（紅外線）串流做 AprilTag，不開彩色、不跑 YOLO。
  - D435i 的 IR 鏡頭是 global shutter（全域快門），在磚頭路震動下「不會果凍」，
    且本來就是灰階，AprilTag 直接吃、不需 cvtColor。
  - 會關閉 IR 投射器（emitter），避免點陣干擾 tag 偵測。

用途：單獨驗證 AprilTag 在 IR 上的穩定度與 FPS（不受彩色/YOLO 干擾）。

注意：
  - pose 估計使用「IR 串流的內參」。
  - 相機需先 attach 進 WSL（或直接接原生主機）才能跑。
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
# ===========================================


def ir_intrinsics(profile: rs.pipeline_profile, index: int):
    sp = profile.get_stream(rs.stream.infrared, index)
    intr = sp.as_video_stream_profile().get_intrinsics()
    camera_params = (intr.fx, intr.fy, intr.ppx, intr.ppy)
    K = np.array([
        [intr.fx, 0.0, intr.ppx],
        [0.0, intr.fy, intr.ppy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)
    return camera_params, K


def disable_ir_emitter(profile: rs.pipeline_profile):
    """關閉 IR 投射器，讓 IR 影像乾淨（沒有點陣），利於 AprilTag。"""
    try:
        depth_sensor = profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 0)
            print("[info] IR emitter 已關閉")
        else:
            print("[warn] 此裝置不支援 emitter_enabled 選項")
    except Exception as e:
        print(f"[warn] 關閉 emitter 失敗: {e}")


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
    config.enable_stream(rs.stream.infrared, IR_INDEX, W, H, rs.format.y8, FPS)

    profile = pipeline.start(config)
    disable_ir_emitter(profile)
    camera_params, K_ir = ir_intrinsics(profile, IR_INDEX)

    fps = 0.0
    prev_t = time.time()
    last_warn = 0.0
    last_prof = time.time()
    acc = {"wait": 0.0, "detect": 0.0, "draw": 0.0, "show": 0.0, "n": 0}
    try:
        while True:
            t0 = time.time()
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                now = time.time()
                if now - last_warn > 2.0:
                    print(f"等待影像逾時: {e}")
                    last_warn = now
                continue
            t1 = time.time()

            ir_frame = frames.get_infrared_frame(IR_INDEX)
            if not ir_frame:
                now = time.time()
                if now - last_warn > 2.0:
                    print(f"未取得 IR(index={IR_INDEX}) frame，重試中...")
                    last_warn = now
                continue

            gray = np.asanyarray(ir_frame.get_data())  # 已是灰階 y8
            results = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=TAG_SIZE_M,
            )
            t2 = time.time()

            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            for r in results:
                corners = r.corners.astype(int)
                for k in range(4):
                    cv2.line(vis, tuple(corners[k]), tuple(corners[(k + 1) % 4]), (0, 255, 0), 2)
                if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
                    rvec, _ = cv2.Rodrigues(r.pose_R)
                    tvec = r.pose_t.reshape(3, 1).astype(np.float32)
                    draw_axes(vis, K_ir, rvec, tvec, length=TAG_SIZE_M * 0.5)

            pairs = pd.update_and_detect(results)
            draw_pair_labels(vis, pairs)

            now = time.time()
            dt = now - prev_t
            prev_t = now
            if dt > 0:
                inst = 1.0 / dt
                fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst

            cv2.putText(
                vis,
                f"FPS={fps:4.1f}  tags={len(results)} pairs={len(pairs)}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA,
            )
            t3 = time.time()

            cv2.imshow("AprilTag (IR global shutter)", vis)
            key = cv2.waitKey(1) & 0xFF
            t4 = time.time()

            # 分段計時：每秒印一次各階段平均毫秒
            acc["wait"] += (t1 - t0)
            acc["detect"] += (t2 - t1)
            acc["draw"] += (t3 - t2)
            acc["show"] += (t4 - t3)
            acc["n"] += 1
            if now - last_prof > 1.0 and acc["n"] > 0:
                n = acc["n"]
                print(
                    f"[prof] wait={acc['wait']/n*1000:5.1f}ms "
                    f"detect={acc['detect']/n*1000:5.1f}ms "
                    f"draw={acc['draw']/n*1000:5.1f}ms "
                    f"show={acc['show']/n*1000:5.1f}ms  fps={fps:4.1f}"
                )
                acc = {"wait": 0.0, "detect": 0.0, "draw": 0.0, "show": 0.0, "n": 0}
                last_prof = now

            if key == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
