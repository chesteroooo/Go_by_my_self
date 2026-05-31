#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense D435i 本機辨識節點 (Local RealSense Detection Node)

用途：
  - 使用 pyrealsense2 讀取 RGB 影像
  - AprilTag 偵測 + pose 估計
  - 發布 /target_info (geometry_msgs/Pose)
  - 測試用，不影響既有流程

WSL2 USB 轉接（Windows 端執行一次）：
  1) 安裝 usbipd-win：https://github.com/dorssel/usbipd-win
  2) 列出裝置：   usbipd list
  3) 附加到 WSL2：usbipd attach --wsl --busid <BUSID>

依賴：
  - librealsense + pyrealsense2
  - pupil_apriltags, rospy, opencv-python
"""

import os
import sys

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import Pose
from pupil_apriltags import Detector

from pair_detector_balance import BalancePairDetector
from pair_detector_setting import draw_axes, draw_pair_labels

# ================= 參數設定 =================
COLOR_W = 640
COLOR_H = 480
FPS = 30

TAG_SIZE_M = 0.08  # Tag 邊長（公尺）
FRAME_TIMEOUT_MS = 5000

# True 會啟用 depth stream（此版本僅用 RGB 偵測）
ENABLE_DEPTH = False
# ===========================================


def get_color_intrinsics(profile: rs.pipeline_profile):
    stream = profile.get_stream(rs.stream.color)
    vsp = stream.as_video_stream_profile()
    intr = vsp.get_intrinsics()
    camera_params = (intr.fx, intr.fy, intr.ppx, intr.ppy)
    K = np.array([
        [intr.fx, 0.0, intr.ppx],
        [0.0, intr.fy, intr.ppy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)
    return intr, camera_params, K, intr.width, intr.height


def main():
    rospy.init_node("realsense_detect_node", anonymous=True)

    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.5,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
    )
    pd = BalancePairDetector(history_len=6, stable_threshold=4)
    pub = rospy.Publisher("/target_info", Pose, queue_size=1)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    if ENABLE_DEPTH:
        config.enable_stream(rs.stream.depth, COLOR_W, COLOR_H, rs.format.z16, FPS)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color) if ENABLE_DEPTH else None

    _, camera_params, K_color, color_w, _ = get_color_intrinsics(profile)
    rate = rospy.Rate(FPS)

    fps = 0.0
    prev_t = rospy.get_time()
    try:
        while not rospy.is_shutdown():
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                rospy.logwarn_throttle(2.0, f"等待影像逾時: {e}")
                rate.sleep()
                continue
            if ENABLE_DEPTH:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                rospy.logwarn_throttle(2.0, "未取得 color frame，重試中...")
                rate.sleep()
                continue

            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            results = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=TAG_SIZE_M,
            )

            # 繪製 Tag 邊框與座標軸
            for r in results:
                corners = r.corners.astype(int)
                for k in range(4):
                    cv2.line(frame, tuple(corners[k]), tuple(corners[(k + 1) % 4]), (0, 255, 0), 2)
                if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
                    rvec, _ = cv2.Rodrigues(r.pose_R)
                    tvec = r.pose_t.reshape(3, 1).astype(np.float32)
                    draw_axes(frame, K_color, rvec, tvec, length=TAG_SIZE_M * 0.5)

            # Pair 偵測
            pairs = pd.update_and_detect(results)
            draw_pair_labels(frame, pairs)

            # 畫面顯示 depth_diff
            for p in pairs:
                tl, tr = p.get("t_left"), p.get("t_right")
                if tl is not None and tr is not None:
                    diff = tr[2] - tl[2]
                    cx_p, cy_p = p["center"]
                    cv2.putText(
                        frame, f"dDep={diff:+.3f}m",
                        (cx_p + 8, cy_p - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2, cv2.LINE_AA,
                    )

            # 建立 id → 偵測結果的查找表，用於取得第二個 tag 的像素位置
            id_to_result = {int(r.tag_id): r for r in results}

            # 打包 /target_info
            msg = Pose()
            if pairs:
                p = pairs[0]
                id_a, id_b = p["members"]
                msg.orientation.x = float(id_a)
                msg.orientation.y = float(id_b)
                msg.orientation.w = 1.0

                # 對齊第二個 tag (id_b) 的像素中心
                r_b = id_to_result.get(id_b)
                if r_b is not None:
                    cx_b = float(np.mean(r_b.corners[:, 0]))
                    msg.position.x = cx_b - (color_w / 2.0)
                else:
                    cx_p, _ = p["center"]
                    msg.position.x = float(cx_p - (color_w / 2.0))

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

            pub.publish(msg)

            # 計算 FPS（指數平滑）
            now = rospy.get_time()
            dt = now - prev_t
            prev_t = now
            if dt > 0:
                inst_fps = 1.0 / dt
                fps = inst_fps if fps == 0.0 else 0.9 * fps + 0.1 * inst_fps

            cv2.putText(
                frame,
                f"FPS={fps:4.1f}  tags={len(results)} pairs={len(pairs)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("RealSense Detection", frame)
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
