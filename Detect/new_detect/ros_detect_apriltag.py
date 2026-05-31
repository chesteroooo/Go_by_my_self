#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense D435i 本機辨識節點 — IR 串流版 (Local RealSense Detection Node, IR stream)

使用 D435i 的 IR（紅外線）串流做 AprilTag 偵測：
  - Global shutter：在機器人行進震動時不會產生「果凍」失真
  - 本來就是灰階，AprilTag 直接使用，無需 cvtColor
  - 關閉 IR 投射器（emitter），避免點陣干擾 tag 偵測

發布 /target_info (geometry_msgs/Pose)：
  orientation.x  — 左  tag ID
  orientation.y  — 右  tag ID
  orientation.w  — 1.0 = 偵測到, 0.0 = 未偵測到
  position.x     — 水平像素誤差 (pair 中心 - 畫面中心), 左=負 右=正
  position.y     — depth_diff = t_right.z - t_left.z (公尺)
  position.z     — 到 pair 中心的 3D 歐幾里得距離 (公尺)

Ubuntu RealSense 設定（首次使用前）：
  sudo apt install librealsense2-dkms librealsense2-utils -y
  pip3 install pyrealsense2
  realsense-viewer   # 確認相機連接正常
"""

import os
import sys

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "apriltag_setting"))
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
W   = 848
H   = 480
FPS = 60

IR_INDEX         = 1       # 左 IR = 1，右 IR = 2（AprilTag 用左眼即可）
TAG_SIZE_M       = 0.08    # Tag 邊長（公尺）
FRAME_TIMEOUT_MS = 5000
# ===========================================


def ir_intrinsics(profile: rs.pipeline_profile, index: int):
    sp   = profile.get_stream(rs.stream.infrared, index)
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


def main():
    rospy.init_node("realsense_detect_node", anonymous=True)

    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1,
        quad_sigma=0.5,
        refine_edges=True,
        decode_sharpening=0.5,
    )
    pd  = BalancePairDetector(history_len=6, stable_threshold=4)
    pub = rospy.Publisher("/target_info", Pose, queue_size=1)

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.infrared, IR_INDEX, W, H, rs.format.y8, FPS)

    profile = pipeline.start(config)
    disable_ir_emitter(profile)
    camera_params, K_ir, ir_w = ir_intrinsics(profile, IR_INDEX)

    rospy.loginfo(f"RealSense IR 串流已開啟：{W}x{H} @ {FPS}fps")

    rate  = rospy.Rate(FPS)
    fps   = 0.0
    prev_t = rospy.get_time()

    try:
        while not rospy.is_shutdown():
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                rospy.logwarn_throttle(2.0, f"等待影像逾時: {e}")
                rate.sleep()
                continue

            ir_frame = frames.get_infrared_frame(IR_INDEX)
            if not ir_frame:
                rospy.logwarn_throttle(2.0, "未取得 IR frame，重試中...")
                rate.sleep()
                continue

            gray = np.asanyarray(ir_frame.get_data())  # 已是灰階 y8

            results = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=TAG_SIZE_M,
            )

            # 轉 BGR 用於視覺化（保留彩色標記）
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            for r in results:
                corners = r.corners.astype(int)
                for k in range(4):
                    cv2.line(frame, tuple(corners[k]), tuple(corners[(k + 1) % 4]), (0, 255, 0), 2)
                if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
                    rvec, _ = cv2.Rodrigues(r.pose_R)
                    tvec = r.pose_t.reshape(3, 1).astype(np.float32)
                    draw_axes(frame, K_ir, rvec, tvec, length=TAG_SIZE_M * 0.5)

            pairs = pd.update_and_detect(results)
            draw_pair_labels(frame, pairs)

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

            id_to_result = {int(r.tag_id): r for r in results}

            msg = Pose()
            if pairs:
                p      = pairs[0]
                id_a, id_b = p["members"]
                msg.orientation.x = float(id_a)
                msg.orientation.y = float(id_b)
                msg.orientation.w = 1.0

                r_b = id_to_result.get(id_b)
                if r_b is not None:
                    cx_b = float(np.mean(r_b.corners[:, 0]))
                    msg.position.x = cx_b - (ir_w / 2.0)
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

            pub.publish(msg)

            now = rospy.get_time()
            dt  = now - prev_t
            prev_t = now
            if dt > 0:
                inst_fps = 1.0 / dt
                fps = inst_fps if fps == 0.0 else 0.9 * fps + 0.1 * inst_fps

            cv2.putText(
                frame,
                f"FPS={fps:4.1f}  tags={len(results)} pairs={len(pairs)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA,
            )

            cv2.imshow("RealSense Detection (IR)", cv2.resize(frame, (640, 360)))
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
