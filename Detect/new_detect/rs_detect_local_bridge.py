#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense D435i 本機辨識節點 — rosbridge 版 (roslibpy)

與 rs_detect_local.py 的差別：
  - 不使用 rospy / ROS_MASTER_URI，改用 roslibpy 透過 websocket 連到機器人的 rosbridge。
  - 視覺邏輯（AprilTag + pose + pair）完全相同，只改「ROS 通訊」那幾段。
  - 適合在「沒裝整套 ROS」的機器（Windows / 原生 Linux）上跑視覺，
    只把小訊息 /target_info 經 rosbridge 送給機器人。

前置需求：
  - 本機：pip install roslibpy pyrealsense2 pupil-apriltags opencv-python numpy
  - 機器人(ROSBRIDGE_HOST)：先啟動 rosbridge
        sudo apt install ros-noetic-rosbridge-server
        roslaunch rosbridge_server rosbridge_websocket.launch   # 預設 port 9090

注意：
  - 影像/遮罩等大資料「不要」過 rosbridge，只傳濃縮後的小訊息（這裡是 Pose）。
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
import roslibpy
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

# rosbridge 連線（機器人那台）
ROSBRIDGE_HOST = "10.0.11.2"
ROSBRIDGE_PORT = 9090
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


def make_pose(px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, ow=0.0):
    """roslibpy 的訊息是純 dict，結構需對應 geometry_msgs/Pose。"""
    return {
        "position": {"x": px, "y": py, "z": pz},
        "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
    }


def warn_throttle(state, key, period, msg):
    """簡易節流列印（取代 rospy.logwarn_throttle）。"""
    now = time.time()
    if now - state.get(key, 0.0) > period:
        print(msg)
        state[key] = now


def main():
    # ---- 連到 rosbridge ----
    client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    print(f"[info] 連線 rosbridge ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT} ...")
    client.run()  # 非阻塞，背景開 websocket
    # 等待連線建立
    t_wait = time.time()
    while not client.is_connected and time.time() - t_wait < 5.0:
        time.sleep(0.1)
    if not client.is_connected:
        print("[error] 連不上 rosbridge，請確認機器人有跑 rosbridge_websocket，且 IP/port 正確")
        return
    print("[info] rosbridge 已連線")

    pub = roslibpy.Topic(client, "/target_info", "geometry_msgs/Pose")
    pub.advertise()

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
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    if ENABLE_DEPTH:
        config.enable_stream(rs.stream.depth, COLOR_W, COLOR_H, rs.format.z16, FPS)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color) if ENABLE_DEPTH else None

    _, camera_params, K_color, color_w, _ = get_color_intrinsics(profile)

    fps = 0.0
    prev_t = time.time()
    warn_state = {}
    try:
        while client.is_connected:
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                warn_throttle(warn_state, "timeout", 2.0, f"等待影像逾時: {e}")
                continue
            if ENABLE_DEPTH:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                warn_throttle(warn_state, "nocolor", 2.0, "未取得 color frame，重試中...")
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

            # 打包 /target_info（純 dict）
            if pairs:
                p = pairs[0]
                id_a, id_b = p["members"]
                msg = make_pose(ox=float(id_a), oy=float(id_b), ow=1.0)

                # 對齊第二個 tag (id_b) 的像素中心
                r_b = id_to_result.get(id_b)
                if r_b is not None:
                    cx_b = float(np.mean(r_b.corners[:, 0]))
                    msg["position"]["x"] = cx_b - (color_w / 2.0)
                else:
                    cx_p, _ = p["center"]
                    msg["position"]["x"] = float(cx_p - (color_w / 2.0))

                tl, tr = p.get("t_left"), p.get("t_right")
                if tl is not None and tr is not None:
                    msg["position"]["y"] = float(tr[2] - tl[2])

                t_comb = p.get("t")
                if t_comb is not None:
                    msg["position"]["z"] = float(np.linalg.norm(t_comb))
            else:
                msg = make_pose(ow=0.0, ox=-1.0, oy=-1.0)

            pub.publish(roslibpy.Message(msg))

            # 計算 FPS（指數平滑）
            now = time.time()
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

            cv2.imshow("RealSense Detection (rosbridge)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        try:
            pub.unadvertise()
        except Exception:
            pass
        client.terminate()


if __name__ == "__main__":
    main()
