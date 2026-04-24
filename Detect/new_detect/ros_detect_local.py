#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本機攝影機偵測節點 (Local Camera Detection Node)

攝影機直接接在執行此程式的電腦上，不透過 ROS Image topic。
不需要 calib_result.yaml，使用近似相機參數。

WSL2 USB 攝影機設定（Windows 端執行一次）：
  1. 安裝 usbipd-win：https://github.com/dorssel/usbipd-win
  2. 列出裝置：   usbipd list
  3. 附加到 WSL2：usbipd attach --wsl --busid <BUSID>
  4. WSL2 確認：  ls /dev/video*   → 應看到 /dev/video0

/target_info (geometry_msgs/Pose) 欄位說明：
  orientation.x  — 左  tag ID
  orientation.y  — 右  tag ID
  orientation.w  — 1.0 = 偵測到, 0.0 = 未偵測到
  position.x     — 水平像素誤差 (pair 中心 - 畫面中心), 左=負 右=正
  position.y     — depth_diff = t_right.z - t_left.z (公尺)
                   正值 = 右tag較遠 = 機器人偏左 = 需右轉
  position.z     — 到 pair 中心的 3D 歐幾里得距離 (公尺)
"""

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from pupil_apriltags import Detector

from pair_detector_balance import BalancePairDetector
from pair_detector_setting import draw_axes, draw_pair_labels

# ================= 參數設定 =================
CAMERA_INDEX = 0         # 攝影機編號，通常是 0；若有多個可試 1, 2...
FRAME_W      = 640
FRAME_H      = 480
FPS          = 30

TAG_SIZE_M   = 0.08       # Tag 邊長（公尺），量實際 Tag 修改此值

# 近似相機內參（無校正時使用）
# 若要更準確，執行 calibration/calibrate_from_images.py 後改用 calib_result.yaml
APPROX_FX = 600.0         # 水平焦距（像素），可依實際視角微調
APPROX_FY = 600.0
# ===========================================


def build_camera_params(w: int, h: int, fx: float, fy: float):
    cx = w / 2.0
    cy = h / 2.0
    return (fx, fy, cx, cy)


def main():
    rospy.init_node("local_detect_node", anonymous=True)

    detector = Detector(families="tag36h11", nthreads=4)
    pd       = BalancePairDetector(history_len=6, stable_threshold=4)
    pub      = rospy.Publisher("/target_info", Pose, queue_size=1)

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS,          FPS)

    # 讀取實際解析度（可能與設定值不同）
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if not cap.isOpened():
        rospy.logerr(f"無法開啟攝影機 (index={CAMERA_INDEX})，請確認裝置已連接。")
        return

    rospy.loginfo(f"攝影機已開啟：{W}x{H} @ {FPS}fps (index={CAMERA_INDEX})")

    camera_params = build_camera_params(W, H, APPROX_FX, APPROX_FY)
    rate = rospy.Rate(FPS)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn_throttle(2.0, "攝影機讀取失敗，重試中...")
            rate.sleep()
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=TAG_SIZE_M,
        )

        # 繪製 Tag 邊框與座標軸
        K_approx = np.array([
            [APPROX_FX, 0, W / 2],
            [0, APPROX_FY, H / 2],
            [0, 0, 1],
        ], dtype=np.float32)

        for r in results:
            corners = r.corners.astype(int)
            for k in range(4):
                cv2.line(frame, tuple(corners[k]), tuple(corners[(k + 1) % 4]), (0, 255, 0), 2)
            if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
                rvec, _ = cv2.Rodrigues(r.pose_R)
                tvec = r.pose_t.reshape(3, 1).astype(np.float32)
                draw_axes(frame, K_approx, rvec, tvec, length=TAG_SIZE_M * 0.5)

        # Pair 偵測
        pairs = pd.update_and_detect(results)
        draw_pair_labels(frame, pairs)

        # 畫面顯示 depth_diff
        for p in pairs:
            tl, tr = p.get("t_left"), p.get("t_right")
            if tl is not None and tr is not None:
                diff   = tr[2] - tl[2]
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
            p    = pairs[0]
            id_a, id_b = p["members"]
            msg.orientation.x = float(id_a)
            msg.orientation.y = float(id_b)
            msg.orientation.w = 1.0

            # 對齊第二個 tag (id_b) 的像素中心
            r_b = id_to_result.get(id_b)
            if r_b is not None:
                cx_b = float(np.mean(r_b.corners[:, 0]))
                msg.position.x = cx_b - W / 2
            else:
                cx_p, _ = p["center"]
                msg.position.x = float(cx_p - W / 2)

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

        cv2.imshow("Local Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
