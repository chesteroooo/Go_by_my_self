#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import yaml
import numpy as np
import math
from pathlib import Path

# 使用 Pose 來傳遞資訊
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose

# 引入 AprilTag 與 Pair 相關工具
from pupil_apriltags import Detector
from pair_detector_setting import PairDetector, draw_pair_labels, draw_axes

# ================= 參數設定 =================
CALIB_FILE = "calib_result.yaml"
TAG_SIZE_M = 0.08  # Tag 的邊長 (公尺)
ALPHA = 0.0        # OpenCV 校正裁切係數
# ===========================================

class RemotePerception:
    def __init__(self):
        rospy.init_node('remote_perception_node', anonymous=True)
        self.init_calibration()
        self.detector = Detector(families="tag36h11")
        self.pd = PairDetector(history_len=6, stable_threshold=4)

        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw/compressed",  
            CompressedImage, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        self.target_pub = rospy.Publisher('/target_info', Pose, queue_size=1)

        rospy.loginfo("感知節點啟動！(X=純水平誤差, Z=3D直線距離)")

    def init_calibration(self):
        root = Path(__file__).parent
        calib_path = root / CALIB_FILE
        if not calib_path.exists():
            rospy.logwarn(f"找不到校正檔: {calib_path}，使用預設參數")
            self.W, self.H = 640, 480
            self.K = np.array([[600,0,320],[0,600,240],[0,0,1]], dtype=np.float32)
            self.dist = np.zeros((5,1), dtype=np.float32)
        else:
            cfg = yaml.safe_load(calib_path.read_text(encoding="utf-8"))
            self.K = np.array(cfg["camera_matrix"], dtype=np.float32)
            self.dist = np.array(cfg["distortion_coefficients"], dtype=np.float32).reshape(-1,1)
            self.W = cfg.get("image_width", 640)
            self.H = cfg.get("image_height", 480)

        self.newK, _ = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (self.W, self.H), ALPHA)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.K, self.dist, None, self.newK, (self.W, self.H), cv2.CV_16SC2)
        fx_u, fy_u, cx_u, cy_u = float(self.newK[0,0]), float(self.newK[1,1]), float(self.newK[0,2]), float(self.newK[1,2])
        self.camera_params_rect = (fx_u, fy_u, cx_u, cy_u)

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception:
            return

        und = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
        
        # 1. 偵測
        results = self.detector.detect(gray, estimate_tag_pose=True, camera_params=self.camera_params_rect, tag_size=TAG_SIZE_M)
        
        # 繪圖
        for r in results:
            corners = r.corners.astype(int)
            for k in range(4):
                cv2.line(und, tuple(corners[k]), tuple(corners[(k+1)%4]), (0,255,0), 2)
            if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
                R = r.pose_R
                t = r.pose_t.reshape(3)
                rvec, _ = cv2.Rodrigues(R)
                tvec = t.reshape(3,1).astype(np.float32)
                draw_axes(und, self.newK.astype(np.float32), rvec, tvec, length=TAG_SIZE_M*0.5)

        # 2. Pair 邏輯
        pairs = self.pd.update_and_detect(results)
        draw_pair_labels(und, pairs)
        
        # 3. 打包數據
        target_msg = Pose()
        
        if len(pairs) > 0:
            p = pairs[0]
            id_1, id_2 = p["members"]
            
            # --- (A) 身分區 ---
            target_msg.orientation.x = float(id_1)
            target_msg.orientation.y = float(id_2)
            target_msg.orientation.w = 1.0
            
            # --- (B) 位置與距離計算 ---
            
            # [1. 純水平橫向誤差]
            # cx: Tag 中心的 X 像素座標
            # cx - img_center_x:
            #   Tag 在左 (例如 100) -> 100 - 320 = -220 (負數)
            #   Tag 在右 (例如 500) -> 500 - 320 = +180 (正數)
            cx, _ = p["center"]  
            img_center_x = self.W / 2
            target_msg.position.x = float(cx - img_center_x)
            
            # [2. 3D 直線距離 (包含深度)]
            # 使用 Norm (歐幾里得距離)，這代表鏡頭到 Tag 的真實連線距離
            t_vec = p.get("t")
            if t_vec is not None:
                # 恢復使用 norm，這會計算 (x^2 + y^2 + z^2) 開根號
                target_msg.position.z = float(np.linalg.norm(t_vec))
            
        else:
            target_msg.orientation.w = 0.0
            target_msg.orientation.x = -1.0
            target_msg.orientation.y = -1.0
            target_msg.position.x = 0.0
            target_msg.position.z = 0.0

        self.target_pub.publish(target_msg)

        cv2.imshow("Perception View", und)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("Quit")

if __name__ == "__main__":
    try:
        RemotePerception()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()