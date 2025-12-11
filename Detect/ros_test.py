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
ALPHA = 0.0        # OpenCV 校正裁切係數 (0=裁掉黑邊, 1=保留所有像素)
# ===========================================

class RemotePerception:
    def __init__(self):
        # 1. 初始化 ROS 節點
        rospy.init_node('remote_perception_node', anonymous=True)
        
        # 2. 載入校正參數
        self.init_calibration()
        
        # 3. 初始化偵測器
        self.detector = Detector(families="tag36h11")
        self.pd = PairDetector(history_len=6, stable_threshold=4)

        # 4. 訂閱影像 (請確認 Topic 名稱是否正確)
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw/compressed",  
            CompressedImage, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # 5. 發布目標資訊 (Topic: /target_info)
        self.target_pub = rospy.Publisher('/target_info', Pose, queue_size=1)

        rospy.loginfo("感知節點啟動！模式: Pair Detection (Clean Protocol)")

    def init_calibration(self):
        """讀取 yaml 校正檔並設定 OpenCV remap"""
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

        # 計算 Undistort Maps (只算一次以節省效能)
        self.newK, _ = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (self.W, self.H), ALPHA)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.K, self.dist, None, self.newK, (self.W, self.H), cv2.CV_16SC2)
        
        # AprilTag 需要的相機參數 (fx, fy, cx, cy)
        fx_u, fy_u, cx_u, cy_u = float(self.newK[0,0]), float(self.newK[1,1]), float(self.newK[0,2]), float(self.newK[1,2])
        self.camera_params_rect = (fx_u, fy_u, cx_u, cy_u)

    def image_callback(self, msg):
        try:
            # === 1. 解碼影像 ===
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"影像解碼失敗: {e}")
            return

        # === 2. 影像校正 (Undistort) ===
        und = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
        
        # === 3. 偵測單張 Tag ===
        results = self.detector.detect(gray, estimate_tag_pose=True, camera_params=self.camera_params_rect, tag_size=TAG_SIZE_M)
        
        # 繪製單張 Tag 的框線與座標軸
        for r in results:
            corners = r.corners.astype(int)
            for k in range(4):
                cv2.line(und, tuple(corners[k]), tuple(corners[(k+1)%4]), (0,255,0), 2)
            
            # 畫紅綠藍座標軸 (需使用 pair_detector_setting 補上的 draw_axes)
            if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
                R = r.pose_R
                t = r.pose_t.reshape(3)
                rvec, _ = cv2.Rodrigues(R)
                tvec = t.reshape(3,1).astype(np.float32)
                draw_axes(und, self.newK.astype(np.float32), rvec, tvec, length=TAG_SIZE_M*0.5)

        # === 4. 偵測 Pair (成對標籤) ===
        pairs = self.pd.update_and_detect(results)
        draw_pair_labels(und, pairs)
        
        # === 5. 打包數據 (使用 Clean Protocol) ===
        target_msg = Pose()
        
        if len(pairs) > 0:
            # 取第一組 Pair 作為追蹤目標
            p = pairs[0]
            id_1, id_2 = p["members"]
            
            # --- (A) 身分區 (Orientation) ---
            target_msg.orientation.x = float(id_1)  # ID 1
            target_msg.orientation.y = float(id_2)  # ID 2
            target_msg.orientation.w = 1.0          # 有效旗標 (Valid)
            
            # --- (B) 控制區 (Position) ---
            # 計算橫向誤差 (像素): 畫面中心 - Pair中心
            cx, cy = p["center"]
            img_center_x = self.W / 2
            target_msg.position.x = float(img_center_x - cx)
            
            # 計算直線距離 (公尺)
            t_vec = p.get("t")
            if t_vec is not None:
                # 使用 3D 距離 (Norm)
                target_msg.position.z = float(np.linalg.norm(t_vec))
            
            # (選用) 在終端機印出資訊方便除錯
            # print(f"Pair: {id_1}_{id_2} | Dist: {target_msg.position.z:.2f}m")

        else:
            # 未偵測到目標：旗標設為 0
            target_msg.orientation.w = 0.0
            
            # 將其他數值設為無效值 (便於除錯)
            target_msg.orientation.x = -1.0
            target_msg.orientation.y = -1.0
            target_msg.position.x = 0.0
            target_msg.position.z = 0.0

        # === 6. 發送與顯示 ===
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