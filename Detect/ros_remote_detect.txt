#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import yaml
import numpy as np
import math
from pathlib import Path
from typing import Tuple

# ROS 訊息格式
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

# AprilTag 偵測器
from pupil_apriltags import Detector

# 從你現有的設定檔匯入 PairDetector 與畫 label 函式
from pair_detector_setting import PairDetector, draw_pair_labels

# ================= 參數設定 =================
# ⚠️ 請將 calib_result.yaml 複製到與此程式同一層資料夾
CALIB_FILE = "calib_result.yaml"
TAG_SIZE_M = 0.08
ALPHA = 0.0

# 控制參數 (若要讓電腦控制車子)
TARGET_DIST = 1.0
# ===========================================

def rotationMatrixToEulerXYZ(R: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.degrees(math.atan2(R[2,1], R[2,2]))
        y = math.degrees(math.atan2(-R[2,0], sy))
        z = math.degrees(math.atan2(R[1,0], R[0,0]))
    else:
        x = math.degrees(math.atan2(-R[1,2], R[1,1]))
        y = math.degrees(math.atan2(-R[2,0], sy))
        z = 0.0
    return x, y, z

def draw_axes(img, K, rvec, tvec, length=0.03):
    axis = np.float32([[0,0,0],[length,0,0],[0,length,0],[0,0,length]]).reshape(-1,3)
    dist0 = np.zeros((5,1), np.float32)
    try:
        pts, _ = cv2.projectPoints(axis, rvec, tvec, K, dist0)
        pts = pts.reshape(-1,2).astype(int)
        o, x, y, z = pts
        cv2.line(img, tuple(o), tuple(x), (0,0,255), 2)
        cv2.line(img, tuple(o), tuple(y), (0,255,0), 2)
        cv2.line(img, tuple(o), tuple(z), (255,0,0), 2)
    except:
        pass

class RemoteTagDetector:
    def __init__(self):
        # 1. 初始化 ROS 節點
        rospy.init_node('remote_tag_detector', anonymous=True)
        
        # 2. 載入校正與初始化參數
        self.init_calibration()
        
        # 3. 初始化偵測器
        self.detector = Detector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=1.0, quad_sigma=0.8,
            refine_edges=True,
            decode_sharpening=0.25,
        )
        self.pd = PairDetector(history_len=6, stable_threshold=4)

        # 4. 訂閱壓縮影像 (關鍵！使用 CompressedImage 以降低頻寬需求)
        # 如果你的 Topic 名稱不同，請修改這裡 (例如 /usb_cam/image_raw/compressed)
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw/compressed", 
            CompressedImage, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24 # 增加 buffer 防止影像撕裂
        )
        
        # 5. (選用) 發布控制指令給車子
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.loginfo("遠端偵測程式啟動！等待影像中...")

    def init_calibration(self):
        # 自動抓取當前檔案位置
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
            self.W = cfg.get("image_width") or 640
            self.H = cfg.get("image_height") or 480
            rospy.loginfo(f"校正檔載入成功: {self.W}x{self.H}")

        # 預先計算 undistort map (只算一次，節省 CPU)
        self.newK, _ = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (self.W, self.H), ALPHA)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.K, self.dist, None, self.newK, (self.W, self.H), cv2.CV_16SC2)
        
        fx_u, fy_u, cx_u, cy_u = float(self.newK[0,0]), float(self.newK[1,1]), float(self.newK[0,2]), float(self.newK[1,2])
        self.camera_params_rect = (fx_u, fy_u, cx_u, cy_u)

    def image_callback(self, msg):
        print("收到圖了！")
        try:
            # === 關鍵解碼步驟 ===
            # 將 ROS 壓縮訊息轉為 numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # 使用 OpenCV 解碼
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"解碼失敗: {e}")
            return

        # 1. 影像校正 (Undistort)
        und = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)

        # 2. AprilTag 偵測
        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params_rect,
            tag_size=TAG_SIZE_M
        )

        # 3. 繪製結果
        for r in results:
            corners = r.corners.astype(int)
            for k in range(4):
                cv2.line(und, tuple(corners[k]), tuple(corners[(k+1)%4]), (0,255,0), 2)
            
            # 繪製座標軸
            if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
                R = r.pose_R
                t = r.pose_t.reshape(3)
                rvec, _ = cv2.Rodrigues(R)
                tvec = t.reshape(3,1).astype(np.float32)
                draw_axes(und, self.newK.astype(np.float32), rvec, tvec, length=TAG_SIZE_M*0.5)

        # 4. Pair Detector 邏輯
        pairs = self.pd.update_and_detect(results)
        draw_pair_labels(und, pairs)

        # 5. 顯示畫面 (在電腦上執行，所以可以看到視窗！)
        cv2.imshow("Remote AprilTag View", und)
        
        # 6. 按 'q' 離開
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User pressed q")

if __name__ == "__main__":
    try:
        node = RemoteTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()