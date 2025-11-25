import time, math, cv2, yaml, numpy as np
from pathlib import Path
from pupil_apriltags import Detector

# ========= 需要你確認的參數 =========
CALIB_FILE = "calib_result.yaml"   # 你前面輸出的校正檔
TAG_SIZE_M = 0.15                  # AprilTag 黑框邊長(公尺)；請改成你的實際尺寸
CAM_INDEX = 0                      # 攝影機索引
ALPHA = 0                          # 0:裁到有效區域；1:保留全部視角(邊緣可能有黑邊)
# ==================================

def rotationMatrixToEulerXYZ(R):
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    if sy >= 1e-6:
        x = math.degrees(math.atan2(R[2,1], R[2,2]))  # roll
        y = math.degrees(math.atan2(-R[2,0], sy))     # pitch
        z = math.degrees(math.atan2(R[1,0], R[0,0]))  # yaw
    else:  # 近奇異
        x = math.degrees(math.atan2(-R[1,2], R[1,1]))
        y = math.degrees(math.atan2(-R[2,0], sy))
        z = 0.0
    return x, y, z

def draw_axes(img, K, rvec, tvec, length=0.03):
    """ 在影像上畫 3D 軸；使用 undistorted 影像 ⇒ dist=0 """
    axis = np.float32([[0,0,0],[length,0,0],[0,length,0],[0,0,length]]).reshape(-1,3)
    dist0 = np.zeros((5,1), np.float32)
    pts, _ = cv2.projectPoints(axis, rvec, tvec, K, dist0)
    pts = pts.reshape(-1,2).astype(int)
    o, x, y, z = pts
    cv2.line(img, tuple(o), tuple(x), (0,0,255), 2)    # X red
    cv2.line(img, tuple(o), tuple(y), (0,255,0), 2)    # Y green
    cv2.line(img, tuple(o), tuple(z), (255,0,0), 2)    # Z blue

# ---------- 讀取校正檔 ----------
cfg_path = Path(CALIB_FILE)
if not cfg_path.exists():
    raise FileNotFoundError(f"找不到 {CALIB_FILE}，請先完成相機校正。")

cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
W, H = int(cfg["image_width"]), int(cfg["image_height"])
K = np.array(cfg["camera_matrix"], dtype=np.float32)
dist = np.array(cfg["distortion_coefficients"], dtype=np.float32).reshape(-1,1)

# 取得 undistort 所需的新內參與對應表
newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (W, H), ALPHA)
map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (W, H), cv2.CV_16SC2)

# 供 AprilTag 偵測使用的內參 (對應 undistorted 影像)
fx_u, fy_u, cx_u, cy_u = float(newK[0,0]), float(newK[1,1]), float(newK[0,2]), float(newK[1,2])
camera_params_rect = (fx_u, fy_u, cx_u, cy_u)

# ---------- 建立 AprilTag 偵測器 ----------
detector = Detector(
    families="tag36h11",
    nthreads=2,
    quad_decimate=1.0,    # 小/中型標籤建議 1.0；若很小可用 0.5（較慢）
    quad_sigma=0.8,
    refine_edges=True,
    decode_sharpening=0.25,
)

# ---------- 打開攝影機 ----------
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    raise RuntimeError("無法開啟攝影機")

retW, retH = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
if (retW, retH) != (W, H):
    print(f"⚠️  警告：校正是 {W}x{H}，目前攝影機是 {retW}x{retH}。請切回校正解析度或重新校正。")

print("按 q 離開。畫面已『先去畸變』再偵測與繪圖。")

# ---------- 主迴圈 ----------
t0, frames, fps = time.time(), 0, 0.0
while True:
    ok, frame = cap.read()
    if not ok:
        break

    frames += 1
    now = time.time()
    if now - t0 >= 1.0:
        fps = frames / (now - t0)
        t0, frames = now, 0

    # 先去畸變
    und = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
    gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)

    # 用 newK (rectified intrinsics) 做姿態估計
    results = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params_rect,
        tag_size=TAG_SIZE_M
    )

    # 繪製結果
    for r in results:
        corners = r.corners.astype(int)
        for i in range(4):
            cv2.line(und, tuple(corners[i]), tuple(corners[(i+1) % 4]), (0,255,0), 2)

        cX, cY = r.center.astype(int)
        cv2.circle(und, (cX, cY), 3, (0,0,255), -1)

        R = r.pose_R
        t = r.pose_t.reshape(3)
        dist_m = float(np.linalg.norm(t))
        roll, pitch, yaw = rotationMatrixToEulerXYZ(R)

        cv2.putText(und, f"id={r.tag_id} m={r.decision_margin:.1f}",
                    (cX+6, cY-24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,255), 2, cv2.LINE_AA)
        cv2.putText(und, f"tz={t[2]:.3f} m | dist={dist_m:.3f} m",
                    (cX+6, cY-6),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(und, f"rpy=({roll:.1f},{pitch:.1f},{yaw:.1f}) deg",
                    (cX+6, cY+12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)

        # 畫 3D 軸（注意：使用 newK，且 dist=0）
        rvec, _ = cv2.Rodrigues(R)
        tvec = t.reshape(3,1).astype(np.float32)
        draw_axes(und, newK.astype(np.float32), rvec, tvec, length=TAG_SIZE_M*0.5)

    cv2.putText(und, f"{W}x{H}  UNDIST  FPS:{fps:.1f}  tags:{len(results)}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    cv2.imshow("AprilTag Pose (undistorted first)", und)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
