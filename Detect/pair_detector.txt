import time
import math
from pathlib import Path
from typing import Tuple

import cv2
import yaml
import numpy as np
from pupil_apriltags import Detector

# 從你現有的設定檔匯入 PairDetector 與畫 label 函式
from pair_detector_setting import PairDetector, draw_pair_labels

# ------------ 可調參數 --------------
CALIB_FILE = "C:\\Users\\user\\Documents\\project\\AprilTag\\calib_result.yaml"   # 放在同目錄或以絕對路徑
TAG_SIZE_M = 0.08                 # 改成你的 tag 大小 (公尺)
CAM_INDEX = 0
ALPHA = 0.0
# ------------------------------------

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
    pts, _ = cv2.projectPoints(axis, rvec, tvec, K, dist0)
    pts = pts.reshape(-1,2).astype(int)
    o, x, y, z = pts
    cv2.line(img, tuple(o), tuple(x), (0,0,255), 2)
    cv2.line(img, tuple(o), tuple(y), (0,255,0), 2)
    cv2.line(img, tuple(o), tuple(z), (255,0,0), 2)

def load_calib(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"calib file not found: {path}")
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    K = np.array(cfg["camera_matrix"], dtype=np.float32)
    dist = np.array(cfg["distortion_coefficients"], dtype=np.float32).reshape(-1,1)
    # try common keys for image size if present
    W = cfg.get("image_width") or cfg.get("width") or cfg.get("img_w") or None
    H = cfg.get("image_height") or cfg.get("height") or cfg.get("img_h") or None
    if W is None or H is None:
        # fallback: try shape in camera_matrix or default
        W, H = 640, 480
    return int(W), int(H), K, dist

def main():
    root = Path(__file__).parent
    calib_path = root / CALIB_FILE
    try:
        W, H, K, dist = load_calib(calib_path)
    except Exception as e:
        print("Load calib failed:", e)
        return

    # undistort mapping
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (W, H), ALPHA)
    map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (W, H), cv2.CV_16SC2)
    fx_u, fy_u, cx_u, cy_u = float(newK[0,0]), float(newK[1,1]), float(newK[0,2]), float(newK[1,2])
    camera_params_rect = (fx_u, fy_u, cx_u, cy_u)

    # apriltag detector
    detector = Detector(
        families="tag36h11",
        nthreads=2,
        quad_decimate=1.0, quad_sigma=0.8,
        refine_edges=True,
        decode_sharpening=0.25,
    )

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        print("Cannot open camera", CAM_INDEX)
        return
    retW = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    retH = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (retW, retH) != (W, H):
        print(f"Warning: camera returned {(retW,retH)} but calib expects {(W,H)}")

    pd = PairDetector(history_len=6, stable_threshold=4)

    print("Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        # undistort first
        und = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)

        results = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params_rect,
            tag_size=TAG_SIZE_M
        )

        # draw single tags
        for r in results:
            corners = r.corners.astype(int)
            for k in range(4):
                cv2.line(und, tuple(corners[k]), tuple(corners[(k+1)%4]), (0,255,0), 2)
            cX, cY = r.center.astype(int)
            cv2.circle(und, (cX, cY), 3, (0,0,255), -1)
            # pose text if available
            if hasattr(r, "pose_R") and hasattr(r, "pose_t"):
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
                rvec, _ = cv2.Rodrigues(R)
                tvec = t.reshape(3,1).astype(np.float32)
                draw_axes(und, newK.astype(np.float32), rvec, tvec, length=TAG_SIZE_M*0.5)

        # get pairs from PairDetector and draw labels
        pairs = pd.update_and_detect(results)
        draw_pair_labels(und, pairs)

        cv2.imshow("pair_detector", und)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()