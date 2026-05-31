import glob
import cv2
import yaml
import numpy as np
from pathlib import Path

# ====== 依你的棋盤設定 ======
# 7x10 "格子" => 內角點為 (9,6)
PATTERN_SIZE = (9, 6)        # (cols, rows) = 內角點數
SQUARE_SIZE_M = 0.010        # 單格邊長(公尺)。只影響外參尺度，可先填 0.02。
_ROOT = Path(__file__).parent.parent
IMAGES_GLOB = str(_ROOT / "calib_images" / "*.png")   # 你的影像路徑樣式；也可用 *.jpg

# ====== 找角點設定 ======
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

# ====== 蒐集 3D/2D 對應點 ======
objp = np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_M

objpoints = []   # 3D (棋盤座標)
imgpoints = []   # 2D (影像像素)
img_size = None

images = sorted(glob.glob(IMAGES_GLOB))
if not images:
    raise SystemExit(f"找不到影像：{IMAGES_GLOB}")

print(f"找到 {len(images)} 張影像，開始偵測角點…")

good = 0
for fn in images:
    img = cv2.imread(fn, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"讀取失敗：{fn}")
        continue

    if img_size is None:
        img_size = (img.shape[1], img.shape[0])  # (w, h)

    ret, corners = cv2.findChessboardCorners(img, PATTERN_SIZE, flags)
    if ret:
        # 亞像素化角點
        corners = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), CRITERIA)
        objpoints.append(objp.copy())
        imgpoints.append(corners)
        good += 1
    else:
        print(f"沒有偵測到內角點：{fn}")

print(f"成功偵測角點的影像：{good} / {len(images)}")
if good < 8:
    raise SystemExit("樣本太少（建議 15~25 張、角度距離多樣），請多拍幾張再試。")

# ====== 校正 ======
print("開始校正…")
rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_size, None, None
)

print("\n=== 校正結果 ===")
print("RMS reprojection error:", rms)
print("K (camera matrix):\n", K)
print("distortion coefficients:\n", dist.ravel())

# ====== 計算平均重投影誤差（可選） ======
tot_err = 0
tot_pts = 0
for i in range(len(objpoints)):
    proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
    err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
    tot_err += err**2
    tot_pts += len(objpoints[i])
mean_err = np.sqrt(tot_err / tot_pts)
print("Mean reprojection error (px):", float(mean_err))

# ====== 儲存結果（YAML） ======
out = {
    "image_width": img_size[0],
    "image_height": img_size[1],
    "camera_matrix": K.tolist(),
    "distortion_coefficients": dist.ravel().tolist(),
    "fx_fy_cx_cy": [float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])],
    "rms": float(rms),
    "mean_reprojection_error": float(mean_err),
    "pattern_size_inner_corners": [PATTERN_SIZE[0], PATTERN_SIZE[1]],
    "square_size_m": float(SQUARE_SIZE_M),
}
_out_path = _ROOT / "Detect" / "old_detect" / "calib_result.yaml"
_out_path.write_text(yaml.dump(out, sort_keys=False), encoding="utf-8")
print(f"已儲存：{_out_path}")

# ====== Undistort 示範 ======
# 取一張原始影像示範校正前/後
demo_img_path = images[0]
demo = cv2.imread(demo_img_path)
h, w = demo.shape[:2]

# 方法A：cv2.undistort（簡單）
newK_A, roiA = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0)
und_A = cv2.undistort(demo, K, dist, None, newK_A)

# 方法B：remap（大量即時影像建議這個，先建立map，加速）
newK_B, roiB = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0)
map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK_B, (w, h), cv2.CV_16SC2)
und_B = cv2.remap(demo, map1, map2, cv2.INTER_LINEAR)

cv2.imwrite("undistort_A.png", und_A)
cv2.imwrite("undistort_B.png", und_B)
print("已輸出範例：undistort_A.png、undistort_B.png")
print("完成。")
