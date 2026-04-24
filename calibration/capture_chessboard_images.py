import cv2, os, time
import numpy as np
from pathlib import Path

# 你的 7x10 格棋盤 ⇒ 內角點 9x6
CHESSBOARD = (9, 6)  # (cols, rows) = inner corners
SAVE_DIR = str(Path(__file__).parent.parent / "calib_images")
os.makedirs(SAVE_DIR, exist_ok=True)

# 建議用與之後使用相同解析度（例如 1280x720）
W, H = 1280, 720

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

print("按 s 儲存(需偵測到內角點)，q 離開")
count = 0

while True:
    ok, frame = cap.read()
    if not ok:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 偵測 9x6 內角點
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD, flags)

    view = frame.copy()
    if ret:
        # 亞像素化角點 + 繪製可視化
        corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), CRITERIA)
        cv2.drawChessboardCorners(view, CHESSBOARD, corners_refined, ret)

    # HUD
    cv2.putText(view, f"Detected: {bool(ret)}   Saved: {count}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    cv2.putText(view, "s=save, q=quit",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,255), 2)

    cv2.imshow("Capture calibration images", view)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
    if k == ord('s'):
        if ret:
            # 存原始彩色影像（不要存灰階或去畸變後）
            path = os.path.join(SAVE_DIR, f"img_{int(time.time()*1000)}.png")
            cv2.imwrite(path, frame)
            count += 1
            print(f"Saved: {path}")
        else:
            print("尚未偵測到 9x6 內角點，調整距離/角度/光線再按 s")

cap.release()
cv2.destroyAllWindows()
print(f"圖片以儲存到資料夾：{SAVE_DIR}")
print(f"總計存檔：{count} 張，接著執行你的 calibrate_from_images.py")
