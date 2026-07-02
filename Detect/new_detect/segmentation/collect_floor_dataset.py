#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO 地板 segmentation 訓練資料收集工具 (Floor Dataset Collector)

直接從 D435i 的 Color（RGB）串流抓「乾淨原始幀」存檔，
解析度與 ros_detect_dual.py 的 YOLO 輸入完全一致（640x480 @ 30fps），
這樣訓練資料才會跟推論時相機看到的畫面相同。

為何不用螢幕錄影：
  - cv2.imshow 顯示的是縮放+疊加文字/遮罩的畫面，不是原始幀
  - 螢幕錄影會再壓縮一次，色彩/解析度都失真
  - segmentation 標註最終要的是「靜態圖片」，不是影片

特色：
  - 每次執行 = 一個新 session 資料夾（new_detect/train_data/session_YYYYmmdd_HHMMSS/）
    方便依場景/批次辨識
  - 依固定時間間隔自動存檔（避免連續幀幾乎相同、浪費標註）
  - 也可按 s 手動補拍、按 p 暫停/繼續、按 q 離開
  - 存的是「未疊加任何標註」的原始 BGR 幀

Ubuntu RealSense 設定（首次使用前）：
  sudo apt install librealsense2-dkms librealsense2-utils -y
  pip3 install pyrealsense2 opencv-python numpy
  realsense-viewer   # 確認相機連接正常

用法：
  python3 collect_floor_dataset.py                 # 預設每 0.5s 存一張
  python3 collect_floor_dataset.py --interval 1.0  # 每 1.0s 存一張
  python3 collect_floor_dataset.py --name lab_floor # session 資料夾加備註名稱
"""

import argparse
import os
import time
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs

# ================= 參數設定 =================
# 與 ros_detect_dual.py 的 YOLO Color 輸入一致
COLOR_W = 640
COLOR_H = 480
FPS     = 30

FRAME_TIMEOUT_MS = 5000

# 預設 session 資料夾根目錄：new_detect/train_data/（本檔已移入 segmentation/，故往上一層）
TRAIN_DATA_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "train_data")
# ===========================================


def make_session_dir(root: str, note: str = "") -> str:
    """每次執行建立一個帶時間戳的 session 資料夾，方便辨識批次。"""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = f"session_{stamp}" + (f"_{note}" if note else "")
    path = os.path.join(root, folder)
    os.makedirs(path, exist_ok=True)
    return path


def parse_args():
    ap = argparse.ArgumentParser(description="D435i Color 串流地板訓練資料收集工具")
    ap.add_argument("--interval", type=float, default=0.5,
                    help="自動存檔間隔（秒），預設 0.5")
    ap.add_argument("--name", type=str, default="",
                    help="session 資料夾的備註名稱（例如場景名）")
    ap.add_argument("--ext", type=str, default="jpg", choices=["jpg", "png"],
                    help="存檔格式，預設 jpg")
    return ap.parse_args()


def main():
    args = parse_args()

    session_dir = make_session_dir(TRAIN_DATA_ROOT, args.name)
    print(f"[資料夾] 本次 session 存檔位置：{session_dir}")
    print(f"[間隔]   每 {args.interval:.2f}s 自動存一張（.{args.ext}）")
    print("[操作]   s=手動存一張  p=暫停/繼續自動存  q=離開")

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)

    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"[錯誤] 無法開啟 RealSense 相機：{e}")
        print("[提示] D435i 一次只能被『一個』程式開啟。請先關閉其他相機程式"
              "（ros_detect_*.py / ros_test_*.py / realsense-viewer）再執行。")
        return
    print(f"[相機]   RealSense Color 串流已開啟：{COLOR_W}x{COLOR_H} @ {FPS}fps")

    cv2.namedWindow("Collect Floor Dataset", cv2.WINDOW_AUTOSIZE)

    saved      = 0
    last_save  = 0.0   # 上次自動存檔時間
    auto_save  = True  # 是否啟用自動存檔

    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                print(f"[警告] 等待影像逾時：{e}")
                continue

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # 原始 BGR 幀 —— 存檔用這個，絕不疊加任何標註
            frame = np.asanyarray(color_frame.get_data())

            now = time.time()
            do_save = False

            # 自動依間隔存檔
            if auto_save and (now - last_save) >= args.interval:
                do_save = True
                last_save = now

            # 預覽畫面（疊加資訊只給人看，不影響存檔的 frame）
            vis = frame.copy()
            status = "AUTO" if auto_save else "PAUSED"
            cv2.putText(vis, f"[{status}] saved={saved}  interval={args.interval:.2f}s",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(vis, "s=save  p=pause  q=quit",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow("Collect Floor Dataset", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("p"):
                auto_save = not auto_save
                print(f"[切換] 自動存檔 = {'ON' if auto_save else 'OFF'}")
            elif key == ord("s"):
                do_save = True   # 手動補拍

            if do_save:
                fname = f"frame_{saved:05d}_{datetime.now().strftime('%H%M%S_%f')[:-3]}.{args.ext}"
                fpath = os.path.join(session_dir, fname)
                cv2.imwrite(fpath, frame)
                saved += 1
                print(f"[存檔] {fname}  (累計 {saved})")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"\n[完成] 共存 {saved} 張到：{session_dir}")


if __name__ == "__main__":
    main()
