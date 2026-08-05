#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
measure_cam.py — 用深度地面平面擬合實測「相機離地高度」與「鏡頭下傾角」

為什麼不用手機水平儀量：水平儀誤差 ±2–3°，3 m 處會讓高度換算差 15 cm，
比 H_PASS(6 cm) 大得多 —— 地面會被當成障礙，或障礙被當成地面。
這支直接從深度點雲擬合地面平面，量到的是「光學中心」，與深度資料同座標系，
所以捲尺量外殼會比它大幾 cm（2026-08-03：外殼 57.3 cm vs 擬合 54.4 cm），以擬合值為準。

量出來的兩個數字填回 ros_test_bypass.py / ros_test_ground_bypass.py 的
CAM_TILT_DEG 與 CAM_HEIGHT_M。**每次調整雲台角度或重新掛載相機都要重量。**

現場擺法（很重要，擺錯數字就沒意義）：
  - 車停在**平坦**地面，前方 1–3 m 淨空（沒有箱子、沒有斜坡、沒有路緣）
  - 畫面下半部要看得到地面，不要對著牆或草叢
  - 車不要動、不要有人在畫面正前方走動

用法：
  python3 measure_cam.py                 # 30 幀，印平均值與標準差
  python3 measure_cam.py -n 60           # 多量幾幀
  python3 measure_cam.py --max-range 3.0 # 只取 3 m 內的地面（戶外遠處深度雜訊大）

注意：D435i 同時只能被一支程式開啟 —— 跑這支之前先關掉所有 ros_detect_*/ros_test_*。
"""

import argparse
import math
import sys

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("找不到 pyrealsense2。先 conda activate wheeltec，或 pip3 install pyrealsense2")

# ================= 參數設定 =================
W, H, FPS = 640, 480, 30
WARMUP_FRAMES = 30           # 前幾幀自動曝光/深度還沒穩，丟掉
FRAME_TIMEOUT_MS = 5000

ROI_TOP_RATIO = 0.45         # 只取畫面下方這個比例以下的列（上半部多半是天空/牆）
PIXEL_STRIDE = 4             # 降採樣，夠用又快
MIN_RANGE_M = 0.35           # 太近的深度不可靠
MAX_RANGE_M = 4.0            # 太遠的深度雜訊大，會把平面拉歪

RANSAC_ITERS = 500           # 加了水平性篩選後很多候選會被直接丟掉，多抽一些
RANSAC_THRESH_M = 0.02       # 點到平面 2 cm 內算 inlier
MIN_POINTS = 500             # 有效點少於此數 → 這幀不算（多半是對著空曠處）
MIN_INLIER_RATIO = 0.35      # inlier 太少代表畫面裡不是一片平地

# 合理性範圍：擬合到的最大平面有可能是牆或桌面，不是地面
PLAUSIBLE_HEIGHT_M = (0.20, 1.50)
PLAUSIBLE_TILT_DEG = (-20.0, 60.0)
MAX_ROLL_DEG = 25.0          # 地面的側傾不該這麼大；超過代表擬到的是牆或斜面


def fit_plane_ransac(pts, rng, horizontal_only=True):
    """對 (N,3) 點雲擬合平面，回傳 (normal, d, inlier_mask)，平面為 n·p + d = 0。

    horizontal_only=True 時只考慮「法向量接近朝上」的候選平面 —— 不然車頭前
    0.7 m 有面牆時，最大的平面是牆不是地面（實測就踩到這個坑）。
    """
    n_pts = len(pts)
    best_mask, best_count = None, 0
    for _ in range(RANSAC_ITERS):
        idx = rng.choice(n_pts, 3, replace=False)
        p0, p1, p2 = pts[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(nrm)
        if norm < 1e-6:                      # 三點共線
            continue
        nrm = nrm / norm
        if horizontal_only:
            n_up = -nrm if nrm[1] > 0 else nrm          # 先朝上再判角度
            _, tilt, roll = plane_to_pose(n_up, 0.0)
            if not (PLAUSIBLE_TILT_DEG[0] <= tilt <= PLAUSIBLE_TILT_DEG[1]):
                continue
            if abs(roll) > MAX_ROLL_DEG:
                continue
        dist = np.abs(pts @ nrm - nrm @ p0)
        mask = dist < RANSAC_THRESH_M
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask
    if best_mask is None:
        return None, None, None

    # 用 inlier 做最小平方精修（SVD：質心 + 最小奇異向量 = 法向量）
    inliers = pts[best_mask]
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
    nrm = vt[-1]
    nrm = nrm / np.linalg.norm(nrm)
    d = -float(nrm @ centroid)

    # 讓法向量朝「上」：相機座標 y 軸向下，所以地面朝上的法向量 y 分量為負
    if nrm[1] > 0:
        nrm, d = -nrm, -d
    return nrm, d, best_mask


def plane_to_pose(nrm, d):
    """平面 → (離地高度 m, 下傾角 deg, 側傾/roll deg)。

    相機座標：x 向右、y 向下、z 向前。相機下傾 θ 時，
    地面朝上法向量在相機座標為 (0, -cosθ, -sinθ)，故 θ = atan2(-n_z, -n_y)。
    """
    height = abs(d)                                   # 原點(光學中心)到平面的距離
    tilt = math.degrees(math.atan2(-nrm[2], -nrm[1]))  # 下傾為正
    roll = math.degrees(math.atan2(-nrm[0], -nrm[1]))  # 右邊比左邊低為正
    return height, tilt, roll


def geometry_check(height, tilt_deg, intr, label, img_h):
    """印出這個高度/俯角下的視野幾何 —— 看得到多近的地面、3 m 處看得到多高。

    ★ 要看 **Color** 的數字：資料集影像、best.pt、VLM 看到的都是彩色畫面。
    Depth 的 FOV 比 Color 寬很多（D435i 深度 87°×58° vs 彩色 69°×43°），
    拿 depth 的 FOV 去談「1 m 處的地面拍不拍得到」會過度樂觀。
    """
    fy, cy = intr.fy, intr.ppy
    a_down = math.atan2(img_h - 1 - cy, fy)   # 畫面最下緣相對光軸的俯角
    a_up = math.atan2(cy, fy)                 # 畫面最上緣相對光軸的仰角
    vfov = math.degrees(a_down + a_up)
    tilt = math.radians(tilt_deg)

    ang_bottom = tilt + a_down                # 最下緣射線與水平面夾角
    nearest = height / math.tan(ang_bottom) if ang_bottom > 1e-3 else float('inf')
    top_at_3m = height + 3.0 * math.tan(a_up - tilt)

    print(f"  [{label}] 垂直 FOV {vfov:.1f}° (fy={fy:.1f}, cy={cy:.1f})   "
          f"地面最近可見 {nearest:.2f} m   3 m 處可見高度 {top_at_3m:.2f} m")


def main():
    ap = argparse.ArgumentParser(description="深度地面平面擬合 → 相機離地高度與下傾角")
    ap.add_argument('-n', '--frames', type=int, default=30, help='量測幀數 (預設 30)')
    ap.add_argument('--max-range', type=float, default=MAX_RANGE_M, help='採樣的最遠距離 m')
    ap.add_argument('--roi-top', type=float, default=ROI_TOP_RATIO,
                    help='只取畫面此比例以下的列 (0=全畫面, 預設 0.45)')
    ap.add_argument('--min-inlier', type=float, default=MIN_INLIER_RATIO,
                    help='inlier 比例門檻 (預設 0.35)。畫面裡地面佔比本來就小時可調低，'
                         '但要自己確認擬到的是地面')
    ap.add_argument('--any-plane', action='store_true',
                    help='除錯用：不限制平面必須接近水平，看看畫面裡最大的平面是什麼')
    args = ap.parse_args()

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)   # 只為了取彩色內參
    profile = pipeline.start(cfg)
    try:
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        fx, fy, ppx, ppy = intr.fx, intr.fy, intr.ppx, intr.ppy

        print(f"深度內參 fx={fx:.1f} fy={fy:.1f} cx={ppx:.1f} cy={ppy:.1f} "
              f"depth_scale={depth_scale}")
        print(f"暖機 {WARMUP_FRAMES} 幀…")
        for _ in range(WARMUP_FRAMES):
            pipeline.wait_for_frames(FRAME_TIMEOUT_MS)

        # 預先算好像素網格（只算一次）
        vs = np.arange(int(args.roi_top * H), H, PIXEL_STRIDE)
        us = np.arange(0, W, PIXEL_STRIDE)
        uu, vv = np.meshgrid(us, vs)
        kx = (uu - ppx) / fx
        ky = (vv - ppy) / fy

        rng = np.random.default_rng(0)
        heights, tilts, rolls, ratios = [], [], [], []

        print(f"量測 {args.frames} 幀…")
        for i in range(args.frames):
            frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            depth = frames.get_depth_frame()
            if not depth:
                continue
            z = np.asanyarray(depth.get_data())[vs][:, us].astype(np.float32) * depth_scale
            valid = (z > MIN_RANGE_M) & (z < args.max_range)
            if valid.sum() < MIN_POINTS:
                print(f"  [{i+1:2d}] 有效深度點太少 ({int(valid.sum())})，跳過 "
                      f"—— 前方是不是太空曠或太近？")
                continue

            zv = z[valid]
            pts = np.stack([kx[valid] * zv, ky[valid] * zv, zv], axis=1)
            nrm, d, mask = fit_plane_ransac(pts, rng, horizontal_only=not args.any_plane)
            if nrm is None:
                print(f"  [{i+1:2d}] 跳過：找不到接近水平的平面 "
                      f"—— 畫面下半部應該是地面，現在對到的是牆或障礙物？"
                      f"（--any-plane 可看它到底擬到什麼）")
                continue
            ratio = mask.mean()
            height, tilt, roll = plane_to_pose(nrm, d)

            # 被打回票時也把擬到的平面印出來，才知道是「地面但雜訊多」還是「根本擬到牆」
            reject = None
            if ratio < args.min_inlier:
                reject = f"inlier 只有 {ratio*100:.0f}% (<{args.min_inlier*100:.0f}%)"
            elif not (PLAUSIBLE_HEIGHT_M[0] <= height <= PLAUSIBLE_HEIGHT_M[1]):
                reject = f"高度 {height*100:.0f} cm 不合理"
            elif not (PLAUSIBLE_TILT_DEG[0] <= tilt <= PLAUSIBLE_TILT_DEG[1]):
                reject = f"下傾 {tilt:.1f}° 不合理"
            if reject:
                print(f"  [{i+1:2d}] 跳過：{reject}   "
                      f"(最大平面 = 高度 {height*100:.0f} cm / 下傾 {tilt:.1f}° / "
                      f"側傾 {roll:+.1f}°) —— 擬到的是地面還是牆？")
                continue

            heights.append(height); tilts.append(tilt)
            rolls.append(roll); ratios.append(ratio)
            print(f"  [{i+1:2d}] 高度 {height*100:5.1f} cm   下傾 {tilt:5.2f}°   "
                  f"側傾 {roll:+5.2f}°   inlier {ratio*100:3.0f}%  ({len(pts)} 點)")
    finally:
        pipeline.stop()

    if not heights:
        sys.exit("\n沒有任何一幀擬合成功。把車停到平坦地面、前方 1–3 m 淨空再試一次。")

    h = np.array(heights); t = np.array(tilts); r = np.array(rolls)
    print("\n================ 結果 ================")
    print(f"成功幀數        : {len(h)} / {args.frames}   "
          f"(平均 inlier {np.mean(ratios)*100:.0f}%)")
    print(f"相機離地高度    : {h.mean()*100:.1f} cm   (標準差 {h.std()*100:.1f} cm)")
    print(f"鏡頭下傾角      : {t.mean():.2f}°   (標準差 {t.std():.2f}°)")
    print(f"側傾 roll       : {r.mean():+.2f}°   (標準差 {r.std():.2f}°)"
          + ("   ← > 1° 代表相機沒擺正，先調平" if abs(r.mean()) > 1.0 else ""))
    if h.std() * 100 > 1.0 or t.std() > 0.3:
        print("⚠ 標準差偏大：車或雲台在晃、地面不平、或畫面裡有非地面物體。")
    print("\n---- 視野幾何檢查（用上面的平均值算） ----")
    geometry_check(h.mean(), t.mean(), color_intr, 'Color', color_intr.height)
    geometry_check(h.mean(), t.mean(), intr, 'Depth', H)
    print("  ★ 談構圖／資料集拍不拍得到，看 Color 那行（深度 FOV 比較寬，會過度樂觀）")
    print("\n---- 填回 ros_test_bypass.py / ros_test_ground_bypass.py ----")
    print(f"CAM_TILT_DEG    = {t.mean():.1f}")
    print(f"CAM_HEIGHT_M    = {h.mean():.3f}")
    print("（同一組數字也要更新 paper/三級處置標註規則.md A.2 表格）")


if __name__ == '__main__':
    main()
