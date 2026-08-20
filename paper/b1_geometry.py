#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
B1 — 深度 + 幾何規則基線（論文 E2 三配置的最底層）

B1/B2/B3 是**嚴格巢狀**的：B2 = B1 + COCO 查表、B3 = B1 + VLM。
所以這支算出來的「走廊內最高障礙點離地多高」是三個配置共用的輸入，
寫錯的話三個結果會一起錯，而且從主表上看不出來。

B1 的輸出空間**只有 L0 / L1，永遠推不出 L2** —— 這不是實作偷懶，是本文的核心論證：
高度量測回答得了「要不要繞」，回答不了「該不該停車通報」。
所以 B1 的錯誤集合恆等於「所有 L2 物件」（延長線、碎陶片、人）。

--------------------------------------------------------------------------
判定規則（對應 paper/三級處置標註規則.md 的決策樹）
    走廊內沒有高於地面的東西          → L0
    有，但最高點 ≤ H_PASS             → L0（輾得過去）
    有，且最高點 > H_PASS             → L1（要繞）
--------------------------------------------------------------------------

平台參數一律取自 A.2 的**實測值**，不使用 ros_test_*.py 裡的調參：
  W_CORRIDOR = 0.84 m   車寬 64 cm 實測 + 左右各 10 cm
  H_PASS     = 0.06 m   底盤最低點離地 12 cm × 0.5
（`ros_test_bypass.py` 的 CORRIDOR_HALF_W=0.25 是舊的測試值，它的註解假設車寬 0.35 m，
  與實測的 64 cm 差很多，拿來當論文基線會被質疑基線沒調好。）

**地面平面每張自己擬合，不信任固定外參。** 雲台被撞歪、地面有坡度都會自動吸收，
論文可寫「地面平面由每幀深度估計，不依賴人工標定外參」。作法與 measure_cam.py 相同。

用法：
    python3 paper/b1_geometry.py                      # 跑所有 session
    python3 paper/b1_geometry.py --session <路徑>     # 只跑一個
    python3 paper/b1_geometry.py --h-pass 0.08        # 換高度門檻
    python3 paper/b1_geometry.py --sweep              # 掃描門檻（論文要寫「取最佳值」）
    python3 paper/b1_geometry.py --debug <影像檔名>   # 印單張的細節

輸出：每個 session 底下的 b1_result.csv
"""

import argparse
import glob
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("需要 opencv-python：pip3 install opencv-python")

# ================= 平台參數（來自標註規則 A.2 的實測值）=================
# 走廊 = **可行駛路面寬度**，不是車輛掃掠寬度。2026-08-04 由空景 12 張實測：
#   前方 2 m 處路面寬 184 cm、3 m 處 204 cm，左側草地邊界穩定落在 +80～+86 cm。
#   （1 m 處量到 120 cm 是相機水平 FOV 的上限，不是路真的變窄。）
# 取 ±0.9 m 作為對稱走廊：左到草地邊界、右仍在鋪面上。
#
# 為什麼不用「車寬 + 餘裕」：本車的導航是**分割路面 + 車道置中**
# （`ros_detect_dual.py`），不是沿固定直線前進。物體只要在它要走的路面上，
# 就是必須處置的障礙 —— 「這次直走剛好不會撞到」不等於可以忽略。
W_CORRIDOR   = 1.80      # 行駛走廊寬度 (m) — 實測可行駛路面寬
H_PASS       = 0.06      # 可輾過的高度上限 (m) — 底盤最低點 0.12 × 0.5
# =======================================================================

# ---- 幾何取樣 ----
PIXEL_STRIDE = 2         # 深度圖降採樣；2 已足夠且能保住細小物體（延長線）
MIN_RANGE_M  = 0.35      # 太近的深度不可靠
MAX_RANGE_M  = 4.0       # 太遠雜訊大；拍攝最遠檔位是 3 m

# ---- 地面平面 RANSAC（與 measure_cam.py 同一套）----
ROI_TOP_RATIO   = 0.40   # 只用畫面下方擬合地面，上半部多是天空/建物
RANSAC_ITERS    = 400
RANSAC_THRESH_M = 0.02
MIN_PLANE_PTS   = 500
MIN_INLIER_RATIO = 0.30
PLAUSIBLE_HEIGHT_M = (0.30, 0.90)   # 相機離地；擬到牆或桌面時會落在這之外
MAX_ROLL_DEG       = 25.0

# ---- 障礙判定 ----
H_NOISE     = 0.03       # 低於此高度視為地面雜訊（RANSAC 容差 2cm + 邊際）
MIN_OBS_PTS = 40         # 走廊內高於 H_NOISE 的點少於此數 → 視為沒有障礙
H_PCTL      = 95         # 取障礙高度的百分位數，避免單一雜訊點灌高
Z_PCTL      = 10         # 取障礙距離的百分位數，代表「最近的那部分」

LEVELS = ("L0", "L1", "L2")


# --------------------------------------------------------------------------
# 地面平面
# --------------------------------------------------------------------------
def plane_to_pose(nrm):
    """平面法向量 →(下傾角, 側傾角) deg。相機座標 x右 y下 z前。"""
    tilt = math.degrees(math.atan2(-nrm[2], -nrm[1]))
    roll = math.degrees(math.atan2(-nrm[0], -nrm[1]))
    return tilt, roll


def fit_ground_plane(pts, rng):
    """RANSAC 擬合地面平面，回傳 (normal, d, inlier_ratio)；平面為 n·p + d = 0。

    只接受「接近水平」的候選 —— 車頭前有牆或有大箱子時，畫面裡最大的平面不是地面。
    """
    n = len(pts)
    best_mask, best_count = None, 0
    for _ in range(RANSAC_ITERS):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[idx]
        nv = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(nv)
        if norm < 1e-6:
            continue
        nv = nv / norm
        n_up = -nv if nv[1] > 0 else nv
        tilt, roll = plane_to_pose(n_up)
        if not (-20.0 <= tilt <= 60.0) or abs(roll) > MAX_ROLL_DEG:
            continue
        mask = np.abs(pts @ nv - nv @ p0) < RANSAC_THRESH_M
        c = int(mask.sum())
        if c > best_count:
            best_count, best_mask = c, mask
    if best_mask is None:
        return None, None, 0.0

    inl = pts[best_mask]
    centroid = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - centroid, full_matrices=False)
    nrm = vt[-1] / np.linalg.norm(vt[-1])
    d = -float(nrm @ centroid)
    if nrm[1] > 0:                      # 讓法向量朝上（相機 y 軸向下）
        nrm, d = -nrm, -d
    return nrm, d, float(best_mask.mean())


def ground_frame(nrm):
    """由地面法向量建立「車體座標」：x=橫向, y=離地高度, z=前進距離。

    前進軸 = 相機光軸投影到地面後正規化；橫向 = 前進 × 法向量。
    這樣俯角與側傾都被自動吸收，不必再用固定外參補償。
    """
    up = nrm                                        # 朝上
    fwd = np.array([0.0, 0.0, 1.0]) - np.dot([0.0, 0.0, 1.0], up) * up
    fwd = fwd / np.linalg.norm(fwd)
    lat = np.cross(up, fwd)
    lat = lat / np.linalg.norm(lat)
    return lat, up, fwd


# --------------------------------------------------------------------------
# 單張影像
# --------------------------------------------------------------------------
def analyse(depth_path, intr, rng):
    """回傳 dict：地面擬合結果 + 走廊內最高障礙。失敗時 ok=False。"""
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    if depth is None:
        return {"ok": False, "why": "深度圖讀不到"}

    H, W = depth.shape[:2]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]
    scale = intr["depth_scale_m_per_unit"]

    vs = np.arange(0, H, PIXEL_STRIDE)
    us = np.arange(0, W, PIXEL_STRIDE)
    uu, vv = np.meshgrid(us, vs)
    z = depth[vs][:, us].astype(np.float32) * scale
    valid = (z > MIN_RANGE_M) & (z < MAX_RANGE_M)
    if valid.sum() < MIN_PLANE_PTS:
        return {"ok": False, "why": f"有效深度點太少 ({int(valid.sum())})"}

    zv = z[valid]
    pts = np.stack([(uu[valid] - cx) / fx * zv,
                    (vv[valid] - cy) / fy * zv,
                    zv], axis=1)

    # 地面只用畫面下方擬合，避免把建物立面或樹冠當成平面
    ground_rows = vv[valid] >= ROI_TOP_RATIO * H
    if ground_rows.sum() < MIN_PLANE_PTS:
        return {"ok": False, "why": "下半部有效點太少"}
    nrm, d, ratio = fit_ground_plane(pts[ground_rows], rng)
    if nrm is None or ratio < MIN_INLIER_RATIO:
        return {"ok": False, "why": f"地面擬合失敗 (inlier {ratio:.2f})"}

    cam_h = abs(d)
    tilt, roll = plane_to_pose(nrm)
    if not (PLAUSIBLE_HEIGHT_M[0] <= cam_h <= PLAUSIBLE_HEIGHT_M[1]):
        return {"ok": False, "why": f"擬到的平面離地 {cam_h*100:.0f}cm，不像地面"}

    lat, up, fwd = ground_frame(nrm)
    h_all = pts @ up + d          # 離地高度（平面上方為正）
    x_all = pts @ lat             # 橫向（相機在車體中線上）
    z_all = pts @ fwd             # 前進距離

    in_corridor = (np.abs(x_all) <= W_CORRIDOR / 2) & (z_all > MIN_RANGE_M) & (z_all < MAX_RANGE_M)
    obs = in_corridor & (h_all > H_NOISE)
    n_obs = int(obs.sum())

    out = {"ok": True, "why": "", "cam_h": cam_h, "tilt": tilt, "roll": roll,
           "inlier": ratio, "n_obs": n_obs}
    if n_obs < MIN_OBS_PTS:
        out.update({"obs_h": 0.0, "obs_z": float("nan")})
    else:
        out.update({"obs_h": float(np.percentile(h_all[obs], H_PCTL)),
                    "obs_z": float(np.percentile(z_all[obs], Z_PCTL))})
    return out


def b1_level(r, h_pass):
    """B1 的判定。注意它的輸出空間裡沒有 L2 —— 這是本文要證明的限制，不是 bug。"""
    if not r["ok"]:
        return ""
    if r["n_obs"] < MIN_OBS_PTS:
        return "L0"
    return "L1" if r["obs_h"] > h_pass else "L0"


# --------------------------------------------------------------------------
def run_session(sess, rng, h_pass, debug=None):
    import csv
    meta = os.path.join(sess, "metadata.csv")
    intr_p = os.path.join(sess, "intrinsics.json")
    if not (os.path.exists(meta) and os.path.exists(intr_p)):
        return []
    intr = json.load(open(intr_p))
    rows = list(csv.DictReader(open(meta, encoding="utf-8")))
    if not rows:
        return []

    print(f"\n=== {os.path.basename(sess)} — {len(rows)} 張 ===")
    results = []
    for r in rows:
        if debug and debug not in r["filename_color"]:
            continue
        a = analyse(os.path.join(sess, "depth", r["filename_depth"]), intr, rng)
        lvl = b1_level(a, h_pass)
        rec = {
            "session": os.path.basename(sess), "seq": r["seq"],
            "filename_color": r["filename_color"], "pair_id": r["pair_id"],
            "object": r["object"], "expected_level": r["expected_level"],
            "distance_m": r["distance_m"], "lateral": r["lateral"],
            "is_seen_class": r["is_seen_class"],
            "fit_ok": int(a["ok"]), "fit_note": a["why"],
            "cam_height_m": round(a.get("cam_h", float("nan")), 4),
            "tilt_deg": round(a.get("tilt", float("nan")), 2),
            "inlier_ratio": round(a.get("inlier", 0.0), 3),
            "obstacle_h_m": round(a.get("obs_h", float("nan")), 4),
            "obstacle_z_m": round(a.get("obs_z", float("nan")), 3),
            "n_obstacle_pts": a.get("n_obs", 0),
            "b1_level": lvl,
        }
        results.append(rec)
        if debug:
            print(json.dumps(rec, ensure_ascii=False, indent=2))

    if not debug and results:
        out = os.path.join(sess, "b1_result.csv")
        with open(out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            w.writeheader(); w.writerows(results)
        print(f"→ {out}")
    return results


def summarise(res, h_pass):
    """逐物件印出 B1 判定 vs 預期等級 —— 主表的雛形。"""
    ok = [r for r in res if r["fit_ok"]]
    print(f"\n================ B1 結果（H_PASS = {h_pass*100:.0f} cm）================")
    print(f"地面擬合成功 {len(ok)}/{len(res)} 張")
    bad = [r for r in res if not r["fit_ok"]]
    if bad:
        print("  擬合失敗的：")
        for r in bad[:8]:
            print(f"    {r['filename_color']}  —— {r['fit_note']}")

    by = defaultdict(list)
    for r in ok:
        by[(r["pair_id"], r["object"], r["expected_level"])].append(r)

    print(f"\n{'物件':<22}{'預期':<6}{'B1 判定':<18}{'最高點(cm)':>12}   對?")
    print("-" * 78)
    n_right = n_tot = 0
    for (pair, obj, exp), rs in by.items():
        cnt = defaultdict(int)
        for r in rs:
            cnt[r["b1_level"]] += 1
        got = " ".join(f"{k}×{v}" for k, v in sorted(cnt.items()))
        hs = [r["obstacle_h_m"] for r in rs if r["n_obstacle_pts"] >= MIN_OBS_PTS]
        hstr = f"{np.mean(hs)*100:8.1f}" if hs else "       —"
        right = sum(1 for r in rs if r["b1_level"] == exp)
        n_right += right; n_tot += len(rs)
        flag = "✓" if right == len(rs) else ("✗" if right == 0 else "△")
        print(f"{pair+'/'+obj:<22}{exp:<6}{got:<18}{hstr:>12}   {flag} {right}/{len(rs)}")
    print("-" * 78)
    print(f"對照 expected_level 的正確率：{n_right}/{n_tot} = {n_right/max(n_tot,1)*100:.1f}%")
    print("\n※ expected_level 是拍攝時的預期，不是最終 ground truth。"
          "\n  正式數字要等 paper/make_label_sheets.py 標註完成後再算。"
          "\n※ B1 永遠不會輸出 L2 —— 所有 L2 物件必然答錯，這正是本文要呈現的限制。")


def sweep(res):
    """掃描 H_PASS，論文要寫明「B1 的門檻在本資料集上取最佳值」。"""
    ok = [r for r in res if r["fit_ok"]]
    print("\n================ H_PASS 掃描 ================")
    print(f"{'H_PASS(cm)':>11}{'正確率':>10}")
    best = (0, -1)
    for h_cm in range(2, 41, 2):
        h = h_cm / 100.0
        n = sum(1 for r in ok if b1_level(
            {"ok": True, "n_obs": r["n_obstacle_pts"], "obs_h": r["obstacle_h_m"]}, h
        ) == r["expected_level"])
        acc = n / max(len(ok), 1)
        if acc > best[1]:
            best = (h_cm, acc)
        print(f"{h_cm:>11}{acc*100:>9.1f}%")
    print(f"\n最佳門檻 = {best[0]} cm（正確率 {best[1]*100:.1f}%）"
          f"；A.2 的實測值是 {H_PASS*100:.0f} cm")


def main():
    ap = argparse.ArgumentParser(description="B1 — 深度 + 幾何規則基線")
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "paper_data")
    ap.add_argument("--data", default=os.path.normpath(root), help="paper_data 目錄")
    ap.add_argument("--session", default="", help="只跑指定的 session 路徑")
    ap.add_argument("--h-pass", type=float, default=H_PASS, help=f"高度門檻 m (預設 {H_PASS})")
    ap.add_argument("--sweep", action="store_true", help="掃描 H_PASS 取最佳值")
    ap.add_argument("--debug", default="", help="只跑檔名含此字串的影像並印細節")
    args = ap.parse_args()

    sessions = ([args.session] if args.session
                else sorted(glob.glob(os.path.join(args.data, "session_*"))))
    rng = np.random.default_rng(0)
    res = []
    for s in sessions:
        res += run_session(s, rng, args.h_pass, args.debug or None)
    if not res:
        sys.exit("沒有可分析的影像。確認 --data 路徑，或該 session 是空的。")
    if args.debug:
        return
    summarise(res, args.h_pass)
    if args.sweep:
        sweep(res)


if __name__ == "__main__":
    main()
