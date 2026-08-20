#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
merge_site_map.py — 把 A/B/C/D 八段獨立座標系的路線合成「一張整合地圖」

問題：每段一張 `.stcm`，每張圖的原點都是自己那段的起站，八段互不通用。
要在同一張圖上看到全部路線、而且不管跑哪一段都能把車畫在正確位置，就得先把
八段配準到同一個座標系，並記下「該段座標 → 整合座標」的剛體變換。

作法（順序很重要，前兩種直覺做法都會失敗）：
  ✗ 純 ICP 形狀匹配 —— 這條路有一段 292 m 自相似的開闊直走廊，一段 129 m 的路線
    沿走廊滑到哪裡殘差都很小。實測 B_C 殘差 0.15 m 卻被貼到 A–B 之間、方向還相反。
  ✗ 站點接合（拿前一段終點航向接下一段起點航向）—— 各段頭尾都是車停在站點原地
    擺動的區段，單點 yaw 是雜訊；實測算出的 D 偏了 243 m，路徑折回自己。
  ✓ 本工具：以 A_D（單張圖涵蓋全程 A→D）為骨幹＝整合座標系，站點按里程比例落在
    骨幹上，各段先做兩點剛體對位，再跑「站點軟錨定 ICP」——錨點把段固定在正確的
    沿線位置（不會滑），ICP 把形狀貼合到骨幹。實測殘差中位 0.34–1.20 m。

**精度說明**：配準誤差只影響「不同段之間的視覺對位」。位姿和該段路線套用的是同一個
剛體變換，距離不變 —— 所以「車離路線多遠 / 在不在線上」的判定完全不受影響。

用法：
  ./merge_site_map.py                 # 產生 routes_site/
  ./merge_site_map.py --png           # 另外輸出整合預覽圖

產出 routes_site/：
  pass_spine.csv   **去重後的單一道路中心線**。八段跑的是同一條實體路（去程回程各四段
                   都疊在一起），畫八條只會糊成一團 —— 所以地圖骨架只畫這一條。
  leg_<LEG>.csv    各段在整合座標系的折線。不叫 pass_* 是刻意的：route_monitor 只把
                   pass_*.csv 當骨架畫，這些留給「高亮目前跑的那一段」用。
  stations.yaml    A/B/C/D 的整合座標
  transforms.yaml  每段的 (旋轉角, 平移) 與配準品質 —— 儀表板拿它換算即時位姿
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
BACKBONE = "A_D"                    # 單張圖涵蓋全程，用它的座標系當整合座標系
LEGS = ["A_B", "B_C", "C_D", "D_A", "D_C", "C_B", "B_A"]
FORWARD = ["A_B", "B_C", "C_D"]     # 用來按里程比例定出 B、C 在骨幹上的位置
ANCHOR_W = 0.25                     # 站點錨點佔的總權重比例
ICP_ITERS = 30
OUTLIER_PCT = 92                    # 殘差超過這個百分位的點不參與擬合（不重疊段）


def load_leg(root, leg):
    f = root / f"routes_{leg}" / f"plan_{leg}.csv"
    if not f.exists():
        return None
    return np.loadtxt(f, delimiter=",", comments="#", encoding="utf-8")


def cumlen(P):
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])


def at_arc(P, c, s):
    i = min(max(int(np.searchsorted(c, s)), 1), len(P) - 1)
    f = (s - c[i - 1]) / max(c[i] - c[i - 1], 1e-9)
    return P[i - 1] + f * (P[i] - P[i - 1])


def nearest_pts(P, Q, chunk=256):
    """P 每點在折線 Q 上的最近點與距離。"""
    A, B = Q[:-1], Q[1:]
    AB = B - A
    l2 = (AB ** 2).sum(1)
    l2[l2 == 0] = 1e-9
    bd = np.full(len(P), np.inf)
    bp = np.zeros_like(P)
    idx = np.arange(len(P))
    for i in range(0, len(A), chunk):
        a, ab, ll = A[i:i + chunk], AB[i:i + chunk], l2[i:i + chunk]
        t = np.clip(((P[:, None, :] - a) * ab).sum(2) / ll, 0, 1)
        proj = a + t[..., None] * ab
        d = np.linalg.norm(proj - P[:, None, :], axis=2)
        k = d.argmin(1)
        dm = d[idx, k]
        m = dm < bd
        bd[m], bp[m] = dm[m], proj[idx, k][m]
    return bp, bd


def wkabsch(P, X, w):
    """加權剛體對位（只有旋轉+平移，不縮放）。"""
    wc = w[:, None]
    W = w.sum()
    pc, xc = (P * wc).sum(0) / W, (X * wc).sum(0) / W
    H = ((P - pc) * wc).T @ (X - xc)
    U, _S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    Rm = Vt.T @ np.diag([1.0, d]) @ U.T
    return Rm, xc - Rm @ pc


def two_point(P, p0, p1):
    """把 P 的頭尾對到 p0、p1。兩點就唯一決定旋轉+平移，沿線位置不會滑。"""
    v0, v1 = P[-1] - P[0], p1 - p0
    th = np.arctan2(v1[1], v1[0]) - np.arctan2(v0[1], v0[0])
    Rm = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return Rm, p0 - Rm @ P[0]


def soft_icp(P, Q, p0, p1):
    """兩點對位當初值 → 站點軟錨定的 ICP。錨點防滑，ICP 貼形狀。"""
    Rm, t = two_point(P, p0, p1)
    n = len(P)
    src = np.vstack([P, P[0][None], P[-1][None]])
    aw = max(ANCHOR_W * n / 2.0, 1.0)
    base_w = np.concatenate([np.ones(n), [aw, aw]])
    for _ in range(ICP_ITERS):
        tgt, d = nearest_pts(P @ Rm.T + t, Q)
        w = base_w.copy()
        w[:n][d > np.percentile(d, OUTLIER_PCT)] = 0.0
        Rm2, t2 = wkabsch(src, np.vstack([tgt, p0[None], p1[None]]), w)
        if np.allclose(Rm2, Rm, atol=1e-10) and np.allclose(t2, t, atol=1e-10):
            break
        Rm, t = Rm2, t2
    return Rm, t


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=str(HERE), help="routes_* 所在目錄")
    ap.add_argument("--out", default=None, help="輸出資料夾（預設 <root>/routes_site）")
    ap.add_argument("--png", action="store_true", help="輸出整合預覽圖")
    args = ap.parse_args()

    root = Path(args.root)
    out = Path(args.out) if args.out else root / "routes_site"

    bb_rows = load_leg(root, BACKBONE)
    if bb_rows is None:
        sys.exit(f"!! 找不到骨幹路線 routes_{BACKBONE}/plan_{BACKBONE}.csv")
    bb = bb_rows[:, :2]
    cb = cumlen(bb)
    L = cb[-1]

    legs = {}
    for leg in LEGS:
        r = load_leg(root, leg)
        if r is None:
            print(f"（略過 {leg}：找不到 plan CSV）")
            continue
        legs[leg] = r

    missing = [l for l in FORWARD if l not in legs]
    if missing:
        sys.exit(f"!! 缺少去程路段 {missing}，無法定出 B/C 在骨幹上的位置")

    # 站點：A、D 是骨幹兩端；B、C 按去程三段的里程比例落在骨幹上
    fl = [cumlen(legs[l][:, :2])[-1] for l in FORWARD]
    tot = sum(fl)
    sB, sC = fl[0] / tot * L, (fl[0] + fl[1]) / tot * L
    stn = {"A": bb[0].copy(), "B": at_arc(bb, cb, sB),
           "C": at_arc(bb, cb, sC), "D": bb[-1].copy()}
    print(f"骨幹 {BACKBONE}：{L:.1f} m；去程三段合計 {tot:.1f} m")
    print(f"站點里程 A=0  B={sB:.0f}  C={sC:.0f}  D={L:.0f} m")
    for k in "ABCD":
        print(f"  {k}: ({stn[k][0]:8.1f}, {stn[k][1]:8.1f})")

    out.mkdir(parents=True, exist_ok=True)
    for old in list(out.glob("pass_*.csv")) + list(out.glob("leg_*.csv")):
        old.unlink()                      # 清掉上一版，避免殘留舊檔被當成骨架畫出來
    tf, quality = {}, []
    # 骨幹自己就是整合座標系 → 單位變換
    allsets = {BACKBONE: (np.eye(2), np.zeros(2), bb_rows)}
    for leg, rows in legs.items():
        P = rows[:, :2]
        Rm, t = soft_icp(P, bb, stn[leg[0]], stn[leg[2]])
        allsets[leg] = (Rm, t, rows)

    print(f"\n{'段':6s} {'殘差中位':>8s} {'p90':>7s} {'最大':>7s} {'起點':>7s} {'終點':>7s}")
    for leg, (Rm, t, rows) in allsets.items():
        P = rows[:, :2]
        Pt = P @ Rm.T + t
        yaw = rows[:, 2] + np.arctan2(Rm[1, 0], Rm[0, 0])
        _pp, d = nearest_pts(Pt, bb)
        e0 = float(np.linalg.norm(Pt[0] - stn[leg[0]]))
        e1 = float(np.linalg.norm(Pt[-1] - stn[leg[2]]))
        print(f"{leg:6s} {np.median(d):8.2f} {np.percentile(d, 90):7.2f} "
              f"{d.max():7.1f} {e0:7.2f} {e1:7.2f}")

        with open(out / f"leg_{leg}.csv", "w", encoding="utf-8") as f:
            f.write(f"# {leg[0]}→{leg[2]} 整合座標 (x,y,yaw_rad)；"
                    f"由 merge_site_map.py 從 routes_{leg} 配準而來\n")
            for j in range(len(Pt)):
                f.write(f"{Pt[j, 0]:.3f},{Pt[j, 1]:.3f},{yaw[j]:.4f}\n")

        tf[leg] = {"rot_deg": round(float(np.degrees(np.arctan2(Rm[1, 0], Rm[0, 0]))), 4),
                   "tx": round(float(t[0]), 4), "ty": round(float(t[1]), 4),
                   "fit_median_m": round(float(np.median(d)), 2),
                   "fit_p90_m": round(float(np.percentile(d, 90)), 2)}
        quality.append(float(np.median(d)))

    # 去重後的單一道路中心線：八段都在骨幹 ±4.5 m 內（見上表最大殘差），
    # 所以骨幹本身就完整涵蓋這條路，直接拿它當中心線。
    with open(out / "pass_spine.csv", "w", encoding="utf-8") as f:
        f.write(f"# 道路中心線（去重）(x,y,yaw_rad)；來自骨幹 {BACKBONE}，"
                f"八段共用這一條\n")
        for j in range(len(bb)):
            f.write(f"{bb[j, 0]:.3f},{bb[j, 1]:.3f},{bb_rows[j, 2]:.4f}\n")

    (out / "stations.yaml").write_text(
        "# A/B/C/D 在整合座標系（= routes_%s 的座標系）\n"
        "snap_radius: 5.0\nstations:\n" % BACKBONE
        + "".join(f"  {k}: {{x: {stn[k][0]:.1f}, y: {stn[k][1]:.1f}}}\n" for k in "ABCD"),
        encoding="utf-8")

    with open(out / "transforms.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump({
            "backbone": BACKBONE,
            "note": "把該段地圖座標的位姿換算成整合座標："
                    "先繞原點轉 rot_deg，再加 (tx, ty)。",
            "stations": {k: [round(float(stn[k][0]), 2), round(float(stn[k][1]), 2)]
                         for k in "ABCD"},
            "legs": tf,
        }, f, allow_unicode=True, sort_keys=False)

    print(f"\n寫入 {out}/pass_spine.csv（去重後的單一道路中心線）、"
          f"leg_*.csv（{len(allsets)} 段，供高亮用）、stations.yaml、transforms.yaml")
    print(f"整體配準品質：殘差中位數的中位 {np.median(quality):.2f} m")
    print("（配準誤差只影響不同段之間的視覺對位；車離路線多遠的判定用同一個剛體變換，"
          "距離不變，不受影響）")

    if args.png:
        render(bb, stn, out / "preview.png")


def render(spine, stn, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK TC", "Microsoft JhengHei",
                                           "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
    except ImportError:
        print("!! 沒有 matplotlib，略過 preview.png")
        return
    fig, ax = plt.subplots(figsize=(13, 8), facecolor="#f9f9f7")
    ax.set_facecolor("#fcfcfb")
    ax.set_aspect("equal")
    ax.grid(True, color="#e1e0d9", lw=0.6)
    # 八段跑的是同一條實體路，重疊的部分只畫一條中心線
    ax.plot(spine[:, 0], spine[:, 1], color="#2a78d6", lw=2.6,
            label="road centreline (all 8 legs share it)", zorder=4)
    for name, c in stn.items():
        ax.plot(*c, "o", ms=14, mfc="#ffffff", mec="#2b2b2b", mew=2, zorder=9)
        ax.annotate(name, c, ha="center", va="center", fontsize=11,
                    fontweight="bold", zorder=10)
    ax.legend(fontsize=9, framealpha=0.9, edgecolor="#e1e0d9", loc="upper left")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    # 標題用 ASCII：不是每台機器都裝了中文字型，缺字會畫成方框
    ax.set_title("Merged site map - one deduplicated centreline, stations A/B/C/D",
                 fontsize=12, loc="left")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"寫入 {path}")


if __name__ == "__main__":
    main()
