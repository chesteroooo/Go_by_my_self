#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extract_oneway.py — 只抽「單程」示教路徑（例如 A→D），並診斷回程為什麼亂掉

extract_route.py 會把整張圖切成多條 pass，交給 route_graph.py 用圖搜尋接起來；
當回程圖層品質差（重定位偏移、跳點、折返點附近亂繞）時，Dijkstra 有可能挑到
壞掉的片段，或在換圖層時產生座標跳點 —— 這時候更穩的做法是：
**直接指定 keyframe 範圍，把去程那一段原封不動切出來當計畫路線。**

用法（跨平台，需 pip install msgpack numpy pyyaml；不需要 ROS）：

  # 1) 先診斷：看分段、折返點、跳點，以及去程/回程圖層偏移多少
  python3 extract_oneway.py ~/maps/A_D.stcm --report

  # 2) 抽單程（二選一）
  python3 extract_oneway.py ~/maps/A_D.stcm --out routes_A_D --half first --png
  python3 extract_oneway.py ~/maps/A_D.stcm --out routes_A_D --kf 0:3421  --png

輸出（--out 資料夾）：
  pass_00.csv    # kf_id,x,y,yaw_rad —— 與 extract_route.py 同格式
  index.yaml     # 給 route_graph.py 用（想再加中途站時）
  stations.yaml  # A=起點、D=終點（名稱可用 --names 改）
  plan_A_D.csv   # x,y,yaw_rad —— 可直接餵 ros_move_follow_route.py，不必再跑 route_graph
  preview.png    # --png 時：路徑 + 行進方向箭頭
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from extract_route import (JUMP_M, path_length, read_keyframes,  # noqa: E402
                           split_passes, travel_yaw)

GAP_WARN_M = 1.5      # 相鄰 waypoint 間距超過此值 → 提醒（follower 前視 0.8 m）
SAME_CORRIDOR_M = 15.0  # 回程點離去程 < 此距離 → 視為「同一條走廊」，可比較圖層偏移


# ---------------------------------------------------------------- 診斷

def segment_table(ids, pts):
    """切段並標出每個切點的成因（跳點 or 折返）。回傳 [(a, b, why, mag), ...]"""
    cuts = split_passes(pts)
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    bounds = [0] + cuts + [len(pts)]
    segs = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        if a == 0:
            why, mag = "起點", 0.0
        elif step[a - 1] > JUMP_M:
            why, mag = "跳點", float(step[a - 1])
        else:
            why, mag = "折返", 0.0
        segs.append((a, b, why, mag))
    return segs


def layer_offset(out_pts, back_pts):
    """回程每點到去程折線的最近距離；只統計「同一條走廊」的點。

    去程/回程若是同一條實體道路，理想值應該接近車道寬（<1.5 m）。
    中位數 3~4 m 以上 = 兩個方向被建成互相偏移的圖層（重定位沒把它們對齊）。
    """
    if len(out_pts) < 2 or len(back_pts) < 2:
        return None
    d = np.linalg.norm(back_pts[:, None, :] - out_pts[None, :, :], axis=2).min(axis=1)
    near = d[d < SAME_CORRIDOR_M]
    if len(near) < 10:
        return None
    return {
        "n_overlap": int(len(near)),
        "ratio": float(len(near) / len(d)),
        "median": float(np.median(near)),
        "p90": float(np.percentile(near, 90)),
        "max": float(near.max()),
    }


def cmd_report(ids, ts, pts, args):
    segs = segment_table(ids, pts)
    print(f"keyframes: {len(ids)}   總軌跡長度: {path_length(pts):.1f} m")
    if (ts > 0).any():
        span = ts[ts > 0].max() - ts[ts > 0].min()
        print(f"時間跨度: {span:.0f} s（{span/60:.1f} 分）")

    print("\n分段（切點成因：跳點=重定位/session 邊界，折返=原地掉頭）")
    print(f"{'#':>3} {'KF 範圍':>16} {'點數':>6} {'長度m':>8} {'起點':>16} {'終點':>16}  切點")
    for k, (a, b, why, mag) in enumerate(segs):
        L = path_length(pts[a:b])
        tag = f"{why} {mag:.1f} m" if mag else why
        print(f"{k:>3} {ids[a]:>7}→{ids[b-1]:<8} {b-a:>6} {L:>8.1f} "
              f"({pts[a,0]:>6.1f},{pts[a,1]:>6.1f}) ({pts[b-1,0]:>6.1f},{pts[b-1,1]:>6.1f})  {tag}")

    # 大跳點清單（回程亂掉最常見的元兇）
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    big = np.where(step > JUMP_M)[0]
    print(f"\n位置跳點 (> {JUMP_M} m)：{len(big)} 處")
    for i in big:
        print(f"  KF {ids[i]}→{ids[i+1]}：{step[i]:.1f} m  "
              f"({pts[i,0]:.1f},{pts[i,1]:.1f}) → ({pts[i+1,0]:.1f},{pts[i+1,1]:.1f})")

    # 去程 vs 回程圖層偏移
    cut, how = turnaround_index(pts, segs)
    if 0 < cut < len(pts) - 1:
        st = layer_offset(pts[:cut], pts[cut:])
        print(f"\n去程/回程圖層對齊（折返點 = KF {ids[cut]} "
              f"({pts[cut,0]:.1f},{pts[cut,1]:.1f})，由{how}判定）")
        if st is None:
            print("  回程幾乎沒有和去程重疊 —— 走的是另一條路，或回程根本沒建到。")
        else:
            print(f"  重疊點數 {st['n_overlap']} ({st['ratio']*100:.0f}% 的回程點在去程 "
                  f"{SAME_CORRIDOR_M:.0f} m 內)")
            print(f"  回程到去程的距離：中位數 {st['median']:.2f} m、"
                  f"p90 {st['p90']:.2f} m、最大 {st['max']:.2f} m")
            verdict = ("兩方向對齊良好（同一實體車道）" if st["median"] < 1.5 else
                       "偏移偏大，換圖層時會有跳點" if st["median"] < 3.0 else
                       "★ 明顯的方向圖層偏移 —— 回程被建成一條平行的假路徑")
            print(f"  判定：{verdict}")


def turnaround_index(pts, segs):
    """折返點（去程/回程的分界）。

    優先取 split_passes 偵到的「折返」切點；原地掉頭偵得到，但**緩彎掉頭**
    （繞一個大彎回來）偵不到 —— 那就退回「離起點最遠的軌跡點」，對 A↔D
    這種來回路線一定成立。有多個折返時取離最遠點最近的那個。
    """
    far = int(np.argmax(np.linalg.norm(pts - pts[0], axis=1)))
    revs = [a for a, _b, why, _m in segs if why == "折返"]
    if revs:
        return min(revs, key=lambda i: abs(i - far)), "折返切點"
    return far, "離起點最遠點"


# ---------------------------------------------------------------- 抽單程

def quality(seg):
    step = np.linalg.norm(np.diff(seg, axis=0), axis=1)
    return {"n": len(seg), "length_m": float(step.sum()),
            "max_gap_m": float(step.max()), "n_gap_over": int((step > GAP_WARN_M).sum())}


def cmd_extract(ids, ts, pts, args):
    segs = segment_table(ids, pts)

    if args.kf:
        lo, hi = (int(v) for v in args.kf.split(":"))
        sel = np.where((ids >= lo) & (ids <= hi))[0]
        if len(sel) < 5:
            sys.exit(f"!! KF 範圍 {args.kf} 只選到 {len(sel)} 個 keyframe")
        a, b = int(sel[0]), int(sel[-1]) + 1
        how = f"--kf {args.kf}"
    else:
        cut, why = turnaround_index(pts, segs)
        a, b = (0, cut + 1) if args.half == "first" else (cut, len(pts))
        how = f"--half {args.half}（折返點 KF {ids[cut]}，由{why}判定）"

    seg, seg_ids = pts[a:b], ids[a:b]

    if args.exclude:
        # 挖掉中途「停在原地擺動」的 KF 區段再接起來。抽稀救不了這種團：
        # 它在原地來回，0.8 m 的里程只換到十幾公分的直線位移，前視點照樣退化。
        drop = np.zeros(len(seg_ids), bool)
        for rng in args.exclude:
            lo, hi = (int(v) for v in rng.split(":"))
            drop |= (seg_ids >= lo) & (seg_ids <= hi)
        if drop.all():
            sys.exit("!! --exclude 把整段都挖掉了")
        kept = np.where(~drop)[0]
        for k in np.where(np.diff(kept) > 1)[0]:          # 報告每個接縫的落差
            i, j = kept[k], kept[k + 1]
            print(f"  接縫 KF {seg_ids[i]}→{seg_ids[j]}：落差 "
                  f"{np.linalg.norm(seg[j] - seg[i]):.2f} m")
        seg, seg_ids = seg[kept], seg_ids[kept]
        how += f" +挖掉 {','.join(args.exclude)}（{int(drop.sum())} 個點）"

    if args.trim_start or args.trim_end:      # 修掉頭尾（掉頭弧線、起步擺動）
        cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))])
        keep = (cum >= args.trim_start) & (cum <= cum[-1] - args.trim_end)
        if keep.sum() < 5:
            sys.exit("!! --trim-start/--trim-end 修掉太多，剩不到 5 個 waypoint")
        seg, seg_ids = seg[keep], seg_ids[keep]
        how += f" +修頭 {args.trim_start} m/修尾 {args.trim_end} m"

    if args.min_step > 0:
        # 丟掉「沒有前進」的 waypoint（車停在原地時 SLAM 仍持續產生 keyframe）。
        # 這種點擠在數十公分內卻照樣累積里程，pure pursuit 的前視點
        # cum[i_near]+LOOKAHEAD 會落進那一團，Ld 掉到下限 → 車原地亂轉。
        keep = [0]
        for j in range(1, len(seg)):
            if np.linalg.norm(seg[j] - seg[keep[-1]]) >= args.min_step:
                keep.append(j)
        if keep[-1] != len(seg) - 1:
            keep.append(len(seg) - 1)          # 終點一定保留
        n_drop = len(seg) - len(keep)
        seg, seg_ids = seg[keep], seg_ids[keep]
        how += f" +抽稀 {args.min_step} m（丟 {n_drop} 個原地點）"

    yaw = travel_yaw(seg)
    q = quality(seg)
    src, dst = args.names

    print(f"擷取 {how}：KF {seg_ids[0]}→{seg_ids[-1]}，{q['n']} 個 waypoint，{q['length_m']:.1f} m")
    print(f"  起點 {src} ({seg[0,0]:.1f},{seg[0,1]:.1f}) → 終點 {dst} ({seg[-1,0]:.1f},{seg[-1,1]:.1f})")
    print(f"  相鄰 waypoint 最大間距 {q['max_gap_m']:.2f} m"
          f"（> {GAP_WARN_M} m 的有 {q['n_gap_over']} 處）")
    if q["max_gap_m"] > GAP_WARN_M:
        print("  ⚠ 有稀疏段：pure pursuit 前視 0.8 m，經過時會切彎但通常仍可跨過")

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    with open(out / "pass_00.csv", "w", encoding="utf-8") as f:
        f.write("# kf_id,x,y,yaw_rad  (aurora_map 座標系, yaw=建圖時的行進方向)\n")
        for j in range(len(seg)):
            f.write(f"{seg_ids[j]},{seg[j,0]:.3f},{seg[j,1]:.3f},{yaw[j]:.4f}\n")

    with open(out / "index.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump({"source": str(args.stcm), "passes": [{
            "file": "pass_00.csv", "n_wp": q["n"], "length_m": round(q["length_m"], 1),
            "start": [round(float(seg[0, 0]), 1), round(float(seg[0, 1]), 1)],
            "end": [round(float(seg[-1, 0]), 1), round(float(seg[-1, 1]), 1)],
            "kf_range": [int(seg_ids[0]), int(seg_ids[-1])],
        }]}, f, allow_unicode=True, sort_keys=False)

    (out / "stations.yaml").write_text(
        "# 由 extract_oneway.py 產生：單程路徑的兩端\n"
        "snap_radius: 3.0\nstations:\n"
        f"  {src}: {{x: {seg[0,0]:.1f}, y: {seg[0,1]:.1f}}}\n"
        f"  {dst}: {{x: {seg[-1,0]:.1f}, y: {seg[-1,1]:.1f}}}\n", encoding="utf-8")

    plan = out / f"plan_{src}_{dst}.csv"
    with open(plan, "w", encoding="utf-8") as f:
        f.write(f"# 路線 {src}→{dst}, {q['length_m']:.1f} m  (x,y,yaw_rad)  "
                f"來源 {Path(args.stcm).name} KF {seg_ids[0]}-{seg_ids[-1]}\n")
        for j in range(len(seg)):
            f.write(f"{seg[j,0]:.3f},{seg[j,1]:.3f},{yaw[j]:.4f}\n")
    print(f"寫入 {out}/pass_00.csv, index.yaml, stations.yaml, {plan.name}")
    print(f"直接上路：python3 ros_move_follow_route.py --plan {plan}")

    if args.png:
        render(pts, seg, (src, dst), out / "preview.png")


def render(all_pts, seg, names, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.sans-serif"] = ["Microsoft JhengHei",
                                           "Noto Sans CJK TC", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
    except ImportError:
        print("!! 沒有 matplotlib，略過 preview.png")
        return
    fig, ax = plt.subplots(figsize=(12.5, 7.5), facecolor="#f9f9f7")
    ax.set_facecolor("#fcfcfb")
    ax.set_aspect("equal")
    ax.grid(True, color="#e1e0d9", lw=0.6)
    ax.plot(all_pts[:, 0], all_pts[:, 1], color="#c3c2b7", lw=1.0, zorder=2)  # 整張圖
    ax.plot(seg[:, 0], seg[:, 1], color="#2a78d6", lw=2.4, zorder=4)          # 擷取段
    n_arrow = max(len(seg) // 12, 1)
    for i in range(n_arrow // 2, len(seg) - 1, n_arrow):
        ax.annotate("", xy=seg[i + 1], xytext=seg[i],
                    arrowprops=dict(arrowstyle="-|>", color="#2a78d6", lw=1.6), zorder=5)
    ax.plot(*seg[0], "o", ms=11, mfc="#1baf7a", mec="#fcfcfb", mew=2, zorder=7)
    ax.plot(*seg[-1], "s", ms=11, mfc="#e34948", mec="#fcfcfb", mew=2, zorder=7)
    for xy, nm in ((seg[0], names[0]), (seg[-1], names[1])):
        ax.annotate(nm, xy, xytext=(9, 7), textcoords="offset points",
                    fontsize=13, fontweight="bold", zorder=8)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"單程示教路徑 {names[0]}→{names[1]}   (灰=整張圖的完整軌跡)",
                 fontsize=11, loc="left")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"寫入 {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stcm")
    ap.add_argument("--report", action="store_true", help="只診斷，不輸出檔案")
    ap.add_argument("--out", help="輸出資料夾（例如 routes_A_D）")
    ap.add_argument("--kf", help="用 keyframe id 範圍指定，例如 0:3421")
    ap.add_argument("--exclude", action="append", metavar="LO:HI",
                    help="挖掉這段 keyframe id 再把前後接起來（原地擺動用）。可重複")
    ap.add_argument("--half", choices=["first", "last"], default="first",
                    help="沒給 --kf 時：切在第一個折返點，取前半(去程)或後半(回程)")
    ap.add_argument("--names", nargs=2, default=["A", "D"], metavar=("SRC", "DST"))
    ap.add_argument("--trim-start", type=float, default=0.0, metavar="M",
                    help="從頭修掉幾公尺（起步原地擺動）")
    ap.add_argument("--trim-end", type=float, default=0.0, metavar="M",
                    help="從尾修掉幾公尺（掉頭弧線）")
    ap.add_argument("--min-step", type=float, default=0.0, metavar="M",
                    help="抽稀：與上一個保留點距離小於此值就丟掉。"
                         "用來清掉車停在原地時累積的 waypoint（建議 0.25）")
    ap.add_argument("--png", action="store_true")
    ap.add_argument("--dump-kf", metavar="CSV",
                    help="把所有 keyframe 原始資料 (kf_id,ts,x,y) 存成 CSV，供離線細查"
                         "（.stcm 解析很慢，順手在同一趟做掉）")
    args = ap.parse_args()

    print(f"讀取 {args.stcm} ...")
    if str(args.stcm).endswith(".csv"):      # --dump-kf 的產物：秒讀，方便反覆調切點
        rows = np.loadtxt(args.stcm, delimiter=",", comments="#", encoding="utf-8")
        ids, ts, pts = rows[:, 0].astype(int), rows[:, 1], rows[:, 2:4]
    else:
        ids, ts, pts = read_keyframes(args.stcm)
    print(f"  keyframes: {len(ids)}")

    if args.dump_kf:
        with open(args.dump_kf, "w", encoding="utf-8") as f:
            f.write("# kf_id,ts,x,y\n")
            for i in range(len(ids)):
                f.write(f"{ids[i]},{ts[i]:.3f},{pts[i,0]:.3f},{pts[i,1]:.3f}\n")
        print(f"  寫入 {args.dump_kf}")

    if args.report or not args.out:
        cmd_report(ids, ts, pts, args)
        if not args.out:
            return
        print()
    cmd_extract(ids, ts, pts, args)


if __name__ == "__main__":
    main()
