#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extract_route.py — 從 Aurora .stcm 地圖檔離線抽出「示教路徑」(teach path)

.stcm 裡每個 keyframe 都存了最佳化後的世界座標 (Owb)，這條軌跡就是你建圖時
實際開過的路線 —— 比 rosbag 錄的即時位姿更好（迴環修正後的版本）。

本工具把軌跡自動切成多條「單向路段 (pass)」：
  - 位置跳躍 > JUMP_M（session 邊界 / 重定位）處切斷
  - 行進方向迴轉 > 120°（折返點）處切斷
  - 太短的碎段（< min-len）丟棄
每條 pass 是一條「有方向」的路徑：車當時朝哪個方向開，之後就該朝同方向重走
（反方向行駛時視覺重定位會失效 —— 見 compus1.2 的分析）。

用法（跨平台，需 pip install msgpack numpy；不需要 ROS）：
  python3 extract_route.py <map.stcm> --out routes_dir [--png] [--min-len 10]

輸出：
  routes_dir/pass_00.csv ...   # 每條 pass 一個檔：kf_id, x, y, yaw_rad
  routes_dir/index.yaml        # 各 pass 的摘要（長度/端點/KF 範圍）
  routes_dir/preview.png       # --png 時：所有 pass 的俯視預覽圖（需 matplotlib）
"""

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

try:
    import msgpack
except ImportError:
    sys.exit("需要 msgpack：pip3 install msgpack")

JUMP_M = 3.0          # 相鄰 KF 距離超過此值 → session 邊界，切斷
REV_WIN = 8           # 迴轉偵測的前後平滑窗（單位：KF 數）
REV_MIN_MOVE = 1.0    # 窗內位移小於此值不判定迴轉（原地擾動不算）
REV_COS = -0.5        # 前後行進方向 cos < -0.5（>120°）→ 折返點


def read_keyframes(stcm_path):
    """走訪 .stcm 容器（16B 檔頭 + [crc,idx,size,1] 記錄），收集 keyframe 位姿。"""
    kfs = []
    with open(stcm_path, "rb") as f:
        f.seek(16)
        while True:
            hdr = f.read(16)
            if len(hdr) < 16:
                break
            _crc, idx, rsize, _one = struct.unpack("<IIII", hdr)
            payload = f.read(rsize)
            if idx < 5:          # 前 5 筆是 metadata（檔案資訊/atlas/相機/地圖物件）
                continue
            obj = msgpack.unpackb(payload, raw=False, strict_map_key=False)
            if obj.get("type") == "kf":
                kfs.append((obj["id"], obj.get("ts", -1), *obj["Owb"][:2]))
    kfs.sort()
    ids = np.array([k[0] for k in kfs])
    ts = np.array([k[1] for k in kfs], float)
    pts = np.array([[k[2], k[3]] for k in kfs], float)
    return ids, ts, pts


def split_passes(pts):
    """回傳切割點 index 清單（每個 index 表示「此點之前切一刀」）。"""
    n = len(pts)
    cuts = set()

    # 1) 位置跳躍
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    for i in np.where(step > JUMP_M)[0]:
        cuts.add(i + 1)

    # 2) 行進方向迴轉（前窗 vs 後窗的淨位移夾角 > 120°）
    marks = []
    for i in range(REV_WIN, n - REV_WIN):
        v1 = pts[i] - pts[i - REV_WIN]
        v2 = pts[i + REV_WIN] - pts[i]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < REV_MIN_MOVE or n2 < REV_MIN_MOVE:
            continue
        if float(v1 @ v2) / (n1 * n2) < REV_COS:
            marks.append(i)
    # 連續的迴轉標記群取中點各切一刀
    for grp_start in range(len(marks)):
        if grp_start == 0 or marks[grp_start] - marks[grp_start - 1] > REV_WIN:
            grp = [m for m in marks if abs(m - marks[grp_start]) <= REV_WIN * 2]
            cuts.add(grp[len(grp) // 2])

    return sorted(cuts)


def path_length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def travel_yaw(pts):
    """每個 waypoint 的行進方向（前後各 2 點的差分，端點外插）。"""
    n = len(pts)
    yaw = np.zeros(n)
    for i in range(n):
        a, b = max(0, i - 2), min(n - 1, i + 2)
        d = pts[b] - pts[a]
        yaw[i] = np.arctan2(d[1], d[0])
    return yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("stcm")
    ap.add_argument("--out", default="routes", help="輸出資料夾")
    ap.add_argument("--min-len", type=float, default=10.0, help="短於此長度(m)的碎段丟棄")
    ap.add_argument("--png", action="store_true", help="輸出 preview.png（需 matplotlib）")
    args = ap.parse_args()

    print(f"讀取 {args.stcm} ...")
    ids, ts, pts = read_keyframes(args.stcm)
    print(f"  keyframes: {len(ids)}")

    cuts = split_passes(pts)
    bounds = [0] + cuts + [len(pts)]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    passes, dropped = [], 0
    for a, b in zip(bounds[:-1], bounds[1:]):
        seg = pts[a:b]
        if len(seg) < 5 or path_length(seg) < args.min_len:
            dropped += 1
            continue
        passes.append((a, b))

    index = []
    for k, (a, b) in enumerate(passes):
        seg, seg_ids = pts[a:b], ids[a:b]
        yaw = travel_yaw(seg)
        fname = f"pass_{k:02d}.csv"
        with open(out / fname, "w", encoding="utf-8") as f:
            f.write("# kf_id,x,y,yaw_rad  (aurora_map 座標系, yaw=建圖時的行進方向)\n")
            for j in range(len(seg)):
                f.write(f"{seg_ids[j]},{seg[j,0]:.3f},{seg[j,1]:.3f},{yaw[j]:.4f}\n")
        L = path_length(seg)
        index.append({
            "file": fname, "n_wp": int(len(seg)), "length_m": round(L, 1),
            "start": [round(float(seg[0, 0]), 1), round(float(seg[0, 1]), 1)],
            "end": [round(float(seg[-1, 0]), 1), round(float(seg[-1, 1]), 1)],
            "kf_range": [int(seg_ids[0]), int(seg_ids[-1])],
        })
        print(f"  pass_{k:02d}: {len(seg):4d} wp, {L:6.1f} m, "
              f"KF {seg_ids[0]}→{seg_ids[-1]}, "
              f"({seg[0,0]:.0f},{seg[0,1]:.0f})→({seg[-1,0]:.0f},{seg[-1,1]:.0f})")
    if dropped:
        print(f"  （丟棄 {dropped} 條 <{args.min_len} m 的碎段）")

    import yaml
    with open(out / "index.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump({"source": str(args.stcm), "passes": index},
                       f, allow_unicode=True, sort_keys=False)
    print(f"寫入 {out}/index.yaml")

    if args.png:
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
        colors = ["#2a78d6", "#1baf7a", "#eda100", "#008300",
                  "#4a3aa7", "#e34948", "#e87ba4", "#eb6834"]
        fig, ax = plt.subplots(figsize=(12, 7), facecolor="#f9f9f7")
        ax.set_facecolor("#fcfcfb")
        ax.set_aspect("equal")
        ax.grid(True, color="#e1e0d9", lw=0.6)
        for k, (a, b) in enumerate(passes):
            seg = pts[a:b]
            c = colors[k % len(colors)]
            ax.plot(seg[:, 0], seg[:, 1], color=c, lw=1.8)
            ax.annotate(f"pass_{k:02d}", seg[len(seg) // 2],
                        color=c, fontsize=10, fontweight="bold")
            ax.plot(*seg[0], "o", ms=7, mfc=c, mec="#fcfcfb", mew=1.5)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_title("teach passes (圓點=起點, 方向=建圖行進方向)", fontsize=11, loc="left")
        fig.tight_layout()
        fig.savefig(out / "preview.png", dpi=130)
        print(f"寫入 {out}/preview.png")


if __name__ == "__main__":
    main()
