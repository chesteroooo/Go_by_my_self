#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_routes.py — 從「一段一張」的單程 .stcm 批次抽出可直接上路的 plan_*.csv

背景：A/B/C/D 四個站點的路線被分段建圖，每段一張獨立的 .stcm
（A_B, B_C, C_D, D_A 去程；D_C, C_B, B_A 回程）。每張圖都是單一 session、
零跳點、零折返的單向軌跡 —— 整條軌跡本身就是要跑的路線，不需要挑折返點。
（注意 extract_oneway.py 的 --half 自動折返偵測在這批圖上會切錯：它退回「離起點
最遠點」，而迴繞型路線的最遠點在中途，D_A 會被砍掉最後 44 m。所以一律取全段。）

本工具做三件事，這些是 routes_A_D/README.md 那套人工流程的自動化版：
  1. 自動偵測頭尾「停在站點原地擺動」的 keyframe 團 → 算出 --trim-start/--trim-end
     （車在站點停 30–110 s，SLAM 照樣產生 keyframe，那些點擠在 0.2 m 內卻累積里程，
      pure pursuit 的前視點會塌進去，車在起點原地亂轉）
  2. 呼叫 extract_oneway.py 抽全段（--kf 取全部 + --min-step 0.25 抽稀）
  3. 把 index.yaml 的 source: 改寫成 .stcm 路徑
     （autopilot_test.sh / field_test.sh 靠這一行自動配對地圖；指到 kf CSV 會配不到）

用法：
  ./build_routes.py --kf-dir ~/kf                    # 全部（讀 dump 出來的小 CSV，秒級）
  ./build_routes.py --kf-dir ~/kf A_B B_C            # 只做指定幾段
  ./build_routes.py --maps ~/maps                    # 直接讀 .stcm（每張約 1–3 分鐘）
  ./build_routes.py --kf-dir ~/kf --check-only       # 只驗證已存在的路線，不重抽

驗證（--check 預設開）：最大 waypoint 間距、實際前視距離塌陷點數、離線模擬橫向誤差。
"""

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

# 預設要建的路段。A_D 不在列內 —— routes_A_D 是人工調過切點的成果
# （--kf 0:3782 --exclude 3632:3743，見 routes_A_D/README.md），不要覆蓋。
DEFAULT_LEGS = ["A_B", "B_C", "C_D", "D_A", "D_C", "C_B", "B_A"]

CLUSTER_R = 0.6      # 判定「停在原地」的團半徑 (m)
CLUSTER_MIN_N = 8    # 團裡至少幾個 keyframe
TRIM_MARGIN = 0.4    # 修到團尾之後再多修這麼多 (m)
MIN_STEPS = (0.25, 0.35, 0.45)   # 抽稀間距候選 (m)：0.25 同 routes_A_D 的配方，
                                 # 過不了前視檢查就自動加大重抽
LOOKAHEAD = 0.8      # 與 ros_move_follow_route.py 的 LOOKAHEAD 一致（驗證用）
GOAL_TOL = 0.3       # 同上：剩餘里程小於此值就判定到達，不再轉向


def load_kf(path):
    rows = np.loadtxt(path, delimiter=",", comments="#", encoding="utf-8")
    return rows[:, 0].astype(int), rows[:, 2:4]


def stationary_clusters(pts):
    """回傳原地擺動段的 [(a, b), ...]（索引，含端點）。"""
    out, i, n = [], 0, len(pts)
    while i < n:
        j = i + 1
        while j < n and np.linalg.norm(pts[i:j + 1] - pts[i], axis=1).max() < CLUSTER_R:
            j += 1
        if j - i >= CLUSTER_MIN_N:
            out.append((i, j - 1))
        i = j if j > i + 1 else i + 1
    return out


def auto_trim(pts):
    """從頭尾的原地擺動團算出 (trim_start, trim_end)，單位公尺。"""
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    ts = te = 0.0
    for a, b in stationary_clusters(pts):
        # 只修「與端點相連」的那一團。半路上的擺動團交給 --min-step 抽稀處理 ——
        # 把它們也修掉會讓路線起點離站點好幾公尺，車停在站點反而過不了 preflight。
        if cum[a] < 1.0:                              # 貼著起點
            ts = max(ts, cum[b] + TRIM_MARGIN)
        if cum[-1] - cum[b] < 1.0:                    # 貼著終點
            te = max(te, cum[-1] - cum[a] + TRIM_MARGIN)
    return round(ts, 1), round(te, 1)


def check_plan(plan, sim=True):
    """量化驗證一條 plan：間距、前視塌陷、離線模擬橫向誤差。回傳 (ok, 訊息列)。"""
    rows = np.loadtxt(plan, delimiter=",", comments="#", encoding="utf-8")
    wp = rows[:, :2]
    step = np.linalg.norm(np.diff(wp, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(step)])

    # 實際前視距離：pure pursuit 取 cum[i]+LOOKAHEAD 的 waypoint，量它離 wp[i] 多遠。
    # 原地擺動沒清乾淨時，前視點會落在同一團裡 → Ld 塌到近乎 0，車原地亂轉。
    i_t = np.searchsorted(cum, cum + LOOKAHEAD).clip(0, len(wp) - 1)
    ld = np.linalg.norm(wp[i_t] - wp, axis=1)
    # 終點區（剩餘 < GOAL_TOL）不列入：跟隨器在那裡已判定到達、不再取前視點，
    # 而 extract_oneway 為了「終點一定保留」常留下一個貼著終點的重複 waypoint，
    # 它的前視距離必然接近 0，算進來會變成假警報。
    ld = ld[cum[-1] - cum > GOAL_TOL]
    n_collapse = int((ld < 0.4).sum())

    msgs = [f"{len(wp)} wp, {cum[-1]:.1f} m, 最大間距 {step.max():.2f} m, "
            f"前視 {ld.min():.2f}–{ld.max():.2f} m（<0.4 m 有 {n_collapse} 處）"]
    # 壞掉的樣子是「大量 waypoint 前視塌到近乎 0」（routes_A_D 清理前有 62 處、最小
    # 0.005 m）。零星幾點只要沒低於 pure pursuit 的 Ld 下限 0.15 m 就跑得過去。
    ok = step.max() <= 1.5 and ld.min() >= 0.25 and n_collapse <= 3

    if sim:
        r = subprocess.run([sys.executable, str(HERE / "ros_move_follow_route.py"),
                            "--plan", str(plan), "--sim"],
                           capture_output=True, text=True)
        tail = [l for l in r.stdout.splitlines() if "橫向誤差：" in l or "到達終點" in l]
        msgs += ["  " + l.strip() for l in tail]
        if "到達終點" not in r.stdout:
            ok = False
            msgs.append("  ✗ 離線模擬沒到終點")
    return ok, msgs


def build(leg, src_file, stcm_path, outdir, png, min_step):
    src, dst = leg.split("_")[:2]
    ids, pts = load_kf(src_file) if str(src_file).endswith(".csv") else (None, None)
    if ids is None:                            # 直接讀 .stcm：先 dump 成 CSV 再算 trim
        sys.exit("!! --maps 模式請先用 extract_oneway.py --dump-kf 產生 CSV")
    ts, te = auto_trim(pts)

    cmd = [sys.executable, str(HERE / "extract_oneway.py"), str(src_file),
           "--out", str(outdir), "--names", src, dst,
           "--kf", f"{ids[0]}:{ids[-1]}",      # 取全段：這批圖沒有折返點可切
           "--min-step", str(min_step)]
    if ts:
        cmd += ["--trim-start", str(ts)]
    if te:
        cmd += ["--trim-end", str(te)]
    if png:
        cmd += ["--png"]

    print(f"\n=== {leg}  (自動修頭 {ts} m / 修尾 {te} m, 抽稀 {min_step} m)")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        return False
    for line in r.stdout.splitlines():
        if line.startswith(("擷取", "  起點", "  相鄰", "  ⚠")):
            print("  " + line.strip())

    # index.yaml 的 source: 必須指向 .stcm —— autopilot/field_test 靠它自動配對地圖
    idx = Path(outdir) / "index.yaml"
    txt = idx.read_text(encoding="utf-8")
    idx.write_text(txt.replace(f"source: {src_file}", f"source: {stcm_path}", 1),
                   encoding="utf-8")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("legs", nargs="*", default=None, help=f"預設 {' '.join(DEFAULT_LEGS)}")
    ap.add_argument("--kf-dir", default="~/kf", help="--dump-kf 產生的 <LEG>_kf.csv 放哪")
    ap.add_argument("--maps", default="~/maps", help=".stcm 放哪（只用來寫 index.yaml 的 source）")
    ap.add_argument("--out-root", default=str(HERE), help="routes_* 產生在哪")
    ap.add_argument("--check-only", action="store_true", help="只驗證現有路線")
    ap.add_argument("--no-sim", action="store_true", help="跳過離線模擬（快）")
    ap.add_argument("--no-png", dest="png", action="store_false", help="不輸出 preview.png")
    args = ap.parse_args()

    legs = args.legs or DEFAULT_LEGS
    kfd, maps, root = (Path(p).expanduser() for p in (args.kf_dir, args.maps, args.out_root))
    built, skipped, bad = [], [], []

    for leg in legs:
        outdir = root / f"routes_{leg}"
        src_file, stcm = kfd / f"{leg}_kf.csv", maps / f"{leg}.stcm"
        if args.check_only:
            plans = sorted(outdir.glob("plan_*.csv"))
            if not plans:
                skipped.append(leg)
                continue
            ok, msgs = check_plan(plans[0], sim=not args.no_sim)
            print(f"\n=== {leg}\n  驗證 {'✓' if ok else '✗'}  " + "\n".join(msgs))
            (built if ok else bad).append(leg)
            continue

        if not src_file.exists():
            print(f"\n=== {leg}: 找不到 {src_file} —— 先跑 extract_oneway.py --dump-kf")
            skipped.append(leg)
            continue
        # 抽稀間距由小到大試，取第一個通過驗證的（前視塌陷幾乎都是抽稀不夠造成的）
        ok, msgs = False, ["沒有產出"]
        for ms in MIN_STEPS:
            if not build(leg, src_file, stcm, outdir, args.png, ms):
                break
            plans = sorted(outdir.glob("plan_*.csv"))
            if not plans:
                break
            ok, msgs = check_plan(plans[0], sim=not args.no_sim)
            print(f"  驗證 {'✓' if ok else '✗'}  " + "\n".join(msgs))
            if ok:
                break
        (built if ok else bad).append(leg)

    print(f"\n完成 {len(built)}：{' '.join(built) or '—'}")
    if bad:
        print(f"有問題 {len(bad)}：{' '.join(bad)}（看上面的驗證訊息）")
    if skipped:
        print(f"略過 {len(skipped)}：{' '.join(skipped)}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
