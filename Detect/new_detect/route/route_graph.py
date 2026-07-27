#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
route_graph.py — 路線圖（站點 + 示教路段）規劃器

概念：
  - pass（extract_route.py 的輸出）＝建圖時實際開過的「單向」路徑
  - station（stations.yaml，人工定義）＝路線上有名字的地點（A、B、C…）
  - 站點會自動吸附到每條經過它的 pass 上；同一條 pass 上相鄰兩站之間的
    路徑切片就是一條「有向邊」。Dijkstra 在站點之間找最短路線。
  - 邊是有方向的：預設只允許照建圖方向走（反方向視覺重定位會失效）。
    --allow-reverse 可放行逆向邊（加 3 倍成本 + 警告），僅供測試。
  - pass 尾端與另一條 pass 頭端距離 < 2 m 時自動加接續邊（同一趟連續行駛
    被折返點切開的情況）。

用法：
  python3 route_graph.py stations-init --routes <dir>            # 產生 stations.yaml 範本
  python3 route_graph.py info        --routes <dir>              # 看圖的站點/邊
  python3 route_graph.py plan A D    --routes <dir> [--via B]
                                     [--allow-reverse] [--render] # 規劃並輸出 plan_A_D.csv
"""

import argparse
import heapq
import re
import sys
from pathlib import Path

import numpy as np
import yaml

REVERSE_COST = 3.0   # 逆向邊的成本倍率（不建議實際行駛）
CONNECT_M = 2.0      # pass 尾→頭自動接續的距離門檻
SEAM_WARN_M = 1.0    # 換 pass 時座標落差超過此值 → 提醒（方向圖層偏移）


# ---------------------------------------------------------------- 載入

def load_passes(routes_dir):
    """回傳 list of dict: {id, wp(N,3: x,y,yaw), kf, cum(N: 沿線里程)}"""
    routes_dir = Path(routes_dir)
    idx = yaml.safe_load((routes_dir / "index.yaml").read_text(encoding="utf-8"))
    passes = []
    for k, meta in enumerate(idx["passes"]):
        rows = np.loadtxt(routes_dir / meta["file"], delimiter=",", comments="#",
                          encoding="utf-8")
        wp = rows[:, 1:4]                    # x, y, yaw
        seg = np.linalg.norm(np.diff(wp[:, :2], axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        passes.append({"id": k, "wp": wp, "kf": rows[:, 0].astype(int), "cum": cum})
    return passes


def load_stations(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    st = {name: np.array([v["x"], v["y"]], float)
          for name, v in data["stations"].items()}
    return st, float(data.get("snap_radius", 6.0))


# ---------------------------------------------------------------- 建圖

def nearest_idx(pass_, xy):
    d = np.linalg.norm(pass_["wp"][:, :2] - xy, axis=1)
    i = int(np.argmin(d))
    return i, float(d[i])


def build_graph(passes, stations, snap_radius, allow_reverse=False, loose_junctions=False):
    """回傳 edges: list of dict{u, v, pass_id, i0, i1, length, reverse}"""
    edges = []

    # 站點吸附：每條 pass 上，站點依里程排序，相鄰兩站成一條有向邊
    # 接續點（_pXstart/_pXend）只吸附到自己的 pass —— 吸到幾何重疊的別條 pass
    # 會讓 Dijkstra 跨圖層抄捷徑，產生 >1 m 的座標跳點（follower 會急停）
    junc_re = re.compile(r"^_p(\d+)(?:start|end)$")
    for p in passes:
        on_pass = []
        for name, xy in stations.items():
            jm = junc_re.match(name)
            if jm and int(jm.group(1)) != p["id"] and not loose_junctions:
                continue
            i, d = nearest_idx(p, xy)
            if d <= snap_radius:
                on_pass.append((i, name))
        on_pass.sort()
        for (i0, u), (i1, v) in zip(on_pass[:-1], on_pass[1:]):
            if i1 < i0:
                continue
            if i1 == i0:
                # 兩站吸附到同一路徑點（例如折返點 = 前一 pass 終點 = 下一 pass 起點）
                # → 互為別名，補雙向零長度邊，否則該站會變成死路
                for uu, vv in ((u, v), (v, u)):
                    edges.append({"u": uu, "v": vv, "pass_id": p["id"],
                                  "i0": i0, "i1": i1, "length": 0.0, "reverse": False})
                continue
            L = p["cum"][i1] - p["cum"][i0]
            edges.append({"u": u, "v": v, "pass_id": p["id"],
                          "i0": i0, "i1": i1, "length": L, "reverse": False})
            if allow_reverse:
                edges.append({"u": v, "v": u, "pass_id": p["id"],
                              "i0": i0, "i1": i1, "length": L, "reverse": True})

    # pass 尾→另一 pass 頭的自動接續（折返點被切開的連續行駛）
    for pa in passes:
        for pb in passes:
            if pa["id"] == pb["id"]:
                continue
            gap = np.linalg.norm(pa["wp"][-1, :2] - pb["wp"][0, :2])
            if gap < CONNECT_M:
                ja, jb = f"_p{pa['id']}end", f"_p{pb['id']}start"
                stations.setdefault(ja, pa["wp"][-1, :2].copy())
                stations.setdefault(jb, pb["wp"][0, :2].copy())
                # 讓接續點也各自出現在自己 pass 的邊鏈上：
                # （重新吸附由呼叫端統一處理 —— 這裡直接補三條邊）
                edges.append({"u": ja, "v": jb, "pass_id": -1,
                              "i0": 0, "i1": 0, "length": gap, "reverse": False})
    return edges


def rebuild_with_junctions(passes, stations, snap_radius, allow_reverse, loose_junctions=False):
    """先把 pass 間的接續點加進站點集，再建完整的邊圖。"""
    st2 = dict(stations)
    for pa in passes:
        for pb in passes:
            if pa["id"] == pb["id"]:
                continue
            if np.linalg.norm(pa["wp"][-1, :2] - pb["wp"][0, :2]) < CONNECT_M:
                st2[f"_p{pa['id']}end"] = pa["wp"][-1, :2].copy()
                st2[f"_p{pb['id']}start"] = pb["wp"][0, :2].copy()
    edges = build_graph(passes, st2, snap_radius, allow_reverse, loose_junctions)
    return st2, edges


# ---------------------------------------------------------------- 規劃

def dijkstra(edges, src, dst):
    adj = {}
    for k, e in enumerate(edges):
        cost = e["length"] * (REVERSE_COST if e["reverse"] else 1.0)
        adj.setdefault(e["u"], []).append((cost, e["v"], k))
    dist, prev = {src: 0.0}, {}
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == dst:
            break
        if d > dist.get(u, np.inf):
            continue
        for cost, v, k in adj.get(u, []):
            nd = d + cost
            if nd < dist.get(v, np.inf):
                dist[v], prev[v] = nd, (u, k)
                heapq.heappush(pq, (nd, v))
    if dst not in dist:
        return None
    path, node = [], dst
    while node != src:
        u, k = prev[node]
        path.append(k)
        node = u
    return list(reversed(path))


def plan_waypoints(passes, edges, edge_seq):
    """把邊序列串成 waypoint 陣列 (N,3)，回傳 (wp, notes)"""
    chunks, notes = [], []
    for k in edge_seq:
        e = edges[k]
        if e["pass_id"] < 0:
            continue                                  # 接續邊（零長度）
        p = passes[e["pass_id"]]
        wp = p["wp"][e["i0"]:e["i1"] + 1].copy()
        if e["reverse"]:
            wp = wp[::-1].copy()
            wp[:, 2] = np.arctan2(np.sin(wp[:, 2] + np.pi), np.cos(wp[:, 2] + np.pi))
            notes.append(f"⚠ {e['u']}→{e['v']} 逆向行駛 pass_{e['pass_id']:02d}"
                         f"（與建圖方向相反，重定位可能失效）")
        if chunks:
            seam = np.linalg.norm(chunks[-1][-1, :2] - wp[0, :2])
            if seam > SEAM_WARN_M:
                notes.append(f"⚠ {e['u']} 處換圖層：座標落差 {seam:.1f} m"
                             f"（方向圖層偏移，經過時減速並容忍位姿跳動）")
        chunks.append(wp)
    return np.vstack(chunks), notes


# ---------------------------------------------------------------- CLI

def cmd_stations_init(args):
    passes = load_passes(args.routes)
    lines = ["# 站點定義：名稱 + aurora_map 座標 (m)。",
             "# 座標不用準 —— 工具會吸附到 snap_radius 內最近的路徑點。",
             "# 可先看 preview.png / plan --render 的格線來抓座標。",
             "snap_radius: 6.0", "stations:"]
    for p in passes:
        s, e = p["wp"][0], p["wp"][-1]
        lines.append(f"  P{p['id']}S: {{x: {s[0]:.1f}, y: {s[1]:.1f}}}   # pass_{p['id']:02d} 起點")
        lines.append(f"  P{p['id']}E: {{x: {e[0]:.1f}, y: {e[1]:.1f}}}   # pass_{p['id']:02d} 終點")
    out = Path(args.routes) / "stations.yaml"
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"已產生範本 {out} —— 請改成你要的站名（A、B、C…）再執行 plan")


def cmd_info(args):
    passes = load_passes(args.routes)
    stations, snap = load_stations(args.stations or Path(args.routes) / "stations.yaml")
    st2, edges = rebuild_with_junctions(passes, stations, snap, args.allow_reverse,
                                        args.loose_junctions)
    print(f"passes: {len(passes)}   stations: {list(stations)}")
    for e in edges:
        tag = "（逆向,×3成本）" if e["reverse"] else ""
        via = f"pass_{e['pass_id']:02d}" if e["pass_id"] >= 0 else "接續"
        print(f"  {e['u']:>10} → {e['v']:<10} {e['length']:7.1f} m  經 {via} {tag}")


def cmd_plan(args):
    passes = load_passes(args.routes)
    stations, snap = load_stations(args.stations or Path(args.routes) / "stations.yaml")
    st2, edges = rebuild_with_junctions(passes, stations, snap, args.allow_reverse,
                                        args.loose_junctions)

    hops = [args.src] + (args.via or []) + [args.dst]
    edge_seq = []
    for u, v in zip(hops[:-1], hops[1:]):
        seg = dijkstra(edges, u, v)
        if seg is None:
            print(f"!! 找不到 {u} → {v} 的路線。")
            print("   可能原因：該方向沒有示教路段（例如回程只建到一半）。")
            print("   解法：下次建圖補開這一段，或加 --allow-reverse 測試逆向（不建議實駕）。")
            sys.exit(1)
        edge_seq += seg

    wp, notes = plan_waypoints(passes, edges, edge_seq)
    L = float(np.linalg.norm(np.diff(wp[:, :2], axis=0), axis=1).sum())
    print(f"路線 {'→'.join(hops)}：{L:.1f} m，{len(wp)} 個 waypoint，經過：")
    for k in edge_seq:
        e = edges[k]
        if e["pass_id"] >= 0:
            print(f"  {e['u']:>10} → {e['v']:<10} {e['length']:6.1f} m"
                  f"  pass_{e['pass_id']:02d}{'（逆向⚠）' if e['reverse'] else ''}")
    for n in notes:
        print(" ", n)

    out = Path(args.routes) / f"plan_{args.src}_{args.dst}.csv"
    with open(out, "w", encoding="utf-8") as f:
        f.write(f"# 路線 {'→'.join(hops)}, {L:.1f} m  (x,y,yaw_rad)\n")
        for r in wp:
            f.write(f"{r[0]:.3f},{r[1]:.3f},{r[2]:.4f}\n")
    print(f"寫入 {out}")

    if args.render:
        render(passes, st2, stations, wp, hops, Path(args.routes) / f"plan_{args.src}_{args.dst}.png")


def render(passes, st2, stations, wp, hops, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.sans-serif"] = ["Microsoft JhengHei",
                                           "Noto Sans CJK TC", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
    except ImportError:
        print("!! 沒有 matplotlib，略過渲染")
        return
    fig, ax = plt.subplots(figsize=(12.5, 7.5), facecolor="#f9f9f7")
    ax.set_facecolor("#fcfcfb")
    ax.set_aspect("equal")
    ax.grid(True, color="#e1e0d9", lw=0.6)
    for p in passes:                                   # 所有示教路段（淡）
        ax.plot(p["wp"][:, 0], p["wp"][:, 1], color="#c3c2b7", lw=1.2, zorder=2)
    ax.plot(wp[:, 0], wp[:, 1], color="#2a78d6", lw=2.4, zorder=4)   # 規劃路線
    n_arrow = max(len(wp) // 12, 1)                    # 行進方向箭頭
    for i in range(n_arrow // 2, len(wp) - 1, n_arrow):
        ax.annotate("", xy=wp[i + 1, :2], xytext=wp[i, :2],
                    arrowprops=dict(arrowstyle="-|>", color="#2a78d6", lw=1.6), zorder=5)
    for name, xy in stations.items():                  # 站點（實名）
        ax.plot(*xy, "o", ms=9, mfc="#0b0b0b", mec="#fcfcfb", mew=2, zorder=6)
        ax.annotate(name, xy, xytext=(8, 6), textcoords="offset points",
                    color="#0b0b0b", fontsize=12, fontweight="bold", zorder=6)
    ax.plot(*wp[0, :2], "o", ms=11, mfc="#1baf7a", mec="#fcfcfb", mew=2, zorder=7)
    ax.plot(*wp[-1, :2], "s", ms=11, mfc="#e34948", mec="#fcfcfb", mew=2, zorder=7)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"route plan {'→'.join(hops)}   (綠圓=出發, 紅方=目的, 灰=其他示教路段)",
                 fontsize=11, loc="left")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"寫入 {out}")


def main():
    ap = argparse.ArgumentParser(description="路線圖規劃器（站點 + 示教路段）")
    sub = ap.add_subparsers(dest="cmd", required=True)
    common = dict(routes=lambda p: p.add_argument("--routes", default="routes"),
                  stations=lambda p: p.add_argument("--stations", default=None),
                  rev=lambda p: (p.add_argument("--allow-reverse", action="store_true"),
                                 p.add_argument("--loose-junctions", action="store_true",
                                     help="允許接續點吸附到別條 pass（舊行為）：可跨層縫合斷掉的"
                                          "示教鏈（如 routes_compus4），但重疊路段可能產生跳點")))

    p = sub.add_parser("stations-init"); common["routes"](p)
    p.set_defaults(fn=cmd_stations_init)

    p = sub.add_parser("info"); common["routes"](p); common["stations"](p); common["rev"](p)
    p.set_defaults(fn=cmd_info)

    p = sub.add_parser("plan")
    p.add_argument("src"); p.add_argument("dst")
    p.add_argument("--via", nargs="*", default=None, help="中途必經站點")
    p.add_argument("--render", action="store_true")
    common["routes"](p); common["stations"](p); common["rev"](p)
    p.set_defaults(fn=cmd_plan)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
