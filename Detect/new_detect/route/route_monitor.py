#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""route_monitor.py — 現場測試即時儀表板（唯讀，不發 /cmd_vel）

一眼看清楚三件事：
  1. 車在整張地圖的哪裡（真實 SLAM 地圖底圖 + 路線骨架 + 車位姿 + 走過的軌跡 + 站點）
  2. 車有沒有在路線上（到最近路線的橫向誤差；綠=在線上、紅=偏離）
  3. 現在什麼狀態（SLAM 追蹤/重定位/丟失、行進 v/w、路線進度、路面分割覆蓋、位姿新鮮度）

它「只訂閱、不控制」——沒有發 /cmd_vel，所以可與任何駕駛節點（follow_route、
ros_detect_dual、遙控面板）同時跑，零衝突。

「適用任何地圖」：底圖直接來自 Aurora 即時發佈的 OccupancyGrid（/…/map），
自動用地圖原點/解析度定位，換任何 .stcm 都不用改設定。路線 CSV（--routes）是
可選的疊圖；沒有也能只看底圖＋車。

★ 整合地圖模式（--leg）：A/B/C/D 每一段是各自獨立的座標系，載入哪一段的 .stcm，
位姿就在那一段的座標系裡。加上 --leg 之後，本程式用 routes_site/transforms.yaml
把即時位姿換算到整合座標，於是**不管在跑哪一段，看到的都是同一張八段整合地圖**，
車子畫在正確的位置上。位姿和該段路線套用同一個剛體變換，距離不變，所以「離路線
多遠 / 在不在線上」的判定不受配準誤差影響。
（整合座標下 Aurora 的即時 OccupancyGrid 底圖會歪掉，所以 --leg 模式不畫底圖，
 底圖改用八段路線骨架本身。）

介面兩種（比照 ros_teleop_panel.py）：
  python3 route_monitor.py --leg A_B       # ★整合地圖 + 目前跑 A_B（彈出視窗）
  python3 route_monitor.py --leg A_B --web # 改開瀏覽器 http://<本機IP>:8770
  python3 route_monitor.py                 # 只看八段整合地圖（不換算位姿）
  python3 route_monitor.py --demo          # 不連 ROS，用假資料預覽 UI
無畫面（SSH）時彈不出視窗會自動退回 --web。

訂閱的話題（有就顯示、沒有就標「等待中」）：
  /slamware_ros_sdk_server_node/map                   OccupancyGrid 底圖（任何地圖）
  /slamware_ros_sdk_server_node/robot_pose            位姿 (PoseStamped, aurora_map)
  /slamware_ros_sdk_server_node/system_status         裝置狀態 (.status)
  /slamware_ros_sdk_server_node/relocalization_status 重定位狀態 (.status)
  /cmd_vel (Twist) · /route_plan (Path) · /floor_info (Pose) · /floor_detected (Bool)
"""

import argparse
import base64
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

try:
    import yaml
except Exception:
    yaml = None

# ==================== 設定 ====================
PORT            = 8770
POSE_TOPIC      = "/slamware_ros_sdk_server_node/robot_pose"
SYS_TOPIC       = "/slamware_ros_sdk_server_node/system_status"
RELOC_TOPIC     = "/slamware_ros_sdk_server_node/relocalization_status"
MAP_TOPIC       = "/slamware_ros_sdk_server_node/map"
CMD_TOPIC       = "/cmd_vel"
PLAN_TOPIC      = "/route_plan"
FLOOR_TOPIC     = "/floor_info"
FLOOR_DET_TOPIC = "/floor_detected"

ON_ROUTE_M      = 1.5      # 橫向誤差 < 此值算「在路線上」（比照 follow_route 的 MAX_LATERAL）
POSE_STALE_S    = 1.0      # 位姿超過此秒數沒更新算「丟失」（比照 follow_route POSE_TIMEOUT）
TRAIL_MAX       = 1500     # 保留的軌跡點數
FOLLOW_WIN_M    = 34.0     # 「跟隨車」模式的視窗寬（公尺）
# ==============================================


# ------------------------------------------------------------ 路線/站點載入（不需 ROS，可選）
def load_routes(route_dir):
    route_dir = Path(route_dir)
    passes = []
    for csv in sorted(route_dir.glob("pass_*.csv")):
        try:
            rows = np.loadtxt(csv, delimiter=",", comments="#", encoding="utf-8")
            if rows.ndim == 2 and len(rows) >= 2:
                # pass_*.csv 是 kf_id,x,y,yaw（4 欄）→ x,y 在第 1..2 欄；
                # 若只有 x,y,yaw（3 欄）則在第 0..1 欄。
                xy = rows[:, 1:3] if rows.shape[1] >= 4 else rows[:, :2]
                passes.append({"name": csv.stem, "pts": xy.tolist()})
        except Exception:
            pass
    stations = {}
    sf = route_dir / "stations.yaml"
    if yaml and sf.exists():
        try:
            doc = yaml.safe_load(sf.read_text(encoding="utf-8")) or {}
            for name, c in (doc.get("stations") or {}).items():
                stations[name] = [float(c["x"]), float(c["y"])]
        except Exception:
            pass
    allpts = [p for pa in passes for p in pa["pts"]] + list(stations.values())
    if allpts:
        a = np.array(allpts)
        bbox = [float(a[:, 0].min()), float(a[:, 1].min()),
                float(a[:, 0].max()), float(a[:, 1].max())]
    else:
        bbox = None      # 沒有路線檔 → 交給地圖底圖定範圍
    return {"passes": passes, "stations": stations, "bbox": bbox}


def load_site_transform(site_dir, leg):
    """讀 transforms.yaml，回傳把 <leg> 段座標換算成整合座標的函式 (x,y,yaw)->(x,y,yaw)。

    找不到就回 None（呼叫端會退回「不換算」，也就是原本的單段行為）。
    """
    f = Path(site_dir) / "transforms.yaml"
    if not (yaml and f.exists()):
        return None, f"找不到 {f}"
    try:
        doc = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
        e = (doc.get("legs") or {}).get(leg)
        if not e:
            return None, f"{f} 裡沒有 {leg} 這一段"
        th = math.radians(float(e["rot_deg"]))
        tx, ty = float(e["tx"]), float(e["ty"])
        c, sn = math.cos(th), math.sin(th)

        def xf(x, y, yaw):
            return (c * x - sn * y + tx, sn * x + c * y + ty, yaw + th)

        return xf, (f"{leg} → 整合座標：轉 {e['rot_deg']:+.2f}°、移 ({tx:+.1f},{ty:+.1f})"
                    f"（配準殘差中位 {e.get('fit_median_m', '?')} m）")
    except Exception as ex:                              # noqa: BLE001
        return None, f"讀 {f} 失敗：{ex}"


def load_plan_xy(route_dir, plan_name):
    p = Path(route_dir) / plan_name
    if not p.exists():
        p = Path(plan_name)
    if not p.exists():
        return None
    rows = np.loadtxt(p, delimiter=",", comments="#", encoding="utf-8")
    return rows[:, :2]


def polyline_cum(pts):
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def nearest_on_polyline(pts, cum, x, y):
    """回傳 (橫向誤差 m, 沿線里程 m, 最近點索引)。點到線段的真實距離。"""
    a = pts[:-1]
    ab = pts[1:] - a
    l2 = (ab ** 2).sum(1)
    l2[l2 == 0] = 1e-9
    t = np.clip(((np.array([x, y]) - a) * ab).sum(1) / l2, 0, 1)
    proj = a + t[:, None] * ab
    d = np.linalg.norm(proj - (x, y), axis=1)
    k = int(np.argmin(d))
    prog = cum[k] + t[k] * (cum[k + 1] - cum[k])
    return float(d[k]), float(prog), k


def occgrid_to_png(width, height, res, data, origin_xy):
    """OccupancyGrid → (png_bytes, world_extent[l,b,r,t])。灰=未知 白=可走 黑=占據。"""
    import cv2
    if width * height == 0:
        return None, None
    grid = np.array(data, dtype=np.int8).reshape(height, width)
    img = np.full((height, width), 128, np.uint8)   # unknown
    img[grid == 0] = 255                             # free
    img[grid > 0] = 0                                # occupied
    img = cv2.flip(img, 0)                           # ROS 原點在左下 → 影像左上
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        return None, None
    ox, oy = origin_xy
    extent = [ox, oy, ox + width * res, oy + height * res]
    return buf.tobytes(), extent


# ------------------------------------------------------------ 共享狀態
class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.pose = None          # (x, y, yaw, t)
        self.cmd = None           # (v, w, t)
        self.floor = None         # (det, err, head, cover, t)
        self.sys_status = ""
        self.reloc_status = ""
        self.plan = None          # np.array Nx2 (active)
        self.plan_cum = None
        self.plan_src = ""
        self.trail = deque(maxlen=TRAIL_MAX)
        self.map_b64 = None       # OccupancyGrid PNG (base64)
        self.map_extent = None    # [l, b, r, t] world
        self.map_ver = 0

    def set_plan(self, pts, src):
        with self.lock:
            self.plan = np.asarray(pts, float)
            self.plan_cum = polyline_cum(self.plan)
            self.plan_src = src

    def set_map(self, png_bytes, extent):
        with self.lock:
            self.map_b64 = base64.b64encode(png_bytes).decode()
            self.map_extent = extent
            self.map_ver += 1


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def build_state(sh, routes):
    now = time.time()
    with sh.lock:
        pose, cmd, floor = sh.pose, sh.cmd, sh.floor
        sysd, relocd = sh.sys_status, sh.reloc_status
        plan, plan_cum, plan_src = sh.plan, sh.plan_cum, sh.plan_src
        trail = list(sh.trail)

    st = {"t": now, "trail": trail, "plan_src": plan_src}
    st["plan"] = plan.tolist() if plan is not None else None

    if pose:
        age = now - pose[3]
        st["pose"] = {"x": pose[0], "y": pose[1], "yaw": pose[2], "age": age}
        fresh = age < POSE_STALE_S
    else:
        st["pose"] = None
        fresh = False
    st["pose_fresh"] = fresh

    if not pose:
        slam = ("INIT", "等待位姿")
    elif not fresh:
        slam = ("LOST", "位姿逾時 %.1fs" % (now - pose[3]))
    elif "Running" in relocd:
        slam = ("RELOC", "重定位中…")
    elif "Fail" in relocd or "Cancel" in relocd:
        slam = ("LOST", "重定位失敗")
    else:
        slam = ("TRACKING", relocd or sysd or "追蹤中")
    st["slam"] = {"code": slam[0], "detail": slam[1], "sys": sysd, "reloc": relocd}

    lateral, progress, remain = None, None, None
    if pose:
        if plan is not None and len(plan) >= 2:
            lateral, progress, _ = nearest_on_polyline(plan, plan_cum, pose[0], pose[1])
            remain = float(plan_cum[-1] - progress)
            st["plan_len"] = float(plan_cum[-1])
        elif routes["passes"]:
            best = None
            for pa in routes["passes"]:
                pts = np.asarray(pa["pts"])
                if len(pts) < 2:
                    continue
                d, _, _ = nearest_on_polyline(pts, polyline_cum(pts), pose[0], pose[1])
                best = d if best is None else min(best, d)
            lateral = best
    st["lateral"] = lateral
    st["progress"] = progress
    st["remain"] = remain
    st["on_route"] = (lateral is not None and lateral <= ON_ROUTE_M and fresh)

    if pose and routes["stations"]:
        nm, nd = None, None
        for name, c in routes["stations"].items():
            d = math.hypot(c[0] - pose[0], c[1] - pose[1])
            if nd is None or d < nd:
                nm, nd = name, d
        st["nearest_station"] = {"name": nm, "dist": nd}

    if cmd and (now - cmd[2] < POSE_STALE_S):
        st["motion"] = {"v": cmd[0], "w": cmd[1], "age": now - cmd[2],
                        "moving": abs(cmd[0]) > 0.01 or abs(cmd[1]) > 0.01}
    else:
        st["motion"] = None

    if floor and (now - floor[4] < 3.0):
        st["floor"] = {"det": bool(floor[0]), "err": floor[1],
                       "head": floor[2], "cover": floor[3], "age": now - floor[4]}
    else:
        st["floor"] = None

    return st


# ------------------------------------------------------------ HTTP（--web）
def make_handler(sh, routes):
    mapmeta = json.dumps({"passes": routes["passes"], "stations": routes["stations"],
                          "bbox": routes["bbox"]}).encode()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
            elif self.path == "/map":
                self._send(mapmeta, "application/json")
            elif self.path == "/mapimg":
                with sh.lock:
                    d = {"ver": sh.map_ver, "extent": sh.map_extent, "png": sh.map_b64}
                self._send(json.dumps(d).encode(), "application/json")
            elif self.path == "/state":
                self._send(json.dumps(build_state(sh, routes)).encode(), "application/json")
            else:
                self.send_response(204)
                self.end_headers()

    return H


def run_web(sh, routes, is_shutdown, port):
    import subprocess
    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(sh, routes))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        ips = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2).stdout.split()
    except Exception:
        ips = []
    for ip in [i for i in ips if not i.startswith("127.")] or ["<本機IP>"]:
        print(f"    瀏覽器開： http://{ip}:{port}")
    print("Ctrl-C 結束")
    try:
        while not is_shutdown():
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


# ------------------------------------------------------------ Tkinter 彈出視窗（預設）
def pick_cjk_font(root):
    """挑一個「這台機器真的有、而且有中文字」的字型家族。

    原本整個 Tk 介面把字型寫死成 ("Arial", ...)：Arial 沒有中文字，Linux 上
    fontconfig 會換成純拉丁字型，於是所有中文都變成方框。Tk 不像瀏覽器會自動
    fallback 到別的字型，所以得自己挑。
    """
    try:
        import tkinter.font as tkfont
        fams = set(tkfont.families(root))
    except Exception:                                    # noqa: BLE001
        return "TkDefaultFont"
    for f in ("Noto Sans CJK TC", "Noto Sans CJK SC", "Noto Sans CJK JP",
              "Microsoft JhengHei", "PingFang TC", "Heiti TC",
              "WenQuanYi Zen Hei", "WenQuanYi Micro Hei", "Noto Sans TC"):
        if f in fams:
            return f
    # 走到這裡代表系統一個中文字型都沒有 —— 這時候「挑字型」救不了，
    # 得先讓系統有字型。實測：沒字型時每個中文字寬 11px（方框），有字型是 19px。
    print("（注意：這台機器找不到中文字型，介面的中文會顯示成方框）")
    try:
        is_wsl = "microsoft" in Path("/proc/version").read_text().lower()
    except Exception:                                    # noqa: BLE001
        is_wsl = False
    if is_wsl and Path("/mnt/c/Windows/Fonts/msjh.ttc").exists():
        print("  WSL 可以直接借用 Windows 的微軟正黑體，不需要 sudo、不需要網路：")
        print("    mkdir -p ~/.local/share/fonts && \\")
        print("    ln -sf /mnt/c/Windows/Fonts/msjh.ttc ~/.local/share/fonts/ && fc-cache -f")
    else:
        print("  Ubuntu/Debian：sudo apt install fonts-noto-cjk")
    return "TkDefaultFont"


def run_gui(sh, routes, is_shutdown):
    import tkinter as tk

    BG, PANEL, LINE, INK, SUB = "#0e1116", "#161a21", "#232a34", "#e8eaed", "#9aa0a6"
    GOOD, WARN, BAD, ACC, GREY = "#34d399", "#fbbf24", "#f87171", "#4a7fd6", "#39414d"
    root = tk.Tk()
    root.title("Route Monitor")          # 不用 emoji：某些 X11/Tk 對彩色字形會 BadLength 當掉
    FONT = pick_cjk_font(root)
    root.configure(bg=BG)
    root.geometry("1180x680")

    view = {"mode": "fit"}
    cv = tk.Canvas(root, bg=BG, highlightthickness=0)
    cv.pack(side="left", fill="both", expand=True)

    side = tk.Frame(root, bg=PANEL, width=300)
    side.pack(side="right", fill="y")
    side.pack_propagate(False)

    def hdr(t):
        tk.Label(side, text=t, bg=PANEL, fg=SUB, font=(FONT, 10)).pack(anchor="w", padx=14, pady=(10, 0))

    tk.Label(side, text="Route Monitor", bg=PANEL, fg=INK,
             font=(FONT, 14, "bold")).pack(anchor="w", padx=14, pady=(14, 0))
    tk.Label(side, text="現場測試即時狀態（唯讀）", bg=PANEL, fg=SUB,
             font=(FONT, 9)).pack(anchor="w", padx=14)

    badges = {}
    for key, lab in (("slam", "SLAM 狀態"), ("route", "在路線上？"), ("move", "行進")):
        f = tk.Frame(side, bg="#11151b", highlightbackground=LINE, highlightthickness=1)
        f.pack(fill="x", padx=12, pady=5)
        tk.Label(f, text=lab, bg="#11151b", fg=SUB, font=(FONT, 10)).pack(side="left", padx=10, pady=8)
        v = tk.Label(f, text="—", bg="#11151b", fg=SUB, font=(FONT, 14, "bold"))
        v.pack(side="right", padx=10)
        badges[key] = v

    cells = {}
    grid = tk.Frame(side, bg=PANEL)
    grid.pack(fill="x", padx=12, pady=(4, 0))
    for i, (key, lab) in enumerate((("lat", "橫向誤差"), ("age", "位姿新鮮度"),
                                    ("pos", "位置"), ("sta", "最近站點"),
                                    ("prog", "路線進度"), ("floor", "路面分割"))):
        c = tk.Frame(grid, bg="#11151b", highlightbackground=LINE, highlightthickness=1)
        c.grid(row=i // 2, column=i % 2, sticky="ew", padx=3, pady=3)
        grid.grid_columnconfigure(i % 2, weight=1)
        tk.Label(c, text=lab, bg="#11151b", fg=SUB, font=(FONT, 8)).pack(anchor="w", padx=8, pady=(5, 0))
        v = tk.Label(c, text="—", bg="#11151b", fg=INK, font=(FONT, 12, "bold"))
        v.pack(anchor="w", padx=8, pady=(0, 6))
        cells[key] = v

    bar = tk.Frame(side, bg=PANEL)
    bar.pack(fill="x", padx=12, pady=8)
    b_fit = tk.Button(bar, text="整張地圖", bg=ACC, fg="white", relief="flat",
                      command=lambda: view.__setitem__("mode", "fit"))
    b_fit.pack(side="left", expand=True, fill="x", padx=2)
    b_fol = tk.Button(bar, text="跟隨車", bg="#23272f", fg=INK, relief="flat",
                      command=lambda: view.__setitem__("mode", "follow"))
    b_fol.pack(side="left", expand=True, fill="x", padx=2)

    tk.Label(side, text="綠=正常 · 黃=注意 · 紅=異常／偏離\n只訂閱、不發 /cmd_vel，可與駕駛節點同時跑",
             bg=PANEL, fg=SUB, font=(FONT, 8), justify="left").pack(anchor="w", padx=14, pady=10)

    def world_box():
        b = list(routes["bbox"]) if routes["bbox"] else None
        with sh.lock:
            e = sh.map_extent
        if e:
            b = e if b is None else [min(b[0], e[0]), min(b[1], e[1]),
                                     max(b[2], e[2]), max(b[3], e[3])]
        return b or [-1, -1, 1, 1]

    def redraw():
        if is_shutdown():
            root.destroy()
            return
        cv.delete("all")
        W, H = cv.winfo_width(), cv.winfo_height()
        if W < 4 or H < 4:
            root.after(150, redraw)
            return
        st = build_state(sh, routes)
        pose = st.get("pose")
        if view["mode"] == "follow" and pose:
            scale = min(W, H) / FOLLOW_WIN_M
            cx, cy = pose["x"], pose["y"]
        else:
            b = world_box()
            bw, bh = max(b[2] - b[0], 1), max(b[3] - b[1], 1)
            scale = min((W - 40) / bw, (H - 40) / bh)
            cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        sx = lambda x: W / 2 + (x - cx) * scale
        sy = lambda y: H / 2 - (y - cy) * scale

        for pa in routes["passes"]:
            pts = pa["pts"]
            flat = []
            for p in pts:
                flat += [sx(p[0]), sy(p[1])]
            if len(flat) >= 4:
                cv.create_line(*flat, fill=GREY, width=2)
        if st.get("plan"):
            flat = []
            for p in st["plan"]:
                flat += [sx(p[0]), sy(p[1])]
            if len(flat) >= 4:
                cv.create_line(*flat, fill=ACC, width=3)
        if len(st.get("trail") or []) > 1:
            flat = []
            for p in st["trail"]:
                flat += [sx(p[0]), sy(p[1])]
            cv.create_line(*flat, fill="#1f6f4e", width=2)
        for name, c in routes["stations"].items():
            X, Y = sx(c[0]), sy(c[1])
            cv.create_oval(X - 5, Y - 5, X + 5, Y + 5, fill=WARN, outline=BG)
            cv.create_text(X + 12, Y, text=name, fill=INK, font=(FONT, 11, "bold"), anchor="w")
        if pose:
            col = GOOD if (st["pose_fresh"] and st["on_route"]) else BAD
            X, Y, ya = sx(pose["x"]), sy(pose["y"]), pose["yaw"]
            dx, dy = math.cos(ya), -math.sin(ya)
            r = 11
            cv.create_polygon(X + dx * r * 1.6, Y + dy * r * 1.6,
                              X - dx * r - dy * r, Y - dy * r + dx * r,
                              X - dx * r + dy * r, Y - dy * r - dx * r,
                              fill=col, outline=BG, width=2)

        # ---- 面板文字 ----
        def setb(w, color, text):
            badges[w].config(fg=color, text=text)
        sl = st["slam"]["code"]
        setb("slam", {"TRACKING": GOOD, "RELOC": WARN, "LOST": BAD}.get(sl, SUB),
             {"TRACKING": "追蹤中", "RELOC": "重定位中", "LOST": "丟失", "INIT": "等待位姿"}.get(sl, "—"))
        if pose and st["lateral"] is not None:
            setb("route", GOOD if st["on_route"] else BAD, "在路線上" if st["on_route"] else "偏離路線")
        else:
            setb("route", SUB, "—")
        m = st.get("motion")
        if m:
            setb("move", GOOD if m["moving"] else SUB,
                 ("行進 %.2f" % m["v"]) if m["moving"] else "停止")
        else:
            setb("move", SUB, "無 /cmd_vel")

        lat = st["lateral"]
        cells["lat"].config(text="—" if lat is None else "%.2f m" % lat,
                            fg=INK if lat is None else (GOOD if lat <= 1.5 else (WARN if lat <= 2.5 else BAD)))
        cells["age"].config(text="—" if not pose else "%.2f s" % pose["age"],
                            fg=SUB if not pose else (GOOD if st["pose_fresh"] else BAD))
        cells["pos"].config(text="—" if not pose else "(%.1f, %.1f)" % (pose["x"], pose["y"]))
        ns = st.get("nearest_station")
        cells["sta"].config(text="—" if not ns else "%s·%.0fm" % (ns["name"], ns["dist"]))
        if st.get("progress") is not None and st.get("plan_len"):
            cells["prog"].config(text="%.0f/%.0f m" % (st["progress"], st["plan_len"]))
        else:
            cells["prog"].config(text="（等待出發）" if st["plan_src"] == "preload" else "—")
        fl = st.get("floor")
        if fl:
            cells["floor"].config(text="%s %.0f%%" % ("OK" if fl["det"] else "x", 100 * fl["cover"]),
                                  fg=GOOD if fl["cover"] >= 0.10 else BAD)
        else:
            cells["floor"].config(text="無 seg", fg=SUB)

        b_fit.config(bg=ACC if view["mode"] == "fit" else "#23272f")
        b_fol.config(bg=ACC if view["mode"] == "follow" else "#23272f")
        root.after(150, redraw)

    root.bind("<KeyPress-q>", lambda e: root.destroy())
    print("彈出視窗已開啟（遠端手機/平板請用 --web）")
    redraw()
    root.mainloop()


# ------------------------------------------------------------ ROS
def run_ros(sh, plan_pre, xf=None):
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist, Pose
    from nav_msgs.msg import Path as PathMsg, OccupancyGrid
    from std_msgs.msg import Bool

    rospy.init_node("route_monitor", anonymous=True, disable_signals=True)

    def pose_cb(m):
        x, y = m.pose.position.x, m.pose.position.y
        yaw = yaw_from_quat(m.pose.orientation)
        if xf:                                   # 該段座標 → 整合座標
            x, y, yaw = xf(x, y, yaw)
        with sh.lock:
            sh.pose = (x, y, yaw, time.time())
            sh.trail.append([x, y])

    def cmd_cb(m):
        with sh.lock:
            sh.cmd = (m.linear.x, m.angular.z, time.time())

    def plan_cb(m):
        # follow_route 發的 /route_plan 也在該段座標系，一樣要換算
        pts = [[p.pose.position.x, p.pose.position.y] for p in m.poses]
        if xf:
            pts = [list(xf(x, y, 0.0)[:2]) for x, y in pts]
        if len(pts) >= 2:
            sh.set_plan(pts, "route_plan")

    def floor_cb(m):
        with sh.lock:
            sh.floor = (m.orientation.w, m.position.x, m.position.y, m.position.z, time.time())

    def map_cb(m):
        try:
            png, extent = occgrid_to_png(m.info.width, m.info.height, m.info.resolution,
                                         m.data, (m.info.origin.position.x, m.info.origin.position.y))
            if png:
                sh.set_map(png, extent)
        except Exception as e:
            rospy.logwarn_throttle(30, f"map render failed: {e}")

    rospy.Subscriber(POSE_TOPIC, PoseStamped, pose_cb, queue_size=1)
    rospy.Subscriber(CMD_TOPIC, Twist, cmd_cb, queue_size=1)
    rospy.Subscriber(PLAN_TOPIC, PathMsg, plan_cb, queue_size=1)
    rospy.Subscriber(FLOOR_TOPIC, Pose, floor_cb, queue_size=1)
    rospy.Subscriber(FLOOR_DET_TOPIC, Bool, lambda m: None, queue_size=1)
    if xf is None:
        # 整合座標模式下這張 OccupancyGrid 是「該段」的地圖、而且是軸對齊的點陣，
        # 轉過去會歪，所以不訂閱；底圖改用八段路線骨架。
        rospy.Subscriber(MAP_TOPIC, OccupancyGrid, map_cb, queue_size=1)

    try:
        from slamware_ros_sdk.msg import SystemStatus, RelocalizationStatus
        rospy.Subscriber(SYS_TOPIC, SystemStatus,
                         lambda m: sh.__setattr__("sys_status", m.status), queue_size=1)
        rospy.Subscriber(RELOC_TOPIC, RelocalizationStatus,
                         lambda m: sh.__setattr__("reloc_status", m.status), queue_size=1)
    except Exception as e:
        print(f"（注意：載不到 slamware_ros_sdk 訊息型別，SLAM 狀態改用位姿推斷：{e}）")

    if plan_pre is not None:
        sh.set_plan(plan_pre, "preload")
    return rospy


# ------------------------------------------------------------ DEMO（無 ROS）
def run_demo(sh, routes, plan_pre):
    pts = plan_pre
    if pts is None:
        for pa in routes["passes"]:
            if len(pa["pts"]) >= 20:
                pts = np.asarray(pa["pts"]); break
    if pts is None:
        pts = np.array([[0, 0], [10, 0]], float)
    sh.set_plan(pts, "preload")
    cum = polyline_cum(pts)

    # 假底圖：把所有 pass 附近標成可走，示範「任何地圖」的底圖渲染路徑
    if routes["bbox"]:
        try:
            import cv2
            b, res, mg = routes["bbox"], 0.3, 6
            ox, oy = b[0] - mg, b[1] - mg
            Wc = max(int((b[2] - b[0] + 2 * mg) / res), 2)
            Hc = max(int((b[3] - b[1] + 2 * mg) / res), 2)
            img = np.full((Hc, Wc), -1, np.int8)
            for pa in routes["passes"]:
                for x, y in pa["pts"]:
                    cv2.circle(img, (int((x - ox) / res), int((y - oy) / res)),
                               int(2.0 / res), 0, -1)   # 0 = free
            png, extent = occgrid_to_png(Wc, Hc, res, img.flatten().tolist(), (ox, oy))
            if png:
                sh.set_map(png, extent)
        except Exception:
            pass

    def loop():
        s, rng = 0.0, np.random.default_rng(1)
        while True:
            s = (s + 0.2 * 0.1) % cum[-1]
            i = min(max(int(np.searchsorted(cum, s)), 1), len(pts) - 1)
            p0, p1 = pts[i - 1], pts[i]
            yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            base = 0.15 * math.sin(s * 0.6)
            exc = 1.8 if (int(s) % 40 < 4) else 0.0
            off = base + exc
            nx, ny = -math.sin(yaw), math.cos(yaw)
            x, y = p1[0] + nx * off, p1[1] + ny * off
            now = time.time()
            with sh.lock:
                sh.pose = (x, y, yaw + rng.normal(0, 0.02), now)
                sh.trail.append([x, y])
                sh.cmd = (0.0 if exc else 0.2, 0.1 * math.sin(s), now)
                sh.floor = (0.0 if exc else 1.0, base * 0.5, 0.02,
                            0.05 if exc else 0.35 + 0.1 * math.sin(s * 2), now)
                sh.sys_status = "DeviceRunning"
                sh.reloc_status = "RelocalizationSucceed"
            time.sleep(0.1)

    threading.Thread(target=loop, daemon=True).start()


# ------------------------------------------------------------ 網頁
PAGE = r"""<!DOCTYPE html>
<html lang="zh-TW"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Route Monitor</title>
<style>
  :root{--bg:#0e1116;--panel:#161a21;--line:#232a34;--ink:#e8eaed;--sub:#9aa0a6;
        --good:#34d399;--warn:#fbbf24;--bad:#f87171;--accent:#4a7fd6;}
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  body{margin:0;background:var(--bg);color:var(--ink);font-family:system-ui,"Noto Sans CJK TC",sans-serif;
       display:flex;height:100vh;overflow:hidden}
  #left{flex:1;position:relative;min-width:0}
  canvas{position:absolute;inset:0;width:100%;height:100%;display:block}
  #hud{position:absolute;left:10px;top:10px;font-size:12px;color:var(--sub);
       background:rgba(14,17,22,.6);padding:6px 9px;border-radius:8px;line-height:1.5}
  #btns{position:absolute;right:10px;top:10px;display:flex;gap:6px}
  #btns button{background:var(--panel);color:var(--ink);border:1px solid var(--line);
       border-radius:8px;padding:7px 11px;font-size:13px;cursor:pointer}
  #btns button.on{background:var(--accent);border-color:var(--accent)}
  #panel{width:320px;background:var(--panel);border-left:1px solid var(--line);
         padding:14px;overflow-y:auto;display:flex;flex-direction:column;gap:10px}
  h1{font-size:15px;margin:0 0 2px;font-weight:600}
  .sub{color:var(--sub);font-size:12px;margin:0 0 6px}
  .badge{border-radius:10px;padding:10px 12px;display:flex;align-items:center;
         justify-content:space-between;background:#11151b;border:1px solid var(--line)}
  .badge .lab{font-size:12px;color:var(--sub)}
  .badge .val{font-size:17px;font-weight:700;display:flex;align-items:center;gap:7px}
  .dot{width:11px;height:11px;border-radius:50%;flex:none}
  .g{color:var(--good)} .w{color:var(--warn)} .b{color:var(--bad)} .n{color:var(--sub)}
  .dg{background:var(--good)} .dw{background:var(--warn)} .db{background:var(--bad)} .dn{background:var(--sub)}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .cell{background:#11151b;border:1px solid var(--line);border-radius:9px;padding:8px 10px}
  .cell .k{font-size:11px;color:var(--sub)} .cell .v{font-size:15px;font-weight:600;margin-top:2px;
         font-variant-numeric:tabular-nums}
  .bar{height:7px;border-radius:4px;background:#11151b;border:1px solid var(--line);overflow:hidden;margin-top:5px}
  .bar>i{display:block;height:100%;background:var(--accent)}
  #foot{color:var(--sub);font-size:11px;margin-top:auto;line-height:1.6}
  @media(max-width:760px){body{flex-direction:column}#panel{width:100%;height:46%;border-left:0;border-top:1px solid var(--line)}}
</style></head>
<body>
<div id="left">
  <canvas id="cv"></canvas>
  <div id="hud">—</div>
  <div id="btns"><button id="bFit" class="on">整張地圖</button><button id="bFollow">跟隨車</button></div>
</div>
<div id="panel">
  <div><h1>🛰️ Route Monitor</h1><p class="sub">現場測試即時狀態（唯讀）</p></div>
  <div class="badge"><span class="lab">SLAM 狀態</span><span class="val" id="bSlam"><span class="dot dn"></span><span class="n">—</span></span></div>
  <div class="badge"><span class="lab">在路線上？</span><span class="val" id="bRoute"><span class="dot dn"></span><span class="n">—</span></span></div>
  <div class="badge"><span class="lab">行進</span><span class="val" id="bMove"><span class="dot dn"></span><span class="n">—</span></span></div>
  <div class="grid">
    <div class="cell"><div class="k">橫向誤差</div><div class="v" id="cLat">—</div></div>
    <div class="cell"><div class="k">位姿新鮮度</div><div class="v" id="cAge">—</div></div>
    <div class="cell"><div class="k">位置 (x, y)</div><div class="v" id="cPos" style="font-size:13px">—</div></div>
    <div class="cell"><div class="k">最近站點</div><div class="v" id="cSta">—</div></div>
  </div>
  <div class="cell"><div class="k">路線進度</div><div class="v" id="cProg">—</div>
       <div class="bar"><i id="cProgBar" style="width:0%"></i></div></div>
  <div class="cell"><div class="k">路面分割（覆蓋 / 偏移 / 頭向）</div><div class="v" id="cFloor" style="font-size:13px">—</div>
       <div class="bar"><i id="cCover" style="width:0%;background:var(--good)"></i></div></div>
  <div id="foot">綠=正常 · 黃=注意 · 紅=異常／偏離<br>此面板只訂閱話題、不發 /cmd_vel，可與駕駛節點同時跑。</div>
</div>
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
let MAP=null, VIEW='fit', MAPIMG={ver:-1,img:null,extent:null};
const bFit=document.getElementById('bFit'),bFollow=document.getElementById('bFollow');
bFit.onclick=()=>{VIEW='fit';bFit.classList.add('on');bFollow.classList.remove('on')};
bFollow.onclick=()=>{VIEW='follow';bFollow.classList.add('on');bFit.classList.remove('on')};
function fit(){const r=cv.getBoundingClientRect();cv.width=r.width*devicePixelRatio;cv.height=r.height*devicePixelRatio;}
addEventListener('resize',fit);
fetch('/map').then(r=>r.json()).then(m=>{MAP=m;fit();});
let S=null;
async function poll(){try{S=await (await fetch('/state')).json();}catch(e){}}
setInterval(poll,150);
async function pollMap(){try{const d=await (await fetch('/mapimg')).json();
  if(d&&d.png&&d.ver!==MAPIMG.ver){const im=new Image();
    im.onload=()=>{MAPIMG.img=im;MAPIMG.extent=d.extent;MAPIMG.ver=d.ver;};
    im.src='data:image/png;base64,'+d.png;}}catch(e){}}
setInterval(pollMap,1500);pollMap();

function worldBox(){let b=(MAP&&MAP.bbox)?MAP.bbox.slice():null;const e=MAPIMG.extent;
  if(e){b=b?[Math.min(b[0],e[0]),Math.min(b[1],e[1]),Math.max(b[2],e[2]),Math.max(b[3],e[3])]:e.slice();}
  return b||[-1,-1,1,1];}
function tf(st){const W=cv.width,H=cv.height,pad=28*devicePixelRatio;let cx,cy,scale;
  if(VIEW==='follow'&&st&&st.pose){scale=Math.min(W,H)/34;cx=st.pose.x;cy=st.pose.y;}
  else{const b=worldBox(),bw=Math.max(b[2]-b[0],1),bh=Math.max(b[3]-b[1],1);
    scale=Math.min((W-2*pad)/bw,(H-2*pad)/bh);cx=(b[0]+b[2])/2;cy=(b[1]+b[3])/2;}
  return {sx:x=>W/2+(x-cx)*scale, sy:y=>H/2-(y-cy)*scale, scale};}

function draw(){requestAnimationFrame(draw);if(!MAP){return;}
  const W=cv.width,H=cv.height,st=S,dp=devicePixelRatio;
  ctx.clearRect(0,0,W,H);ctx.fillStyle='#0e1116';ctx.fillRect(0,0,W,H);
  const T=tf(st);
  if(MAPIMG.img&&MAPIMG.extent){const e=MAPIMG.extent;
    ctx.imageSmoothingEnabled=false;ctx.globalAlpha=0.9;
    ctx.drawImage(MAPIMG.img,T.sx(e[0]),T.sy(e[3]),(e[2]-e[0])*T.scale,(e[3]-e[1])*T.scale);
    ctx.globalAlpha=1;}
  ctx.lineWidth=1.5*dp;ctx.strokeStyle='#39414d';
  for(const pa of MAP.passes){ctx.beginPath();
    pa.pts.forEach((p,i)=>{const X=T.sx(p[0]),Y=T.sy(p[1]);i?ctx.lineTo(X,Y):ctx.moveTo(X,Y);});ctx.stroke();}
  if(st&&st.plan){ctx.lineWidth=3*dp;ctx.strokeStyle='#4a7fd6';ctx.beginPath();
    st.plan.forEach((p,i)=>{const X=T.sx(p[0]),Y=T.sy(p[1]);i?ctx.lineTo(X,Y):ctx.moveTo(X,Y);});ctx.stroke();}
  if(st&&st.trail&&st.trail.length>1){ctx.lineWidth=2*dp;ctx.strokeStyle='rgba(52,211,153,.55)';ctx.beginPath();
    st.trail.forEach((p,i)=>{const X=T.sx(p[0]),Y=T.sy(p[1]);i?ctx.lineTo(X,Y):ctx.moveTo(X,Y);});ctx.stroke();}
  ctx.font=(12*dp)+'px system-ui,"Noto Sans CJK TC",sans-serif';ctx.textBaseline='middle';
  for(const [name,c] of Object.entries(MAP.stations)){const X=T.sx(c[0]),Y=T.sy(c[1]);
    ctx.fillStyle='#fbbf24';ctx.beginPath();ctx.arc(X,Y,5*dp,0,7);ctx.fill();
    ctx.fillStyle='#e8eaed';ctx.fillText(name,X+9*dp,Y);}
  if(st&&st.pose){const X=T.sx(st.pose.x),Y=T.sy(st.pose.y),ya=st.pose.yaw,r=10*dp;
    const col=!st.pose_fresh?'#f87171':(st.on_route?'#34d399':'#f87171');
    const dx=Math.cos(ya),dy=-Math.sin(ya);
    ctx.fillStyle=col;ctx.beginPath();ctx.moveTo(X+dx*r*1.6,Y+dy*r*1.6);
    ctx.lineTo(X-dx*r-dy*r,Y-dy*r+dx*r);ctx.lineTo(X-dx*r+dy*r,Y-dy*r-dx*r);ctx.closePath();ctx.fill();
    ctx.strokeStyle='#0e1116';ctx.lineWidth=2*dp;ctx.stroke();}
  const hud=document.getElementById('hud');
  hud.textContent=(MAPIMG.img?'底圖✓ · ':'底圖等待中 · ')+MAP.passes.length+' pass · '+
     Object.keys(MAP.stations).length+' 站 · '+(VIEW==='fit'?'全圖':'跟車 34m');
  updatePanel(st);}
function setBadge(id,code,text){const el=document.getElementById(id);
  const map={g:['dg','g'],w:['dw','w'],b:['db','b'],n:['dn','n']},c=map[code]||map.n;
  el.innerHTML='<span class="dot '+c[0]+'"></span><span class="'+c[1]+'">'+text+'</span>';}
function updatePanel(st){if(!st){return;}
  const sl=st.slam||{},sc={TRACKING:['g','追蹤中'],RELOC:['w','重定位中'],LOST:['b','丟失'],INIT:['n','等待位姿']}[sl.code]||['n','—'];
  setBadge('bSlam',sc[0],sc[1]);
  if(st.pose&&st.lateral!=null) setBadge('bRoute',st.on_route?'g':'b',st.on_route?'在路線上':'偏離路線');
  else setBadge('bRoute','n','—');
  if(st.motion) setBadge('bMove',st.motion.moving?'g':'n',st.motion.moving?('行進 '+st.motion.v.toFixed(2)+' m/s'):'停止');
  else setBadge('bMove','n','無 /cmd_vel');
  const g=id=>document.getElementById(id);
  g('cLat').textContent=st.lateral!=null?st.lateral.toFixed(2)+' m':'—';
  g('cLat').className='v '+(st.lateral==null?'':(st.lateral<=1.5?'g':(st.lateral<=2.5?'w':'b')));
  g('cAge').textContent=st.pose?(st.pose.age.toFixed(2)+' s'):'—';
  g('cAge').className='v '+(!st.pose?'':(st.pose_fresh?'g':'b'));
  g('cPos').textContent=st.pose?('('+st.pose.x.toFixed(1)+', '+st.pose.y.toFixed(1)+')'):'—';
  g('cSta').textContent=st.nearest_station?(st.nearest_station.name+' · '+st.nearest_station.dist.toFixed(0)+'m'):'—';
  if(st.progress!=null&&st.plan_len){g('cProg').textContent=st.progress.toFixed(1)+' / '+st.plan_len.toFixed(0)+' m  (剩 '+st.remain.toFixed(0)+')';
     g('cProgBar').style.width=Math.max(0,Math.min(100,100*st.progress/st.plan_len))+'%';}
  else{g('cProg').textContent=st.plan_src==='preload'?'（等待出發）':'—';g('cProgBar').style.width='0%';}
  if(st.floor){g('cFloor').textContent=(st.floor.det?'✓ ':'✗ ')+(100*st.floor.cover).toFixed(0)+'% · 偏 '+st.floor.err.toFixed(2)+' · 頭 '+st.floor.head.toFixed(2);
     g('cCover').style.width=Math.max(0,Math.min(100,100*st.floor.cover))+'%';
     g('cCover').style.background=st.floor.cover<0.10?'var(--bad)':'var(--good)';}
  else{g('cFloor').textContent='無 /floor_info';g('cCover').style.width='0%';}}
draw();
</script></body></html>
"""


# ------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description="現場測試即時儀表板（唯讀）")
    ap.add_argument("--routes", default="routes_site",
                    help="路線資料夾（含 pass_*.csv 與 stations.yaml）；預設為八段整合地圖")
    ap.add_argument("--leg", default=None, metavar="LEG",
                    help="★車目前跑的路段（例 A_B）：把即時位姿從該段座標換算到整合座標")
    ap.add_argument("--site", default="routes_site",
                    help="整合地圖資料夾（含 transforms.yaml），--leg 用它換算")
    ap.add_argument("--plan", default=None, help="預先高亮的 plan CSV（例：plan_A_B.csv）")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--web", action="store_true", help="改用瀏覽器面板（手機/平板遠端看）")
    ap.add_argument("--demo", action="store_true", help="不連 ROS，用假資料預覽 UI")
    args = ap.parse_args()

    here = Path(__file__).resolve().parent
    def resolve(p):
        return Path(p) if Path(p).is_absolute() else (here / p)

    site_dir = resolve(args.site)
    xf, xf_note = None, None
    route_dir = resolve(args.routes)
    if args.leg:
        xf, xf_note = load_site_transform(site_dir, args.leg)
        print(f"整合地圖：{xf_note}")
        if xf is None:
            print("  → 換算不了，改用單段模式（位姿照原座標畫）"
                  "；先跑 merge_site_map.py 產生 routes_site/")
        else:
            route_dir = site_dir            # 顯示整合地圖
    routes = load_routes(route_dir)
    bb = routes["bbox"]
    print(f"載入 {len(routes['passes'])} 條 pass、{len(routes['stations'])} 個站點；" +
          (f"路線範圍 x[{bb[0]:.0f},{bb[2]:.0f}] y[{bb[1]:.0f},{bb[3]:.0f}] m"
           if bb else "沒有路線檔 → 底圖範圍改由 ROS 地圖決定"))

    plan_pre = None
    if args.plan:
        plan_pre = load_plan_xy(route_dir, args.plan)
        print(f"預載計畫路線 {args.plan}：{'找不到' if plan_pre is None else str(len(plan_pre)) + ' 點'}")
    elif args.leg and xf is not None:
        # 整合地圖模式：直接把該段（已在整合座標的）折線當作計畫路線，
        # 這樣進度/橫向誤差是對「正在跑的那一段」算的，且距離完全精確。
        plan_pre = load_plan_xy(site_dir, f"leg_{args.leg}.csv")
        if plan_pre is not None:
            print(f"高亮目前路段 {args.leg}：{len(plan_pre)} 點")

    sh = Shared()
    if args.demo:
        run_demo(sh, routes, plan_pre)
        is_shutdown = lambda: False
        print("=== DEMO 模式（未連 ROS，假資料）===")
    else:
        try:
            rospy = run_ros(sh, plan_pre, xf)
            is_shutdown = rospy.is_shutdown
        except Exception as e:
            print(f"連 ROS 失敗（{e}）。改用 --demo 可離線預覽 UI。")
            return

    if args.web:
        run_web(sh, routes, is_shutdown, args.port)
    else:
        try:
            run_gui(sh, routes, is_shutdown)
        except Exception as e:
            print(f"開不了視窗（{e}）→ 自動改用網頁版")
            run_web(sh, routes, is_shutdown, args.port)


if __name__ == "__main__":
    main()
