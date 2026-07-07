#!/usr/bin/env python3
"""手動遙控面板 (Teleop Panel) — 預設彈出視窗，--web 改用瀏覽器

直接執行就彈出控制視窗（Tkinter）：
  python3 Detect/new_detect/ros_teleop_panel.py           # 彈出視窗，發布 /cmd_vel
  python3 Detect/new_detect/ros_teleop_panel.py --sim     # 室內測試：只印指令，不需要 ROS
  python3 Detect/new_detect/ros_teleop_panel.py --web     # 改開網頁版 http://<車上PC的IP>:8765
                                                          # （Windows/手機瀏覽器遠端遙控用）

操作（視窗版與網頁版相同）：
  - 3x3 方向鍵：前進/後退/原地左右轉；斜角鍵 = 邊走邊轉（弧線行駛）
  - 「按住才動、放開就停」；視窗關閉 / 網頁斷線 0.6 秒內自動停車（watchdog）
  - 兩條滑桿即時調整線速度 (m/s) 與角速度 (rad/s)
  - 鍵盤：W/S 前後、A/D 原地左右轉、Q/E 前進弧線、Z/C 後退弧線、空白鍵 = 立即停車
    （方向鍵 ↑↓←→ 同 WASD；可同時按 W+A 組合出弧線）

注意：
  - 本節點不開相機，可與任何 ros_detect_*.py 感知節點同時跑
  - 勿與「同樣在發 /cmd_vel」的節點同跑（drive 模式的 ros_detect_dual.py、ros_move_*.py）
  - 面板閒置時不發 /cmd_vel（放開按鍵 1 秒後停止發布），不會佔住話題
  - SSH 無畫面環境開不了視窗時會自動退回網頁版
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from xmlrpc.client import ServerProxy

# ==================== 設定 ====================
PORT          = 8765
PUB_RATE_HZ   = 20.0    # /cmd_vel 發布頻率（底盤有逾時保護，必須連續發）
WATCHDOG_SEC  = 0.6     # 超過這個秒數沒收到 UI 心跳 → 視為斷線/凍結，停車
ZERO_TAIL_SEC = 1.0     # 停止後再發 1 秒的零速度，確保底盤收到，之後閉嘴

V_MIN, V_MAX, V_DEFAULT = 0.05, 0.60, 0.20   # 線速度滑桿範圍 (m/s)
W_MIN, W_MAX, W_DEFAULT = 0.10, 1.50, 0.50   # 角速度滑桿範圍 (rad/s)
# ==============================================

# 3x3 方向鍵：(符號, 說明, vx, wz)。wz=+1 為左轉（ROS 慣例 CCW）
PAD_LAYOUT = [
    ("⬉", "fwd-left",  1,  1), ("⬆", "forward",  1, 0), ("⬈", "fwd-right",  1, -1),
    ("⟲", "spin left", 0,  1), ("■", "stop",     0, 0), ("⟳", "spin right", 0, -1),
    ("⬋", "back-left", -1, -1), ("⬇", "backward", -1, 0), ("⬊", "back-right", -1, 1),
]


class CmdState:
    """UI 最新指令 + watchdog。vx/wz 是 -1/0/+1 方向，v/w 是滑桿速度值。"""

    def __init__(self):
        self._lock = threading.Lock()
        self.vx = 0.0
        self.wz = 0.0
        self.v  = V_DEFAULT
        self.w  = W_DEFAULT
        self._ts = 0.0

    def set(self, vx, wz, v, w):
        with self._lock:
            self.vx = max(-1.0, min(1.0, float(vx)))
            self.wz = max(-1.0, min(1.0, float(wz)))
            self.v  = max(V_MIN, min(V_MAX, float(v)))
            self.w  = max(W_MIN, min(W_MAX, float(w)))
            self._ts = time.time()
            return self.vx * self.v, self.wz * self.w

    def twist(self):
        """回傳 (linear, angular, active)。心跳逾時（斷線/UI 凍結）一律回零。"""
        with self._lock:
            if time.time() - self._ts > WATCHDOG_SEC:
                return 0.0, 0.0, False
            lin = self.vx * self.v
            ang = self.wz * self.w
            return lin, ang, (lin != 0.0 or ang != 0.0)


def pub_loop(state, publish, stop_evt, is_shutdown):
    """按住 → 連續發 /cmd_vel；放開/斷線 → 發 1 秒零速度後閉嘴。"""
    period = 1.0 / PUB_RATE_HZ
    zero_tail_until = 0.0
    while not stop_evt.is_set() and not is_shutdown():
        lin, ang, active = state.twist()
        now = time.time()
        if active:
            publish(lin, ang)
            zero_tail_until = now + ZERO_TAIL_SEC
        elif now < zero_tail_until:
            publish(0.0, 0.0)
        time.sleep(period)


# ==================== Tkinter 視窗版 ====================

def run_gui(state, sim, is_shutdown):
    import tkinter as tk

    BG, PANEL, BTN, BTN_ON, FG, SUB = ("#14171c", "#1c2026", "#23272f",
                                       "#2b5cab", "#e8eaed", "#9aa0a6")
    root = tk.Tk()
    root.title("🚗 Car Teleop" + ("  [SIM 未連 ROS]" if sim else ""))
    root.configure(bg=BG)
    root.resizable(False, False)

    status = tk.Label(root, text="idle", font=("Arial", 12, "bold"),
                      bg=PANEL, fg=SUB, width=36, pady=8)
    status.grid(row=0, column=0, padx=14, pady=(14, 8))

    # ── 3x3 方向鍵（按住才動）──
    btn_cmd = [None]          # 按鈕按住時的指令（優先於鍵盤）
    pressed = {}              # 鍵盤按住狀態
    release_jobs = {}         # X11 autorepeat 的假放開 → 延遲確認

    pad = tk.Frame(root, bg=BG)
    pad.grid(row=1, column=0, padx=14)
    btn_map = {}
    for i, (sym, name, vx, wz) in enumerate(PAD_LAYOUT):
        b = tk.Button(pad, text=f"{sym}\n{name}", font=("Arial", 13),
                      width=8, height=3, bg=BTN, fg=FG,
                      activebackground=BTN_ON, activeforeground=FG,
                      relief="flat", takefocus=0)
        b.grid(row=i // 3, column=i % 3, padx=5, pady=5)
        b.bind("<ButtonPress-1>",   lambda e, c=(vx, wz): btn_cmd.__setitem__(0, c))
        b.bind("<ButtonRelease-1>", lambda e: btn_cmd.__setitem__(0, None))
        btn_map[b] = (vx, wz)

    def stop_all():
        btn_cmd[0] = None
        pressed.clear()

    estop = tk.Button(root, text="STOP  (SPACE)", font=("Arial", 14, "bold"),
                      bg="#c62828", fg="white", activebackground="#ff5252",
                      relief="flat", height=2, takefocus=0, command=stop_all)
    estop.grid(row=2, column=0, sticky="ew", padx=14, pady=(10, 4))

    def make_scale(row, label, lo, hi, res, default):
        s = tk.Scale(root, from_=lo, to=hi, resolution=res, orient="horizontal",
                     label=label, length=330, bg=PANEL, fg=FG, troughcolor=BTN,
                     highlightthickness=0, font=("Arial", 10), takefocus=0)
        s.set(default)
        s.grid(row=row, column=0, padx=14, pady=4)
        return s

    v_scale = make_scale(3, "Linear speed 線速度 (m/s)",   V_MIN, V_MAX, 0.01, V_DEFAULT)
    w_scale = make_scale(4, "Angular speed 角速度 (rad/s)", W_MIN, W_MAX, 0.05, W_DEFAULT)

    tk.Label(root, text="按住才會動，放開就停 · 關窗自動停車\n"
                        "W/S 前後 · A/D 原地轉 · Q/E/Z/C 弧線 · SPACE 停",
             font=("Arial", 9), bg=BG, fg=SUB, justify="center"
             ).grid(row=5, column=0, pady=(6, 12))

    # ── 鍵盤（處理 X11 autorepeat：放開後 60ms 內又按下 = 假放開，忽略）──
    KEYS = {"w", "a", "s", "d", "q", "e", "z", "c", "up", "down", "left", "right"}

    def key_cmd():
        vx = wz = 0
        if pressed.get("w") or pressed.get("up"):      vx = 1
        elif pressed.get("s") or pressed.get("down"):  vx = -1
        if pressed.get("a") or pressed.get("left"):    wz = 1
        elif pressed.get("d") or pressed.get("right"): wz = -1
        for k, (a, b) in {"q": (1, 1), "e": (1, -1), "z": (-1, 1), "c": (-1, -1)}.items():
            if pressed.get(k):
                vx, wz = a, b
        return vx, wz

    def on_press(ev):
        k = ev.keysym.lower()
        if k == "space":
            stop_all()
            return
        if k in KEYS:
            job = release_jobs.pop(k, None)
            if job:
                root.after_cancel(job)
            pressed[k] = True

    def on_release(ev):
        k = ev.keysym.lower()
        if k in KEYS:
            release_jobs[k] = root.after(60, lambda: (release_jobs.pop(k, None),
                                                      pressed.pop(k, None)))

    root.bind("<KeyPress>", on_press)
    root.bind("<KeyRelease>", on_release)

    # ── 心跳：每 120ms 把目前指令寫進 CmdState（UI 凍結 → watchdog 停車）──
    def tick():
        if is_shutdown():
            root.destroy()
            return
        vx, wz = btn_cmd[0] if btn_cmd[0] is not None else key_cmd()
        lin, ang = state.set(vx, wz, v_scale.get(), w_scale.get())
        if lin != 0.0 or ang != 0.0:
            status.config(text=f"DRIVING   v={lin:+.2f} m/s   w={ang:+.2f} rad/s",
                          fg="#6cd97e", bg="#103a1c")
        else:
            status.config(text="idle", fg=SUB, bg=PANEL)
        for b, c in btn_map.items():
            b.config(bg=BTN_ON if c == (vx, wz) and c != (0, 0) else BTN)
        root.after(120, tick)

    root.protocol("WM_DELETE_WINDOW", root.destroy)
    tick()
    print("遙控視窗已開啟（遠端瀏覽器版請用 --web）")
    root.mainloop()


# ==================== 網頁版 ====================

PAGE = """<!DOCTYPE html>
<html lang="zh-TW">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Car Teleop</title>
<style>
  * { box-sizing: border-box; -webkit-user-select: none; user-select: none;
      -webkit-tap-highlight-color: transparent; }
  body { margin: 0; background: #14171c; color: #e8eaed; font-family: system-ui, sans-serif;
         display: flex; flex-direction: column; align-items: center; min-height: 100vh; }
  h1 { font-size: 1.1rem; margin: 12px 0 4px; font-weight: 600; }
  #status { font-size: .95rem; margin: 4px 0 10px; padding: 4px 14px; border-radius: 6px;
            background: #23272f; color: #9aa0a6; min-width: 260px; text-align: center; }
  #status.run  { background: #103a1c; color: #6cd97e; }
  #status.dead { background: #4a1414; color: #ff8a80; }
  #pad { display: grid; grid-template-columns: repeat(3, 96px); grid-gap: 10px; margin: 6px 0; }
  #pad button { height: 96px; font-size: 1.5rem; border: 1px solid #3a4048; border-radius: 14px;
                background: #23272f; color: #e8eaed; touch-action: none; cursor: pointer; }
  #pad button small { display: block; font-size: .62rem; color: #9aa0a6; margin-top: 2px; }
  #pad button.on, #pad button:active { background: #2b5cab; border-color: #4a7fd6; }
  #estop { width: 318px; height: 58px; margin: 12px 0 6px; font-size: 1.2rem; font-weight: 700;
           border: 0; border-radius: 12px; background: #c62828; color: #fff; cursor: pointer; }
  #estop:active { background: #ff5252; }
  .slider { width: 318px; margin: 8px 0; background: #1c2026; border-radius: 10px; padding: 10px 14px; }
  .slider label { display: flex; justify-content: space-between; font-size: .9rem; color: #bdc1c6; }
  .slider input { width: 100%; margin-top: 6px; accent-color: #4a7fd6; }
  .slider b { color: #e8eaed; font-variant-numeric: tabular-nums; }
  #hint { font-size: .75rem; color: #6b7178; margin: 8px 12px 16px; text-align: center; line-height: 1.5; }
</style>
</head>
<body oncontextmenu="return false">
<h1>🚗 Car Teleop Panel</h1>
<div id="status">connecting…</div>

<div id="pad">__PAD_BUTTONS__</div>

<button id="estop">STOP&nbsp;(SPACE)</button>

<div class="slider">
  <label>Linear speed 線速度 <b><span id="vVal"></span> m/s</b></label>
  <input id="vSlider" type="range" min="__V_MIN__" max="__V_MAX__" step="0.01" value="__V_DEF__">
</div>
<div class="slider">
  <label>Angular speed 角速度 <b><span id="wVal"></span> rad/s</b></label>
  <input id="wSlider" type="range" min="__W_MIN__" max="__W_MAX__" step="0.05" value="__W_DEF__">
</div>

<div id="hint">按住按鍵才會動，放開就停 · 斷線 0.6s 自動停車<br>
鍵盤：W/S 前後 · A/D 原地轉 · Q/E/Z/C 弧線（或 W+A 組合）· SPACE 停</div>

<script>
const vSlider = document.getElementById('vSlider');
const wSlider = document.getElementById('wSlider');
const vVal = document.getElementById('vVal');
const wVal = document.getElementById('wVal');
const statusEl = document.getElementById('status');
const pads = Array.from(document.querySelectorAll('#pad button'));

let btnCmd = null;          // 按鈕按住時的指令（優先）
const keys = {};            // 鍵盤按住狀態

function keyCmd() {
  let vx = 0, wz = 0;
  if (keys['w'] || keys['arrowup'])    vx = 1;
  else if (keys['s'] || keys['arrowdown']) vx = -1;
  if (keys['a'] || keys['arrowleft'])  wz = 1;
  else if (keys['d'] || keys['arrowright']) wz = -1;
  if (keys['q']) { vx = 1;  wz = 1; }
  if (keys['e']) { vx = 1;  wz = -1; }
  if (keys['z']) { vx = -1; wz = 1; }
  if (keys['c']) { vx = -1; wz = -1; }
  return { vx, wz };
}
function current() { return btnCmd !== null ? btnCmd : keyCmd(); }

function updateUI() {
  vVal.textContent = (+vSlider.value).toFixed(2);
  wVal.textContent = (+wSlider.value).toFixed(2);
  const c = current();
  pads.forEach(b => b.classList.toggle('on',
    +b.dataset.vx === c.vx && +b.dataset.wz === c.wz && (c.vx !== 0 || c.wz !== 0)));
}

function send() {
  const c = current();
  fetch('/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ vx: c.vx, wz: c.wz, v: +vSlider.value, w: +wSlider.value }) })
  .then(r => r.json()).then(j => {
    const moving = j.lin !== 0 || j.ang !== 0;
    statusEl.className = moving ? 'run' : '';
    statusEl.textContent = (moving ? 'DRIVING ' : 'idle ') +
      ' v=' + j.lin.toFixed(2) + ' m/s  w=' + j.ang.toFixed(2) + ' rad/s' +
      (j.sim ? '  [SIM]' : '');
  })
  .catch(() => {
    statusEl.className = 'dead';
    statusEl.textContent = 'DISCONNECTED — car stops itself in 0.6 s';
  });
  updateUI();
}
setInterval(send, 150);     // 心跳：斷了 server 端 watchdog 會自動停車

function stopAll() {
  btnCmd = null;
  for (const k in keys) keys[k] = false;
  send();
}

// ── 方向鍵（pointer 事件涵蓋滑鼠與觸控）──
pads.forEach(b => {
  b.addEventListener('pointerdown', e => {
    e.preventDefault();
    btnCmd = { vx: +b.dataset.vx, wz: +b.dataset.wz };
    send();
  });
  ['pointerup', 'pointercancel', 'pointerleave'].forEach(ev =>
    b.addEventListener(ev, () => { if (btnCmd) { btnCmd = null; send(); } }));
});

document.getElementById('estop').addEventListener('click', stopAll);

// ── 鍵盤 ──
window.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;   // 滑桿聚焦時不搶方向鍵
  const k = e.key.toLowerCase();
  if (k === ' ') { e.preventDefault(); stopAll(); return; }
  if ('wasdqezc'.includes(k) || k.startsWith('arrow')) {
    e.preventDefault();
    if (!keys[k]) { keys[k] = true; send(); }
  }
});
window.addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if (keys[k]) { keys[k] = false; send(); }
});
window.addEventListener('blur', stopAll);   // 切視窗 = 停車

vSlider.addEventListener('input', updateUI);
wSlider.addEventListener('input', updateUI);
vSlider.addEventListener('change', () => vSlider.blur());
wSlider.addEventListener('change', () => wSlider.blur());
updateUI();
</script>
</body>
</html>
"""

_pad_html = "".join(
    f'<button data-vx="{vx}" data-wz="{wz}"'
    + (' class="stopbtn"' if (vx, wz) == (0, 0) else "")
    + f'>{sym}<small>{name}</small></button>'
    for sym, name, vx, wz in PAD_LAYOUT)

PAGE = (PAGE
        .replace("__PAD_BUTTONS__", _pad_html)
        .replace("__V_MIN__", str(V_MIN)).replace("__V_MAX__", str(V_MAX))
        .replace("__V_DEF__", str(V_DEFAULT))
        .replace("__W_MIN__", str(W_MIN)).replace("__W_MAX__", str(W_MAX))
        .replace("__W_DEF__", str(W_DEFAULT)))


def make_handler(state, sim):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass   # 心跳每 150ms 一次，別洗版終端機

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(204)
                self.end_headers()

        def do_POST(self):
            if self.path != "/cmd":
                self._json({"ok": False}, 404)
                return
            try:
                n = int(self.headers.get("Content-Length", 0))
                d = json.loads(self.rfile.read(n))
                lin, ang = state.set(d.get("vx", 0), d.get("wz", 0),
                                     d.get("v", V_DEFAULT), d.get("w", W_DEFAULT))
                self._json({"ok": True, "lin": lin, "ang": ang, "sim": sim})
            except Exception as e:
                self._json({"ok": False, "err": str(e)}, 400)

    return Handler


def master_online(uri, timeout=3.0):
    """rospy.init_node 在 master 連不上時會無聲卡死 — 先用有 timeout 的方式確認。"""
    old = socket.getdefaulttimeout()
    socket.setdefaulttimeout(timeout)
    try:
        ServerProxy(uri).getPid("/teleop_panel")
        return True
    except Exception:
        return False
    finally:
        socket.setdefaulttimeout(old)


def local_ips():
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2)
        return [ip for ip in out.stdout.split() if not ip.startswith("127.")]
    except Exception:
        return []


def run_web(state, sim, is_shutdown, port):
    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(state, sim))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    for ip in local_ips() or ["<此機IP>"]:
        print(f"    瀏覽器開： http://{ip}:{port}")
    print("按 Ctrl-C 結束（結束前會發零速度停車）\n")
    try:
        while not is_shutdown():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


# ==================== 主程式 ====================

def main():
    ap = argparse.ArgumentParser(description="Teleop panel -> /cmd_vel（預設彈出視窗）")
    ap.add_argument("--web", action="store_true",
                    help="改用瀏覽器面板（Windows/手機遠端遙控）")
    ap.add_argument("--port", type=int, default=PORT, help="網頁版埠號")
    ap.add_argument("--sim", action="store_true",
                    help="室內測試：不連 ROS，只在終端機印出指令")
    args = ap.parse_args()

    state = CmdState()

    if args.sim:
        is_shutdown = lambda: False
        last_print = [""]

        def publish(lin, ang):
            line = f"[SIM] cmd_vel  linear.x={lin:+.2f}  angular.z={ang:+.2f}"
            if line != last_print[0]:
                print(line)
                last_print[0] = line
    else:
        uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        print(f"連線 ROS master：{uri} ...")
        if not master_online(uri):
            print(f"""
✘ ROS master 連不上（{uri}）— 面板不啟動。
  可能原因與解法：
    1. 車子（wheeltec）沒開機或不在網路上 → 開機並確認 ping 10.0.11.2 通
    2. 車上還沒跑 roslaunch turn_on_wheeltec_robot mapping.launch
    3. 想在室內先試面板（不動車）→ 加 --sim
""")
            sys.exit(1)
        import rospy
        from geometry_msgs.msg import Twist
        rospy.init_node("teleop_panel", anonymous=True, disable_signals=True)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        is_shutdown = rospy.is_shutdown

        def publish(lin, ang):
            t = Twist()
            t.linear.x  = lin
            t.angular.z = ang
            pub.publish(t)

    print(f"=== Teleop Panel {'[SIM 模式，未連 ROS]' if args.sim else ''} ===")

    stop_evt = threading.Event()
    th = threading.Thread(target=pub_loop, args=(state, publish, stop_evt, is_shutdown),
                          daemon=True)
    th.start()
    try:
        if args.web:
            run_web(state, args.sim, is_shutdown, args.port)
        else:
            try:
                run_gui(state, args.sim, is_shutdown)
            except Exception as e:
                print(f"開不了視窗（{e}）→ 自動改用網頁版")
                run_web(state, args.sim, is_shutdown, args.port)
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        th.join(timeout=1.0)
        for _ in range(3):
            publish(0.0, 0.0)
            time.sleep(0.03)
        print("已停車，面板結束")


if __name__ == "__main__":
    main()
