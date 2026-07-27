#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_move_follow_route.py — 沿規劃路線行駛（pure pursuit）

輸入：route_graph.py 產生的 plan_X_Y.csv（x, y, yaw_rad @ aurora_map 座標系）
定位：訂閱 Aurora 位姿 /slamware_ros_sdk_server_node/robot_pose (PoseStamped)
輸出：/cmd_vel (Twist)；另發 nav_msgs/Path 到 /route_plan 供 RViz 顯示

安全機制：
  - 空白鍵 = 緊急停止/解除，q = 停車結束（比照 ros_move_pair_task.py）
  - 位姿逾時 POSE_TIMEOUT 沒更新 → 停車（SLAM 斷線/追蹤丟失）
  - 橫向誤差 > MAX_LATERAL → 停車（位姿跳層/重定位錯誤的保險）

出發前檢查（重要）：
  車頭必須朝路線行進方向、且已重定位成功（aurora_status.py 看狀態）。
  路線是「方向性」的 —— 只能沿建圖方向走，逆向會失去視覺重定位。

離線模擬（不需 ROS，Windows 可跑）：
  python3 ros_move_follow_route.py --plan plan_A_D.csv --sim --render sim.png
實車：
  python3 ros_move_follow_route.py --plan plan_A_D.csv
"""

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

# ------------------------------------------------------------ 參數
MAX_SPEED_V = 0.6        # 巡航速度 (m/s) = 60 cm/s（原 0.2；現場測試提速）
MAX_TURN_W = 0.6         # 角速度上限 (rad/s)
LOOKAHEAD = 0.8          # pure pursuit 前視距離 (m)
GOAL_TOL = 0.3           # 到達判定 (m)
MAX_LATERAL = 1.5        # 橫向誤差超過即停車 (m)（位姿跳層保險）
POSE_TIMEOUT = 1.0       # 位姿逾時 (s)
RATE_HZ = 20


def load_plan(path):
    rows = np.loadtxt(path, delimiter=",", comments="#", encoding="utf-8")
    wp = rows[:, :2]
    seg = np.linalg.norm(np.diff(wp, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    return wp, cum


class PurePursuit:
    """給定位姿 → 回傳 (v, w, 診斷)。內部記住路徑進度避免回跳。"""

    def __init__(self, wp, cum):
        self.wp, self.cum = wp, cum
        self.i_near = 0
        self.inited = False

    def step(self, x, y, yaw):
        # 第一次呼叫：全路徑找最近點（支援從中途重啟）
        if not self.inited:
            self.i_near = int(np.argmin(np.linalg.norm(self.wp - (x, y), axis=1)))
            self.inited = True
        # 之後只在進度視窗內往前找（防止 U 形路徑吸到對向）
        lo = self.i_near
        hi = min(len(self.wp), lo + 200)
        d = np.linalg.norm(self.wp[lo:hi] - (x, y), axis=1)
        self.i_near = lo + int(np.argmin(d))
        lateral = float(d[self.i_near - lo])

        remain = self.cum[-1] - self.cum[self.i_near]
        if remain < GOAL_TOL and np.linalg.norm(self.wp[-1] - (x, y)) < GOAL_TOL:
            return 0.0, 0.0, dict(done=True, lateral=lateral, remain=0.0)

        # 前視點：沿線再走 LOOKAHEAD 公尺的 waypoint
        target_s = self.cum[self.i_near] + LOOKAHEAD
        i_t = int(np.searchsorted(self.cum, target_s))
        i_t = min(i_t, len(self.wp) - 1)
        tx, ty = self.wp[i_t]

        alpha = math.atan2(ty - y, tx - x) - yaw          # 前視點方位誤差
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        Ld = max(math.hypot(tx - x, ty - y), 0.15)
        w = max(-MAX_TURN_W, min(MAX_TURN_W, 2.0 * MAX_SPEED_V * math.sin(alpha) / Ld))
        v = MAX_SPEED_V * max(0.15, 1.0 - abs(alpha) / math.radians(90))  # 大角度先转慢走
        if remain < 1.0:
            v *= max(remain, 0.25)                        # 接近終點減速
        return v, w, dict(done=False, lateral=lateral, remain=remain, alpha=alpha)


# ------------------------------------------------------------ 離線模擬
def run_sim(wp, cum, render_path=None):
    pp = PurePursuit(wp, cum)
    # 從路線起點旁 0.5 m、帶 15° 航向誤差出發，驗證收斂
    yaw0 = math.atan2(*(wp[5] - wp[0])[::-1])
    x, y = wp[0] + np.array([-math.sin(yaw0), math.cos(yaw0)]) * 0.5
    yaw = yaw0 + math.radians(15)
    dt, traj, lat_log = 1.0 / RATE_HZ, [], []
    for step in range(int(cum[-1] / (MAX_SPEED_V * dt) * 3)):
        v, w, info = pp.step(x, y, yaw)
        if info["done"]:
            print(f"到達終點：{step*dt:.0f}s，模擬里程 {cum[-1]:.0f} m")
            break
        if info["lateral"] > MAX_LATERAL:
            print(f"!! 橫向誤差 {info['lateral']:.2f} m 超限（模擬不應發生）")
            break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        traj.append((x, y))
        lat_log.append(info["lateral"])
        if step % (RATE_HZ * 30) == 0:
            print(f"  t={step*dt:5.0f}s  剩餘 {info['remain']:6.1f} m  "
                  f"橫向誤差 {info['lateral']:.2f} m")
    traj = np.array(traj)
    lat = np.array(lat_log)
    print(f"橫向誤差：中位 {np.median(lat):.3f} m，最大 {lat.max():.3f} m"
          f"（含出發時故意偏 0.5 m）")
    if len(lat) > 500:
        print(f"收斂後（>20 s 起）：中位 {np.median(lat[400:]):.3f} m，"
              f"最大 {lat[400:].max():.3f} m")
    if render_path:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            plt.rcParams["font.sans-serif"] = ["Microsoft JhengHei",
                                               "Noto Sans CJK TC", "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
        except ImportError:
            return
        fig, ax = plt.subplots(figsize=(12.5, 7.5), facecolor="#f9f9f7")
        ax.set_facecolor("#fcfcfb"); ax.set_aspect("equal")
        ax.grid(True, color="#e1e0d9", lw=0.6)
        ax.plot(wp[:, 0], wp[:, 1], color="#c3c2b7", lw=3, label="規劃路線")
        ax.plot(traj[:, 0], traj[:, 1], color="#2a78d6", lw=1.4,
                label="模擬行駛（起點偏 0.5 m / 15°）")
        ax.plot(*traj[0], "o", ms=9, mfc="#1baf7a", mec="#fcfcfb", mew=2)
        ax.plot(*traj[-1], "s", ms=9, mfc="#e34948", mec="#fcfcfb", mew=2)
        ax.legend(fontsize=9, framealpha=0.9, edgecolor="#e1e0d9")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_title("pure pursuit 離線模擬", fontsize=11, loc="left")
        fig.tight_layout(); fig.savefig(render_path, dpi=130)
        print(f"寫入 {render_path}")


# ------------------------------------------------------------ ROS 實車
def run_ros(wp, cum, pose_topic):
    import select
    import termios
    import tty

    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Path as PathMsg

    class Keyboard:                                   # 比照 ros_move_pair_task.py
        def __enter__(self):
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            return self

        def read(self):
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
            return None

        def __exit__(self, *a):
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    state = {"pose": None, "t": 0.0}

    def cb(msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        state["pose"] = (msg.pose.position.x, msg.pose.position.y, yaw)
        state["t"] = time.time()

    rospy.init_node("follow_route")
    rospy.Subscriber(pose_topic, PoseStamped, cb, queue_size=1)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    path_pub = rospy.Publisher("/route_plan", PathMsg, queue_size=1, latch=True)

    pm = PathMsg()
    pm.header.frame_id = "aurora_map"
    for x, y in wp[::5]:
        ps = PoseStamped()
        ps.header.frame_id = "aurora_map"
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.w = 1.0
        pm.poses.append(ps)
    path_pub.publish(pm)

    pp = PurePursuit(wp, cum)
    rate = rospy.Rate(RATE_HZ)
    estop = False
    print("等待位姿…（空白鍵=緊急停止/解除, q=結束）")
    with Keyboard() as kb:
        while not rospy.is_shutdown():
            key = kb.read()
            if key == " ":
                estop = not estop
                print("\n[E-STOP]" if estop else "\n[解除]")
            elif key == "q":
                break

            tw = Twist()
            fresh = state["pose"] and (time.time() - state["t"] < POSE_TIMEOUT)
            if estop or not fresh:
                cmd_pub.publish(tw)                    # 停車
                rate.sleep()
                continue

            v, w, info = pp.step(*state["pose"])
            if info["done"]:
                print("\n到達終點，停車。")
                break
            if info["lateral"] > MAX_LATERAL:
                print(f"\n!! 橫向誤差 {info['lateral']:.1f} m 超限 —— 停車"
                      f"（可能位姿跳到另一方向圖層，重新確認重定位）")
                break
            tw.linear.x, tw.angular.z = v, w
            cmd_pub.publish(tw)
            print(f"\r剩餘 {info['remain']:6.1f} m  橫向 {info['lateral']:4.2f} m  "
                  f"v={v:.2f} w={w:+.2f}   ", end="")
            rate.sleep()
    cmd_pub.publish(Twist())                           # 離開前保證停車


def main():
    ap = argparse.ArgumentParser(description="沿 plan CSV 行駛（pure pursuit）")
    ap.add_argument("--plan", required=True, help="route_graph.py 產生的 plan_X_Y.csv")
    ap.add_argument("--sim", action="store_true", help="離線模擬（不需 ROS）")
    ap.add_argument("--render", default=None, help="--sim 時輸出軌跡 PNG")
    ap.add_argument("--pose-topic", default="/slamware_ros_sdk_server_node/robot_pose")
    args = ap.parse_args()

    wp, cum = load_plan(args.plan)
    print(f"路線 {Path(args.plan).name}：{cum[-1]:.1f} m，{len(wp)} waypoints")
    if args.sim:
        run_sim(wp, cum, args.render)
    else:
        run_ros(wp, cum, args.pose_topic)


if __name__ == "__main__":
    main()
