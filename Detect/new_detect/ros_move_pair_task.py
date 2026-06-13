#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任務型 Pair 跟隨控制器 (Pair-following Task Controller) — 去程 + 回程

實測任務：
  去程：從起點出發 → 沿途跟隨 → 在終點 pair 0_4 前 1m 停車 → 原地置中對準 0_4 → 待命
  回程：在終點按 g → 掉頭 ~180° → 沿途跟隨回去 → 通過起點 pair 0_1 後再前進 50cm → 停車

行為（依你的需求）：
  1. 開機看到 tag → 直接置中；看不到 → 原地旋轉掃一圈找 tag
  2. 置中（pixel→0，轉得快）→ 前進，邊走邊用 depth_diff 修正成正對 (head-on)
  3. 去程：看到終點 0_4 且距離 ≤ 1m → 停 → 原地置中對準 0_4 → ARRIVED_WAIT 待命
  4. 按 g → 回程：掉頭 → 重新搜尋/置中/前進 → 通過 0_1 後 dead-reckon 前進 50cm → DONE
  5. 【失控保險】控制器終端機：空白鍵=緊急停止/解除   q=立即停車並結束（任何狀態）

回程 pair 命名：同一對 tag 從另一側看，左右對調（0_4→4_0…），
  但本程式用「集合」比對 goal（frozenset），故 {0,1} 不管顯示成 0_1 或 1_0 都認得。

狀態機：
  INIT_SEARCH → INIT_ALIGN → DRIVING ─(去程到點)→ FINAL_ALIGN → ARRIVED_WAIT
                                   │                                  │ (按 g)
                                   │                            TURN_AROUND
                                   │                                  ↓
                                   └──────────── (回程) ────────→ INIT_SEARCH …
                              DRIVING ─(回程到 0_1)→ DEAD_RECKON → DONE
  ESTOP（空白鍵）/ q 可在任何狀態插入

depth_diff 慣例（pair_detector_balance.py）：
  >0 右遠→車偏左→右轉   <0 左遠→車偏右→左轉   ≈0 正對
"""

import select
import sys
import termios
import time
import tty
from collections import deque
from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import Pose, Twist


class RouteMap:
    """讀取 route_map.yaml，提供 Tag pair → 地點名稱的查詢。"""

    def __init__(self, yaml_path: str):
        data = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8"))
        self.nodes = {n["id"]: n for n in data.get("nodes", [])}

        self._pair_to_node = {}
        for nid, n in self.nodes.items():
            key = frozenset(int(x) for x in n["pair"])
            self._pair_to_node[key] = nid

        self._adj = {nid: [] for nid in self.nodes}
        for e in data.get("edges", []):
            f, t, gd = e["from"], e["to"], e["going_direction"]
            self._adj[f].append({"to": t, "going_direction": gd})
            rev = "returning" if gd == "going" else "going"
            self._adj[t].append({"to": f, "going_direction": rev})

    def find_node(self, id_a: int, id_b: int):
        return self._pair_to_node.get(frozenset([int(id_a), int(id_b)]))

    def node_name(self, node_id: str) -> str:
        return self.nodes.get(node_id, {}).get("name", node_id)

    def bfs_path(self, src: str, dst: str):
        """BFS 最短路徑（供未來任務使用）。"""
        if src == dst:
            return [src]
        visited, queue = {src}, deque([[src]])
        while queue:
            path = queue.popleft()
            for edge in self._adj.get(path[-1], []):
                nxt = edge["to"]
                if nxt not in visited:
                    new_path = path + [nxt]
                    if nxt == dst:
                        return new_path
                    visited.add(nxt)
                    queue.append(new_path)
        return []

# ================= 任務設定（依你的場地修改）=================
GOAL_PAIR_IDS  = (0, 4)    # 去程終點 pair（0_4）
START_PAIR_IDS = (0, 1)    # 回程終點 pair（0_1）

# 去程
OUT_STOP_DISTANCE = 1.0    # 去程：停在 0_4 前 1.0m
OUT_PREREQ_PAIR   = (0, 3) # 去程：須先看到此 pair 才承認終點（防提早停）；None=關閉

# 回程（按 g 觸發）
RET_PREREQ_PAIR   = (0, 2) # 回程：須先看到此 pair 才承認終點
RET_PASS_BEYOND   = 0.5    # 回程：通過 0_1 後再「直行」這麼遠才停 (m)
RET_TRIGGER_DIST  = 0.6    # 回程：距 0_1 ≤ 此值就切成「直行通過」(m)
DEAD_RECKON_V     = 0.15   # 回程通過 0_1 時的直行速度 (m/s)

# 掉頭（按 g 後先轉約 180°，開迴路，依實車微調 TURN_TIME）
TURN_W    = 0.5            # 掉頭角速度 (rad/s)
TURN_TIME = 3.14159 / TURN_W   # ~180° 所需秒數

STOP_TOL      = 0.05       # 到點距離容差 (m)
APPROACH_DIST = 1.6        # 進入此距離開始按比例減速
ROUTE_MAP_FILE = "route_map.yaml"

# 終點停車強健化（防漏看前置點而不停 / 防遠處誤觸）
GOAL_CONFIRM_FRAMES = 3
GOAL_FALLBACK_DIST  = OUT_STOP_DISTANCE + 0.3

# 終點置中（去程到 0_4 後原地對準）
FINAL_ALIGN_TIMEOUT = 4.0  # 置中最多轉幾秒就接受

# ================= 對齊 (INIT_ALIGN / FINAL_ALIGN：純像素置中，轉得快) =================
KP_ALIGN_PIXEL  = 0.006    # pixel_error → 對齊角速度增益
ALIGN_THRESHOLD = 25       # 對齊容許像素誤差 (px)
MAX_ALIGN_W     = 0.9      # 對齊最大角速度
MIN_ALIGN_W     = 0.10     # 對齊最小角速度（克服靜摩擦）

# ================= 前進 (DRIVING：邊走邊用 depth_diff 修正成正對) =================
MAX_SPEED_V    = 0.2       # 巡航線速度
MIN_APPROACH_V = 0.06      # 靠近時最小前進速（克服摩擦）
KP_APPROACH    = 0.4       # 靠近：距離誤差 → 線速度增益
MAX_DRIVE_W    = 0.25      # 前進中最大方向修正角速度
KD_DRIVE       = 1.5       # 前進中 depth_diff → 角速度（修正成正對）
KP_DRIVE       = 0.0006    # 前進中 pixel_error → 角速度（保持置中）

# ================= 搜尋 / 看門狗 =================
INIT_SEARCH_W       = 0.3
SEARCH_FULL_CIRCLE  = 6.2832 / INIT_SEARCH_W * 1.25   # 掃一圈 + 25% 餘量 (秒)
TAG_LOST_TIMEOUT    = 10.0
MSG_FRESH_SEC       = 0.2
MEMORY_W_DECAY_SEC  = 1.0    # 跟丟後延續最後轉向量的衰減時間（秒），之後直走
# ==========================================================


class KeyPoller:
    """非阻塞讀取終端機單一按鍵（不需按 Enter）。無 tty 時自動停用。"""

    def __init__(self):
        self.enabled = False
        try:
            self.fd  = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)   # cbreak 保留 Ctrl-C
            self.enabled = True
        except Exception as e:
            rospy.logwarn(f"鍵盤控制停用（非互動終端機？）：{e}")

    def poll(self):
        if not self.enabled:
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def restore(self):
        if self.enabled:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


class PairTaskController:
    def __init__(self):
        rospy.init_node("pair_task_controller_node", anonymous=True)

        # 載入地圖（僅用於印出位置名稱，非必要）
        self.route_map = None
        if RouteMap is not None:
            try:
                self.route_map = RouteMap(str(Path(__file__).parent / ROUTE_MAP_FILE))
                rospy.loginfo(f"地圖載入完成，共 {len(self.route_map.nodes)} 個節點")
            except Exception as e:
                rospy.logwarn(f"地圖載入失敗（不影響任務）：{e}")

        self.keys = KeyPoller()
        rospy.on_shutdown(self._on_shutdown)

        self.sub = rospy.Subscriber(
            "/target_info", Pose, self._pose_cb, queue_size=1, tcp_nodelay=True
        )
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.current_pose = None
        self.last_msg_time = rospy.Time.now()

        rospy.loginfo("⏳ 等待感知節點 /target_info ...")
        rospy.wait_for_message("/target_info", Pose)
        rospy.loginfo("✅ 連線成功！")

        # 共用狀態變數
        self.estop             = False
        self.current_node      = None
        self.ignore_set        = None    # 剛離開的 pair：暫時忽略，直到看到別的 pair
        self.search_start_time = rospy.Time.now()
        self.turn_start_time   = None
        self.dr_start_time     = None
        self.dr_duration       = 0.0
        self.final_align_start = None
        self.last_goal_dist    = None
        self.last_drive_w      = 0.0   # 跟丟記憶修正：最後一次看到 tag 的轉向量

        # 設定去程任務參數
        self._apply_leg("outbound")
        self.state = "INIT_SEARCH"

        rospy.loginfo(f"🎯 去程：跟隨至 {tuple(GOAL_PAIR_IDS)}，停在 {OUT_STOP_DISTANCE:.2f}m 前並置中")
        rospy.loginfo(f"🔁 回程(按 g)：跟隨至 {tuple(START_PAIR_IDS)}，通過後再前進 {RET_PASS_BEYOND:.2f}m")
        rospy.loginfo("⌨️  控制：[空白鍵]=緊急停止/解除   [g]=到點後開始回程   [q]=立即停車結束")

        rospy.Timer(rospy.Duration(0.05), self._control_loop)   # 20 Hz

    # ---------------- 任務段設定 ----------------
    def _apply_leg(self, leg):
        """切換去程/回程的目標、前置點、停車方式，並重置進度旗標。"""
        self.leg = leg
        if leg == "outbound":
            self.goal_set     = frozenset(int(x) for x in GOAL_PAIR_IDS)
            self.prereq_set   = frozenset(int(x) for x in OUT_PREREQ_PAIR) if OUT_PREREQ_PAIR else None
            self.stop_dist    = OUT_STOP_DISTANCE
            self.pass_through = False    # 去程：停在前方 1m
            self.final_center = True     # 去程：到點後置中
        else:  # return
            self.goal_set     = frozenset(int(x) for x in START_PAIR_IDS)
            self.prereq_set   = frozenset(int(x) for x in RET_PREREQ_PAIR) if RET_PREREQ_PAIR else None
            self.stop_dist    = OUT_STOP_DISTANCE  # 回程不用，pass_through 走 dead-reckon
            self.pass_through = True     # 回程：通過 0_1 後再前進 50cm
            self.final_center = False

        self.prereq_seen      = self.prereq_set is None
        self.goal_close_count = 0
        self.near_goal        = False
        self.no_tag_since     = None
        self.last_goal_dist   = None
        self.final_align_start = None

    # ---------------- callbacks ----------------
    def _pose_cb(self, data):
        self.current_pose  = data
        self.last_msg_time = rospy.Time.now()

    def _on_shutdown(self):
        self._publish_stop(times=5)
        self.keys.restore()

    def _publish_stop(self, times=4):
        """連發數次零速，確保底盤確實煞停（避免單封包遺失）。"""
        stop = Twist()
        for _ in range(times):
            self.cmd_pub.publish(stop)
            time.sleep(0.02)

    def _update_location(self, id_a, id_b):
        if self.route_map is None:
            return
        node_id = self.route_map.find_node(int(id_a), int(id_b))
        if node_id and node_id != self.current_node:
            self.current_node = node_id
            rospy.loginfo(f"📍 位置更新：{self.route_map.node_name(node_id)} ({node_id})")

    def _start_dead_reckon(self, remaining_to_tag):
        """切入開迴路直行：把『到 0_1 的剩餘距離 + 50cm』換算成直行時間。"""
        dr_dist = max(remaining_to_tag, 0.0) + RET_PASS_BEYOND
        self.dr_duration   = dr_dist / DEAD_RECKON_V
        self.dr_start_time = rospy.Time.now()
        rospy.loginfo(f"➡️  通過 0_1：直行 {dr_dist:.2f}m（約 {self.dr_duration:.1f}s）後停車")
        self.state = "DEAD_RECKON"

    # ---------------- keyboard ----------------
    def _poll_keyboard(self):
        key = self.keys.poll()
        if key is None:
            return
        if key == " ":
            self.estop = not self.estop
            if self.estop:
                rospy.logwarn("🛑 緊急停止！(再按一次空白鍵解除)")
            else:
                rospy.loginfo("▶️  解除緊急停止，重新對齊...")
                self.state        = "INIT_ALIGN"
                self.no_tag_since = None
        elif key in ("g", "G"):
            if self.leg == "outbound" and self.state == "ARRIVED_WAIT":
                rospy.loginfo("🔁 收到 g：開始回程，先掉頭 ~180°（回程忽略剛離開的 0_4）")
                left_pair = self.goal_set            # 剛停妥的 pair（0_4）
                self._apply_leg("return")
                self.ignore_set      = left_pair     # 掉頭後忽略它，避免又鎖回去
                self.turn_start_time = None
                self.state = "TURN_AROUND"
            else:
                rospy.logwarn("⚠️ g 無效：請先在終點 0_4 停妥（ARRIVED_WAIT）才能回程")
        elif key in ("q", "Q"):
            rospy.logwarn("👋 收到 q：立即停車並結束節點。")
            self.estop = True
            self._publish_stop(times=5)
            rospy.signal_shutdown("user quit")

    # ---------------- main loop ----------------
    def _control_loop(self, event):
        self._poll_keyboard()

        twist = Twist()
        now   = rospy.Time.now()

        # ── e-stop 凌駕一切 ──
        if self.estop:
            self.cmd_pub.publish(twist)
            return

        # ── 讀取感知 ──
        tag_visible = False
        pixel_error = depth_diff = dist = 0.0
        is_goal = False
        if self.current_pose is not None:
            age = (now - self.last_msg_time).to_sec()
            if age <= MSG_FRESH_SEC and self.current_pose.orientation.w == 1.0:
                id_a = self.current_pose.orientation.x
                id_b = self.current_pose.orientation.y
                pair_set = frozenset((int(id_a), int(id_b)))

                if self.ignore_set is not None and pair_set == self.ignore_set:
                    # 剛離開的 pair → 當作沒看到，讓搜尋/前進略過它
                    rospy.loginfo_throttle(1.0, f"  (略過剛離開的 pair {tuple(self.ignore_set)})")
                else:
                    if self.ignore_set is not None:
                        rospy.loginfo(f"看到新 pair {tuple(pair_set)}，解除忽略 {tuple(self.ignore_set)}")
                        self.ignore_set = None
                    tag_visible = True
                    pixel_error = self.current_pose.position.x
                    depth_diff  = self.current_pose.position.y
                    dist        = self.current_pose.position.z
                    is_goal     = pair_set == self.goal_set
                    if self.prereq_set is not None and not self.prereq_seen and pair_set == self.prereq_set:
                        self.prereq_seen = True
                        rospy.loginfo(f"✅ 已通過前置點 {tuple(self.prereq_set)}")
                    if is_goal and dist <= GOAL_FALLBACK_DIST:
                        self.goal_close_count += 1
                    else:
                        self.goal_close_count = 0
                    self._update_location(id_a, id_b)

        # =================== 狀態機 ===================

        # ── INIT_SEARCH：旋轉掃一圈找 tag ──
        if self.state == "INIT_SEARCH":
            if tag_visible:
                rospy.loginfo("🎯 找到 Tag pair，開始置中...")
                self.state = "INIT_ALIGN"
            else:
                if (now - self.search_start_time).to_sec() > SEARCH_FULL_CIRCLE:
                    rospy.logerr("❌ 旋轉一圈仍找不到 Tag，進入 DEAD（空白鍵可重試）。")
                    self.state = "DEAD"
                else:
                    twist.angular.z = INIT_SEARCH_W

        # ── INIT_ALIGN：原地快速置中 (pixel→0)；head-on 留待 DRIVING 邊走邊修 ──
        elif self.state == "INIT_ALIGN":
            if tag_visible:
                if abs(pixel_error) < ALIGN_THRESHOLD:
                    rospy.loginfo(f"✅ 置中完成 px={pixel_error:.0f} → 前進")
                    self.no_tag_since = None
                    self.state = "DRIVING"
                else:
                    twist.angular.z = self._align_w(pixel_error)
                    rospy.loginfo_throttle(0.5, f"  [ALIGN] px={pixel_error:+.0f} w={twist.angular.z:+.3f}")
            else:
                twist.angular.z = INIT_SEARCH_W * 0.5

        # ── DRIVING：邊走邊置中 + depth_diff 修正成正對 ──
        elif self.state == "DRIVING":
            if tag_visible:
                self.no_tag_since = None
                w = -(KD_DRIVE * depth_diff + KP_DRIVE * pixel_error)
                twist.angular.z = max(min(w, MAX_DRIVE_W), -MAX_DRIVE_W)
                self.last_drive_w = twist.angular.z

                goal_armed = self.prereq_seen or (self.goal_close_count >= GOAL_CONFIRM_FRAMES)

                if is_goal and goal_armed:
                    self.last_goal_dist = dist
                    if dist < APPROACH_DIST:
                        self.near_goal = True

                    if self.pass_through:
                        # 回程：靠近 0_1 就切換成直行通過
                        if dist <= RET_TRIGGER_DIST:
                            self._start_dead_reckon(dist)
                        elif dist < APPROACH_DIST:
                            twist.linear.x = self._approach_v(dist - RET_TRIGGER_DIST)
                            rospy.loginfo_throttle(0.5, f"  [RET-APPROACH] dist={dist:.2f}m v={twist.linear.x:.2f}")
                        else:
                            twist.linear.x = MAX_SPEED_V
                    else:
                        # 去程：停在 stop_dist 前，再置中
                        d_err = dist - self.stop_dist
                        if d_err <= STOP_TOL:
                            twist.angular.z = 0.0
                            if self.final_center:
                                rospy.loginfo(f"🏁 抵達 {dist:.2f}m，原地置中對準 0_4...")
                                self.final_align_start = None
                                self.state = "FINAL_ALIGN"
                            else:
                                self.state = "ARRIVED_WAIT"
                        elif dist < APPROACH_DIST:
                            twist.linear.x = self._approach_v(d_err)
                            rospy.loginfo_throttle(0.5, f"  [APPROACH] dist={dist:.2f}m v={twist.linear.x:.2f} w={twist.angular.z:+.3f}")
                        else:
                            twist.linear.x = MAX_SPEED_V
                else:
                    twist.linear.x = MAX_SPEED_V
                    if is_goal:
                        rospy.loginfo_throttle(1.0, f"  [DRIVE] 看到終點但尚未解鎖，繼續前進 dist={dist:.2f}m")
                    else:
                        rospy.loginfo_throttle(1.0, f"  [DRIVE] dist={dist:.2f}m dDep={depth_diff:+.3f} w={twist.angular.z:+.3f}")
            else:
                # 看不到 tag
                if self.near_goal:
                    if self.pass_through:
                        rospy.logwarn("0_1 近距離跟丟 → 直行通過")
                        self._start_dead_reckon(self.last_goal_dist if self.last_goal_dist is not None else RET_TRIGGER_DIST)
                    else:
                        rospy.logwarn("🏁 終點前跟丟（已很靠近），視為抵達。")
                        self.state = "ARRIVED_WAIT"
                else:
                    if self.no_tag_since is None:
                        self.no_tag_since = now
                        rospy.loginfo("⚠️ 看不到 Tag，沿最後方向修正後直走...")
                    elapsed_no_tag = (now - self.no_tag_since).to_sec()
                    if elapsed_no_tag >= TAG_LOST_TIMEOUT:
                        rospy.logwarn(f"🛑 {TAG_LOST_TIMEOUT:.0f}s 未見 Tag，停車。")
                        self.state = "STOPPED"
                    else:
                        twist.linear.x = MAX_SPEED_V
                        # 記憶修正：延續最後的轉向量並線性衰減，避免跟丟瞬間方向歸零
                        if elapsed_no_tag < MEMORY_W_DECAY_SEC:
                            decay = 1.0 - elapsed_no_tag / MEMORY_W_DECAY_SEC
                            twist.angular.z = self.last_drive_w * decay

        # ── FINAL_ALIGN：去程到點後原地置中對準 0_4 ──
        elif self.state == "FINAL_ALIGN":
            if self.final_align_start is None:
                self.final_align_start = now
            elapsed   = (now - self.final_align_start).to_sec()
            centered  = tag_visible and abs(pixel_error) < ALIGN_THRESHOLD
            timed_out = elapsed >= FINAL_ALIGN_TIMEOUT
            if tag_visible and not centered and not timed_out:
                twist.angular.z = self._align_w(pixel_error)
                rospy.loginfo_throttle(0.5, f"  [FINAL_ALIGN] px={pixel_error:+.0f} w={twist.angular.z:+.3f}")
            else:
                why = "置中完成" if centered else ("逾時" if timed_out else "跟丟")
                rospy.loginfo(f"🏁 已抵達並{why}，停在 0_4 前。按 [g] 開始回程。")
                self.state = "ARRIVED_WAIT"

        # ── TURN_AROUND：回程開始前掉頭 ~180° ──
        elif self.state == "TURN_AROUND":
            if self.turn_start_time is None:
                self.turn_start_time = now
            if (now - self.turn_start_time).to_sec() < TURN_TIME:
                twist.angular.z = TURN_W
            else:
                rospy.loginfo("✅ 掉頭完成，搜尋回程 tag...")
                self.search_start_time = now
                self.state = "INIT_SEARCH"

        # ── DEAD_RECKON：回程通過 0_1 後開迴路直行 50cm ──
        elif self.state == "DEAD_RECKON":
            if (now - self.dr_start_time).to_sec() < self.dr_duration:
                twist.linear.x = DEAD_RECKON_V
            else:
                rospy.loginfo("🏁 已通過 0_1 並前進 50cm，回程完成，停車。")
                self.state = "DONE"

        # ── STOPPED：中途長時間跟丟，等 tag 再出現 ──
        elif self.state == "STOPPED":
            if tag_visible:
                rospy.loginfo("✅ 重新看到 Tag，繼續。")
                self.no_tag_since = None
                self.state = "DRIVING"

        # ── 終端狀態：停住 ──
        elif self.state in ("ARRIVED_WAIT", "DONE", "DEAD"):
            pass

        self.cmd_pub.publish(twist)

    # ---------------- 小工具 ----------------
    def _align_w(self, pixel_error):
        w = -KP_ALIGN_PIXEL * pixel_error
        if abs(w) < MIN_ALIGN_W:
            w = MIN_ALIGN_W if w > 0 else -MIN_ALIGN_W
        return max(min(w, MAX_ALIGN_W), -MAX_ALIGN_W)

    def _approach_v(self, dist_error):
        return max(min(KP_APPROACH * dist_error, MAX_SPEED_V), MIN_APPROACH_V)


if __name__ == "__main__":
    try:
        PairTaskController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
