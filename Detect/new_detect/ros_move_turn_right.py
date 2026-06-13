#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
右轉任務控制器 (Turn-Right Mission Controller)

任務：起點出發 → 沿 pair 前進 → 開到 0_2「正上方」停車 → 原地右轉 ~90°
      → 重新搜尋前進（0_3 為途經點，只確認不停車）→ 在 0_4 前
      FINAL_STOP_DIST 停車 → DONE

「停在 tag 正上方」的作法：
  地面 tag 在太近時會掉出視野下緣，無法用視覺一路導到正上方。
  所以距 0_2 ≤ TURN_TRIGGER_DIST 時，把「剩餘 3D 距離」換算成水平距離
  （用鏡頭高度做勾股），再開迴路直行該距離 — 與 ros_move_pair_task.py
  回程通過 0_1 的 DEAD_RECKON 作法相同。

右轉 90° 為開迴路計時旋轉：實轉不足 90° 把 TURN_TIME_SCALE 調大，
轉過頭調小（地面摩擦不同會差很多，務必實車校一次）。

鍵盤（控制器終端機）：空白鍵=緊急停止/解除   q=立即停車結束

狀態機：
  INIT_SEARCH → INIT_ALIGN → DRIVING(往0_2) ─(夠近)→ DR_OVER_TAG → TURN_RIGHT
        ↑                                                             │
        └────────── (忽略剛離開的 0_2，重新搜尋) ←─────────────────────┘
  INIT_SEARCH → INIT_ALIGN → DRIVING(往0_4) ─(到點)→ DONE
"""

import math
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import Pose, Twist

from ros_move_pair_task import KeyPoller, RouteMap   # 共用鍵盤工具與路線地圖

# ================= 任務設定 =================
TURN_PAIR_IDS     = (0, 2)   # 開到正上方後右轉的 pair
WAYPOINT_PAIR_IDS = (0, 3)   # 途經點（只確認位置）
GOAL_PAIR_IDS     = (0, 4)   # 終點

# 「停在 0_2 正上方」
TURN_TRIGGER_DIST = 0.70     # 距 0_2 ≤ 此值（3D, m）就改用航位推算駛到正上方
CAM_HEIGHT        = 0.25     # 鏡頭離地高度 (m) — 依實車量測，用來換算水平距離
OVER_TAG_EXTRA    = 0.10     # 多走一點讓「車體中心」而非鏡頭壓在 tag 上 (m)，依車型調
DEAD_RECKON_V     = 0.12     # 航位推算直行速度 (m/s)

# 右轉 ~90°（開迴路，輸出負角速度 = 順時針）
TURN_W          = 0.5        # 右轉角速度大小 (rad/s)
TURN_TIME_SCALE = 1.00       # 實轉不足 90° 調大、轉過頭調小
TURN_TIME       = (math.pi / 2.0) / TURN_W * TURN_TIME_SCALE

# 終點停車
FINAL_STOP_DIST = 0.5        # 停在 0_4 前這麼遠 (m)
STOP_TOL        = 0.05       # 到點距離容差 (m)
APPROACH_DIST   = 1.6        # 進入此距離開始按比例減速
MIN_APPROACH_V  = 0.06       # 靠近時最小前進速（克服摩擦）
KP_APPROACH     = 0.4        # 距離誤差 → 線速度增益

# ================= 對齊 / 前進（與 ros_move_pair_task.py 同調）=================
KP_ALIGN_PIXEL  = 0.006      # pixel_error → 對齊角速度增益
ALIGN_THRESHOLD = 25         # 對齊容許像素誤差 (px)
MAX_ALIGN_W     = 0.9
MIN_ALIGN_W     = 0.10

MAX_SPEED_V = 0.2            # 巡航線速度
MAX_DRIVE_W = 0.25           # 前進中最大方向修正角速度
KD_DRIVE    = 1.5            # depth_diff → 角速度
KP_DRIVE    = 0.0006         # pixel_error → 角速度

# ================= 搜尋 / 看門狗 =================
INIT_SEARCH_W      = 0.3
SEARCH_FULL_CIRCLE = 6.2832 / INIT_SEARCH_W * 1.25   # 掃一圈 + 25% 餘量 (秒)
TAG_LOST_TIMEOUT   = 10.0
MSG_FRESH_SEC      = 0.2
MEMORY_W_DECAY_SEC = 1.0     # 跟丟後延續最後轉向量的衰減時間（秒）

ROUTE_MAP_FILE = "route_map.yaml"
# ==========================================================


class TurnRightMission:
    def __init__(self):
        rospy.init_node("turn_right_mission_node", anonymous=True)

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

        self.current_pose  = None
        self.last_msg_time = rospy.Time.now()

        rospy.loginfo("⏳ 等待感知節點 /target_info ...")
        rospy.wait_for_message("/target_info", Pose)
        rospy.loginfo("✅ 連線成功！")

        self.turn_set = frozenset(int(x) for x in TURN_PAIR_IDS)
        self.goal_set = frozenset(int(x) for x in GOAL_PAIR_IDS)

        # 任務階段：to_turn = 往 0_2，to_goal = 右轉後往 0_4
        self.phase = "to_turn"

        self.estop             = False
        self.current_node      = None
        self.ignore_set        = None    # 剛離開的 pair：暫時忽略，直到看到別的 pair
        self.search_start_time = rospy.Time.now()
        self.turn_start_time   = None
        self.dr_start_time     = None
        self.dr_duration       = 0.0
        self.no_tag_since      = None
        self.near_target       = False   # 已接近本階段目標 pair
        self.last_target_dist  = None
        self.last_drive_w      = 0.0     # 跟丟記憶修正：最後一次看到 tag 的轉向量

        self.state = "INIT_SEARCH"

        rospy.loginfo(f"🎯 任務：到 {tuple(TURN_PAIR_IDS)} 正上方 → 右轉90° → "
                      f"經 {tuple(WAYPOINT_PAIR_IDS)} → 停在 {tuple(GOAL_PAIR_IDS)} 前 "
                      f"{FINAL_STOP_DIST:.2f}m")
        rospy.loginfo("⌨️  控制：[空白鍵]=緊急停止/解除   [q]=立即停車結束")

        rospy.Timer(rospy.Duration(0.05), self._control_loop)   # 20 Hz

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

    def _start_over_tag(self, dist_3d):
        """把剩餘 3D 距離換算成水平距離，開迴路直行到 tag 正上方。"""
        horizontal = math.sqrt(max(dist_3d ** 2 - CAM_HEIGHT ** 2, 0.0)) + OVER_TAG_EXTRA
        self.dr_duration   = horizontal / DEAD_RECKON_V
        self.dr_start_time = rospy.Time.now()
        rospy.loginfo(f"➡️  距 0_2 {dist_3d:.2f}m → 直行 {horizontal:.2f}m"
                      f"（約 {self.dr_duration:.1f}s）到正上方")
        self.state = "DR_OVER_TAG"

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
        is_target = False
        target_set = self.turn_set if self.phase == "to_turn" else self.goal_set

        if self.current_pose is not None:
            age = (now - self.last_msg_time).to_sec()
            if age <= MSG_FRESH_SEC and self.current_pose.orientation.w == 1.0:
                id_a = self.current_pose.orientation.x
                id_b = self.current_pose.orientation.y
                pair_set = frozenset((int(id_a), int(id_b)))

                if self.ignore_set is not None and pair_set == self.ignore_set:
                    rospy.loginfo_throttle(1.0, f"  (略過剛離開的 pair {tuple(self.ignore_set)})")
                else:
                    if self.ignore_set is not None:
                        rospy.loginfo(f"看到新 pair {tuple(pair_set)}，解除忽略 {tuple(self.ignore_set)}")
                        self.ignore_set = None
                    tag_visible = True
                    pixel_error = self.current_pose.position.x
                    depth_diff  = self.current_pose.position.y
                    dist        = self.current_pose.position.z
                    is_target   = pair_set == target_set
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

        # ── INIT_ALIGN：原地快速置中 (pixel→0) ──
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

        # ── DRIVING：邊走邊修正；依階段在 0_2 / 0_4 觸發動作 ──
        elif self.state == "DRIVING":
            if tag_visible:
                self.no_tag_since = None
                w = -(KD_DRIVE * depth_diff + KP_DRIVE * pixel_error)
                twist.angular.z = max(min(w, MAX_DRIVE_W), -MAX_DRIVE_W)
                self.last_drive_w = twist.angular.z

                if is_target:
                    self.last_target_dist = dist
                    if dist < APPROACH_DIST:
                        self.near_target = True

                    if self.phase == "to_turn":
                        # 往 0_2：夠近就切換成「直行到正上方」
                        if dist <= TURN_TRIGGER_DIST:
                            twist.angular.z = 0.0
                            self._start_over_tag(dist)
                        elif dist < APPROACH_DIST:
                            twist.linear.x = self._approach_v(dist - TURN_TRIGGER_DIST)
                            rospy.loginfo_throttle(0.5, f"  [TO-0_2] dist={dist:.2f}m v={twist.linear.x:.2f}")
                        else:
                            twist.linear.x = MAX_SPEED_V
                    else:
                        # 往 0_4：停在 FINAL_STOP_DIST 前
                        d_err = dist - FINAL_STOP_DIST
                        if d_err <= STOP_TOL:
                            twist.angular.z = 0.0
                            rospy.loginfo(f"🏁 抵達 0_4 前 {dist:.2f}m，任務完成。")
                            self.state = "DONE"
                        elif dist < APPROACH_DIST:
                            twist.linear.x = self._approach_v(d_err)
                            rospy.loginfo_throttle(0.5, f"  [TO-0_4] dist={dist:.2f}m v={twist.linear.x:.2f}")
                        else:
                            twist.linear.x = MAX_SPEED_V
                else:
                    twist.linear.x = MAX_SPEED_V
                    rospy.loginfo_throttle(1.0, f"  [DRIVE] dist={dist:.2f}m dDep={depth_diff:+.3f} "
                                                f"w={twist.angular.z:+.3f}")
            else:
                # 看不到 tag
                if self.near_target:
                    if self.phase == "to_turn":
                        rospy.logwarn("0_2 近距離跟丟（掉出視野下緣）→ 直行到正上方")
                        self._start_over_tag(self.last_target_dist
                                             if self.last_target_dist is not None
                                             else TURN_TRIGGER_DIST)
                    else:
                        rospy.logwarn("🏁 終點前跟丟（已很靠近），視為抵達。")
                        self.state = "DONE"
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
                        # 記憶修正：延續最後的轉向量並線性衰減
                        if elapsed_no_tag < MEMORY_W_DECAY_SEC:
                            decay = 1.0 - elapsed_no_tag / MEMORY_W_DECAY_SEC
                            twist.angular.z = self.last_drive_w * decay

        # ── DR_OVER_TAG：開迴路直行到 0_2 正上方 ──
        elif self.state == "DR_OVER_TAG":
            if (now - self.dr_start_time).to_sec() < self.dr_duration:
                twist.linear.x = DEAD_RECKON_V
            else:
                rospy.loginfo("🅿  已停在 0_2 正上方，開始右轉 ~90°...")
                self.turn_start_time = None
                self.state = "TURN_RIGHT"

        # ── TURN_RIGHT：原地右轉 ~90°（負角速度 = 順時針）──
        elif self.state == "TURN_RIGHT":
            if self.turn_start_time is None:
                self.turn_start_time = now
            if (now - self.turn_start_time).to_sec() < TURN_TIME:
                twist.angular.z = -TURN_W
            else:
                rospy.loginfo("✅ 右轉完成，搜尋下一段路線（忽略剛離開的 0_2）...")
                self.phase             = "to_goal"
                self.ignore_set        = self.turn_set
                self.near_target       = False
                self.last_target_dist  = None
                self.no_tag_since      = None
                self.search_start_time = now
                self.state = "INIT_SEARCH"

        # ── STOPPED：中途長時間跟丟，等 tag 再出現 ──
        elif self.state == "STOPPED":
            if tag_visible:
                rospy.loginfo("✅ 重新看到 Tag，繼續。")
                self.no_tag_since = None
                self.state = "DRIVING"

        # ── 終端狀態：停住 ──
        elif self.state in ("DONE", "DEAD"):
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
        TurnRightMission()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
