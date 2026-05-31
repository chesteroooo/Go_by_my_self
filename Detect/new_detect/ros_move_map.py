#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地圖感知移動控制器 v2 (Map-aware Movement Controller)

行走邏輯：
  - 開機旋轉找到第一個 Tag → 做一次初始對齊 → 進入 DRIVING 一直走
  - DRIVING 狀態：
      * 看到 Tag  → 用 depth_diff + pixel_error 調整方向，同時前進
      * 看不到Tag → 繼續直走（不旋轉、不停車）
      * 超過 TAG_TIMEOUT 秒完全看不到 Tag → STOPPED（停車）
  - STOPPED 狀態：
      * 再次看到 Tag → 回 DRIVING，不需重新對齊

對齊速度修正（INIT_ALIGN）：
  先前問題：MAX_SPEED_W = MIN_SPEED_W = 0.08 → 永遠只有一個速度，無比例控制
  修正後：MAX_ALIGN_W = 0.4，MIN_ALIGN_W = 0.05
  → 誤差大時轉快，誤差小時轉慢，不會過衝或停不準

狀態機：
  INIT_SEARCH → INIT_ALIGN → DRIVING → STOPPED
                                ↑____________|  (看到 tag 重新開始)
"""

from collections import deque
from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import Pose, Twist

# ================= 參數調校區 =================
# 初始對齊
KD_DEPTH        = 2.0        # depth_diff  → 角速度增益
KP_PIXEL        = 0.0005     # pixel_error → 角速度增益（輔助）
DEPTH_THRESHOLD = 0.03       # 對齊容許深度差 (m)，0.03 比原本 0.015 更寬鬆好達到
ALIGN_THRESHOLD = 20         # 對齊容許像素誤差 (px)

MAX_ALIGN_W = 0.4            # 對齊時最大角速度（比原本 0.08 大，誤差大時轉更快）
MIN_ALIGN_W = 0.05           # 對齊時最小角速度（比原本 0.08 小，誤差小時轉慢）

# 前進
MAX_SPEED_V    = 0.2         # 前進線速度
MAX_DRIVE_W    = 0.15        # 前進中最大修正角速度（不要太大，避免蛇行）
KD_DRIVE       = 1.0         # 前進中 depth_diff → 角速度增益
KP_DRIVE       = 0.0003      # 前進中 pixel_error → 角速度增益（輔助）

# 停車條件
TAG_TIMEOUT    = 10.0        # 連續看不到 tag 幾秒後停車

# 初始搜尋
INIT_SEARCH_W  = 0.3         # 初始搜尋旋轉速度
INIT_SEARCH_TIMEOUT = 35.0   # 初始搜尋超時秒數（超過進 DEAD）

ROUTE_MAP_FILE = "route_map.yaml"
# ==============================================


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


class MapMoveController:
    def __init__(self):
        rospy.init_node("map_move_controller_node", anonymous=True)

        map_path = Path(__file__).parent / ROUTE_MAP_FILE
        self.route_map = RouteMap(str(map_path))
        rospy.loginfo(f"地圖載入完成，共 {len(self.route_map.nodes)} 個節點")

        self.sub = rospy.Subscriber(
            "/target_info", Pose, self._pose_cb, queue_size=1, tcp_nodelay=True
        )
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.current_pose = None

        rospy.loginfo("⏳ 等待感知節點連線...")
        rospy.wait_for_message("/target_info", Pose)
        rospy.loginfo("✅ 連線成功！開始旋轉搜尋初始 Tag...")

        # 狀態機
        # INIT_SEARCH → INIT_ALIGN → DRIVING → STOPPED
        self.state             = "INIT_SEARCH"
        self.last_msg_time     = rospy.Time.now()
        self.search_start_time = rospy.Time.now()
        self.no_tag_since      = None    # 開始計算看不到 tag 的時間

        # 位置感知
        self.current_node = None

        rospy.Timer(rospy.Duration(0.05), self._control_loop)

    def _pose_cb(self, data):
        self.current_pose  = data
        self.last_msg_time = rospy.Time.now()

    def _update_location(self, id_a: float, id_b: float):
        node_id = self.route_map.find_node(int(id_a), int(id_b))
        if node_id and node_id != self.current_node:
            self.current_node = node_id
            rospy.loginfo(f"📍 位置更新：{self.route_map.node_name(node_id)}  ({node_id})")

    def _control_loop(self, event):
        twist        = Twist()
        current_time = rospy.Time.now()

        # ── 讀取感知資料 ──
        tag_visible = False
        pixel_error = 0.0
        depth_diff  = 0.0

        if self.current_pose is not None:
            age = (current_time - self.last_msg_time).to_sec()
            if age <= 0.2 and self.current_pose.orientation.w == 1.0:
                tag_visible = True
                id_a = self.current_pose.orientation.x
                id_b = self.current_pose.orientation.y
                pixel_error = self.current_pose.position.x
                depth_diff  = self.current_pose.position.y
                self._update_location(id_a, id_b)

        # =================== 狀態機 ===================

        # ── INIT_SEARCH：開機旋轉，找到第一個 tag 就進對齊 ──
        if self.state == "INIT_SEARCH":
            if tag_visible:
                rospy.loginfo("🎯 找到 Tag，開始初始對齊...")
                self.state = "INIT_ALIGN"
            else:
                elapsed = (current_time - self.search_start_time).to_sec()
                if elapsed > INIT_SEARCH_TIMEOUT:
                    rospy.logerr("❌ 初始搜尋超時，進入 DEAD。")
                    self.state = "DEAD"
                else:
                    twist.angular.z = INIT_SEARCH_W

        # ── INIT_ALIGN：停車，旋轉到 depth_diff ≈ 0 且置中 ──
        elif self.state == "INIT_ALIGN":
            twist.linear.x = 0.0
            if tag_visible:
                depth_ok = abs(depth_diff)  < DEPTH_THRESHOLD
                pixel_ok = abs(pixel_error) < ALIGN_THRESHOLD

                if depth_ok and pixel_ok:
                    rospy.loginfo(
                        f"✅ 初始對齊完成  dDep={depth_diff:+.3f}m  px={pixel_error:.0f}px"
                        f"  → 開始前進"
                    )
                    self.no_tag_since = None
                    self.state        = "DRIVING"
                else:
                    # 比例控制：誤差大 → 轉快，誤差小 → 轉慢
                    w = -(KD_DEPTH * depth_diff + KP_PIXEL * pixel_error)
                    if abs(w) < MIN_ALIGN_W:
                        w = MIN_ALIGN_W if w > 0 else -MIN_ALIGN_W
                    twist.angular.z = max(min(w, MAX_ALIGN_W), -MAX_ALIGN_W)
                    rospy.loginfo_throttle(
                        0.5,
                        f"  [ALIGN] dDep={depth_diff:+.4f}m  px={pixel_error:+.0f}  "
                        f"w={twist.angular.z:+.3f}  "
                        f"({'depth' if not depth_ok else ''}"
                        f"{'&px' if not pixel_ok and not depth_ok else ''}"
                        f"{'px' if depth_ok and not pixel_ok else ''} 未達標)"
                    )
            else:
                # 對齊過程中 tag 消失 → 慢慢旋轉找回來
                twist.angular.z = INIT_SEARCH_W * 0.5

        # ── DRIVING：一直走，依 tag 調整方向 ──
        elif self.state == "DRIVING":
            if tag_visible:
                self.no_tag_since = None
                # 前進 + 持續方向修正
                w = -(KD_DRIVE * depth_diff + KP_DRIVE * pixel_error)
                twist.linear.x  = MAX_SPEED_V
                twist.angular.z = max(min(w, MAX_DRIVE_W), -MAX_DRIVE_W)
                rospy.loginfo_throttle(
                    1.0,
                    f"  [DRIVE] dDep={depth_diff:+.4f}m  px={pixel_error:+.0f}  "
                    f"w={twist.angular.z:+.3f}"
                )
            else:
                # 看不到 tag：繼續直走，計時
                if self.no_tag_since is None:
                    self.no_tag_since = current_time
                    rospy.loginfo("⚠️ 看不到 Tag，繼續直走...")

                elapsed_no_tag = (current_time - self.no_tag_since).to_sec()
                if elapsed_no_tag >= TAG_TIMEOUT:
                    rospy.logwarn(f"🛑 {TAG_TIMEOUT:.0f} 秒未偵測到 Tag，停車。")
                    self.state = "STOPPED"
                else:
                    twist.linear.x  = MAX_SPEED_V
                    twist.angular.z = 0.0   # 直走

        # ── STOPPED：停車等待 tag 重新出現 ──
        elif self.state == "STOPPED":
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            if tag_visible:
                rospy.loginfo("✅ 偵測到 Tag，繼續前進！")
                self.no_tag_since = None
                self.state        = "DRIVING"

        # ── DEAD ──
        elif self.state == "DEAD":
            pass

        self.cmd_pub.publish(twist)


if __name__ == "__main__":
    try:
        MapMoveController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
