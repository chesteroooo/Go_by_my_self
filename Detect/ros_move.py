#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Pose

# ================= 參數調校區 =================
# ================= 參數調校區 =================
# 1. 任務設定
TARGET_TASK_DIST = 1.5     # 觸發後要盲走/修正的總距離 (1.0 公尺)
COOLDOWN_TIME = 3.0        # 任務完成後，忽略 Tag 的冷卻時間 (秒)

# 2. PID 參數 (轉向)
# 【修正】降低 P 值，讓方向盤打得更柔和
KP_ANGULAR = 0.0005  

# 3. 速度限制
MAX_SPEED_V = 0.2          # 執行任務時的恆定前進速度
# 【修正】最高轉速設為 0.05 rad/s (大約每秒轉 3 度，避免甩尾)
MAX_SPEED_W = 0.05         

# 4. 【關鍵】最小啟動速度 (克服摩擦力)
# 【修正】最低轉速設為 0.017 rad/s (精準符合你說的「每秒調整 1 度」)
MIN_SPEED_W = 0.017        
# ============================================
# ============================================

class RemoteController:
    def __init__(self):
        rospy.init_node('remote_controller_node', anonymous=True)
        
        self.sub = rospy.Subscriber('/target_info', Pose, self.pose_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.current_pose = None
        self.last_msg_time = rospy.Time.now()

        # ================= 新增：狀態機與距離記憶變數 =================
        self.is_executing_task = False       # 是否正在執行「前進1公尺」任務
        self.driven_distance = 0.0           # 紀錄目前已經走了多遠
        self.last_loop_time = rospy.Time.now() # 計算時間差 (dt) 用
        self.cooldown_until = rospy.Time.now() # 紀錄冷卻時間到什麼時候
        # ==========================================================
        
        # 啟動 20Hz 控制迴圈 (0.05秒跑一次)
        rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("控制器啟動！等待 Tag 觸發 1 公尺任務...")

    def pose_callback(self, data):
        """只更新數據，不發送指令"""
        self.current_pose = data
        self.last_msg_time = rospy.Time.now()

    def control_loop(self, event):
        """每秒跑 20 次的主迴圈：負責計算距離與發布速度"""
        twist = Twist()
        current_time = rospy.Time.now()
        
        # 1. 計算本次迴圈經過的時間差 (dt)，單位為秒
        dt = (current_time - self.last_loop_time).to_sec()
        self.last_loop_time = current_time

        # 2. 判斷現在是否「確實有看到 Tag」 (且畫面沒有卡住超時)
        tag_visible = False
        if self.current_pose is not None:
            if (current_time - self.last_msg_time).to_sec() <= 0.5:
                if self.current_pose.orientation.w == 1.0:
                    tag_visible = True

        # ================= 核心：狀態機邏輯 =================
        
        # 狀態 A：觸發新任務 (目前沒任務、看到 Tag、且冷卻時間已過)
        if tag_visible and not self.is_executing_task and current_time > self.cooldown_until:
            self.is_executing_task = True
            self.driven_distance = 0.0
            rospy.loginfo("👀 發現新 Tag！開始執行【前進 1 公尺】任務...")

        # 狀態 B：正在執行 1 公尺任務
        if self.is_executing_task:
            # 任務期間，保持恆定的前進速度
            linear_x = MAX_SPEED_V
            angular_z = 0.0
            
            if tag_visible:
                # 【狀況 B-1：有看到 Tag】 -> 邊走邊修正方向
                error_x = self.current_pose.position.x
                
                if abs(error_x) > 20:
                    angular_z = -1 * KP_ANGULAR * error_x
                    
                    # 轉向動力補償
                    if abs(angular_z) < MIN_SPEED_W:
                        angular_z = MIN_SPEED_W if angular_z > 0 else -MIN_SPEED_W
                        
                # 轉向安全速限
                angular_z = max(min(angular_z, MAX_SPEED_W), -MAX_SPEED_W)
                
            else:
                # 【狀況 B-2：Tag 消失了】 -> 盲走，不轉向，直直往前開
                angular_z = 0.0

            # 3. 累積行駛距離 (利用 速度 × 時間 = 距離)
            self.driven_distance += linear_x * dt
            
            # 4. 檢查是否已經走滿 1 公尺
            if self.driven_distance >= TARGET_TASK_DIST:
                rospy.loginfo("✅ 已完成 1 公尺任務！停車並進入冷卻狀態。")
                self.is_executing_task = False
                linear_x = 0.0
                angular_z = 0.0
                # 設定冷卻時間，避免車子停下來立刻又看到同一個 Tag 而重複觸發
                self.cooldown_until = current_time + rospy.Duration(COOLDOWN_TIME)

        else:
            # 狀態 C：沒有任務時 -> 乖乖停車等待
            linear_x = 0.0
            angular_z = 0.0

        # ==================================================

        # 輸出最終指令給底盤
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    try:
        RemoteController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass