#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Pose

# ================= 參數調校區 =================
# 1. 目標距離
TARGET_DIST = 0.5      # 建議先設 0.5m，太近(0.1)容易煞不住撞上去

# 2. PID 參數
KP_LINEAR = 0.2        # 前進力道 (你覺得這個穩就用這個)
KP_ANGULAR = 0.003     # 轉向力道

# 3. 速度限制
MAX_SPEED_V = 0.2      # 最高前進速 (稍微保守一點)
MAX_SPEED_W = 0.5      # 最高轉速

# 4. 【關鍵】最小啟動速度 (克服摩擦力)
MIN_SPEED_V = 0.2      # 前進沒力時，強制給這個速度
MIN_SPEED_W = 0.3      # 轉向沒力時，強制給這個速度
# ============================================

class RemoteController:
    def __init__(self):
        rospy.init_node('remote_controller_node', anonymous=True)
        
        # 1. 訂閱 (只負責收數據)
        self.sub = rospy.Subscriber('/target_info', Pose, self.pose_callback)
        
        # 2. 發布 (建議先用 cmd_vel，如果不動再改 smoother)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 3. 變數初始化
        self.current_pose = None
        self.last_msg_time = rospy.Time.now()

        # 4. 【核心】啟動 20Hz 控制迴圈 (解決頓挫問題)
        rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("控制器啟動 (左負右正修正版)...")

    def pose_callback(self, data):
        """只更新數據，不發送指令"""
        self.current_pose = data
        self.last_msg_time = rospy.Time.now()

    def control_loop(self, event):
        """每秒跑 20 次的主迴圈"""
        twist = Twist()
        
        # 看門狗：超過 0.5 秒沒影像就停車
        if (rospy.Time.now() - self.last_msg_time).to_sec() > 0.5:
            self.cmd_pub.publish(twist)
            return

        if self.current_pose is None:
            return

        data = self.current_pose
        
        # 1.0 = 有看到 Tag
        if data.orientation.w == 1.0:
            
            # === A. 轉向控制 (Steering) ===
            # data.position.x 定義：左負、右正
            # 我們需要：左轉(正)、右轉(負)
            # 所以公式： angular = -1 * KP * x
            error_x = data.position.x
            
            # 死區：誤差大於 20 像素才轉
            if abs(error_x) > 20:
                # 【修正點】加上 -1 翻轉方向
                angular_z = -1 * KP_ANGULAR * error_x
                
                # 轉向動力補償
                if abs(angular_z) < MIN_SPEED_W:
                    if angular_z > 0:
                        angular_z = MIN_SPEED_W
                    else:
                        angular_z = -MIN_SPEED_W
            else:
                angular_z = 0.0
            
            # === B. 前進控制 (Throttle) ===
            # data.position.z 已經是純深度距離
            dist_error = data.position.z - TARGET_DIST
            
            if abs(dist_error) > 0.05:
                linear_x = KP_LINEAR * dist_error
                
                # 前進動力補償
                if abs(linear_x) < MIN_SPEED_V:
                    if linear_x > 0:
                        linear_x = MIN_SPEED_V
                    else:
                        linear_x = -MIN_SPEED_V
            else:
                linear_x = 0.0

            # === C. 安全速限 ===
            twist.linear.x = max(min(linear_x, MAX_SPEED_V), -MAX_SPEED_V)
            twist.angular.z = max(min(angular_z, MAX_SPEED_W), -MAX_SPEED_W)
            
        else:
            # 沒看到 Tag -> 停車
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    try:
        RemoteController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass