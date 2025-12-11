#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Pose

# ================= 參數調校區 (Tuning) =================
# 1. 目標距離：希望車子離 Tag 保持幾公尺？
#    (設 0.6 代表它會追到剩 60 公分時停下來，太近會撞到)
TARGET_DIST = 0.1

# 2. 前進靈敏度 (KP_LINEAR)：數值越大，加速越猛
#    建議從 0.2 開始慢慢加
KP_LINEAR = 0.2

# 3. 轉向靈敏度 (KP_ANGULAR)：數值越大，轉彎越犀利
#    建議 0.002 ~ 0.005 之間
KP_ANGULAR = 0.003

# 4. 安全速限：怕車子暴衝，這裡鎖死最高速度
MAX_SPEED_V = 0.1  # 直線最高速 (m/s)
MAX_SPEED_W = 0.2  # 轉彎最高速 (rad/s)
# ======================================================

class RemoteController:
    def __init__(self):
        # 初始化節點
        rospy.init_node('remote_controller_node', anonymous=True)
        
        # 訂閱感知資訊 (你的 ros_test.py 發出來的)
        self.sub = rospy.Subscriber('/target_info', Pose, self.control_loop)
        
        # 發布控制指令 (給車子的底盤)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 安全機制：看門狗計時器
        self.last_msg_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.1), self.watchdog)

        rospy.loginfo("控制器啟動！準備追蹤目標...")
        rospy.loginfo(f"目標距離: {TARGET_DIST}m | 前進P值: {KP_LINEAR}")

    def control_loop(self, data):
        # 收到訊息，更新時間戳記 (餵狗)
        self.last_msg_time = rospy.Time.now()
        
        twist = Twist()
        
        # 1. 檢查「有效旗標」 (Clean Protocol: orientation.w)
        #    1.0 = 有看到, 0.0 = 沒看到
        if data.orientation.w == 1.0:
            
            # --- A. 轉向控制 (Steering) ---
            # position.x 是橫向誤差 (像素)
            # 誤差正值代表在左邊 -> 要左轉 (正角速度)
            angular_z = KP_ANGULAR * data.position.x
            
            # --- B. 前進控制 (Throttle) ---
            # position.z 是直線距離 (公尺)
            # 誤差 = 現在距離 - 目標距離
            # 如果現在 2m, 目標 1m -> 誤差 +1 -> 向前加速
            dist_error = data.position.z - TARGET_DIST
            
            # 設定一個 "死區 (Deadzone)" 0.05m
            # 避免車子在目標附近一直前後抖動
            if abs(dist_error) > 0.05:
                linear_x = KP_LINEAR * dist_error
            else:
                linear_x = 0.0

            # --- C. 安全速限 (Safety Clamp) ---
            # 限制速度不要超過我們設定的上限
            twist.linear.x = max(min(linear_x, MAX_SPEED_V), -MAX_SPEED_V)
            twist.angular.z = max(min(angular_z, MAX_SPEED_W), -MAX_SPEED_W)
            
            # (選用) 顯示除錯資訊
            # rospy.loginfo(f"距離: {data.position.z:.2f}m | 速度: {twist.linear.x:.2f}")

        else:
            # 沒看到 Tag -> 強制停車
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        # 發送指令給車子
        self.cmd_pub.publish(twist)

    def watchdog(self, event):
        # 如果超過 0.5 秒沒收到任何訊息 (例如感知程式當掉、網路斷線)
        # 強制發送停車指令
        if (rospy.Time.now() - self.last_msg_time).to_sec() > 0.5:
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)

if __name__ == "__main__":
    try:
        RemoteController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass