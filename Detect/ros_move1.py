#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Pose
import math

# ================= 參數調校區 =================
APPROACH_TIME = 1.0        # 看到新 Tag 後，繼續往前開的緩衝時間(秒)
ALIGN_THRESHOLD = 15       # 對準容許誤差 (像素)
BLIND_FORWARD_TIME = 5.0   # 看不到任何 Tag 後，允許繼續盲走的時間(秒)

# 【核心新增】新標籤確認時間
TAG_CONFIRM_DURATION = 1.5  # 必須連續辨識到新 ID 達 1.5 秒才承認它

KP_ANGULAR = 0.0005
# 速度設定
MAX_SPEED_V = 0.2          
MAX_SPEED_W = 0.08         
MIN_SPEED_W = 0.08         

# 安全設定
TIMEOUT_SEC = 5.0          
SEARCH_SPEED_W = 0.37
SEARCH_TIMEOUT = 35.0
# ============================================

class RemoteController:
    def __init__(self):
        rospy.init_node('remote_controller_node', anonymous=True)
        # 加上 tcp_nodelay 減少通訊延遲
        self.sub = rospy.Subscriber('/target_info', Pose, self.pose_callback, queue_size=1, tcp_nodelay=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.current_pose = None
        
        rospy.loginfo("⏳ 等待相機連線中...")
        rospy.wait_for_message('/target_info', Pose) 
        rospy.loginfo("✅ 連線成功！啟動導航。")

        # 狀態機初始化
        self.state = 'ADJUSTING' 
        self.last_msg_time = rospy.Time.now()
        self.last_tag_time = rospy.Time.now() 
        self.last_target_id = None
        
        # 計時器變數
        self.approach_start_time = None
        self.search_start_time = None
        self.blind_start_time = None
        
        # 【新增】記錄第一次看到新標籤的時間與 ID
        self.new_tag_detect_start_time = None
        self.potential_new_id = None
        
        rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def pose_callback(self, data):
        self.current_pose = data
        self.last_msg_time = rospy.Time.now()

    def control_loop(self, event):
        twist = Twist()
        current_time = rospy.Time.now()
        
        # 1. 影像狀態檢查
        tag_visible = False
        current_id = None
        
        if self.current_pose is not None:
            if (current_time - self.last_msg_time).to_sec() <= 0.2: # 提高反應靈敏度
                if self.current_pose.orientation.w == 1.0:
                    tag_visible = True
                    self.last_tag_time = current_time
                    current_id = (self.current_pose.orientation.x, self.current_pose.orientation.y)

        # 2. 全域安全看門狗
        if (current_time - self.last_tag_time).to_sec() > TIMEOUT_SEC:
            if self.state not in ['SEARCHING', 'DEAD']:
                rospy.logwarn("⚠️ 遺失目標超過安全時間，啟動原地搜尋！")
                self.state = 'SEARCHING'
                self.search_start_time = current_time

        # ================= 3. 核心狀態機邏輯 =================
        
        if self.state == 'FORWARD':
            if tag_visible:
                self.blind_start_time = None # 只要看到東西就重置盲走
                
                # 檢查是不是新的 ID
                if current_id != self.last_target_id:
                    # 如果跟剛才觀察到的潛在 ID 不同，或是剛開始觀察，就重置計時器
                    if self.potential_new_id != current_id:
                        self.potential_new_id = current_id
                        self.new_tag_detect_start_time = current_time
                        rospy.loginfo(f"❓ 疑似新目標 {current_id}，開始 1.5 秒確認...")
                    
                    # 檢查是否已經「連續」看滿 1.5 秒
                    if (current_time - self.new_tag_detect_start_time).to_sec() >= TAG_CONFIRM_DURATION:
                        rospy.loginfo(f"🎯 確認目標 {current_id} 為下一站！啟動逼近模式。")
                        self.state = 'APPROACHING'
                        self.approach_start_time = current_time
                        self.new_tag_detect_start_time = None
                        self.potential_new_id = None
                        twist.linear.x = MAX_SPEED_V
                    else:
                        # 還在 1.5 秒觀察期內，保持直走
                        twist.linear.x = MAX_SPEED_V
                else:
                    # 看到的是舊 ID，重置確認計時器並繼續走
                    self.new_tag_detect_start_time = None
                    self.potential_new_id = None
                    twist.linear.x = MAX_SPEED_V
            else:
                # 沒看到任何東西，重置確認計時器並啟動/維持盲走
                self.new_tag_detect_start_time = None
                self.potential_new_id = None
                
                if self.blind_start_time is None:
                    self.blind_start_time = current_time
                
                if (current_time - self.blind_start_time).to_sec() >= BLIND_FORWARD_TIME:
                    rospy.logwarn("🛑 盲走超時，轉為搜尋模式。")
                    self.state = 'SEARCHING'
                    self.search_start_time = current_time
                else:
                    twist.linear.x = MAX_SPEED_V

        elif self.state == 'APPROACHING':
            # 逼近階段：保持前進直到時間到
            if (current_time - self.approach_start_time).to_sec() >= APPROACH_TIME:
                twist.linear.x = 0.0
                if tag_visible:
                    rospy.loginfo("🛑 抵達站點，開始對齊。")
                    self.state = 'ADJUSTING'
                else:
                    self.state = 'SEARCHING'
                    self.search_start_time = current_time
            else:
                twist.linear.x = MAX_SPEED_V

        elif self.state == 'ADJUSTING':
            twist.linear.x = 0.0  
            if tag_visible:
                error_x = self.current_pose.position.x
                if abs(error_x) <= ALIGN_THRESHOLD:
                    rospy.loginfo(f"✅ 對齊完成，目標 {current_id} 存入記憶，出發！")
                    self.state = 'FORWARD'
                    self.last_target_id = current_id
                else:
                    # 原地旋轉對齊
                    angular_z = -1 * KP_ANGULAR * error_x
                    if abs(angular_z) < MIN_SPEED_W:
                        angular_z = MIN_SPEED_W if angular_z > 0 else -MIN_SPEED_W
                    twist.angular.z = max(min(angular_z, MAX_SPEED_W), -MAX_SPEED_W)

        elif self.state == 'SEARCHING':
            # 搜尋模式為了反應快，不強制等 1.5 秒，看到新 ID 直接去對準
            if tag_visible and current_id != self.last_target_id:
                rospy.loginfo("🎯 尋回目標，進行對齊。")
                self.state = 'ADJUSTING'
                self.last_tag_time = current_time 
            else:
                if (current_time - self.search_start_time).to_sec() > SEARCH_TIMEOUT:
                    self.state = 'DEAD'  
                else:
                    twist.angular.z = SEARCH_SPEED_W

        elif self.state == 'DEAD':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    try:
        RemoteController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass