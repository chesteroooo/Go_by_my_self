================================================================
ROS 遠端視覺導航通訊協議 (Clean Pair Version)
================================================================
[Topic] /target_info
[Type]  geometry_msgs/Pose

我們將 Pose 分為「控制數據 (Position)」與「識別數據 (Orientation)」兩區。

----------------------------------------------------------------
(A) Position (控制數據區 - 核心座標定義)
    - position.x : [橫向誤差 Horizontal Error] (單位: Pixel)
        - 定義: (Tag中心 X座標) - (畫面中心 X座標)
        - 計算方式: cx - (img_width / 2)
        - 符號意義:
            (-) 負數: Tag 在畫面左邊
            (+) 正數: Tag 在畫面右邊
            0: Tag 在正中間
        * 特性: 純水平誤差，不受 Tag 高度 (Y軸) 影響。

    - position.z : [深度距離 Depth Distance] (單位: Meter)
        - 定義: Tag 到相機平面的垂直深度 (Camera Z-axis)
        - 計算方式: t_vec[2]
        - 符號意義: 數值越小代表越近，數值越大代表越遠。
        * 特性: 純深度距離，不受 Tag 上下左右移動影響 (非歐幾里得距離)。

(B) Orientation (身分與狀態區)
    - orientation.x : [Tag ID 1] (float格式)
    - orientation.y : [Tag ID 2] (float格式)
    - orientation.w : [有效旗標 Valid Flag]
        - 1.0: 看到目標，數據有效 -> 執行 PID 控制
        - 0.0: 目標丟失 -> 執行強制停車