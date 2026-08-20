#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
preflight.py — 出發前檢查：Aurora 位姿真的落在這條路線上、而且車頭朝對方向嗎？

這是自動駕駛前最後、也是最實在的一道關卡。重定位「成功」不等於定位「正確」，
而位姿有在發佈更是完全不能當定位成功的證據 —— Aurora 沒對上地圖時照樣會在一個
全新的 session 座標系裡發位姿，車就會照著別張圖的座標開出去（0706 的失敗模式）。

檢查三件事：
  離路線距離 <= --max-off、與路線行進方向夾角 <= --max-yaw，並印出「對到的里程」
  —— 從起點出發時里程應該接近 0，數字很大就表示重定位把車定到路線別的地方去了。

用法：
  python3 preflight.py --plan routes_A_B/plan_A_B.csv
  python3 preflight.py --plan ... --max-off 3.0 --max-yaw 120 --timeout 10

退出碼：0 = 通過，1 = 沒通過／收不到位姿
"""

import argparse
import math
import sys

import numpy as np

POSE_TOPIC = "/slamware_ros_sdk_server_node/robot_pose"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", required=True, help="plan_X_Y.csv")
    ap.add_argument("--max-off", type=float, default=3.0, help="離路線多遠以內放行 (m)")
    ap.add_argument("--max-yaw", type=float, default=120.0, help="方向差多少以內放行 (deg)")
    ap.add_argument("--timeout", type=float, default=10.0, help="等位姿的秒數")
    args = ap.parse_args()

    import rospy
    from geometry_msgs.msg import PoseStamped

    wp = np.loadtxt(args.plan, delimiter=",", comments="#", encoding="utf-8")[:, :2]
    rospy.init_node("preflight", anonymous=True, disable_signals=True)
    try:
        m = rospy.wait_for_message(POSE_TOPIC, PoseStamped, timeout=args.timeout)
    except rospy.ROSException:
        print(f"✗ {args.timeout:.0f} 秒內收不到位姿（Aurora 節點沒起來？）")
        return 1

    x, y = m.pose.position.x, m.pose.position.y
    q = m.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    d = np.linalg.norm(wp - (x, y), axis=1)
    i = int(d.argmin())
    lat = float(d[i])
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(wp, axis=0), axis=1))])
    mileage = float(cum[min(i, len(cum) - 1)])

    j = min(i + 3, len(wp) - 1)
    if j == i:
        i = max(0, i - 3)
    pd = math.atan2(wp[j][1] - wp[i][1], wp[j][0] - wp[i][0])
    alpha = math.degrees(math.atan2(math.sin(pd - yaw), math.cos(pd - yaw)))

    print(f"位姿 ({x:+.2f},{y:+.2f})  yaw {math.degrees(yaw):+.0f}°  "
          f"離路線 {lat:.2f} m  與路線方向差 {alpha:+.0f}°")
    print(f"對到路線里程 {mileage:.0f} m / 全長 {cum[-1]:.0f} m"
          + ("   ← 從起點出發的話這裡應該接近 0" if mileage > 20 else ""))

    if lat <= args.max_off and abs(alpha) <= args.max_yaw:
        print(f"✓ 出發前檢查通過（門檻 {args.max_off:.1f} m / {args.max_yaw:.0f}°）")
        return 0
    print(f"✗ 檢查未通過（門檻 {args.max_off:.1f} m / {args.max_yaw:.0f}°）："
          "車不在路線上、車頭朝錯方向，或重定位其實沒成功")
    return 1


if __name__ == "__main__":
    sys.exit(main())
