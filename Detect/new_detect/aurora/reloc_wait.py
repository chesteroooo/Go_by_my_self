#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reloc_wait.py — 要求 Aurora 重定位，並可靠地等到結果

為什麼不沿用 `rostopic echo … | grep -m1 Succeed`（field_test.sh 原本的做法）：

  1. **成功狀態是 edge-triggered。** 裝置持續回報 SUCCEED 時，ROS topic 上發的是
     "RelocalizationNone" —— server_workers.cpp 會拿 lastRelocalizationStatus_
     比對，一樣就改發 None。所以 "RelocalizationSucceed" 每次重定位只出現**一則**，
     訂閱建立得比它晚就永遠等不到，腳本等滿逾時判定失敗，但車其實已經定位好了。
     本程式**先訂閱、確認訂閱真的接上發佈者，才送出請求**，把這個競態消掉。

  2. **殘留的請求會讓重試變成空操作。** 前一次還沒結束時，relocalization service
     直接回 success=false（"Relocalization already in progress"），再按幾次都沒用。
     預設先發 relocalization/cancel 清掉殘留狀態。

另外拿「位姿跳變」當獨立佐證：車靜止時，位姿只有在重定位把它拉進地圖座標系的那一刻
會跳。跳超過 POSE_JUMP_M 就認定成功 —— 這條在 topic 那則訊息真的被漏掉時仍然有效。
（前提是重定位期間車不要動，這本來就是建議做法；要關掉用 --no-pose-jump。）

用法：
  python3 reloc_wait.py                  # 發出重定位並等待
  python3 reloc_wait.py --timeout 240
  python3 reloc_wait.py --no-request     # 只等待，不主動發（別人已經發過了）
  python3 reloc_wait.py --no-cancel      # 不要先取消殘留請求

退出碼：0 = 重定位成功；1 = 未成功（逾時／失敗／Aurora 節點不在）
"""

import argparse
import sys
import time

NS = "/slamware_ros_sdk_server_node"
POSE_JUMP_M = 1.0        # 車靜止時位姿跳這麼多 = 重定位把位姿拉進地圖座標系了
SUB_WAIT_S = 10.0        # 等訂閱接上發佈者的上限


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timeout", type=float, default=180.0, help="等待上限（秒）")
    ap.add_argument("--no-request", dest="request", action="store_false",
                    help="不發出重定位請求，只等結果")
    ap.add_argument("--no-cancel", dest="cancel", action="store_false",
                    help="不要先取消殘留的重定位請求")
    ap.add_argument("--no-pose-jump", dest="pose_jump", action="store_false",
                    help="不把位姿跳變當成功佐證（車在重定位期間會移動時用）")
    args = ap.parse_args()

    import rospy
    from geometry_msgs.msg import PoseStamped
    try:
        from slamware_ros_sdk.msg import (RelocalizationCancelRequest,
                                          RelocalizationStatus)
        from slamware_ros_sdk.srv import RelocalizationRequest
    except ImportError:
        print("!! 匯入不到 slamware_ros_sdk —— 先 source ~/aurora_ros/devel/setup.bash")
        return 1

    rospy.init_node("reloc_wait", anonymous=True, disable_signals=True)
    state = {"status": "", "pose": None, "t_status": 0.0}

    def cb_status(m):
        state["status"], state["t_status"] = m.status, time.time()

    def cb_pose(m):
        state["pose"] = (m.pose.position.x, m.pose.position.y)

    sub = rospy.Subscriber(NS + "/relocalization_status", RelocalizationStatus,
                           cb_status, queue_size=10)
    rospy.Subscriber(NS + "/robot_pose", PoseStamped, cb_pose, queue_size=1)

    # 先確認訂閱真的接上發佈者，否則那唯一一則 Succeed 會在建立連線的空窗被漏掉
    t0 = time.time()
    while sub.get_num_connections() == 0 and time.time() - t0 < SUB_WAIT_S:
        time.sleep(0.1)
    if sub.get_num_connections() == 0:
        print(f"!! {SUB_WAIT_S:.0f} 秒內接不到 {NS}/relocalization_status"
              f"（Aurora 節點沒起來？）")
        return 1

    t0 = time.time()
    while state["pose"] is None and time.time() - t0 < 5.0:
        time.sleep(0.1)
    pose0 = state["pose"]
    if pose0 is None:
        print("!! 收不到位姿，Aurora 節點可能沒在發佈")
        return 1
    print(f"訂閱就緒，起始位姿 ({pose0[0]:+.2f},{pose0[1]:+.2f})")

    if args.request:
        if args.cancel:
            # 清掉可能殘留的 relocalization_active_，否則新請求會被直接回絕
            pub = rospy.Publisher(NS + "/relocalization/cancel",
                                  RelocalizationCancelRequest, queue_size=1)
            t0 = time.time()
            while pub.get_num_connections() == 0 and time.time() - t0 < 3.0:
                time.sleep(0.1)
            pub.publish(RelocalizationCancelRequest())
            time.sleep(0.5)
        try:
            rospy.wait_for_service(NS + "/relocalization", timeout=10.0)
            ok = rospy.ServiceProxy(NS + "/relocalization", RelocalizationRequest)().success
            print(f"已送出重定位請求（service success={ok}）"
                  + ("" if ok else " —— 裝置說還有一次在進行中，繼續等結果"))
        except Exception as e:                       # noqa: BLE001 — 服務掛掉也要繼續等
            print(f"（重定位 service 呼叫失敗：{e}；仍然等 topic 結果）")

    print(f"等待重定位結果（最多 {args.timeout:.0f} 秒，車放特徵多的地方、靜止不動比較快）…")
    t0, last = time.time(), ""
    while time.time() - t0 < args.timeout:
        if rospy.is_shutdown():
            return 1
        s = state["status"]
        if s != last and s:
            print(f"  狀態 {s}")
            last = s
        if "Succeed" in s:
            print("✓ 重定位成功（收到 RelocalizationSucceed）")
            return 0
        if any(k in s for k in ("Fail", "Cancel", "Abort")):
            print(f"✗ 重定位未成功（{s}）")
            return 1
        if args.pose_jump and state["pose"]:
            d = ((state["pose"][0] - pose0[0]) ** 2
                 + (state["pose"][1] - pose0[1]) ** 2) ** 0.5
            if d > POSE_JUMP_M:
                print(f"✓ 位姿跳了 {d:.1f} m → ({state['pose'][0]:+.2f},"
                      f"{state['pose'][1]:+.2f})，判定重定位成功"
                      f"（車靜止時只有重定位會讓位姿跳）")
                return 0
        el = time.time() - t0
        print(f"\r  已等 {el:4.0f}s / {args.timeout:.0f}s   ", end="", flush=True)
        time.sleep(0.2)

    print(f"\n✗ 逾時 {args.timeout:.0f} 秒仍沒有結果（最後狀態：{last or '無'}）")
    return 1


if __name__ == "__main__":
    sys.exit(main())
