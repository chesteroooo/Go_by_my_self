#!/usr/bin/env python3
# =============================================================================
# aurora_status.py — headless "status light" for the Aurora (no Aurora Remote app).
# Live one-line readout of: device status, relocalization status, pose rate, and
# current pose — so you can tell when SLAM has INITIALIZED (safe to move / save)
# and when it has RELOCALIZED into a loaded map.
#
# Usage (node running, same ROS master):
#   source ~/aurora_ros/devel/setup.bash
#   python3 aurora_status.py
# Ctrl+C to quit.
# =============================================================================
import math, sys, rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
try:
    from slamware_ros_sdk.msg import SystemStatus, RelocalizationStatus
except ModuleNotFoundError:
    sys.exit("slamware_ros_sdk not found — source the Aurora workspace first:\n"
             "  source /opt/ros/noetic/setup.bash\n"
             "  source ~/aurora_ros/devel/setup.bash")

NS = "/slamware_ros_sdk_server_node"
S = {"sys": "?", "reloc": "?", "conn": "?", "pose": None, "stamps": []}


def _yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    rospy.init_node("aurora_status", anonymous=True)
    rospy.Subscriber(NS + "/system_status", SystemStatus, lambda m: S.update(sys=m.status))
    rospy.Subscriber(NS + "/relocalization_status", RelocalizationStatus,
                     lambda m: S.update(reloc=m.status))
    rospy.Subscriber(NS + "/state", String, lambda m: S.update(conn=m.data))

    def cb_pose(m):
        S["pose"] = m.pose
        now = rospy.get_time()
        S["stamps"].append(now)
        S["stamps"] = [t for t in S["stamps"] if now - t < 3.0]
    rospy.Subscriber(NS + "/robot_pose", PoseStamped, cb_pose)

    t0 = rospy.get_time()
    r = rospy.Rate(2)
    print("watching Aurora (Ctrl+C to quit)...")
    while not rospy.is_shutdown():
        st = S["stamps"]
        hz = (len(st) - 1) / (st[-1] - st[0]) if len(st) > 1 else 0.0
        p = S["pose"]
        pos = (f"x={p.position.x:+.2f} y={p.position.y:+.2f} yaw={_yaw(p.orientation):+6.1f}"
               if p else "pose=none          ")
        if hz > 5 and S["sys"] == "DeviceRunning":
            health = "TRACKING OK "
        elif hz == 0:
            health = "INIT / WAIT "
        else:
            health = "WEAK        "
        line = (f"[t+{rospy.get_time()-t0:5.0f}s] {health} sys={S['sys']:13s} "
                f"reloc={S['reloc']:18s} pose={hz:4.1f}Hz  {pos}")
        print("\r" + line + "   ", end="", flush=True)
        r.sleep()


if __name__ == "__main__":
    main()
