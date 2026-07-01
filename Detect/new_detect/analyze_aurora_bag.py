#!/usr/bin/env python3
# =============================================================================
# analyze_aurora_bag.py — offline SLAM-quality report from a record_aurora bag.
#
# Reads /slamware_ros_sdk_server_node/robot_pose (+ state / tf / status) and
# reports: trajectory stats, suspicious pose JUMPS (relocalization/drift),
# SLAM state transitions, and the TF frame tree (to confirm aurora_* renaming).
# Optionally saves an XY trajectory plot (PNG) if matplotlib is available.
#
# Usage:
#   source /opt/ros/noetic/setup.bash
#   source ~/aurora_ros/devel/setup.bash          # so custom msgs deserialize
#   python3 analyze_aurora_bag.py [path/to.bag]   # default: newest in ~/aurora_bags
# =============================================================================
import glob, math, os, sys
import rosbag

POSE_TOPIC = "/slamware_ros_sdk_server_node/robot_pose"
STATE_TOPIC = "/slamware_ros_sdk_server_node/state"
SYS_TOPIC = "/slamware_ros_sdk_server_node/system_status"
RELOC_TOPIC = "/slamware_ros_sdk_server_node/relocalization_status"
JUMP_SPEED = 2.5  # m/s between consecutive poses => flag as a jump (walking ~1.4)


def pick_bag(argv):
    if len(argv) > 1:
        return argv[1]
    bags = sorted(glob.glob(os.path.expanduser("~/aurora_bags/*.bag")),
                  key=os.path.getmtime)
    if not bags:
        sys.exit("no bag given and none found in ~/aurora_bags/")
    return bags[-1]


def main():
    bag_path = pick_bag(sys.argv)
    print(f"== analyzing: {bag_path} ==\n")

    poses, states, tf_edges = [], [], set()
    status = {SYS_TOPIC: [], RELOC_TOPIC: []}  # (t, status_string)
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages():
            if topic == POSE_TOPIC:
                p = msg.pose.position
                poses.append((msg.header.stamp.to_sec(), p.x, p.y, p.z))
            elif topic == STATE_TOPIC:
                states.append((t.to_sec(), msg.data))
            elif topic in status:
                status[topic].append((t.to_sec(), msg.status))
            elif topic in ("/tf", "/tf_static"):
                for tr in msg.transforms:
                    tf_edges.add((tr.header.frame_id, tr.child_frame_id))

    # ---- trajectory ----
    if len(poses) < 2:
        print("!! too few poses to analyze"); return
    poses.sort()
    t0 = poses[0][0]
    path_len = 0.0
    jumps = []
    xs = [p[1] for p in poses]; ys = [p[2] for p in poses]; zs = [p[3] for p in poses]
    for a, b in zip(poses, poses[1:]):
        dt = b[0] - a[0]
        d = math.dist(a[1:], b[1:])
        path_len += d
        if dt > 0 and d / dt > JUMP_SPEED:
            jumps.append((b[0] - t0, d, d / dt))
    disp = math.dist(poses[0][1:], poses[-1][1:])

    print("TRAJECTORY")
    print(f"  poses          : {len(poses)}  over {poses[-1][0]-t0:.1f}s")
    print(f"  path length    : {path_len:.2f} m")
    print(f"  net displacement: {disp:.2f} m  (start->end straight line)")
    print(f"  X range        : {max(xs)-min(xs):.2f} m")
    print(f"  Y range        : {max(ys)-min(ys):.2f} m")
    print(f"  Z range        : {max(zs)-min(zs):.2f} m  (should be ~small if level)")

    # ---- jumps (relocalization / drift) ----
    print("\nPOSE JUMPS (> %.1f m/s between frames — possible relocalization/drift)" % JUMP_SPEED)
    if not jumps:
        print("  none — pose is continuous (good: no tracking snaps)")
    else:
        for ts, d, sp in jumps[:20]:
            print(f"  t+{ts:6.1f}s : jumped {d:.2f} m ({sp:.1f} m/s)")
        print(f"  total jumps: {len(jumps)}")

    # ---- SLAM state transitions ----
    print("\nSLAM STATE (/state) transitions")
    if states:
        last = None
        for ts, s in states:
            if s != last:
                print(f"  t+{ts-t0:6.1f}s : {s}")
                last = s
    else:
        print("  (no /state messages)")

    # ---- device status transitions (health / relocalization) ----
    print("\nDEVICE STATUS transitions")
    for topic, label in ((SYS_TOPIC, "system_status"), (RELOC_TOPIC, "relocalization_status")):
        seq = status[topic]
        if not seq:
            print(f"  {label}: (none)"); continue
        last = None
        for ts, s in seq:
            if s != last:
                print(f"  t+{ts-t0:6.1f}s : {label} -> {s}")
                last = s
    print("  note: 'DeviceRunning' + 'RelocalizationNone' throughout = device saw no fault;")
    print("        relocalization_status only becomes meaningful after LOADING a saved map.")

    # ---- TF tree (verify aurora_* renaming) ----
    print("\nTF EDGES (parent -> child)")
    for parent, child in sorted(tf_edges):
        print(f"  {parent:20s} -> {child}")

    # ---- optional plot ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        out = os.path.splitext(bag_path)[0] + "_traj.png"
        plt.figure(figsize=(6, 6))
        plt.plot(xs, ys, "-", lw=1)
        plt.plot(xs[0], ys[0], "go", label="start")
        plt.plot(xs[-1], ys[-1], "rs", label="end")
        for ts, d, sp in jumps:
            pass  # jumps already listed; markers optional
        plt.axis("equal"); plt.grid(True); plt.legend()
        plt.title(os.path.basename(bag_path))
        plt.xlabel("x (m)"); plt.ylabel("y (m)")
        plt.savefig(out, dpi=110, bbox_inches="tight")
        print(f"\nsaved trajectory plot -> {out}")
    except Exception as e:
        print(f"\n(plot skipped: {e})")


if __name__ == "__main__":
    main()
