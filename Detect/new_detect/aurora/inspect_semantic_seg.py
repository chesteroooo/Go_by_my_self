#!/usr/bin/env python3
# =============================================================================
# inspect_semantic_seg.py — TEST the Aurora's BUILT-IN semantic segmentation.
#
# The Aurora publishes /slamware_ros_sdk_server_node/semantic_segmentation as a
# sensor_msgs/Image whose pixels are per-pixel CLASS IDs (onboard AI model).
# This grabs one frame, reports which class IDs appear (and how much of the
# image each covers), colorizes the mask, and saves a PNG so you can see
# whether it usefully separates road / grass / vegetation / building on YOUR scene.
#
# Usage:
#   1) run the Aurora:  roslaunch ~/Go_by_my_self/Detect/new_detect/aurora_slam.launch
#      (point it at the real red road for the meaningful test)
#   2) same ROS master in this terminal, then:
#        python3 inspect_semantic_seg.py [out.png]
# =============================================================================
import sys, numpy as np, cv2, rospy
from sensor_msgs.msg import Image

SEG_TOPIC = "/slamware_ros_sdk_server_node/semantic_segmentation"
OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/im27car/aurora_bags/semantic_seg.png"


def to_np(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("mono8", "8uc1"):
        return buf.reshape(msg.height, msg.width)
    if enc in ("bgr8", "rgb8", "8uc3"):
        a = buf.reshape(msg.height, msg.width, 3)
        # collapse to a single label channel if it's really a label image
        return a[:, :, 0] if np.array_equal(a[:, :, 0], a[:, :, 2]) else a
    if enc in ("mono16", "16uc1"):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    return buf.reshape(msg.height, msg.width, -1)


def main():
    rospy.init_node("inspect_semantic_seg", anonymous=True)
    print(f"waiting for one message on {SEG_TOPIC} (10s)...")
    try:
        msg = rospy.wait_for_message(SEG_TOPIC, Image, timeout=10.0)
    except rospy.ROSException:
        sys.exit("!! no semantic_segmentation message — is the Aurora node running "
                 "and pointed at the scene? (topic silent)")

    lab = to_np(msg)
    print(f"encoding={msg.encoding}  size={msg.width}x{msg.height}  shape={lab.shape}")
    if lab.ndim != 2:
        print("(not a single-channel label image; saving raw)")
        cv2.imwrite(OUT, lab); return

    ids, counts = np.unique(lab, return_counts=True)
    total = lab.size
    print(f"\n{len(ids)} class IDs present:")
    for i, c in sorted(zip(ids, counts), key=lambda z: -z[1]):
        print(f"  id {int(i):3d} : {100.0*c/total:5.1f}%  of image")

    # colorize labels for a human-viewable mask
    palette = (cv2.applyColorMap((lab.astype(np.float32) *
               (255.0 / max(1, lab.max()))).astype(np.uint8), cv2.COLORMAP_JET))
    cv2.imwrite(OUT, palette)
    print(f"\nsaved colorized mask -> {OUT}")
    print("Look at it: does one distinct color cleanly cover the brick bike-lane? "
          "If yes, that class ID is your 'road' signal — no training needed.")


if __name__ == "__main__":
    main()
