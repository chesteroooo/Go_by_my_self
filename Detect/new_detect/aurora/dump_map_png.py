#!/usr/bin/env python3
# =============================================================================
# dump_map_png.py — pull the Aurora's current 2D map out of ROS as a PNG
# + print coverage stats, so a saved .stcm can be visually inspected/analyzed.
#
# Typical use (Aurora powered, App CLOSED — single-client device):
#   T1: roslaunch Detect/new_detect/aurora/aurora_slam.launch
#   T2: Detect/new_detect/aurora/aurora_map.sh load ~/maps/compus1.stcm
#       python3 Detect/new_detect/aurora/dump_map_png.py ~/maps/compus1_map.png
#
# White = free space (the walked route), black = occupied, grey = unknown.
# =============================================================================
import sys
import numpy as np
import cv2
import rospy
from nav_msgs.msg import OccupancyGrid

MAP_TOPIC = "/slamware_ros_sdk_server_node/map"
OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/im27car/aurora_bags/aurora_map.png"


def main():
    rospy.init_node("dump_map_png", anonymous=True)
    print(f"waiting for one map on {MAP_TOPIC} (20s)...")
    try:
        msg = rospy.wait_for_message(MAP_TOPIC, OccupancyGrid, timeout=20.0)
    except rospy.ROSException:
        sys.exit("!! no map received — is the node running and a map loaded/built?")

    W, H, res = msg.info.width, msg.info.height, msg.info.resolution
    grid = np.array(msg.data, dtype=np.int8).reshape(H, W)

    img = np.full((H, W), 128, np.uint8)      # unknown -> grey
    img[grid == 0] = 255                       # free    -> white
    img[grid > 0] = 0                          # occupied-> black
    img = cv2.flip(img, 0)                     # ROS origin bottom-left -> image top-left

    free = int((grid == 0).sum())
    occ = int((grid > 0).sum())
    known = free + occ
    print(f"map: {W}x{H} cells @ {res:.3f} m/cell  =  {W*res:.0f} x {H*res:.0f} m")
    print(f"known cells : {known}  ({100.0*known/grid.size:.1f}% of grid)")
    print(f"free space  : {free*res*res:.0f} m^2   occupied: {occ*res*res:.0f} m^2")
    cv2.imwrite(OUT, img)
    print(f"saved -> {OUT}")


if __name__ == "__main__":
    main()
