# Jetson Orin Nano — On-Device Structure & Deployment Plan

Companion to [JETSON_MIGRATION.md](JETSON_MIGRATION.md) (that doc = the phased *how-to*;
this doc = the *live inventory*, the **protected-file map**, the **isolation boundary**, and the
**"never delete without permission" safety protocol**).

> **Assessed live on 2026-07-27** over the 5G uplink (`ssh wheeltec@10.0.11.2`, ping ~1.1 s).
> The direct-Ethernet `orin` link (192.168.50.2) was down; use it when present for speed.

---

## 1. What the Jetson actually is (corrected Phase-0 facts)

| Item | Live value (2026-07-27) | Consequence |
|---|---|---|
| Board / RAM | Orin **Nano** Dev Kit, **4 GB** (3.3 GiB usable) | tight — swap + TensorRT + small imgsz mandatory |
| L4T / JetPack | **R35.6.0 → JetPack 5.1.4** | ✅ Ubuntu 20.04 → **ROS Noetic native** |
| OS / arch | Ubuntu **20.04.6** / **aarch64** | plan holds as written |
| Power mode | nvpmodel **DEFAULT=0** (MAXN for this board) | run `jetson_clocks` to lock; that's the ceiling |
| Disk | 116 G, **21 G free (82 % used)** | enough for swapfile + builds, but watch it |
| Swap | **zram only** (6×285 M ≈ 1.7 GiB), **NO disk swapfile** | **add an 8 GB swapfile** before any big build |
| ROS | **noetic** present; `ROS_MASTER_URI=10.0.11.2`, `ROS_IP=10.0.11.2` preset in `.bashrc` | master already lives here |
| System Python | **3.8.10** (`/usr/bin/python3`) — what ROS uses | target this interpreter |
| TensorRT | **8.5.2.2 present (system)** | ✅ can export `.engine` on-device |
| OpenCV | **THREE coexist**: manual **3.4.5** in `/usr/local` (this is what `import cv2` gives), apt **libopencv 4.2** (runtime), apt **4.5.4** (dev). `darknet_ros` links `libopencv_core.so.3.4` from `/usr/local` → **load-bearing**. `cv_bridge` links apt **4.x**, not 3.4.5. | fragile; don't disturb the 3.4.5 `/usr/local` install or `darknet_ros` breaks |
| torch / torchvision | **Working CUDA torch 1.14 lives in the conda `wheeltec` env** (with torchvision 0.14.1, opencv 4.5.4, numpy). **System** python has torchvision 0.14.1 **with no torch behind it** (half-broken). | use the conda env; the `~/torch-*.whl` is the source already installed there |
| ultralytics / pyrealsense2 / pupil_apriltags | **missing** (from conda env too) | add to the conda `wheeltec` env; librealsense = build from source (RSUSB) |
| Conda | Anaconda3; **`wheeltec` env = the real GPU ML runtime** (chosen 2026-07-27). `conda init` in `.bashrc` (interactive only). | run our nodes from this env |
| D435i | **not plugged in** (only BT `8087:0a2b` on USB) | plug into Orin USB3 (Phase 3) |
| Aurora NIC | **`eth0` is DOWN → FREE** | ✅ Aurora gets the built-in RJ45 at `192.168.11.x` |

**Network map:** `usb0 = 10.0.11.2/24` (robot LAN via the 5G/RNDIS modem — my current path),
`wlan0 = 192.168.0.100` (up), `eth0 = DOWN` (reserved for Aurora), `docker0` present (Docker installed).

---

## 2. Protected — pre-existing work, DO NOT delete/move/overwrite without asking

The home dir is full of prior work. **Everything below is off-limits** unless you explicitly OK it:

- **Catkin workspaces** (all sourced from `.bashrc`): `~/wheeltec_robot`, `~/wheeltec_arm`,
  `~/wheeltec_lidar`, `~/cartographer_noetic`, `~/Autoware` / `~/.autoware` / `~/autoware.ai`
- **Old detection copy**: `~/Detect/` (`ros_detect.py`, `ros_move.py`, `pair_detector_setting.py`,
  `calib_result.yaml`) — an early old_detect snapshot, **separate** from our repo. Not our source of truth, but **leave it**.
- **Car control scripts** in `~/`: `car_*.py`, `carctrl.py`, `carmap.py`, `carstop.py`, `clearMap.py`,
  `cwm_*.py`, `get_lidar*.py`, `CarMove*Srv.py`, `UDP_cam2Jpeg.py`, `camlist.py`, `capcam.py`
- **Builds / SDKs**: `~/opencv-3.4.5/`, `~/anaconda3/`, `~/yolov5-pytorch/`, `~/yolov5-tf2/`,
  `~/3rdparty/`, `~/cJSON/`, `~/data/`, `~/wheeltec_arm`… and the `(NewCarCtrl)` dir
- **Reusable wheels (reuse, don't delete)**: `~/torch-1.14.0a0+nv23.02-cp38…whl`,
  `~/torchvision/`, `~/tensorflow-2.10.0+nv22.10…whl`
- **Running service (ours, keep)**: `5g-watchdog.service`
- **`~/.bashrc`** — append-only, and only after showing you the diff. Never rewrite it.

---

## 3. Isolation boundary — everything WE add lives here (and only here)

| Path | Purpose | Collision risk |
|---|---|---|
| `~/Go_by_my_self/` | the repo (rsync from laptop, then git) | **none** — does not exist yet |
| `~/aurora_ros/` | SLAMTEC SDK workspace (`catkin_make`) | none — does not exist yet |
| `~/maps/` | saved `.stcm` SLAM maps (gitignored) | none — does not exist yet |
| `~/Go_by_my_self/.venv-jetson/` | **Python venv** (recommended, see §4) | none — sandboxed |

**Rule:** anything created by this migration is confined to those four paths. Nothing else in `~/`
is created, moved, or deleted without your explicit go-ahead.

---

## 4. Python environment — DECISION (2026-07-27): conda `wheeltec` env

**No venv.** A real, persistent CUDA environment already exists — the conda `wheeltec` env — with
torch 1.14 (CUDA) + torchvision 0.14.1 + opencv 4.5.4 + numpy + tensorflow. We adopt it as THE
runtime and only *add* the missing pieces:
```bash
conda activate wheeltec
pip install ultralytics pupil-apriltags rospkg catkin_pkg   # + pyrealsense2 (built from source)
```
Rationale: it's already GPU-ready, and it keeps us **off the fragile system Python** that the vendor
base bringup depends on (system has torchvision-without-torch, numpy pinned 1.20, and a manual
OpenCV 3.4.5 in `/usr/local` that `darknet_ros` links against). Running ROS from conda needs the ROS
`dist-packages` on `PYTHONPATH` (from `source /opt/ros/noetic/setup.bash`) — python is 3.8 in both,
so `rospy`/`cv_bridge` load fine; watch for a libstdc++ mismatch and prefer the system one if it bites.

**Rejected — system-wide install:** would require bumping system `numpy`/OpenCV that `darknet_ros`
and the wheeltec base nodes are pinned to. High risk to the wheel controller, low upside.

---

## 5. Function → placement map (what runs where on the Orin)

| Function | File(s) | Runtime need on Orin | Code change? |
|---|---|---|---|
| AprilTag detect | `ros_detect_apriltag.py` | pyrealsense2 + pupil_apriltags, IR stream (CPU) | none |
| Dual detect + road seg | `ros_detect_dual.py` | **GPU** torch / `best3.engine`; drop N100 2+2 split; raise imgsz | **yes** — `_device:=0`, accept `.engine`, conditional thread-split |
| Route following | `route/ros_move_follow_route.py`, `route_graph.py`, `extract_route.py` | pure Python + Aurora pose (light) | none |
| Aurora visual SLAM | `aurora/aurora_slam.launch` (+ `~/aurora_ros`) | SDK aarch64 build; `eth0`=192.168.11.x | none (rebuild SDK) |
| Teleop / diagnostics | `ros_teleop_panel.py`, `diagnose_base.sh` | light | none |
| Task controllers | `ros_move_pair_task.py`, `ros_move_turn_right.py` | light + `/target_info` | none |
| Base bringup (existing) | `turn_on_wheeltec_robot mapping.launch` | already on-device | none — reuse |

---

## 6. Build order (respecting the 4 GB ceiling)

1. **8 GB swapfile** (`/swapfile` or on nvme) — before any build, or torch/librealsense OOM.
2. `sudo nvpmodel -m 0 && sudo jetson_clocks` — max clocks for builds & runtime.
3. Seed the repo: `rsync` laptop → `~/Go_by_my_self/` (incl. gitignored `best*.pt`), then `git`.
4. venv (§4) → **torch from local wheel** → torchvision → ultralytics (no opencv 4.x).
5. **librealsense** from source, `-DFORCE_RSUSB_BACKEND=ON` → `pyrealsense2` (biggest time sink).
6. `pupil_apriltags` (pip aarch64, else source).
7. `~/aurora_ros` → `catkin_make` (SLAMTEC ships aarch64 libs).
8. Export `best3.engine` on-device: `yolo export model=best3.pt format=engine device=0 half=True`.
9. Bring nodes up one at a time watching `tegrastats` (Phase 3→5 of the migration doc).

---

## 7. Safety protocol — "don't delete old files without permission"

1. **No `rm`/`mv` outside `~/Go_by_my_self`, `~/aurora_ros`, `~/maps`** without explicit approval.
2. **rsync into the repo only**, never with `--delete` against `~/`. The existing `~/Detect` is a
   *different path* and stays untouched.
3. **`.bashrc` = append-only**, shown as a diff first (it already sources 5 workspaces + conda —
   one broken line breaks every login).
4. **apt = additive only**; no `apt remove` / `autoremove`.
5. **pip stays in the venv** (one deletable folder) — system site-packages untouched.
6. **Manifest before touching anything** so we can prove exactly what we added:
   ```bash
   ls -la ~/ > ~/pre_migration_home.txt
   pip3 freeze > ~/pre_migration_pip.txt
   dpkg -l > ~/pre_migration_dpkg.txt
   ```
7. Reuse, don't re-fetch: the torch/tf wheels are already on disk.

---

## 8. Old-files audit (assessed 2026-07-27) — necessity, health, disk

Disk is **82 % full (21 G free)**. Classification:

| Item | Size | Verdict | Evidence |
|---|---|---|---|
| `wheeltec_robot` / `_lidar` / `_arm` — **the wheel controller** | 2.8 G | **KEEP — OFF-LIMITS** | base bringup, motors, lidar, `darknet_ros`. User: do not touch. |
| `cartographer_noetic` | 1.3 G | **KEEP** | `mapping.launch` uses cartographer (20 refs) |
| conda `wheeltec` env | 2.9 G | **KEEP (our runtime)** | CUDA torch/torchvision/opencv/numpy/tf |
| `/usr/local` OpenCV **3.4.5** install | (system) | **KEEP — load-bearing** | `darknet_ros` links `libopencv_core.so.3.4` |
| **Autoware** (`Autoware`+`.autoware`+`autoware_shared_dir`+`autoware.ai`) | **~15.7 G** | **APPROVED to remove** (pending final go + verify independence) | abandoned; not in bringup/boot |
| conda **base** env | ~5.3 G | bloat, not approved | auto-activates; unused by ROS |
| `opencv-3.4.5/` source tree | 4.0 G | not approved | build leftover (install stays in `/usr/local`) |
| loose `Anaconda3-*.sh`, `tensorflow-*.whl`, `torch-*.whl` | ~1.8 G | not approved | installers/wheels |
| `yolov5-pytorch`, `yolov5-tf2` | 0.7 G | not approved | likely superseded by ultralytics |
| `~/Detect`, `car_*.py`, `cwm_*.py`, … | small | KEEP | old scripts, harmless |

**Environment health = fragile:** system torchvision has no torch; system numpy pinned 1.20;
three OpenCVs coexist. The vendor base is pinned to those — hence the conda-env decision (§4).

---

## 9. Decisions & pending actions

**Decided (2026-07-27):**
- Runtime = **conda `wheeltec` env** (§4). No venv, no system-wide ML install.
- Cleanup scope = **Autoware stack only**. The **wheel controller is off-limits.**

**DONE (2026-07-27):**
- Manifest snapshotted on device: `~/pre_migration_{home,dpkg}.txt`, `~/pre_migration_conda_wheeltec_pip.txt`.
- Verified Autoware independent of wheel controller/boot/docker (only link = one `.bashrc` line).
- **Autoware stack removed** (`~/Autoware ~/.autoware ~/autoware_shared_dir ~/autoware.ai`) →
  freed **15 G** (82 %→67 %, now **37 G free**). `.bashrc` line 143 commented; backup at `~/.bashrc.pre_autoware`.

**Still available to reclaim (not yet approved):**
- Docker image `myautoware_gnss:latest` (**12 G** in `/var/lib/docker`) — no container uses it. `docker rmi` frees ~12 G.
- conda base bloat, `opencv-3.4.5/` source tree (4 G), loose installers/wheels (~1.8 G), yolov5 dirs (0.7 G).

**DONE (2026-07-27) — runtime environment built & verified in conda `wheeltec` env:**
- ✅ 8 GB swapfile (persisted in fstab; total swap 9.7 G)
- ✅ repo + `best*.pt` on Jetson at `~/Go_by_my_self/`
- ✅ `torch 1.14 (CUDA=True)`, `torchvision 0.14.1`, `numpy 1.24.4`, `cv2 5.0`, `rospkg`
- ✅ `ultralytics 8.4.107` — loads `best.pt`, classes `{0:grass,1:road,2:sidewalk}`
- ✅ `pupil_apriltags 1.0.4`
- ✅ **`pyrealsense2 2.54.2`** built from source (RSUSB, against conda py3.8, `.so` copied into
  env site-packages; udev rules installed) — **sees the D435i** (SN 021222072869, FW 5.17.0.10)

**Internet note:** Jetson has no native internet (WiFi router dead; 5G = restricted tunnel).
Laptop internet-sharing failed (5G modem in the middle). Fix used = **phone hotspot `gugu`** on
`wlan0` (10.204.176.x). Env build needs no internet once done; hotspot can be off.

**Temporary/runtime changes to clean up later (all clear on reboot):** laptop iptables MASQUERADE +
`ip_forward` + `rp_filter=0`; Jetson `resolvectl dns usb0`. Nothing written to config files.

**Watch:** ultralytics pulled **opencv-python 5.0** + numpy 1.24 into the conda env — if a node uses
`cv_bridge` (built against apt OpenCV 4.x) alongside cv2, check for ABI friction (Python side passes
numpy arrays, so likely fine).

**DONE (2026-07-27, cont'd):**
- ✅ `ros_detect_dual.py` GPU changes (auto `_device`, `_imgsz`, GPU skips N100 thread-split, prefers `.engine`).
- ✅ Camera smoke-test: **GPU seg 42 ms / 24 fps** on real D435i frames.
- ✅ Aurora onto Jetson `eth0` = `192.168.11.2/24` → `192.168.11.1` reachable.
- ✅ **Aurora SDK built** (`~/aurora_ros`, `slamware_ros_sdk_server_node`). Fix: `target_link_libraries`
  was missing `${OpenCV_LIBRARIES}` **and** cmake grabbed darknet's `/usr/local` OpenCV 3.4.5 → forced
  `-DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4` (4.5.4, matches cv_bridge).
- ✅ **Launcher scripts**: `~/robot_menu.sh` (menu) + `~/Go_by_my_self/autopilot_test.sh` (one-click:
  pick route → base + Aurora SLAM + load map + wait-localize → follow route). Teleop = GUI popup.
- ✅ **Map `~/maps/compus5.2.stcm`** on Jetson (692 MB, verified) — autopilot auto-selects it.

**BLOCKER — Aurora streams no sensor data:** node **connects OK** (SDK 2.1.1-rtm = firmware), no stale
client, but **IMU/pose/images/status ALL silent**. Not network/build/version. Suspect **power**
(board up, sensor/SLAM subsystem off on marginal USB-C) or device not activated. Decisive test:
open **Aurora Remote app** on the Jetson desktop → connect `192.168.11.1` → if app shows preview/pose,
it's the ROS wrapper (needs a start-mapping call?); if app also blank, it's device/power → use DC
9–24 V jack, power-cycle. Until this streams pose, autopilot (`robot_menu.sh` → 4) waits at localize.

**TensorRT engine export deferred** — JetPack `tensorrt` not visible in conda env + broken onnx/protobuf;
seg already runs on GPU via torch, so it's a later speedup, not a blocker.
