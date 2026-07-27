# Jetson Orin Nano Migration + Remote-Dev Plan

Goal: move **all** perception/control from the x86 laptop onto the robot's **Jetson Orin Nano**
so the robot is **standalone**. Laptop becomes dev + monitor only.

## Topology (target)

```
                 robot (standalone)
   ┌───────────────────────────────────────────────┐
   │  Jetson Orin Nano  (10.0.11.2, ROS master)     │
   │   ├─ wheeltec base bringup (mapping.launch)    │
   │   ├─ D435i  ── USB3 ──┐                         │
   │   ├─ Aurora S ─ Eth ──┤  perception + control   │
   │   │      (192.168.11.1)│  all on-board (GPU)    │
   │   └─ roscore / /cmd_vel → motors               │
   └───────────────────────────────────────────────┘
              ▲  10.0.11.0/24 (robot LAN, WiFi/AP)
              │
   ┌──────────┴───────────┐
   │  Laptop 10.0.11.3     │  dev (VS Code Remote-SSH) + monitor (rviz/rostopic)
   │  eth0 = USB net card  │  NOT in the runtime path anymore
   └──────────────────────┘
```

Key facts:
- `10.0.11.2` = Orin (robot). `10.0.11.3` = laptop. ROS master already on the Orin.
- Laptop reaches the robot LAN via the **USB net card `eth0`** — needs an IP (see Phase 1).
- Aurora is a separate subnet `192.168.11.x`; see Phase 4 for how it attaches to the Orin.

---

## Phase 0 — RESULTS (assessed 2026-07-24)

| Item | Value | Consequence |
|---|---|---|
| OS | Ubuntu **20.04** | ✅ ROS Noetic native — plan holds as written |
| ROS | **noetic** installed | ✅ base present |
| Board / RAM | Orin Nano Dev Kit, **4 GB** (3.3 GiB usable) | ⚠️ tight — TensorRT + small imgsz + more swap are mandatory |
| Disk / Swap | **21 GB** free / **1.7 GiB** swap | add an **8 GB swapfile** before building torch/librealsense (OOM risk on 4 GB) |
| Power | nvpmodel **10 W** (= max for 4 GB) | run `jetson_clocks` to lock clocks; that's the ceiling |
| torch/torchvision | **MISSING** | install NVIDIA Jetson CUDA wheels |
| pyrealsense2 | **MISSING** | build librealsense from source |
| ultralytics | **MISSING** | pip after torch |
| pupil_apriltags | **MISSING** | pip/source |
| OpenCV | **3.4.5** (old, no CUDA) | verify nodes don't need cv2 4.x APIs; consider upgrading |
| repo / aurora_ros | **absent** | rsync repo+models over the link; clone+build SDK on Orin |
| D435i on USB | **not connected** | plug into Orin USB3 (Phase 3) |

**4 GB reality check:** do NOT expect torch + YOLO + Aurora SLAM + AprilTag + ROS all resident at once
without help. Mandatory: (1) an 8 GB swapfile, (2) TensorRT `.engine` instead of full torch at runtime,
(3) small `imgsz`, (4) bring nodes up one at a time and watch `tegrastats` for OOM.

---

## Phase 0 — Connect + assess the Orin (do first, once robot is powered on)

Bring the laptop's robot-link up, then SSH in:

```bash
# on the laptop
sudo ip addr add 10.0.11.3/24 dev eth0     # persist later via netplan/NM
ping -c2 10.0.11.2
ssh wheeltec@10.0.11.2
```

Then run this assessment ON THE ORIN and record the output — it decides the rest:

```bash
# --- OS / JetPack ---
cat /etc/nv_tegra_release            # L4T version → JetPack version
lsb_release -a                       # Ubuntu 20.04? (needed for native ROS Noetic)
uname -m                             # aarch64
# --- compute ---
nvidia-smi 2>/dev/null || sudo tegrastats --interval 1000   # GPU present, live load
nvpmodel -q                          # current power mode
free -h                              # RAM: 4GB vs 8GB matters a LOT
df -h /                              # disk headroom for torch/CUDA/librealsense builds
# --- what's already installed ---
ls /opt/ros/                         # noetic present?
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())" 2>&1
python3 -c "import pyrealsense2 as rs; print(rs.__version__)" 2>&1
python3 -c "import ultralytics, pupil_apriltags; print('ok')" 2>&1
# --- networking / ports (for sensors) ---
ip -brief addr                       # is 10.0.11.x on WiFi or the RJ45? (frees Eth for Aurora)
lsusb                                # USB3 controller for the D435i
```

**Decision gate:** if `lsb_release` shows **Ubuntu 22.04 (JetPack 6)**, ROS Noetic is not native →
either run Noetic in a container / RoboStack, or plan a jump to ROS 2. If **20.04 (JetPack 5)**,
continue natively (assumed below, since the wheeltec base already runs Noetic there).

---

## Phase 1 — Remote-dev workflow (VS Code Remote-SSH)

1. **Passwordless SSH** from laptop:
   ```bash
   ssh-keygen -t ed25519            # if you don't have a key
   ssh-copy-id wheeltec@10.0.11.2
   ```
   Add to `~/.ssh/config` on the laptop:
   ```
   Host orin
       HostName 10.0.11.2
       User wheeltec
   ```
2. **VS Code**: install the *Remote - SSH* extension → connect to `orin`. VS Code auto-installs its
   aarch64 server into `~/.vscode-server` on the Orin. (On a 4GB Orin, keep extensions minimal — the
   server uses RAM.)
3. **Get the repo onto the Orin.** GitHub is slow here (partial clone), so seed it over the fast
   local link, then keep git for pulls:
   ```bash
   # from the laptop — copies working tree incl. gitignored models
   rsync -avz --exclude '.git' /home/im27car/Go_by_my_self/ wheeltec@10.0.11.2:~/Go_by_my_self/
   # models are gitignored — copy them explicitly if not already in the tree
   rsync -avz /home/im27car/Go_by_my_self/Detect/new_detect/best*.pt wheeltec@10.0.11.2:~/Go_by_my_self/Detect/new_detect/
   ```
   On the Orin, `git init`/`git remote add` (or clone once) so it's the source of truth. Edit live via
   Remote-SSH; commit on the Orin.

---

## Phase 2 — Build the aarch64 + CUDA software stack (the hard part)

Order matters — install CUDA torch **before** ultralytics so it doesn't pull a CPU wheel.

| Component | How on Jetson (NOT plain `pip install`) |
|---|---|
| **PyTorch + torchvision** | NVIDIA Jetson wheels matched to your JetPack (Jetson Zoo / NVIDIA pip index). Gives CUDA torch. |
| **ultralytics (YOLOv8-seg)** | `pip install ultralytics` *after* torch is in place; verify `YOLO(...).to('cuda')`. |
| **pyrealsense2** | Build **librealsense** from source: `-DBUILD_PYTHON_BINDINGS=ON -DFORCE_RSUSB_BACKEND=ON` (RSUSB avoids kernel patching). Install udev rules. (JetsonHacks `installLibrealsense` script works.) |
| **pupil-apriltags** | Try pip aarch64 wheel; if none, build from source. |
| **cv-bridge / vision-opencv** | `sudo apt install ros-noetic-cv-bridge ros-noetic-vision-opencv`. |
| **Aurora SDK** | Rebuild `~/aurora_ros` (`slamware_ros_sdk`) with `catkin_make` on the Orin — SLAMTEC ships aarch64 libs. |

Performance setup on the Orin:
```bash
sudo nvpmodel -m 0        # MAXN
sudo jetson_clocks        # lock clocks high
```
Export the seg model to **TensorRT** on the Orin (engines are device/version specific — build here):
```bash
yolo export model=best3.pt format=engine device=0 half=True   # → best3.engine
```

---

## Phase 3 — Move the D435i onto the Orin

- Plug D435i into an Orin **USB3** port. `realsense-viewer` or `rs-enumerate-devices` to confirm.
- Remember the rule from CLAUDE.md: **only one program opens the camera at a time** — now on the Orin.
- Smoke test: `python3 Detect/new_detect/ros_detect_apriltag.py` (IR/AprilTag path first — lightest).

---

## Phase 4 — Move Aurora onto the Orin

Aurora needs a `192.168.11.x` Ethernet link. Check Phase-0 `ip addr`:
- **If the robot LAN (10.0.11.x) rides on the Orin's WiFi** → the built-in **RJ45 is free**: plug Aurora
  straight into it, give that interface `192.168.11.2/24`. Cleanest.
- **If the RJ45 is already the robot LAN** → add a **USB3-Gigabit adapter** on the Orin dedicated to
  Aurora, *or* move Aurora onto the robot switch and re-IP it into `10.0.11.x`.
- Only one client may hold the Aurora — close the Aurora Remote app first.
- Bring up: `roslaunch ~/Go_by_my_self/Detect/new_detect/aurora/aurora_slam.launch`; check
  `rostopic hz /slamware_ros_sdk_server_node/robot_pose`.

---

## Phase 5 — Perception + control on the Orin

Bring nodes up one at a time (each validated on the laptop already):
1. `ros_detect_apriltag.py` — AprilTag only.
2. `ros_detect_dual.py` — AprilTag(IR) + seg(Color). Now **GPU**, so:
   - run with `device=0` / the `.engine` model,
   - drop the N100 2+2 CPU thread-split assumption,
   - you can raise `YOLO_IMGSZ` above 320.
3. Control: `ros_move_pair_task.py` / lane-centering drive mode.

Env on the Orin (local master):
```bash
export ROS_MASTER_URI=http://10.0.11.2:11311
export ROS_IP=10.0.11.2
```
Laptop stays a monitor: same `ROS_MASTER_URI`, `ROS_IP=10.0.11.3`, plus `/etc/hosts` entries both ways
so hostnames resolve.

### Code changes needed (small)
- `ros_detect_dual.py`: make YOLO **device configurable** (`_device:=0`), accept a `.engine` model,
  make the N100 thread-split conditional (skip on Orin), allow larger `imgsz`.
- Confirm model auto-find picks the Orin-local `best3.pt`/`best3.engine`.
- `sys.path.append('../apriltag_setting')` relative imports work as long as the repo layout is preserved.

---

## Phase 6 — Autostart (standalone robot)

systemd units on the Orin so it comes up without a laptop:
- `roscore` (or the wheeltec base's existing bringup service) →
- perception (`ros_detect_dual.py`) `After=` base →
- control.
Add `nvpmodel -m 0` + `jetson_clocks` at boot. Keep an easy kill switch (teleop e-stop / a `stop` unit).

---

## Risk / watch-list
- **4GB vs 8GB Orin**: YOLO + Aurora SLAM + AprilTag + ROS together is tight on 4GB — watch for OOM.
- **pyrealsense2 build** is the most likely time-sink; RSUSB backend avoids kernel patching.
- **JetPack 6 / Ubuntu 22.04** would break native ROS Noetic — verify in Phase 0.
- **Slow GitHub/PyPI** here: prefer local `rsync` for the repo/models; expect long pip/build times.
- **TensorRT engines are not portable** — build on the Orin, rebuild after any JetPack/torch bump.
- **Aurora's second subnet** needs a free NIC on the Orin — resolve in Phase 4 before field use.
```
