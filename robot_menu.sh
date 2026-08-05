#!/bin/bash
# =============================================================================
# robot_menu.sh — one menu to launch every test on the Jetson.
# Run it ON THE JETSON (its own desktop for the GUI tests, or over SSH for the
# headless ones). Movement items (1,7) auto-start the robot base; autopilot (4)
# starts everything it needs.
# =============================================================================
# full ROS workspace chain (same as ~/.bashrc) so ALL packages resolve:
# turn_on_wheeltec_robot (base) + cartographer/lidar/arm + Aurora msgs
source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/cartographer_noetic/devel_isolated/setup.bash --extend 2>/dev/null
source ~/wheeltec_robot/devel/setup.bash --extend 2>/dev/null
source ~/wheeltec_lidar/devel/setup.bash --extend 2>/dev/null
source ~/wheeltec_arm/devel/setup.bash --extend 2>/dev/null
source ~/aurora_ros/devel/setup.bash --extend 2>/dev/null   # --extend: don't drop wheeltec pkgs
source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null
conda activate wheeltec 2>/dev/null
ND=~/Go_by_my_self/Detect/new_detect

# start the robot base (motor driver) if it isn't already up — needed to MOVE
ensure_base(){
  if rosnode list 2>/dev/null | grep -q wheeltec_robot; then echo "[base] already running."; return; fi
  echo "[base] starting robot base (motors) in the background..."
  nohup roslaunch turn_on_wheeltec_robot mapping.launch >~/base.log 2>&1 &
  echo "[base] waiting ~8s for it to come up..."; sleep 8
}

while true; do
  clear
  echo "=============================================="
  echo "   ROBOT TEST MENU  (Jetson $(hostname -I | awk '{print $1}'))"
  echo "   env: $(python3 -c 'import torch;print("torch",torch.__version__,"CUDA",torch.cuda.is_available())' 2>/dev/null)"
  [ -n "$DISPLAY" ] && echo "   display: $DISPLAY (GUI ok)" || echo "   display: NONE (only headless 3/6 work here)"
  echo "=============================================="
  echo "  PERCEPTION"
  echo "   1) Segmentation + lane-centering  (GPU, drives)  [GUI]"
  echo "   2) Segmentation — perception only (no motors)    [GUI]"
  echo "   3) Camera smoke test              (headless)"
  echo "  AUTOPILOT"
  echo "   4) *** AUTOPILOT TEST ***  pick a route, it does the rest"
  echo "  AURORA / DEBUG"
  echo "   5) Aurora built-in segmentation test             [GUI]"
  echo "   6) Aurora status monitor            (headless)"
  echo "  MANUAL"
  echo "   7) Teleop panel  (pop-up window)                 [GUI]"
  echo "  PAPER — TANET 2026 (deadline 08-15)"
  echo "   8) Capture paper dataset  color+depth+metadata   [GUI]"
  echo "   9) Capture + live best.pt overlay (see failures) [GUI]"
  echo "  10) Floor dataset collector (color only, legacy)  [GUI]"
  echo "  11) Capture progress report         (headless)"
  echo "   q) quit"
  echo "----------------------------------------------"
  read -rp "choose: " c
  c="${c//[[:space:]]/}"       # strip stray spaces/CR from the RDP keyboard
  case "$c" in
    1) ensure_base; python3 "$ND/ros_detect_dual.py" ;;
    2) python3 "$ND/ros_detect_dual.py" _drive:=false ;;
    3) python3 ~/seg_smoke.py ;;
    4) bash ~/Go_by_my_self/autopilot_test.sh ;;
    5) python3 "$ND/aurora/inspect_semantic_seg.py" ;;
    6) python3 "$ND/aurora/aurora_status.py" ;;
    7) ensure_base; python3 "$ND/ros_teleop_panel.py" ;;
    8|9)
       read -rp "拍攝地點代號 (預設 A): " loc; loc="${loc//[[:space:]]/}"; loc="${loc:-A}"
       SEG=""; [ "$c" = "9" ] && SEG="--seg"
       python3 "$ND/segmentation/capture_paper_dataset.py" --location "$loc" $SEG ;;
    10) python3 "$ND/segmentation/collect_floor_dataset.py" ;;
    11)
       python3 - "$ND" <<'PY'
# 拍攝進度報表 —— 目標值全部由 capture_paper_dataset.py 的 PRESETS 推導，
# 所以改物件清單時這份報表不會過期。合併所有 session（兩個地點 = 兩次執行 = 兩個 session）。
import ast, csv, glob, os, re, sys, collections

ND = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/Go_by_my_self/Detect/new_detect")
SRC = os.path.join(ND, "segmentation", "capture_paper_dataset.py")
ROOT = os.path.join(ND, "paper_data")
PER_OBJ, N_EMPTY, N_NEG = 6, 6, 34          # 每物件 3距離x2橫向；空景 6；負樣本 ~34

try:
    presets = ast.literal_eval(re.search(r"PRESETS = (\[.*?\n\])",
                               open(SRC, encoding="utf-8").read(), re.S).group(1))
except Exception as e:
    print("!! 讀不到 PRESETS:", e); raise SystemExit
obs = [p for p in presets if p[0] not in ("EMPTY", "NEG")]
meta = {p[1]: p for p in obs}
target = len(obs) * PER_OBJ + N_EMPTY + N_NEG

sessions = sorted(glob.glob(os.path.join(ROOT, "session_*")))
if not sessions:
    print("尚無 session（還沒開始拍）"); raise SystemExit
rows = []
for s in sessions:
    p = os.path.join(s, "metadata.csv")
    if os.path.exists(p):
        rows += list(csv.DictReader(open(p, encoding="utf-8")))
print(f"session {len(sessions)} 個，最新: {os.path.basename(sessions[-1])}")
if not rows:
    print("!! 所有 session 都沒有 metadata.csv"); raise SystemExit

locs = sorted({r["location"] for r in rows})
print(f"總筆數: {len(rows)} / 目標 {target}   地點: {', '.join(locs)}")

print(f"--- 障礙物件進度 (每個目標 {PER_OBJ} 張 = 3距離 x 2橫向) ---")
cnt = collections.Counter(r["object"] for r in rows)
cell = collections.defaultdict(set)
for r in rows:
    cell[r["object"]].add((r["distance_m"], r["lateral"]))
for pid, ob, lvl, seen, zh in obs:
    n = cnt.get(ob, 0)
    tag = "已見" if seen else "未見"
    flag = "OK " if n >= PER_OBJ else "!! "
    line = f"  {flag}{pid:6s} {ob:15s} {tag} {lvl}  {n:3d}/{PER_OBJ}"
    if 0 < n < PER_OBJ:                      # 還沒滿就列出缺哪幾格，現場可直接補
        want = {(str(d), l) for d in (1, 2, 3) for l in ("center", "left")}
        miss = sorted(want - cell[ob])
        if miss:
            line += "  缺: " + " ".join(f"{d}m/{l}" for d, l in miss)
    print(line)

print("--- 空景 / 負樣本 ---")
for L in locs:
    ne = sum(1 for r in rows if r["location"] == L and r["pair_id"] == "EMPTY")
    nn = sum(1 for r in rows if r["location"] == L and r["pair_id"] == "NEG")
    e = "OK " if ne >= N_EMPTY else "!! "
    print(f"  {e}地點 {L}: EMPTY {ne}/{N_EMPTY}   負樣本 {nn}")
print(f"  誤觸發率分母合計: {sum(1 for r in rows if r['pair_id'] in ('EMPTY','NEG'))} 張")

# 最關鍵的一項：未見組的 L2。B1 靠高度分級、高度推不出 L2，
# 所以 B1 的錯誤集合 = 所有 L2 物件；未見組沒有 L2，B1 在未見組就會全對，4.1 主表垮掉。
unseen_l2 = [(ob, cnt.get(ob, 0)) for _, ob, lvl, seen, _ in obs if not seen and lvl == "L2"]
done = [o for o, n in unseen_l2 if n >= PER_OBJ]
print(f"--- 未見組 L2 (主結果命脈): {len(done)}/{len(unseen_l2)} 完成 ---")
for o, n in unseen_l2:
    print(f"  {'OK ' if n >= PER_OBJ else '!! '}{o:15s} {n:3d}/{PER_OBJ}")
if not done:
    print("  !!!! 未見 L2 一個都沒拍完 —— 現在就去補，這是離開現場前的第一優先")
elif len(done) < 2:
    print("  !! 只有一個未見 L2 → 主表在未見組的差距只有一格，限制一節必須載明")

print("--- 等級分布 ---", dict(collections.Counter(r["expected_level"] for r in rows)))
bad = [r["filename_color"] for r in rows if float(r["depth_median_mm"] or 0) <= 0]
if bad: print(f"!! 深度可疑 {len(bad)} 筆:", bad[:5])
noseg = [s for s in sessions if not glob.glob(os.path.join(s, "seg_vis", "*"))]
if noseg:
    print("!! 這些 session 沒有 seg_vis（忘了加 --seg，圖 2 會沒素材）:",
          ", ".join(os.path.basename(s) for s in noseg))
PY
       ;;
    q|Q) exit 0 ;;
    *) echo "unknown choice: '$c'"; sleep 1 ;;
  esac
  echo; read -rp "[done] press Enter to return to menu..." _
done
