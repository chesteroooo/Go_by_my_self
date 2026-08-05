#!/usr/bin/env bash
# =============================================================================
# field_test.sh — 一鍵現場測試（單一終端跑完整流程）
#
#   ./field_test.sh                 # ★正常就這樣跑：用預設地圖 ~/maps/A_D.stcm
#   MAP=~/maps/compus5.2.stcm PLANS=routes_compus5_2 ./field_test.sh   # 換別張圖
#     ⚠ MAP 和 PLANS 必須是同一張圖 —— 不同 .stcm 的座標系不通用，
#       混用會把車開到完全錯的地方。腳本啟動時會擋下不一致的組合。
#
# 依序自動完成：
#   0. 連線檢查（機器人 WiFi / Aurora 乙太網 / 地圖檔）
#   1. 車上底盤 roslaunch（ssh，一次密碼；已在線則跳過）
#   2. Aurora SLAM（背景啟動；已在發佈位姿則跳過）
#   3. rosbag 錄製（背景，結束時自動關檔＋跑分析）
#   4. 載入地圖（sync_set_stcm，~75 秒位姿會暫停—正常）
#   5. ★重定位（上次 0706 測試失敗的根因：load 不會自動重定位！）
#      沒看到 RelocalizationSucceed 不放行
#   6. 行駛選單（出發前自動檢查「在路線上＋朝向正確」才啟動跟線）
#
# 結束（選單按 q 或 Ctrl-C）：自動停 rosbag、關 Aurora、跑 analyze_aurora_bag.py
# 底盤 roslaunch 留在車上繼續跑（不影響下次執行）。
# =============================================================================
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"      # Go_by_my_self
ROUTE_DIR="$ROOT/Detect/new_detect/route"
AURORA_DIR="$ROOT/Detect/new_detect/aurora"
PLANS="${PLANS:-$ROUTE_DIR/routes_A_D}"
[ -d "$PLANS" ] || PLANS="$ROUTE_DIR/$PLANS"          # 允許只給資料夾名
MAP="${MAP:-$HOME/maps/A_D.stcm}"
BAGDIR="$HOME/aurora_bags"
TS=$(date +%m%d_%H%M)
LOG="$BAGDIR/logs_$TS"
ROBOT=wheeltec@10.0.11.2
NS=/slamware_ros_sdk_server_node

mkdir -p "$BAGDIR" "$LOG"
source /opt/ros/noetic/setup.bash
[ -f "$HOME/aurora_ros/devel/setup.bash" ] && source "$HOME/aurora_ros/devel/setup.bash"
export ROS_MASTER_URI=http://10.0.11.2:11311
export ROS_IP=10.0.11.3

say() { echo -e "\n\033[1;36m== $* ==\033[0m"; }
die() { echo -e "\033[1;31m!! $*\033[0m"; exit 1; }
pose_alive() { timeout 3 rostopic echo -n1 $NS/robot_pose >/dev/null 2>&1; }

# ---------------------------------------------------------------- 收尾
AURORA_PID=""
BAG_PID=""
MON_PID=""
cleanup() {
    say "收尾"
    [ -n "$MON_PID" ] && kill "$MON_PID" 2>/dev/null      # 關即時儀表板
    if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
        echo "停止 rosbag（乾淨關檔，避免 .active）…"
        kill -INT "$BAG_PID" 2>/dev/null
        wait "$BAG_PID" 2>/dev/null
    fi
    [ -n "$AURORA_PID" ] && kill -INT "$AURORA_PID" 2>/dev/null
    local bagfile
    bagfile=$(ls -t "$BAGDIR"/test_"$TS"*.bag 2>/dev/null | head -1)
    if [ -n "$bagfile" ]; then
        say "自動分析 $bagfile"
        python3 "$AURORA_DIR/analyze_aurora_bag.py" "$bagfile" || true
        echo "（log 在 $LOG）"
    fi
}
trap cleanup EXIT

# ---------------------------------------------------------------- 重定位（可重入，選單也會用）
do_reloc() {
    say "重定位（沒有這步，位姿就不在地圖座標系！）"
    while true; do
        : > "$LOG/reloc.txt"
        # 先開 watcher 再呼叫 service，避免錯過短暫的 Succeed
        ( timeout 180 rostopic echo $NS/relocalization_status/status \
              | grep -m1 -E "Succeed|Failed|Canceled" > "$LOG/reloc.txt" ) &
        local watcher=$!
        "$AURORA_DIR/aurora_map.sh" reloc || echo "（service 呼叫失敗，等 watcher）"
        echo "等待重定位結果（最多 180 秒，車放特徵多的地方比較快）…"
        wait "$watcher" 2>/dev/null
        local r
        r=$(tr -d ' "' < "$LOG/reloc.txt")
        case "$r" in
            *Succeed*) echo -e "\033[1;32m✓ 重定位成功\033[0m"; return 0 ;;
            *) echo -e "\033[1;31m✗ 重定位未成功（${r:-逾時}）\033[0m"
               read -rp "把車移到特徵豐富處再試一次？Enter=重試 / q=放棄: " a
               [ "$a" = "q" ] && return 1 ;;
        esac
    done
}

# ---------------------------------------------------------------- 出發前檢查
preflight() {   # $1 = plan csv；位姿要在路線 1.5 m 內、朝向差 <90°
    python3 - "$1" <<'PY'
import sys, math, numpy as np, rospy
from geometry_msgs.msg import PoseStamped
wp = np.loadtxt(sys.argv[1], delimiter=",", comments="#", encoding="utf-8")[:, :2]
rospy.init_node("preflight", anonymous=True)
try:
    m = rospy.wait_for_message("/slamware_ros_sdk_server_node/robot_pose",
                               PoseStamped, timeout=5)
except rospy.ROSException:
    print("✗ 5 秒內收不到位姿"); sys.exit(1)
x, y = m.pose.position.x, m.pose.position.y
q = m.pose.orientation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
d = np.linalg.norm(wp - (x, y), axis=1)
i = int(d.argmin()); lat = float(d[i])
j = min(i+3, len(wp)-1)
if j == i: i = max(0, i-3)
pd = math.atan2(wp[j][1]-wp[i][1], wp[j][0]-wp[i][0])
alpha = math.degrees(math.atan2(math.sin(pd-yaw), math.cos(pd-yaw)))
print(f"位姿 ({x:+.2f},{y:+.2f})  yaw {math.degrees(yaw):+.0f}°  "
      f"離路線 {lat:.2f} m  與路線方向差 {alpha:+.0f}°")
if lat <= 1.5 and abs(alpha) <= 90:
    print("✓ 出發前檢查通過"); sys.exit(0)
print("✗ 檢查未通過：車不在路線上、車頭朝錯方向，或重定位其實沒成功（0706 就是這樣）")
sys.exit(1)
PY
}

drive() {       # $1 = plan csv
    if ! preflight "$1"; then
        read -rp "仍要強制出發？輸入 YES（其他=取消）: " f
        [ "$f" = "YES" ] || return
    fi
    echo "（空白鍵=急停/解除，q=結束回選單）"
    python3 "$ROUTE_DIR/ros_move_follow_route.py" --plan "$1"
}

snapshot() {
    echo "-- 位姿 --"
    timeout 3 rostopic echo -n1 $NS/robot_pose/pose/position 2>/dev/null || echo "沒有位姿！"
    echo "-- 位姿頻率（應 ~15Hz）--"
    timeout 5 rostopic hz $NS/robot_pose 2>/dev/null | tail -1
    echo "-- 重定位狀態 --"
    timeout 3 rostopic echo -n1 $NS/relocalization_status/status 2>/dev/null
}

# ================================================================ 主流程
say "0/6 連線檢查"
[ -f "$MAP" ] || die "找不到地圖 $MAP"
[ -d "$PLANS" ] || die "找不到路線資料夾 $PLANS"
# 路線與地圖必須出自同一張 .stcm —— 不同 session 的座標系不通用
PLAN_SRC=$(awk '/^source:/{print $2; exit}' "$PLANS/index.yaml" 2>/dev/null)
PLAN_SRC="${PLAN_SRC##*[\\/]}"          # 有些舊 index.yaml 存的是 Windows 路徑
if [ -n "$PLAN_SRC" ] && [ "$PLAN_SRC" != "$(basename "$MAP")" ]; then
    die "路線 $(basename "$PLANS") 抽自 $PLAN_SRC，但要載的是 $(basename "$MAP")。
   座標系不通用，車會照著別張圖的座標亂開。請讓 MAP= 和 PLANS= 一致。"
fi
ping -c1 -W2 10.0.11.2   >/dev/null || die "連不上機器人 10.0.11.2（WiFi 網卡接了嗎？）"
ping -c1 -W2 192.168.11.1 >/dev/null || die "連不上 Aurora 192.168.11.1（乙太網路線？）"
echo "✓ 地圖 $MAP／機器人／Aurora 都通"

say "1/6 車上底盤"
if rostopic list >/dev/null 2>&1; then
    echo "✓ ROS master 已在線（底盤已啟動，跳過）"
else
    echo "啟動車上 roslaunch（輸入 wheeltec 的密碼）…"
    ssh "$ROBOT" 'bash -ic "nohup roslaunch turn_on_wheeltec_robot mapping.launch >/tmp/base.log 2>&1 & sleep 3; echo 底盤啟動中"' \
        || die "ssh 失敗"
    for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
    rostopic list >/dev/null 2>&1 || die "等不到 ROS master（看車上 /tmp/base.log）"
    echo "✓ ROS master 上線"
fi

say "2/6 Aurora SLAM"
if pose_alive; then
    echo "✓ Aurora 已在發佈位姿（跳過啟動）"
else
    echo "背景啟動 Aurora（手機 App 要先關掉—單一 client！）…"
    roslaunch "$AURORA_DIR/aurora_slam.launch" >"$LOG/aurora.log" 2>&1 &
    AURORA_PID=$!
    echo -n "等待位姿"
    for _ in $(seq 1 40); do pose_alive && break; echo -n .; done; echo
    pose_alive || die "等不到 Aurora 位姿（看 $LOG/aurora.log；App 關了嗎？）"
    echo "✓ Aurora 位姿上線"
fi

say "3/6 rosbag 錄製（背景）"
rosbag record -O "$BAGDIR/test_$TS.bag" \
    $NS/robot_pose $NS/system_status $NS/relocalization_status \
    /cmd_vel /route_plan >"$LOG/rosbag.log" 2>&1 &
BAG_PID=$!
echo "✓ 錄到 $BAGDIR/test_$TS.bag"

say "3.5/6 即時儀表板（唯讀，會彈出視窗）"
# 只訂閱不發 /cmd_vel，可與駕駛節點同時跑。底圖來自 Aurora 即時地圖。
python3 "$ROUTE_DIR/route_monitor.py" --routes "$PLANS" >"$LOG/monitor.log" 2>&1 &
MON_PID=$!
echo "✓ 儀表板已啟動（Tk 視窗；無畫面/出錯會自動退回網頁，URL 見 $LOG/monitor.log）"

say "4/6 載入地圖"
read -rp "車放 A 點、車頭朝路線方向後按 Enter 載圖（這次開機已載過就輸入 s 跳過）: " ans
if [ "$ans" != "s" ]; then
    echo "上傳中（$(du -h "$MAP" | cut -f1)，約 $(( $(stat -c%s "$MAP") / 5000000 )) 秒，期間位姿暫停是正常的）…"
    "$AURORA_DIR/aurora_map.sh" load "$MAP" || die "載圖失敗"
    echo -n "等位姿恢復"
    for _ in $(seq 1 60); do pose_alive && break; echo -n .; done; echo
    pose_alive || die "載圖後位姿沒恢復（0706 後段就是位姿死掉——重開 Aurora 電源再來）"
    echo "✓ 位姿恢復"
fi

# 5/6 —— 上次漏掉的關鍵步驟
do_reloc || die "沒有重定位成功就不能自動駕駛（0706 的教訓）"

say "6/6 行駛選單"
# 選單由 $PLANS/plan_*.csv 自動產生 —— 換地圖/加路線都不必再改這支腳本
mapfile -t PLANFILES < <(ls "$PLANS"/plan_*.csv 2>/dev/null)
[ ${#PLANFILES[@]} -eq 0 ] && die "$PLANS 裡沒有任何 plan_*.csv"
while true; do
    echo
    echo "──────────────────────────────────"
    echo " 地圖 $(basename "$MAP")   路線 $(basename "$PLANS")"
    for i in "${!PLANFILES[@]}"; do
        # 第一行註解就是「路線 A→D, 433.7 m …」
        echo " $((i+1))) $(head -1 "${PLANFILES[$i]}" | sed 's/^# *//; s/ *(x,y,yaw_rad)//')"
    done
    echo " s) 位姿/狀態快照      r) 重新重定位"
    echo " q) 結束（自動關 bag＋跑分析）"
    echo "──────────────────────────────────"
    read -rp "> " c
    case "$c" in
        ''|*[!0-9]*)
            case "$c" in
                s) snapshot ;;
                r) do_reloc ;;
                q) break ;;
                *) echo "?" ;;
            esac ;;
        *)  f="${PLANFILES[$((c-1))]:-}"
            if [ -n "$f" ] && [ "$c" -ge 1 ]; then drive "$f"; else echo "?"; fi ;;
    esac
done
