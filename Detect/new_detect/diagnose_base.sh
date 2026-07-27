#!/usr/bin/env bash
# =============================================================================
# diagnose_base.sh — 進水後「連得上、但不會動」的離線診斷
#
#   用法：插上 5G WiFi 網卡、連到機器人 AP 後，本機執行：
#       ./diagnose_base.sh          # 只讀診斷，不會讓車移動
#       ./diagnose_base.sh --move   # 最後多做「輪子離地」的微動測試（會問你確認）
#
# 判讀（看輸出）：
#   /odom 沒更新 或 底盤節點不在  → STM32/序列埠是進水受害者（USB-serial 濕了、
#       或 STM32 板受損）。先查機器人上 /dev/ttyUSB* 或 /dev/wheeltec 有沒有出現。
#   /odom 有更新 + 電壓正常，但下 cmd_vel 輪子不轉 → 馬達驅動板/馬達電源受損，
#       或 e-stop 沒解除。查實體急停鈕、馬達電源開關/保險絲。
#   電壓偏低 → 先充電/換電池，低壓會讓馬達停動。
# =============================================================================
set -u
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://10.0.11.2:11311
export ROS_IP=10.0.11.3
ROBOT=wheeltec@10.0.11.2

hr() { echo -e "\n\033[1;36m== $* ==\033[0m"; }

hr "0. 網路：連得到機器人嗎"
if ping -c1 -W2 10.0.11.2 >/dev/null 2>&1; then
    echo "✓ ping 10.0.11.2 通"
else
    echo "✗ ping 不到 10.0.11.2 —— 5G 網卡插好了？連到機器人 AP 了？車開機了？"
    exit 1
fi

hr "1. ROS master / 話題"
if timeout 5 rostopic list >/tmp/topics.txt 2>&1; then
    echo "✓ master 在線，話題數：$(wc -l </tmp/topics.txt)"
else
    echo "✗ 連不到 ROS master —— 車上 roslaunch 沒起來（ssh 進去看 /tmp/base.log）"
    exit 1
fi

hr "2. 底盤驅動節點在不在"
timeout 5 rosnode list 2>/dev/null | grep -iE "wheeltec|base|serial|robot" \
    || echo "✗ 找不到底盤節點 —— mapping.launch 的底盤 driver 掛了（多半是序列埠斷）"

hr "3. /odom 有沒有在更新（序列埠/STM32 是否活著）"
echo "應該有頻率（~20-50Hz）；卡住不動 = STM32 序列鏈路斷了"
timeout 6 rostopic hz /odom 2>/dev/null | tail -2 || echo "✗ /odom 沒資料 → 序列埠/STM32 是受害者"

hr "4. 電池電壓（低壓會禁止馬達）"
for t in /PowerVoltage /power_voltage /voltage /battery /Power; do
    grep -qx "$t" /tmp/topics.txt && { echo "讀 $t："; timeout 3 rostopic echo -n1 "$t" 2>/dev/null; }
done
grep -qiE "voltage|power|battery" /tmp/topics.txt || echo "(沒有電壓話題，改用萬用表量電池)"

hr "5. 誰在發 /cmd_vel（有沒有殘留控制器/急停在灌 0）"
timeout 4 rostopic info /cmd_vel 2>/dev/null || echo "(沒有 /cmd_vel 話題)"

hr "6.（可選）機器人端序列裝置 —— 需輸入 wheeltec 密碼"
read -rp "要 ssh 進車看 /dev 序列裝置嗎？(y/Enter 跳過): " a
if [ "$a" = "y" ]; then
    ssh "$ROBOT" 'ls -l /dev/ttyUSB* /dev/ttyACM* /dev/wheeltec* 2>/dev/null || echo "沒有序列裝置 → USB-serial 沒被系統認到（進水受害者）"'
fi

# ---------------------------------------------------------------- 微動測試
if [ "${1:-}" = "--move" ]; then
    hr "7. 微動測試 —— ⚠ 先把車架高、輪子離地！"
    read -rp "輪子已離地、周圍淨空了嗎？輸入 YES 才動: " ok
    if [ "$ok" = "YES" ]; then
        echo "發 0.05 m/s 前進 1.5 秒，看每個輪子…"
        timeout 1.5 rostopic pub -r10 /cmd_vel geometry_msgs/Twist \
            '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' 2>/dev/null
        rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}' >/dev/null 2>&1   # 確保停
        echo "→ 全部不轉：馬達電源/驅動板受損或 e-stop。"
        echo "→ 只有某輪不轉：那顆馬達/驅動通道是進水受害者。"
        echo "→ 都會轉：馬達沒事，問題在別處（先前哪一步卡住就往那查）。"
    else
        echo "已取消微動測試。"
    fi
fi

hr "診斷結束"
