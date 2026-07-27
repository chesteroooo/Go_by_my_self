# routes_compus5_2 — compus5.2 地圖的路線

由 `~/maps/compus5.2.stcm`（693 MB，3632 keyframes，2026-07-15 建）抽出。
**座標系與 compus4/compus4.2 都不同**（新建圖 session）——plan 不可跨圖沿用。

這張圖是一個「環」：去程沿走廊到最遠端，回程走**南側斜路**（另一條路）回來，
但回程沒開回原點，停在 C (15,-75)。

## 站點（stations.yaml，snap_radius 3.0）
- **A** (-0.2, 0.0)：起點（建圖原點，pass_00 起點）
- **B** (350.8, -2.2)：最遠端折返點（pass_07 終點＝pass_08 起點，示教在此掉頭）
- **C** (15.1, -74.6)：回程示教終點，離 A 還有 ~76 m ——**沒有開回 A**

## 計畫（皆通過離線模擬 @0.6 m/s）
- `plan_A_B.csv` / `.png` — **去程 A→B，507 m**（pass_00→…→07 連續，照建圖方向；
  相鄰 waypoint 最大間距 0.58 m，模擬收斂後橫向誤差最大 0.36 m）
- `plan_B_C.csv` / `.png` — **回程 B→C，515 m**（pass_08 一條到底，走南側斜路；
  最大間距 1.60 m，模擬最大橫向誤差 0.60 m。到 B 先遙控原地掉頭再啟動；
  到 C 之後請遙控回 A）
- `sim_A_B.png` / `sim_B_C.png` — 離線模擬軌跡
- pass_09~12 是原點附近另一 session 的不連通碎段，規劃用不到（留著參考）

## field_test.sh 已接好
預設 `MAP=~/maps/compus5.2.stcm`、`PLANS=routes_compus5_2`，選單 1=A→B、2=B→C。

## 重新產生（若日後重抽/改站點）
```bash
python3 extract_route.py ~/maps/compus5.2.stcm --out routes_compus5_2 --png
python3 route_graph.py plan A B --routes routes_compus5_2 --render
python3 route_graph.py plan B C --routes routes_compus5_2 --render
python3 ros_move_follow_route.py --plan routes_compus5_2/plan_A_B.csv --sim --render sim_A_B.png
```
