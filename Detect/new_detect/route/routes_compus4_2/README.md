# routes_compus4_2 — compus4.2 地圖的路線

由 `~/maps/compus4.2.stcm`（541 MB，2923 keyframes，2026-07-15 建）抽出。
比 compus4 大很多：主走廊直線一路開到 x≈291（單程 ~395 m）。
**座標系與 compus4 不同**（新建圖 session）——不可沿用 routes_compus4 的 plan。

## 站點（stations.yaml，snap_radius 3.0）
- **A** (2.3, 6.0)：起點，連通鏈開頭（pass_01 起點，靠近建圖原點）
- **B** (290.8, -0.5)：最遠端折返點（pass_03 終點＝pass_04 起點，示教在此掉頭）
- **C** (147.9, -22.8)：回程示教終點 ——**示教沒有開回 A**，所以沒有全程回程

## 計畫（皆通過離線模擬 @0.6 m/s，收斂後橫向誤差 <0.8 m）
- `plan_A_B.csv` / `.png` — **去程 A→B，585 m**（pass_01→02→03，全程照建圖方向）
  - 注意 pass_02 含建圖時的繞行/擺動段（~168 m），車會照著重走
  - pass_03 直線段 keyframe 較疏，相鄰 waypoint 最大間距 2.76 m（先天如此，follower 可跨）
- `plan_B_C.csv` / `.png` — **回程 B→C，189 m**（pass_04→05→06；到 B 先遙控原地掉頭再啟動）
  - 到 C 之後沒有示教路徑，剩下請遙控
- `sim_A_B.png` / `sim_B_C.png` — 離線模擬軌跡

## field_test.sh 已接好
預設 `MAP=~/maps/compus4.2.stcm`、`PLANS=routes_compus4_2`，選單 1=A→B、2=B→C。

## 重新產生（若日後重抽/改站點）
```bash
python3 extract_route.py ~/maps/compus4.2.stcm --out routes_compus4_2 --png
python3 route_graph.py plan A B --routes routes_compus4_2 --render
python3 route_graph.py plan B C --routes routes_compus4_2 --render
python3 ros_move_follow_route.py --plan routes_compus4_2/plan_A_B.csv --sim --render sim_A_B.png
```
註：route_graph 已修正「兩站吸附同一路徑點會斷路」的 bug（B 折返點需要它），
且接續點預設只吸附自己的 pass（避免跨圖層抄捷徑產生 >1.5 m 跳點）；
舊圖（如 routes_compus4）若需要跨層縫合可加 `--loose-junctions`。
