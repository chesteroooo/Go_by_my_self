# routes_compus4 — compus4 地圖的路線（已建好，可直接測）

由 `~/maps/compus4.stcm`（108 MB，575 keyframes）抽出。這是一條 **~80 m 出去＋回來** 的走廊路線。

## 檔案
- `pass_00/01/02.csv` `index.yaml` `preview.png` — extract_route.py 抽出的示教路段
- `stations.yaml` — 站點：**A**＝原點(0,0)、**B**＝折返點(49.6,-27.6)
- `plan_A_B.csv` / `.png` — **去程 A→B，99 m**（連續、全程照建圖方向 → 視覺重定位有效）
- `plan_B_A.csv` / `.png` — **回程 B→A，103 m**（同上，開回原點）

兩條 plan 都已驗證：相鄰 waypoint 最大間距 < 0.8 m（無跳點），且 0% 逆向行駛。

## field_test.sh 已接好
`field_test.sh` 預設 `MAP=~/maps/compus4.stcm`、`PLANS=routes_compus4`，選單：
1) 去程 A→B（99 m）  2) 回程 B→A（先遙控原地掉頭再選，103 m）

## 重新產生路線（若日後重抽）
```bash
python3 extract_route.py ~/maps/compus4.stcm --out routes_compus4 --png
python3 route_graph.py plan A B --routes routes_compus4 --render
python3 route_graph.py plan B A --routes routes_compus4 --render
```
註：本圖從原點出發的去程示教段有斷點，plan 由 route_graph 的圖搜尋接起；改站點後
若 `plan A B` 找不到路線，是正常的圖連通性問題——沿用現成的 plan_A_B.csv 即可。
