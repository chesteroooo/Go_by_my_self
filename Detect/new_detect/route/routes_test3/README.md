# routes_test3 — 明天建好 test3 地圖後再產生路線

`field_test.sh` 已指向 `~/maps/test3.stcm` 與這個資料夾。地圖與路線是**同一個座標系**
綁定的，所以路線一定要用 **test3 自己的地圖**抽，不能沿用 compus1.2 的（座標對不上，
車會開錯地方）。即時儀表板不受影響——底圖直接來自 Aurora 即時地圖，換 test3 免設定。

## 明天流程（建完 test3.stcm 後，都不需要車在線上，離線就能跑）

```bash
cd Detect/new_detect/route

# 1. 從地圖抽示教路徑（產生 pass_*.csv + index.yaml + preview.png）
python3 extract_route.py ~/maps/test3.stcm --out routes_test3 --png

# 2. 產生站點範本，然後編輯座標（看 preview.png 抓 A/B/… 的位置）
python3 route_graph.py stations-init --routes routes_test3
#   → 編輯 routes_test3/stations.yaml（座標不用準，會吸附到最近路徑點）

# 3. 規劃 plan（drive 選單要用的檔）。站名要跟 field_test.sh 選單一致：
python3 route_graph.py plan A M1 --routes routes_test3 --render
python3 route_graph.py plan M1 A --routes routes_test3 --render
#   （要跑全程再加 plan A B / plan B A）
```

做完 `field_test.sh` 的行駛選單就能用，儀表板也會自動疊上 `routes_test3` 的路線。

> 注意：`field_test.sh` 選單目前寫死 `plan_A_M1 / plan_M1_A / plan_A_B / plan_B_A`。
> 若 test3 的站點命名不同，抽完路線後改選單裡的 plan 檔名（或把站點就命名成 A/M1/B）。
