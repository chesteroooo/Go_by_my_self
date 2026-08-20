# route/ — A/B/C/D 分段路線

四個站點 **A → B → C → D** 的示教路線，**每一段一張獨立的 `.stcm`**，去回程各四段共八段。
路線用 `build_routes.py` 從地圖的 keyframe 軌跡離線抽出，可直接餵 `ros_move_follow_route.py`。

## 八段一覽（實測值，`build_routes.py --check-only` 產生）

| 路線 | 資料夾 | 地圖 | 長度 | waypoint | 最大間距 | 離線模擬橫向誤差（中位／最大） |
|---|---|---|---|---|---|---|
| A→B | `routes_A_B` | `A_B.stcm` | 164.7 m | 281 | 0.88 m | 0.153 / 0.612 m |
| B→C | `routes_B_C` | `B_C.stcm` | 129.2 m | 219 | 0.89 m | 0.155 / 0.602 m |
| C→D | `routes_C_D` | `C_D.stcm` | 151.8 m | 259 | 0.85 m | 0.156 / 0.600 m |
| D→A | `routes_D_A` | `D_A.stcm` | 432.7 m | 682 | 0.98 m | 0.163 / 0.571 m |
| D→C | `routes_D_C` | `D_C.stcm` | 152.5 m | 250 | 0.89 m | 0.158 / 0.508 m |
| C→B | `routes_C_B` | `C_B.stcm` | 131.5 m | 226 | 0.88 m | 0.152 / 0.602 m |
| B→A | `routes_B_A` | `B_A.stcm` | 167.4 m | 276 | 0.91 m | 0.158 / 0.592 m |
| A→D | `routes_A_D` | `A_D.stcm` | 432.7 m | 752 | 1.25 m | 0.148 / 0.623 m |

模擬條件：起點故意偏 0.5 m、航向偏 15°，速度 0.6 m/s（`ros_move_follow_route.py --sim`）。
**A→D / D→A 是「一次開完全程」的整段圖**，和 A→B→C→D 三段走的是同一條路
（三段合計 445.7 m vs 整段 432.7 m，差 3%，來自行車路線與頭尾修剪的差異）。
往返長度都對得上（A↔B 164.7/167.4、B↔C 129.2/131.5、C↔D 151.8/152.5、A↔D 432.7/432.7），
是路段命名正確的交叉驗證。

## ★ 每一段是各自獨立的座標系 —— 跑完一段要換圖重定位

每張 `.stcm` 的**原點就是那一段的起站**（八段的第一個 waypoint 都在自己地圖的 ±0.9 m 內）。
所以 `A_B.stcm` 裡的 B 和 `B_C.stcm` 裡的 B **不是同一組座標**，跑完 A→B 之後：

1. 載入 `B_C.stcm`（約 60–80 秒，期間位姿暫停是正常的）
2. 重定位成功（`aurora/reloc_wait.py`）
3. 才能跑 `plan_B_C.csv`

**不能把三段接起來連續開。** 想一趟開完 A→D，請用整段的 `routes_A_D` + `A_D.stcm`。
（顯示上例外：`routes_site/` 把八段配準到同一座標系，見下一節 —— 那是給人看的整合地圖，
不改變「跑車時位姿仍在該段座標系」這件事。）

分段圖的好處正是**起點比較好抓**：一張 82 MB 的 `A_B.stcm` 只涵蓋 165 m，重定位的候選
keyframe 少、歧義低；`compus5.2.stcm` 661 MB／涵蓋整個校區，同樣站在起點卻要在幾萬個
keyframe 裡比對。起點抓不到時，優先改用分段圖。

## 整合地圖 `routes_site/`（跨路段的統一視圖）

八段各自獨立的座標系不方便看全局，`merge_site_map.py` 把它們配準到同一個框架：

```bash
python3 merge_site_map.py --png        # 產生 routes_site/
```

以 `routes_A_D`（單張圖涵蓋全程）的座標系為骨幹，站點按里程比例落在骨幹上
（A=0、B=160、C=285、D=433 m），各段先兩點剛體對位、再跑站點軟錨定 ICP。
實測殘差中位 **0.34–1.20 m**（整體 0.82 m）。

**兩種直覺作法都會失敗，別再走回頭路：**

- ✗ **純 ICP 形狀匹配** —— 這條路有一段 292 m 自相似的開闊直走廊，一段 129 m 的路線
  沿走廊滑到哪都貼得很好。實測 `B_C` 殘差只有 0.15 m，卻被貼到 A–B 之間、方向還相反。
- ✗ **站點接合**（拿前段終點航向接下段起點航向）—— 各段頭尾都是車停在站點原地擺動的
  區段，單點 yaw 是雜訊。實測算出的 D 偏了 243 m，路徑還折回自己。

產出：

| 檔案 | 內容 |
|---|---|
| `pass_spine.csv` | **去重後的單一道路中心線**。八段跑的是同一條實體路（去程回程各四段全部疊在一起），畫八條只會糊成一團，所以地圖骨架只畫這一條 |
| `leg_<LEG>.csv` | 各段在整合座標系的折線。刻意**不叫** `pass_*` —— 儀表板只把 `pass_*.csv` 當骨架畫，這些留給「高亮目前跑的那一段」 |
| `stations.yaml` | A/B/C/D 的整合座標 |
| `transforms.yaml` | 每段的旋轉/平移＋配準品質，儀表板拿它換算即時位姿 |
| `preview.png` | 靜態預覽（單一中心線＋四個站點） |

### 儀表板：不管跑哪一段都看同一張整合地圖

```bash
python3 route_monitor.py --leg A_B          # 彈出視窗
python3 route_monitor.py --leg A_B --web    # 瀏覽器 :8770（手機/平板）
python3 route_monitor.py                    # 只看整合地圖，不換算位姿
```

`--leg` 用 `transforms.yaml` 把即時位姿從該段座標換算到整合座標，於是車子會畫在整合
地圖的正確位置上，同時顯示航向、走過的軌跡、SLAM/重定位狀態、v/w、離路線多遠、
最近站點、路線進度。`field_test.sh` 與 `autopilot_test.sh` 都已自動帶入正確的 `--leg`。

**精度**：位姿和該段路線套用的是**同一個剛體變換**，距離不變 —— 所以「離路線多遠 /
在不在線上」完全不受配準誤差影響（已驗證誤差 < 0.5 mm）。配準誤差只表現為不同段之間
約 1 m 的視覺錯位。

畫面上只有**一條**道路中心線（`pass_spine.csv`）＋四個站點，目前跑的那一段用不同顏色
疊在上面。

**限制**：整合座標下 Aurora 的即時 OccupancyGrid 底圖會歪掉（它是該段座標系的軸對齊
點陣），所以 `--leg` 模式不畫該底圖，底圖改用道路中心線本身。不加 `--leg` 時底圖照常。

### 離線預覽（不需要 ROS / Aurora / 車）

```bash
python3 route_monitor.py --demo              # 彈出視窗
python3 route_monitor.py --demo --web        # 瀏覽器，SSH 進來看就用這個
python3 route_monitor.py --demo --leg B_C    # 順便高亮某一段
```

不加 `--demo` 而沒有 ROS master 的話，會一直卡在 `Unable to register with master node`
重試，不會有畫面。

### 中文字型（介面中文變方框時看這裡）

有兩個獨立的原因，兩個都修掉了：

1. **Tk 介面把字型寫死成 `Arial`** —— Arial 沒有中文字，Tk 又不像瀏覽器會自動 fallback。
   現在改成開視窗時從系統實際有的字型家族裡挑（`Noto Sans CJK TC` 優先）。
2. **系統一個中文字型都沒有** —— 這時候「挑字型」救不了，得先讓系統有字型。
   挑不到時程式會印出對應這台機器的安裝指令。

怎麼判斷是哪一種：`fc-list :lang=zh | wc -l`，是 0 就是第 2 種。
（實測：沒字型時 Tk 量到每個中文字寬 11 px＝方框；有字型是 19 px＝全形字。）

| 機器 | 狀況 | 做法 |
|---|---|---|
| Jetson | 已有 29 個 Noto CJK | 不用做，實測選到 `Noto Sans CJK TC` |
| WSL | 預設 0 個 | 借用 Windows 的微軟正黑體，**不用 sudo、不用網路**（見下） |
| 一般 Ubuntu | 視安裝而定 | `sudo apt install fonts-noto-cjk` |

WSL 借用 Windows 字型：

```bash
mkdir -p ~/.local/share/fonts
ln -sf /mnt/c/Windows/Fonts/msjh.ttc ~/.local/share/fonts/     # 微軟正黑體
ln -sf /mnt/c/Windows/Fonts/msjhbd.ttc ~/.local/share/fonts/   # 粗體
fc-cache -f
```

要還原就把 `~/.local/share/fonts/msjh*.ttc` 刪掉再 `fc-cache -f`。

網頁版（`--web`）的 CSS 本來就有中文 fallback，不受影響；`preview.png` 的標題刻意用
英文，避免在沒有中文字型的機器上產生方框。

## 重新產生

```bash
# 1) 先把 .stcm 的 keyframe 抽成小 CSV（.stcm 純 Python 解析慢，每張 1–3 分鐘；
#    之後調參數直接吃 CSV，秒級）
cd ~/Go_by_my_self/Detect/new_detect/route
for M in A_B B_C C_D D_A D_C C_B B_A; do
  python3 extract_oneway.py ~/maps/$M.stcm --dump-kf ~/kf/${M}_kf.csv --report > ~/kf/${M}_report.txt
done

# 2) 批次抽路線 + 驗證（自動決定修頭/修尾/抽稀）
python3 build_routes.py --kf-dir ~/kf

# 只重驗現有路線，不重抽
python3 build_routes.py --check-only
```

`build_routes.py` 把 `routes_A_D/README.md` 那套人工流程自動化：

- **整段取用**：這批圖都是單一 session、零跳點、零折返的單向軌跡，整條軌跡就是路線。
  不要用 `extract_oneway.py --half` —— 它的自動折返偵測在這批圖上會退回「離起點最遠點」，
  而迴繞型路線的最遠點在中途，`D_A` 會被砍掉最後 44 m。
- **自動修頭尾**：車在站點停 30–110 秒，SLAM 照樣產生 keyframe，那些點擠在 0.2 m 內卻累積
  里程，pure pursuit 的前視點會塌進去、車在起點原地亂轉。只修「與端點相連」的那一團——
  把半路上的擺動團也修掉會讓路線起點離站點好幾公尺，車停在站點反而過不了出發前檢查。
- **抽稀** `--min-step`：從 0.25 m 起試，過不了前視檢查就自動加大（0.35 / 0.45）。
- **驗證**：最大間距、實際前視距離、離線模擬到不到終點與橫向誤差，全部要過才算數。
- **改寫 `index.yaml` 的 `source:`** 成 `.stcm` 路徑 —— `autopilot_test.sh` / `field_test.sh`
  靠這一行自動配對地圖（只取檔名），指到 kf CSV 會配不到。

## 怎麼跑

```bash
# 一鍵（會自己配對地圖、載圖、重定位、出發前檢查）
bash ~/Go_by_my_self/autopilot_test.sh          # 或 robot_menu.sh 選 4

# 現場完整流程（含 rosbag 錄製 + 即時儀表板）
MAP=~/maps/A_B.stcm PLANS=routes_A_B ./field_test.sh
```

出發前檢查的門檻（`field_test.sh`）：預設離路線 **3.0 m**、方向差 **120°** 以內放行，
可用 `PREFLIGHT_OFF_M` / `PREFLIGHT_YAW_DEG` 覆寫。它會印出「對到路線里程」——
**從起點出發時這個數字應該接近 0，很大就表示重定位把車定到路線別的地方去了。**

## 相關工具

| 檔案 | 用途 |
|---|---|
| `build_routes.py` | 批次抽路線 + 品質驗證（本文件主角） |
| `extract_oneway.py` | 單段抽取／`--report` 診斷／`--dump-kf` 匯出 keyframe |
| `extract_route.py` | 把整張圖切成多條 pass（多 session 的舊圖用） |
| `route_graph.py` | 多 pass 用圖搜尋接成 plan（分段圖用不到） |
| `preflight.py` | 出發前檢查：位姿真的在這條路線上嗎 |
| `ros_move_follow_route.py` | pure pursuit 跟線（`--sim` 可離線模擬） |
| `merge_site_map.py` | 把八段配準成一張整合地圖 `routes_site/` |
| `route_monitor.py` | 唯讀即時儀表板（`--leg` = 整合地圖模式） |
| `../aurora/reloc_wait.py` | 要求重定位並可靠等到結果 |

## 已移除的 campus 路線

`routes_compus1_2` / `routes_compus4` / `routes_compus4_2` / `routes_compus5_2` 已從選單移除。
前三組的 `.stcm` 早就不在 Jetson 的 `~/maps/`，選到只會要求手動挑圖（挑錯就照別張圖的
座標開出去）；`routes_compus5_2` 的地圖還在，是一併移除的。

它們仍在 git 歷史裡，要救回來：

```bash
git checkout HEAD -- Detect/new_detect/route/routes_compus5_2
```
