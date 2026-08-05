# 換機接手指南 — 沒有 Jetson 也能寫完這篇論文

> 建立於 2026-08-06。**Jetson 之後不再可連**，所有現場資料已完整取回並逐檔驗證。
> 這頁講：接手的電腦要準備什麼、剩下的工作有哪些、每一項為什麼不需要 Jetson。

---

## 0. 一句話結論

**剩下的工作全部是離線分析與寫作，沒有一項需要 Jetson 或車子。**
唯一會失去的是「閉迴路實車示範錄影」（原 T3），那是加分項不是主結果 —— 處理方式見 §4。

---

## 1. 新電腦要準備什麼

### 1-A 取得程式碼與文件

```bash
git clone https://github.com/chesteroooo/Go_by_my_self.git
cd Go_by_my_self
```

這包含：所有分析腳本、`paper/` 全部文件、每個 session 的 `metadata.csv` 與 `intrinsics.json`、
以及 `.claude/memory/`（跨對話的專案記憶，Claude Code 會讀）。

### 1-B 取得影像（**不在 git 裡，要另外傳**）

`.gitignore` 排除了 `color/ depth/ depth_vis/ seg_vis/`（161 MB、988 個檔）。
用隨身碟或雲端把整個 `Detect/new_detect/paper_data/` 複製過去，放在同樣的相對路徑。

> **影像裡有 136 張含人像**（協助者）。傳輸與存放請避免公開位置；
> 論文用圖若出現人臉需打碼並載明取得同意（見標註規則 §5）。

驗證有沒有傳完整：

```bash
find Detect/new_detect/paper_data -type f | wc -l     # 應為 1005
python3 paper/b1_geometry.py                          # 能跑完就代表深度與內參都在
```

### 1-C 模型檔（`Detect/new_detect/models/`，也被 gitignore 擋掉，要一起傳）

| 檔案 | 用途 | md5 |
|---|---|---|
| `best_paper.pt` | **分析用**的路面分割模型（= 舊的 `best3.pt`）。走廊寬度實測、圖 2 動機示例都用這顆 | `21aab7c9…` |
| `best_field_jetson.pt` | **現場拍攝當下** Jetson 上跑的那顆，`seg_vis/` 是它產生的 | `250b677f…` |
| `yolov8n.pt` | B2 的 COCO 偵測器（也可讓 ultralytics 自動下載） | — |

> ⚠️ **兩顆 `best.pt` 不是同一個模型。** 資料集裡的 `seg_vis/` 來自 `best_field_jetson.pt`；
> 之後若要重跑分割分析，**明確指定要用哪一顆並在論文寫清楚**，不要混用。

### 1-D Python 環境

```bash
pip3 install numpy opencv-python ultralytics pandas matplotlib scikit-learn
# B3 需要：pip3 install google-generativeai   （另需 GEMINI_API_KEY）
```

**不需要** `pyrealsense2`、ROS、`rospy` —— 那些只有拍攝與車控才用得到。

---

## 2. 手上的資料（已逐檔 md5 驗證，與 Jetson 完全一致）

| Session | 張數 | 內容 |
|---|---|---|
| `session_19700102_062831` | 90 | **B 地點主力**：12 物件 × 6 + 兩組空景。時間戳為 1970（當時 RTC 掉了），seq 順序正確 |
| `session_20260805_144645` | 57 | **B 地點第二批**：P02 三態 + 空景 + NEG 32 |
| `session_20260803_133904` | 100 | A 地點補充樣本（**無空景**，`empty_ref` 為空，不進主表）|
| | **247** | |

**主力 = 前兩個 session 共 147 張**，論文主結果用這批。

### 逐物件

```
P01  box 9 / person-crouch 16        P02  box-intact 12 / box-flattened 6 / box-covering 6
P03  bag-empty 6 / bag-full 6        P05  toilet-paper 15 / person-stand 18
P06  road-marking(人孔蓋) 15 / cable 6    X-L2  bowl-shards 21
COCO bicycle 6 / umbrella 24 / backpack 6
EMPTY 18    NEG road 31 / grass 10 / sidewalk 16
```

> **P02 是同一個紙箱的三種狀態**：完整 L1、壓扁 L0、**撐起蓋住東西 L2**。
> 同一類別標籤橫跨三個等級 —— 這是全篇最強的單一素材。

---

## 3. 剩下的工作，以及為什麼都不需要 Jetson

| # | 工作 | 用什麼 | 為什麼不需要 Jetson |
|---|---|---|---|
| 1 | **標註 + kappa** | `paper/make_label_sheets.py` | 讀 `color/` 與 CSV，純離線 |
| 2 | **B1 幾何基線** | `paper/b1_geometry.py`（已完成並跑通） | 讀 `depth/` + `intrinsics.json`，**每張自行擬合地面平面**，不需要相機外參也不需要相機 |
| 3 | **B2 COCO 基線** | `ultralytics` + `yolov8n.pt` | 對 `color/` 離線推論 |
| 4 | **B3 VLM** | Gemini API | 對 `color/` 呼叫雲端 API，只需要外網 |
| 5 | **E2 主表 / E4 觸發率 / E5 定性** | pandas + matplotlib | 全部是對上面產出的表格做統計 |
| 6 | **寫作與排版** | `paper/TANET_論文格式.docx` | — |

### B2 實作的兩個必要細節（已驗證，不照做數字會錯）

1. **必須用走廊 + 深度過濾偵測框。** 走道盡頭停著一台真的廂型車，COCO 對**每一張**（含空景）
   都會輸出 `car` 0.4–0.6。不過濾的話 B2 的誤觸發率是 100%。
   走廊寬度用 **`W_CORRIDOR = 1.80 m`**（實測可行駛路面寬，見標註規則 A.2）。
2. **半身影像仍能偵測到人。** 13° 俯角下站立的人只拍到大腿以下，`yolov8n` 仍輸出
   `person` 0.85 —— 所以 B2 的失敗是原理性的，不是構圖造成的。這句可以直接寫進論文。

---

## 4. 失去 Jetson 會少掉什麼（以及怎麼處理）

| 原規劃 | 狀態 | 處理 |
|---|---|---|
| T3 閉迴路實車示範錄影（觸發 → VLM 回 L1 → 實際繞行）| **做不成** | 主結果不受影響。論文改為「處置等級對應到既有的繞障行為」的**設計說明**，不宣稱做過閉迴路示範 |
| 補拍任何影像 | **做不成** | 資料集已完成 247 張，不需要補 |
| `ros_test_bypass.py`（L1 的執行器）| 已從 repo 刪除 | 仍在 git 歷史裡：`git show <刪除前的commit>:Detect/new_detect/ros_test_bypass.py`。**PLAN.md 有 4 處引用它，撰稿時要改寫**（見下） |

### 撰稿時必須改寫的四處

- `PLAN.md:77`、`標註規則 A.1` — L1 的動作定義引用 `ros_test_bypass.py`
- `PLAN.md:296` — 「B1 不是稻草人，是本車現行的避障邏輯」← **防守審稿人的關鍵論證，要保留但改寫措辭**
- `PLAN.md:330` — 「為什麼不用光達」的論證引用它
- `PLAN.md:532` / `拍攝作業手冊 T3` — 閉迴路示範

---

## 5. 已知的資料瑕疵（論文要照實寫）

| 項目 | 說明 |
|---|---|
| **遺失 5 張 NEG road** | 2026-08-05 15:27 拍攝的 seq 56–60，在一次 `rsync --delete` 同步中被覆蓋。seq 61 已從 Jetson 救回並補上 CSV 紀錄。NEG 仍有 57 張，多樣性已驗證（兩兩差異無近似重複），不影響誤觸發率的計算 |
| **A 地點 100 張沒有空景** | `empty_ref` 為空，不能用於觸發層的差分分析；僅作補充樣本 |
| **三批資料的相機外參不同** | 08-03 `14.8°/0.544`、08-04 `13.03°/0.565`、08-05 `13.41°/0.558`。**不影響 B1**（每張自行擬合平面）。同一個配對的兩半都在同一批內，未跨批拆開 |
| **Jetson 時鐘** | 08-04 那批的時間戳是 1970（RTC 掉電）。`seq` 與檔名順序正確，僅 timestamp 欄不可用 |
| **人孔蓋在 CSV 裡叫 `road-marking`** | 現場沿用了舊的 preset 名稱。分析時用 `(object, location)` 或 session 區分 |
| **未見組的 L2 有 3 個** | 延長線、碎陶片、蓋住的紙板 |

---

## 6. 給接手的 Claude Code

`CLAUDE.md` 與 `.claude/memory/` 已隨 repo 一起帶過去。開始工作前請先讀：

1. `paper/PLAN.md` — 論述架構、實驗設計、投稿格式（**單一真相來源**）
2. `paper/三級處置標註規則.md` — ground truth 的操作型定義（**標註開始後即凍結**）
3. 本頁 §3 與 §5

**不要**再嘗試連 `10.0.11.2`，那台已經不在。
