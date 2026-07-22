# TANet 論文計畫(投稿截止:2026-08-15)

## 題目

**主推:**
> 邊緣視覺語言模型於視覺 SLAM 自走車之場景退化偵測與監督
> —— 以 Qualcomm QCS6490 為目標平台之可行性與資源評估
>
> *Edge Vision-Language Models as Scene-State Supervisors for Visual-SLAM
> Autonomous Vehicles: A Feasibility and Resource Study on the Qualcomm QCS6490*

**備選(較短):**
> 以邊緣 SoC 執行視覺語言模型之自走車場景監督系統設計與評估

## 研究目的

1. **驗證可行性**:輕量 VLM(0.5B–4B、4-bit 量化)能否從單張影像預測視覺 SLAM
   的退化場景(開放自相似磚道走廊)?ground truth 來自真實 SLAM 遙測,非人工標註。
2. **量測目標平台**:在 QCS6490 上量測準確度 × 延遲 × 記憶體 × 功耗,
   並與「車載 x86 PC」「雲端 Gemini API」兩個部署點對照。
3. **資源預算**:建立感知堆疊各元件(ORB 監看、YOLOv8n-seg、VLM)在目標 SoC
   上的資源預算與並行負載表現,以數據評估「QCS6490 取代車載 PC」的可行性。
4. **應用延伸**:展示同一 VLM 模組可延伸為場景監控
   (路徑是否被擋/被什麼擋/是否需通報)——深度相機給不了的語意能力。

## 核心論點

- Aurora S 自帶 SLAM 運算(PC 只收 pose)、車底盤有自己的控制板,
  → 車載 PC 的真實工作量(偵測、任務邏輯、ROS 膠水)在 QCS6490 射程內。
- VLM 不進控制迴路:便宜訊號(ORB 特徵數)常駐監看,異常才觸發 VLM 快照判定
  (數秒延遲可接受)——事件觸發式監督者架構。
- 主張邊界:本文以數據 de-risk 遷移決策;完整遷移與閉迴路漂移改善 = 未來工作。

## 系統架構(目標)

```
Aurora S(自算 SLAM)──Ethernet──┐
                                QCS6490:ORB 常駐監看 → 觸發 VLM 判定
RealSense / 錄影回放 ───────────┘        │rosbridge (websocket)
                                         ▼
                    ROS 系統(/scene_state)──→ 分割輔助定位切換 / 管理者告警
```

## 論文章節(TANet 中文格式,約 6–8 頁)

1. **緒論** — 問題鏈(SLAM 弱區實證 → 缺切換觸發器 → 幾何感測缺語意 → 雲端不適合車載)+ 三貢獻
2. **相關研究** — 視覺 SLAM 退化、edge VLM 部署、VLM 於機器人/監控
3. **系統架構與方法** — 目標平台架構、遙測式標籤法、事件觸發設計、prompt 設計
4. **實驗設計** — 資料集、模型、指標、硬體
5. **結果與討論** — E2–E4 表圖 + E5 demo
6. **結論與未來工作** — NPU 加速、完整遷移、閉迴路、事故資料集

**圖表清單**:系統架構圖|校園路線圖標退化區|樣本影像對照|模型規格表|
準確度表(含 baselines)|裝置效能表|三部署點對照表|資源預算+並行負載表|
**準確度-延遲 tradeoff 散點圖(主圖)**|bag 回放時間軸圖|監控 VQA 範例圖

## 研究步驟

| # | 步驟 | 內容 | 時程 |
|---|---|---|---|
| 0 | 資料盤點 | 確認 rosbag 有影像 topic(record_aurora.sh full);沒有→補錄一趟(半天) | 7/22–7/24 |
| 1 | 資料集建置 | analyze_aurora_bag.py 遙測 → 每幀標「退化/正常」;依路段切測試集,兩類平衡各數百張 | 7/24–7/28 |
| 2 | 平台架設 | QCS6490 裝 llama.cpp;下載 GGUF 模型(4-bit);跑通 1 張圖端到端 | 7/25–7/29(與 1 並行) |
| 3 | E2 準確度 | 各 VLM 同一 prompt 跑測試集;baselines:ORB 特徵數門檻、CLIP zero-shot、Gemini | 7/29–8/3 |
| 4 | E3 裝置效能 | 延遲拆解(encode/prefill/decode)、RAM、功耗(USB-C 電表)、熱衰減;**並行負載測試**(ORB+seg 常駐+VLM 觸發) | 8/1–8/5 |
| 5 | E4 三部署點 | 同 harness 在車載 PC 跑一輪(明確標「現行部署平台,無 GPU」——比的是 perf/watt 不是絕對速度;PC 功耗用智慧插座量);Gemini API 延遲/成本/離線性(角色=審稿人必問的「打 API 不就好」,用數據回答;雲端某些欄贏也照實報,主張是邊緣可行+離線自主) | 8/4–8/6 |
| 6 | E5 demo | rosbridge 回放:bag → QCS6490 → /scene_state;監控 VQA 範例(紙箱擋路等) | 8/6–8/9 |
| 7 | 寫作投稿 | 圖表 → 初稿 → 潤稿 → 投稿 | 8/8–8/15 |

## 需要的東西

**硬體**
- QCS6490 開發板(型號待確認:RB3 Gen 2 / Rubik Pi 3 / Thundercomm)+ 散熱
- USB-C inline 電表(量功耗,約 NT$300–800)
- Ubuntu PC(bag 回放、車載 PC 對照組)、Windows PC(分析/寫作)
- 車 + Aurora:僅步驟 0 需補錄時使用
- (選配)支援 12V 的 PD 行動電源——僅車載彩蛋 demo 用

**軟體 / 模型**
- llama.cpp(GGUF + mtmd 多模態)
- 模型:SmolVLM2-500M/2.2B、Moondream2、Qwen3-VL-2B、LFM2-VL-1.6B;(選配)Gemma3-4B 當準確度上限
- OpenCV(ORB baseline + 常駐監看)、ultralytics YOLOv8n-seg(預算列)
- open_clip(CLIP zero-shot baseline)
- rosbridge_server(Ubuntu PC)+ roslibpy(QCS6490)
- 既有:analyze_aurora_bag.py、record_aurora.sh、gemini_filter.py(改接測試集)
- Gemini API key、matplotlib/pandas

**資料**
- 既有 rosbag(compus 路線)、train_data 810+ 張、compus1.2/compus2 地圖分析結果

**其他**
- TANet 論文格式模板、投稿系統帳號

## 風險與對策

| 風險 | 對策 |
|---|---|
| bag 沒錄影像 | 步驟 0 立刻確認;補錄只要半天 |
| 某模型 llama.cpp 跑不起來 | 清單有 5 個,取 3–4 個成功者即可 |
| VLM 準確度不佳 | 仍是有效結果(誠實報告 + tradeoff 分析);CLIP baseline 兜底 |
| 時程崩 | 砍序:E5 監控 demo → 並行負載 → CLIP baseline;E2+E3 是底線 |
| 場域 5G 專網不通外網 | 架設期改用 WiFi/熱點抓模型,或 PC 下載後 scp 進板;PC 用雙網卡+路由分工(專網不設 default gateway)。**此事實寫入動機與 E4:部署網路內雲端 VLM 不可用 → 邊緣必要性的實證** |
