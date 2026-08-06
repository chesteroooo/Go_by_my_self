---
name: tanet-notion-pages
description: "Notion page IDs for the TANET 2026 paper plan and its field-shooting manual subpage, synced from paper/*.md"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 98674eea-ae42-4a37-9519-c20a4ae5773e
  modified: 2026-08-04T14:39:58.063Z
---

TANET 2026 論文計畫 的 Notion 頁面(Notion MCP 已設定,connector 名 `notion`):

- 主頁「TANET 2026 論文計畫 — VLM 於自走車開放詞彙障礙處置之應用(主題 1)」
  `3ab68331-f87a-8159-8f50-ea979f3a27b7`
  ← 同步自 `paper/PLAN.md`
- 子頁「拍攝作業手冊(現場用)」
  `3ac68331-f87a-81c8-87f3-fa48cc37b2cd`
  ← 同步自 `paper/拍攝作業手冊.md`
- 子頁「三級處置標註規則(標註用)」
  `3b068331-f87a-8107-8c18-c7b15988edec`(2026-08-01 建立)
  ← 同步自 `paper/三級處置標註規則.md`
- 子頁「完成論文的步驟與所需技術(重點版)」
  `3b268331-f87a-8123-9513-fb6e023c4b0c`(2026-08-04 建立)
  ← **Notion 原生,repo 沒有對應檔案**;內容是步驟/技術/為什麼的摘要,細節仍以 PLAN.md 為準

同步方向:**repo 是真相來源**,Notion 是副本。全頁重建時必須在主頁內容裡保留
**每一個子頁**的 `<page url="...">` 區塊(目前有 **3 個**子頁),否則 `replace_content` 會判定
子頁要被刪除(`allow_deleting_content` 未開時會直接報錯)。

張數/距離等數字若三方不一致,以 `Detect/new_detect/segmentation/capture_paper_dataset.py`
的 `PRESETS` / `DISTANCES` 為最終依據。
