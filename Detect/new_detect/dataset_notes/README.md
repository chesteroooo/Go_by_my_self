# dataset_notes/ — 已封存資料夾的「資訊殘留」

2026-08-20 清理時，以下資料夾被移到桌面封存（再上雲端），因為它們的**影像全部是
`train_data/` 的逐位元組複本**（md5 全檔比對，8,616 張唯一影像 = train_data 7,628
+ paper_data 988，其餘皆重複）：

`sorted_data/` `cleaned_data/` `label_rest/` `label_500/` `label_Reinforce/`
`autolabel_500/viz/` `autolabel_500/images/`，以及 repo 根目錄的 `data2/ data3/ data4/`

像素可以重來，但**「哪張圖被分到哪一類 / 進了哪個標註批次」是人做的判斷，刪掉就沒了**。
所以那部分留在這裡，每一列都指回還在 repo 裡的 `train_data/` 原檔。

| 檔案 | 內容 |
|---|---|
| `sorted_data_分類.csv` | 4,183 列 —— `gemini_filter.py` 的 clear / minor_issues / bad_data 判定 |
| `cleaned_data_保留清單.csv` | 2,737 列 —— 清洗後留下的是哪些張 |
| `label_500_批次.csv` | 500 列 —— 第一批送標的名單（= `autolabel_500/` 的來源） |
| `label_rest_批次.csv` | 2,237 列 —— 其餘待標名單 |
| `label_Reinforce_批次.csv` | 159 列 —— 補強批次名單 |
| `cleaning_report.txt` | 原 `cleaned_data/` 內的清洗報告 |
| `label_Reinforce_manifest.csv` | 原 `label_Reinforce/` 內的 manifest |

欄位：`archived_path`（在原資料夾裡的相對路徑）、`label`（分類，僅前兩份有）、
`train_data_source`（**還在 repo 裡的對應原檔**）。

要復原任何一個資料夾，照 `train_data_source` 複製即可，不必去雲端拉封存。

> 註：`autolabel_500/` **整個資料夾**（`images/ labels/ viz/ data.yaml`）都已封存到桌面，
> repo 裡不再保留。`labels/` 那 500 個 YOLO 標註是真資產，要重新訓練時從封存取回，
> 並依 `label_500_批次.csv` 確認影像對應關係。
> （`data.yaml` 的 `path:` 停在舊的 Windows 路徑 `C:/Users/user/Desktop/...`，取回後要先修。）
