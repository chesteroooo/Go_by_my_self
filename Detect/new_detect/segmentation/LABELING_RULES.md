# 標註規則 Labeling Rules — road segmentation (grass / road / sidewalk)

給所有標註者（人工或 AI 輔助皆同）。目標：訓練 YOLOv8-seg 讓自駕車在紅磚自行車道上
保持置中。標註不一致 = 訓練雜訊，**遇到不確定的情況：先問，不要猜。**
For all annotators (manual or AI-assisted). Goal: train YOLOv8-seg so the car can
center itself on the red-brick bike lane. Inconsistent labels = training noise.
**When unsure: ask, don't guess.**

## 類別 Classes — ID 固定，永遠不要改號 (fixed forever, never renumber)

| ID | name | 定義 Definition |
|----|----------|-----------------|
| 0 | `grass`    | 草地與植物（含樹冠、灌木）。Grass ground AND vegetation (tree canopy, bushes). |
| 1 | `road`     | **紅磚車道** — 車要行駛的磚面。The red-brick lane the car drives on. |
| 2 | `sidewalk` | **灰色人行舖面** — 車道旁的灰磚／水泥面（含遮棚下）。Grey paving/concrete beside the lane (incl. under shelters). |

其他一切（天空、建築、人、車、圍籬、樓梯…）＝**不標註（背景）**。
Everything else (sky, buildings, people, vehicles, fences, stairs…) = **unlabeled (background)**.

## 核心規則 Golden rules

1. **陰影不改變類別。** 陰影下的紅磚仍是 `road`，陰影下的灰磚仍是 `sidewalk`。
   Shadow does NOT change the class. Shaded red brick is still `road`.
2. **褪色的紅磚仍是 `road`。** 本場地紅磚常偏灰褐色 — 依「它是車道」判斷，不是依顏色。
   Weathered red brick is still `road` — judge by "it is the lane", not by color.
3. **路面上的油漆標記屬於該路面。** 自行車圖標、白線、箭頭 → 包含在 `road` 內，不要挖掉。
   Painted markings (bike symbols, lines, arrows) belong to the surface — keep inside `road`.
4. **障礙物要從路面挖掉。** 人、機車、腳踏車、狗擋住路面時，把它們從 polygon 中排除
   （障礙物本身不標註 — 由另一個現成 COCO 模型負責）。
   Cut obstacles OUT of surface polygons (people/scooters/bikes/dogs). Do not label
   the obstacles themselves — a stock COCO model handles them.
5. **磚縫小草：小於一塊磚 → 算 `road`；成片草區 → 算 `grass`。**
   Grass tufts in brick joints smaller than one paver → `road`; larger patches → `grass`.
6. **樹冠、灌木 → `grass`（類別 0 = 所有植物）。** 不需要把樹幹細摳出來，合理即可。
   Tree canopy/bushes → class 0 (all vegetation). No need to trace trunks pixel-perfectly.
7. **只標到看得清楚為止。** 遠處（接近消失點）邊界看不清就停，不要用猜的延伸。
   Label only as far as boundaries are clearly visible; near the vanishing point, stop —
   never extrapolate.
8. **樓梯、其他材質廣場、水溝蓋 → 背景。** 只有「車道旁的灰色人行面」才是 `sidewalk`。
   Stairs / other paving types / drain covers → background. `sidewalk` is only the grey
   pedestrian surface adjacent to the lane.

## 精細度優先順序 Precision priority

1. **`road` ↔ `sidewalk` 邊界（尤其陰影下）— 最重要，畫到最準。**
   The road↔sidewalk edge (especially in shade) — highest value, trace it carefully.
   （預訓練模型畫不好這條線，人工修正就是為了它。This is the boundary the pretrained
   model fails at — it's the main reason humans are correcting.）
2. `grass` ↔ `road` 邊界 — 照著可見邊緣畫即可。Follow the visible edge.
3. 樹冠外形、遠景 — 合理即可，不用像素級完美。Rough is fine.

## AI 輔助 / 預標註 AI-assist & pre-labels

- 預標註（SegFormer 自動產生）**只是草稿** — 每張圖都必須人工檢查後才算完成。
  Pre-labels are DRAFTS — every image must be human-reviewed before it counts.
- 預標註常見錯誤 Known pre-label errors:
  - `sidewalk` **常常整個缺失**（被併進 road）→ 需要人工補畫。Often missing entirely — add it.
  - 陰影下的 road/sidewalk 邊界會亂飄 → 重畫。Boundary wanders in shade — redraw.
  - 障礙物挖洞大致正確，但邊緣要檢查。Obstacle cut-outs roughly right; check edges.
- 可以用 SAM／smart-polygon 等工具，但結果仍須符合本文件所有規則。
  SAM/smart-polygon tools are fine; the result must still obey every rule here.

## 流程 Workflow

- 平台 Platform: 同一個 Roboflow 專案（Instance Segmentation）。把本文件貼進
  Roboflow 的 Annotation Instructions。One shared Roboflow project; paste this doc
  into its Annotation Instructions.
- 類別名稱必須完全一致：`grass` / `road` / `sidewalk`（小寫）。Exact lowercase names.
- 太小的區域（< 影像 0.5%）可以略過。Skip specks smaller than ~0.5% of the image.
- 不確定的圖：標記為 unsure／留言，讓大家討論後統一 — **規則有新決議就回來更新本文件。**
  Tag unsure images for discussion; when a new decision is made, UPDATE THIS FILE.
- 困難的圖（黃昏、陰影、逆光）**不要跳過** — 它們是最有價值的訓練樣本。
  Do NOT skip hard images (dusk/shade/backlight) — they are the most valuable samples.
