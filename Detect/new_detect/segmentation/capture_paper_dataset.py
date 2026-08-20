#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
論文資料擷取工具 (Paper Dataset Capture) — TANET 2026

一次按鍵 = 抓齊一筆完整樣本：
  1. 彩色原始幀 (PNG 無損，640x480，與 best.pt 推論解析度一致)
  2. 對齊到彩色的深度幀 (16-bit PNG，保留原始 mm 值) —— B1/B2 深度基線要用
  3. 深度可視化圖 (JPG，只給人檢查用)
  4. metadata 一列寫進 CSV
  5. (建議一律加 --seg) best.pt 分割疊圖 —— 現場就能看到它把紙箱標成 road
     注意：分割是在主迴圈裡逐幀同步跑的（非 worker thread），所以預覽會降到推論速度；
     車靜止拍照的情境下無所謂。seg_vis/ 事後可用 color/ 原圖重跑補出來，
     真正補不回來的是「現場看到失效、當場決定多拍幾個角度」這個機會。

相機內參與 depth scale 存成 intrinsics.json，之後把深度換算成公尺/高度時要用。

為什麼一定要用這支而不是 collect_floor_dataset.py：
  - 那支只存 Color，沒有深度 → B1/B2 基線沒資料可跑
  - 那支沒有 metadata → 沒有 ground truth 就算不出準確率
  - 那支存 jpg 有壓縮假影，可能影響分割結果

現場操作（不用打字，全部按鍵切換）：
  n / b   下一個 / 上一個 預設物件（P01/P02/P03/P05/P06/P07 配對 + COCO 組 + 空景/負樣本）
  1 2 3   距離 1m / 2m / 3m（5m 檔已停用：戶外深度品質不可靠）
  c / v   橫向位置 center（正對車道中線）/ left（偏左，測行駛走廊邊緣）
  g       切換光線條件 (sunny/cloudy/backlit/shade)
  SPACE   擷取一筆
  u       復原（刪掉上一筆）
  q       離開

用法：
  python3 capture_paper_dataset.py --location A
  python3 capture_paper_dataset.py --location A --seg      # 加上即時分割疊圖
"""

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    Image = None   # 沒有 Pillow 就退回英文顯示

# ================= 參數設定 =================
COLOR_W, COLOR_H = 640, 480      # 與 ros_detect_dual.py 的 YOLO 輸入一致
DEPTH_W, DEPTH_H = 640, 480
FPS = 30
FRAME_TIMEOUT_MS = 5000

# 輸出根目錄：<repo>/paper/paper_data/（2026-08-20 由 new_detect/ 移進 paper/）
OUT_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "paper", "paper_data"))

# best.pt 位置（--seg 才會載入）
BEST_PT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "best.pt")

# 預設物件清單 —— 對應拍攝作業手冊的 P01–P07 配對
# (pair_id, object, expected_level, is_seen_class, 中文顯示名)
# 注意：object 欄位固定用英文，因為它會進檔名與 CSV；中文只用於畫面顯示。
#
# is_seen_class 的定義（2026-07-30 改）：指「是否在 COCO 80 類內」，
# 亦即 baseline B2（COCO 偵測器 + 類別→等級查表）認不認得。
# 舊定義是「best.pt 的訓練類別」，但 best.pt 已不是 baseline，該定義已作廢。
PRESETS = [
    ("P01", "box",            "L1", False, "紙箱"),
    ("P01", "person-crouch",  "L2", True,  "蹲著的人"),
    # P02 替代方案（2026-07-30）：原為「乾路面 L0 vs 施工濕水泥 L2」，濕水泥現場取得不到。
    # 改用同一個紙箱的兩種狀態 —— 類別完全相同、處置等級相反，狀態論證比原版更乾淨，
    # 且零額外材料。若日後真的遇到施工區，再機會性補拍 wet-cement。
    ("P02", "box-intact",     "L1", False, "完整紙箱（必須繞）"),
    ("P02", "box-flattened",  "L0", False, "壓扁攤平的紙箱（可壓過）"),
    # P03 替代方案（2026-07-31）：原為「淺水痕 L0 vs 深積水 L2」，現場無法製造深積水。
    # 改用同一個塑膠袋的兩種狀態 —— 空的可壓過、裝滿的必須繞，類別與標籤完全相同，
    # 差別只在狀態。這正是「核心主張」自己舉的例子（袋子空的還是裝滿的）。
    # 注意：此替代讓 P03 不再貢獻未見 L2（見 P06 的說明）。
    ("P03", "bag-empty",      "L0", False, "空塑膠袋（可壓過）"),
    ("P03", "bag-full",       "L1", False, "裝滿的塑膠袋（必須繞）"),
    # P04（塑膠袋/磚塊）已於 2026-07-30 移除 —— 現場取得不到，且它只是補涵蓋度，
    # 不承擔任何論證角色，砍掉不影響主結果。
    # P05 的 A 側改過兩次：三角錐 → 垃圾桶 → **衛生紙 6 組裝**（2026-08-01，現場兩者都沒有）。
    # 這一側只要滿足：未見類別（不在 COCO）、L1、站得住。**高度不必跟人相近**——
    # B1 的失敗不是因為高度巧合，而是因為它的輸出空間裡根本沒有 L2（高度門檻只能回答
    # 「有沒有東西擋路」，永遠推不出「要停車通報」）。幾何相近只是讓圖表好看，不是邏輯必要條件。
    #
    # 衛生紙尺寸 30×40×10 cm。**L1 的理由是「高度超過 H_PASS」，不是「硬」** ——
    # 它很軟，但決策樹 Q4 的「能輾過」要求「夠矮 **且** 軟」兩個都成立，第一個就掛了。
    # ⚠ 標註時容易被直覺帶偏成 L0（「衛生紙輾過去沒差」），務必照決策樹走。
    # ⚠ 包裝不可拆（拆開變多個物件）；6 張的擺法（躺平 10cm / 立起 30cm）必須全程一致。
    # 其他可換的（都不在 COCO）：cone 三角錐、broom 立著的掃把、sign-board A字告示牌、bucket 水桶。
    ("P05", "toilet-paper",   "L1", False, "衛生紙6組裝"),
    ("P05", "person-stand",   "L2", True,  "站立的人"),
    # P06（2026-07-31 再改）：「落葉堆/碎玻璃」→「地面標線/橫拉警戒帶」→ 現改為延長線／繩索，
    # 因為現場沒有警戒帶。三者性質相同：平貼地面、不在 COCO、等級相反。
    #
    # ⚠ P06 現在是「未見組唯一的 L2」（深積水與警戒帶皆已無法取得）。
    #   B1 靠高度分級，而高度永遠推不出 L2，所以 B1 的錯誤集合恆等於「所有 L2 物件」。
    #   未見組少了它，B1 會在未見組全對，4.1 主表的未見組就沒有任何一格是幾何答錯的。
    #   → 這一對絕不能砍，且 E4-1 要單獨報它的觸發結果。
    ("P06", "road-marking",   "L0", False, "地面標線／影子"),
    ("P06", "cable",          "L2", False, "橫過路面的延長線／繩索"),
    # P07（垃圾桶/蹲伏的狗）已於 2026-07-31 移除 —— 現場取得不到狗。
    # 它的角色（證明有給對照組公平機會）已由 P01、P05 承擔，論述不因此有洞。
    #
    # 第二個未見 L2（2026-07-31 二次啟用）—— 破碎的碗/碎陶片，是原「碎玻璃」的安全替代。
    # 為什麼重要：B1 靠高度分級，高度永遠推不出 L2，所以 B1 的錯誤集合恆等於「所有 L2 物件」。
    # 未見組每多一個 L2，4.1 主表在未見組的差距就多一格。只有 cable 一項時差距僅一格，
    # 統計上沒有重量；加上 bowl-shards 變成兩格。
    ("X-L2", "bowl-shards",   "L2", False, "破碎的碗／碎陶片"),
    # 第三個未見 L2（2026-08-05 啟用，改為必拍）—— 布蓋住的不明物體。
    # 為什麼從「加分」升成必拍：標註規則 Q3-4 的判準是「不確定的部分會不會改變處置」，
    # 而現有 13 個物件**全都是人一眼認得出的**，這條分支一個樣本都沒有 ——
    # 論文只能寫「規則有定義」，不能寫「驗證過」。
    # 它同時是 E5 可解釋性最好的素材：VLM 能輸出「看不出布底下是什麼，建議停車通報」
    # 這種文字理由，而 B1/B2 在定義上寫不出這句話。
    # ⚠ 布要蓋到**看不出底下是什麼**（蓋一半、看得出是紙箱就沒有意義了）。
    ("X-L2", "covered-object", "L2", False, "布蓋住的不明物體"),
    # 其他候選的未見 L2（機會性補拍，現場看到再啟用）：
    # ("X-L2", "site-barrier",   "L2", False, "施工圍籬"),
    #
    # COCO 涵蓋的額外障礙物 —— 平衡「已見組」樣本數（2026-07-30 新增）。
    # 挑選條件：(a) 在 COCO 80 類內、(b) 搬得動（要擺到車前 1/2/3 m，固定式的 bench 不合用）。
    # chair → potted plant → umbrella（2026-07-31 二次）：現場既沒有可搬動的椅子也沒有盆栽。
    # 其他可替換的 COCO 選項（都搬得動）：suitcase 行李箱、sports ball 球、handbag 手提包。
    # 不建議 bottle 水瓶 —— 3 m 處太小，偵測失敗會變成尺寸問題而非語意問題，論證會被模糊。
    ("COCO", "bicycle",       "L1", True,  "停放腳踏車"),
    ("COCO", "umbrella",      "L1", True,  "雨傘"),
    ("COCO", "backpack",      "L1", True,  "背包"),
    # 無障礙負樣本 —— E4-1 誤觸發率的分母，兼測「無障礙時不該誤報」。
    ("EMPTY", "none",         "L0", False, "空景對照"),
    ("NEG",  "road",          "L0", False, "無障礙負樣本：路面"),
    ("NEG",  "grass",         "L0", False, "無障礙負樣本：草地"),
    ("NEG",  "sidewalk",      "L0", False, "無障礙負樣本：人行道"),
]

LEVEL_ZH = {"L0": "L0 可通行", "L1": "L1 可繞行", "L2": "L2 停車通報"}

# 5m 檔已移除（2026-07-30）：戶外強光下 D435i 在 5m 的深度大面積破洞，資料進不了 B1/B2。
DISTANCES = {ord("1"): 1, ord("2"): 2, ord("3"): 3}
LIGHTING = ["sunny", "cloudy", "backlit", "shade"]
LIGHTING_ZH = {"sunny": "晴", "cloudy": "陰", "backlit": "逆光", "shade": "樹蔭"}

# 中文字型（畫面顯示用；cv2.putText 不支援中文，必須經由 PIL）
CJK_FONT_PATHS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
]

CSV_FIELDS = [
    "seq", "filename_color", "filename_depth", "pair_id", "object",
    "expected_level", "distance_m", "lateral", "is_seen_class",
    "location", "lighting", "timestamp", "empty_ref",
    "depth_median_mm", "notes",
]
# ===========================================


def parse_args():
    ap = argparse.ArgumentParser(description="TANET 2026 論文資料擷取（彩色 + 對齊深度 + metadata）")
    ap.add_argument("--location", default="A", help="拍攝地點代號，寫進 metadata")
    ap.add_argument("--name", default="", help="session 資料夾備註名稱")
    ap.add_argument("--seg", action="store_true",
                    help="載入 best.pt 做即時分割疊圖（現場驗證無聲失效用，會吃記憶體）")
    ap.add_argument("--model", default=BEST_PT, help="分割模型路徑")
    return ap.parse_args()


def load_cjk_font(size=18):
    """找一個能顯示中文的字型；找不到就回 None，畫面退回英文。"""
    if Image is None:
        return None
    for p in CJK_FONT_PATHS:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                continue
    return None


def draw_hud(img_bgr, lines, fonts):
    """把 HUD 文字畫上去。有中文字型就用 PIL，沒有就退回 cv2(僅英文)。

    lines: [(文字, (B,G,R), small_bool), ...]
    fonts: (一般字型, 小字型) 或 None
    """
    if fonts is None:
        for i, (txt, col, _s) in enumerate(lines):
            cv2.putText(img_bgr, txt, (10, 25 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        return img_bgr

    font, font_s = fonts
    heights = [20 if s else 26 for _t, _c, s in lines]
    bar_h = 10 + sum(heights)

    # 先鋪半透明黑底，戶外陽光下才看得清楚
    overlay = img_bgr.copy()
    cv2.rectangle(overlay, (0, 0), (img_bgr.shape[1], bar_h), (0, 0, 0), -1)
    img_bgr = cv2.addWeighted(overlay, 0.55, img_bgr, 0.45, 0)

    pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
    drw = ImageDraw.Draw(pil)
    y = 5
    for (txt, col, small) in lines:
        drw.text((8, y), txt, font=(font_s if small else font),
                 fill=(col[2], col[1], col[0]))   # BGR -> RGB
        y += 20 if small else 26
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def depth_to_vis(depth):
    """深度可視化：每張自動正規化，近距離也看得出結構（只給人檢查，不進實驗）。"""
    valid = depth[depth > 0]
    if valid.size == 0:
        return np.zeros((*depth.shape, 3), np.uint8)
    lo, hi = np.percentile(valid, 2), np.percentile(valid, 98)
    if hi <= lo:
        hi = lo + 1
    norm = np.clip((depth.astype(np.float32) - lo) / (hi - lo), 0, 1)
    vis = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    vis[depth == 0] = (0, 0, 0)        # 無效深度標成黑色，一眼看出破洞
    return vis


def make_session_dir(note=""):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = f"session_{stamp}" + (f"_{note}" if note else "")
    path = os.path.join(OUT_ROOT, folder)
    for sub in ("color", "depth", "depth_vis", "seg_vis"):
        os.makedirs(os.path.join(path, sub), exist_ok=True)
    return path


def save_intrinsics(session_dir, profile, depth_scale):
    """存相機內參 —— 之後要把深度換算成公尺與物體高度時必須用到。"""
    cp = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = cp.get_intrinsics()
    data = {
        "width": intr.width, "height": intr.height,
        "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
        "model": str(intr.model), "coeffs": list(intr.coeffs),
        "depth_scale_m_per_unit": depth_scale,
        "note": "depth PNG 為 16-bit 原始單位；公尺 = 值 * depth_scale_m_per_unit",
    }
    with open(os.path.join(session_dir, "intrinsics.json"), "w") as f:
        json.dump(data, f, indent=2)
    return data


def main():
    args = parse_args()
    session_dir = make_session_dir(args.name)
    csv_path = os.path.join(session_dir, "metadata.csv")

    # ---- 可選：載入分割模型 ----
    seg_model = None
    road_id = None
    if args.seg:
        try:
            from ultralytics import YOLO
            seg_model = YOLO(args.model)
            names = seg_model.names
            for k, v in names.items():
                if str(v).lower() == "road":
                    road_id = k
            print(f"[分割] 已載入 {args.model}，類別={names}，road id={road_id}")
        except Exception as e:
            print(f"[警告] 分割模型載入失敗，改為純擷取模式：{e}")
            seg_model = None

    # ---- 開相機（彩色 + 深度，並對齊到彩色）----
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"[錯誤] 無法開啟 RealSense：{e}")
        print("[提示] D435i 一次只能被一個程式開啟。先關掉 ros_detect_*.py / "
              "ros_test_*.py / realsense-viewer 再跑。")
        return 1

    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    intr = save_intrinsics(session_dir, profile, depth_scale)
    align = rs.align(rs.stream.color)
    print(f"[相機] color+depth {COLOR_W}x{COLOR_H}@{FPS}，depth_scale={depth_scale}")
    print(f"[輸出] {session_dir}")

    # ---- CSV ----
    csv_f = open(csv_path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(csv_f, fieldnames=CSV_FIELDS)
    writer.writeheader()
    csv_f.flush()

    _f, _fs = load_cjk_font(18), load_cjk_font(14)
    fonts = (_f, _fs) if (_f and _fs) else None
    print(f"[畫面] 中文字型：{'已載入' if fonts else '找不到，退回英文顯示'}")

    preset_i = 0
    distance = 2
    lateral = "center"
    light_i = 0
    seq = 0
    last_empty_ref = ""
    rows = []          # 供 undo 使用

    cv2.namedWindow("Paper Dataset Capture", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as e:
                print(f"[警告] 等待影像逾時：{e}")
                continue

            aligned = align.process(frames)
            cf = aligned.get_color_frame()
            df = aligned.get_depth_frame()
            if not cf or not df:
                continue

            color = np.asanyarray(cf.get_data())          # BGR uint8
            depth = np.asanyarray(df.get_data())          # uint16，單位由 depth_scale 決定

            pair_id, obj, level, is_seen, obj_zh = PRESETS[preset_i]

            # ---- vis = 乾淨的疊圖（會被存成論文配圖，不可有 HUD 文字）----
            vis = color.copy()
            if seg_model is not None:
                try:
                    r = seg_model.predict(color, imgsz=320, verbose=False)[0]
                    vis = r.plot()
                except Exception as e:
                    print(f"[警告] 分割推論失敗：{e}")

            # ---- disp = 給人看的畫面，疊 HUD ----
            if fonts is not None:
                hud = [
                    (f"[{preset_i+1}/{len(PRESETS)}] {pair_id}  {obj_zh}  {LEVEL_ZH[level]}"
                     + ("  ★已見類別" if is_seen else ""), (0, 255, 0), False),
                    (f"距離 {distance}m  橫向 {'正中' if lateral == 'center' else '偏左'}"
                     f"  光線 {LIGHTING_ZH[LIGHTING[light_i]]}  地點 {args.location}"
                     f"  已存 {seq} 筆", (0, 255, 255), False),
                    ("空白=擷取 n/b=物件 1~4=距離 c/v=橫向 g=光線 u=復原 q=離開",
                     (255, 255, 0), True),
                ]
            else:
                hud = [
                    (f"[{preset_i+1}/{len(PRESETS)}] {pair_id} {obj} {level}"
                     + ("  (SEEN)" if is_seen else ""), (0, 255, 0), False),
                    (f"dist={distance}m lat={lateral} light={LIGHTING[light_i]}"
                     f" loc={args.location} saved={seq}", (0, 255, 255), False),
                    ("SPACE=cap n/b=obj 1-4=dist c/v=lat g=light u=undo q=quit",
                     (255, 255, 0), True),
                ]
            disp = draw_hud(vis.copy(), hud, fonts)
            cv2.imshow("Paper Dataset Capture", disp)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("n"):
                preset_i = (preset_i + 1) % len(PRESETS)
            elif key == ord("b"):
                preset_i = (preset_i - 1) % len(PRESETS)
            elif key in DISTANCES:
                distance = DISTANCES[key]
            elif key == ord("c"):
                lateral = "center"
            elif key == ord("v"):
                lateral = "left"
            elif key == ord("g"):
                light_i = (light_i + 1) % len(LIGHTING)
            elif key == ord("u"):
                if rows:
                    last = rows.pop()
                    stem = os.path.splitext(last["filename_color"])[0]
                    # 四個目錄都要刪，否則 depth_vis / seg_vis 會留下孤兒檔
                    for sub, fn in (("color",     last["filename_color"]),
                                    ("depth",     last["filename_depth"]),
                                    ("depth_vis", stem + "_dvis.jpg"),
                                    ("seg_vis",   stem + "_seg.jpg")):
                        p = os.path.join(session_dir, sub, fn)
                        if os.path.exists(p):
                            os.remove(p)
                    seq -= 1
                    # CSV 整份重寫（樣本數不大，最簡單也最不會錯）
                    csv_f.close()
                    csv_f = open(csv_path, "w", newline="", encoding="utf-8")
                    writer = csv.DictWriter(csv_f, fieldnames=CSV_FIELDS)
                    writer.writeheader()
                    writer.writerows(rows)
                    csv_f.flush()
                    print(f"[復原] 已刪除 {last['filename_color']}")
                else:
                    print("[復原] 沒有可刪的紀錄")
            elif key == ord(" "):
                base = f"{pair_id}_{obj}_{level}_{distance}m_{lateral}_{seq:03d}"
                fn_color = base + ".png"
                fn_depth = base + "_depth.png"

                cv2.imwrite(os.path.join(session_dir, "color", fn_color), color)
                cv2.imwrite(os.path.join(session_dir, "depth", fn_depth), depth)

                # 深度可視化（只給人檢查，不進實驗）；黑色 = 無效深度
                cv2.imwrite(os.path.join(session_dir, "depth_vis", base + "_dvis.jpg"),
                            depth_to_vis(depth))

                if seg_model is not None:
                    cv2.imwrite(os.path.join(session_dir, "seg_vis", base + "_seg.jpg"), vis)

                # 中央區塊深度中位數 —— 現場快速檢查深度是不是全黑
                h, w = depth.shape
                patch = depth[h // 2 - 40:h // 2 + 40, w // 2 - 40:w // 2 + 40]
                valid = patch[patch > 0]
                dmed = float(np.median(valid)) * depth_scale * 1000.0 if valid.size else 0.0

                if pair_id == "EMPTY":
                    last_empty_ref = fn_color

                row = {
                    "seq": seq,
                    "filename_color": fn_color,
                    "filename_depth": fn_depth,
                    "pair_id": pair_id,
                    "object": obj,
                    "expected_level": level,
                    "distance_m": distance,
                    "lateral": lateral,
                    "is_seen_class": str(is_seen).lower(),
                    "location": args.location,
                    "lighting": LIGHTING[light_i],
                    "timestamp": datetime.now().isoformat(timespec="seconds"),
                    "empty_ref": "" if pair_id == "EMPTY" else last_empty_ref,
                    "depth_median_mm": round(dmed, 1),
                    "notes": "",
                }
                writer.writerow(row)
                csv_f.flush()
                rows.append(row)
                seq += 1

                warn = "  <-- 深度可疑，請檢查！" if dmed <= 0 else ""
                print(f"[存檔] {fn_color}  depth_med={dmed:.0f}mm{warn}  (累計 {seq})")

    finally:
        csv_f.close()
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"\n[完成] 共 {seq} 筆 → {session_dir}")
        print(f"[提醒] metadata.csv 的 expected_level 是拍攝時的預期，"
              f"最終 ground truth 仍需 2-3 人獨立標註並計算 kappa。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
