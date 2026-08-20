#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
三級處置 ground truth 標註表單產生器 + 一致性計分

規則見 paper/三級處置標註規則.md。這支腳本負責層 B 的機械部分：
  1. 從 capture_paper_dataset.py 產生的 metadata.csv 建立**匿名**標註表單
  2. 收回各標註者的表單後，計算 kappa、多數決定案、列出爭議案例

為什麼要匿名：原始檔名 `P01_box_L1_2m_center_001.png` 直接寫著物件名與預期等級，
`metadata.csv` 也有 expected_level 欄。標註者只要看到任何一個，盲標就失效，
kappa 會被錨定效應灌水成假的高一致性。所以表單只給 `I001` 這種不帶資訊的編號，
並在 labels/images/ 下建立同名 symlink 讓標註者看圖。

用法：
    # 產生 3 份空白表單（自動抓 paper/paper_data/ 下所有 session）
    python3 paper/make_label_sheets.py

    # 指定特定 session（例如只想標某一個地點）
    python3 paper/make_label_sheets.py --meta paper/paper_data/session_.../metadata.csv

    # 各自填完 level 欄之後計分
    python3 paper/make_label_sheets.py --score
"""

import argparse
import csv
import glob
import os
import random
import sys
from collections import Counter, defaultdict

LEVELS = ["L0", "L1", "L2"]
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(_HERE, "labels")
# capture_paper_dataset.py 的 OUT_ROOT，每次執行建一個 session_<時間戳> 資料夾
PAPER_DATA = os.path.join(_HERE, "paper_data")
KEY_NAME = "_key_DO_NOT_OPEN.csv"

# 標註者只會看到這三欄，其餘一律不外流
SHEET_FIELDS = ["item_id", "image", "level", "notes"]


# ---------------------------------------------------------------- 產生表單
def discover_meta():
    """沒給 --meta 時，自動撿 paper_data/ 下所有 session 的 metadata.csv。"""
    found = sorted(glob.glob(os.path.join(PAPER_DATA, "session_*", "metadata.csv")))
    if not found:
        sys.exit(
            f"[錯誤] 在 {os.path.normpath(PAPER_DATA)} 底下找不到任何 session。\n"
            "       這支腳本是「拍完之後」用的 —— 先用 capture_paper_dataset.py 拍攝：\n"
            "         python3 Detect/new_detect/segmentation/capture_paper_dataset.py "
            "--location A --seg\n"
            "       已經拍過但放在別的地方，就用 --meta 指定 metadata.csv 的路徑。")
    print(f"自動找到 {len(found)} 個 session：")
    for p in found:
        print(f"  {os.path.normpath(p)}")
    print()
    return found


def load_meta(paths):
    """讀入一或多個 metadata.csv，回傳 [(來源目錄, row), ...]。"""
    rows = []
    for p in paths:
        if not os.path.isfile(p):
            sys.exit(f"[錯誤] 找不到 metadata：{p}")
        base = os.path.dirname(os.path.abspath(p))
        with open(p, newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f):
                if not r.get("filename_color"):
                    continue
                rows.append((base, r))
    if not rows:
        sys.exit("[錯誤] metadata 裡沒有任何影像")
    return rows


def resolve_color(base, name):
    """capture_paper_dataset.py 把彩色圖放在 color/ 子目錄下。"""
    for cand in (os.path.join(base, "color", name), os.path.join(base, name)):
        if os.path.isfile(cand):
            return cand
    return None


def make_sheets(meta_paths, names, out_dir, seed, copy_images):
    rows = load_meta(meta_paths)
    random.Random(seed).shuffle(rows)          # 打散順序，消除序列效應

    img_dir = os.path.join(out_dir, "images")
    os.makedirs(img_dir, exist_ok=True)

    key_rows, sheet_rows, missing = [], [], []
    for i, (base, r) in enumerate(rows, start=1):
        item_id = f"I{i:03d}"
        src = resolve_color(base, r["filename_color"])
        if src is None:
            missing.append(r["filename_color"])
            continue

        ext = os.path.splitext(src)[1] or ".png"
        dst = os.path.join(img_dir, item_id + ext)
        if os.path.lexists(dst):
            os.remove(dst)
        if copy_images:
            import shutil
            shutil.copy2(src, dst)
        else:
            os.symlink(os.path.abspath(src), dst)

        sheet_rows.append({"item_id": item_id,
                           "image": os.path.join("images", item_id + ext),
                           "level": "", "notes": ""})
        key_rows.append({"item_id": item_id,
                         "source_dir": base,
                         "filename_color": r["filename_color"],
                         "pair_id": r.get("pair_id", ""),
                         "object": r.get("object", ""),
                         "expected_level": r.get("expected_level", ""),
                         "distance_m": r.get("distance_m", ""),
                         "lateral": r.get("lateral", ""),
                         "is_seen_class": r.get("is_seen_class", ""),
                         "location": r.get("location", "")})

    if missing:
        print(f"[警告] {len(missing)} 張影像找不到檔案，已跳過：{missing[:5]}"
              + (" ..." if len(missing) > 5 else ""))

    for n in names:
        path = os.path.join(out_dir, f"sheet_{n}.csv")
        if os.path.exists(path):
            sys.exit(f"[錯誤] {path} 已存在。要重建請先手動移除（避免覆蓋已填好的標註）。")
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=SHEET_FIELDS)
            w.writeheader()
            w.writerows(sheet_rows)
        print(f"  空白表單：{path}")

    key_path = os.path.join(out_dir, KEY_NAME)
    with open(key_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(key_rows[0].keys()))
        w.writeheader()
        w.writerows(key_rows)

    print(f"\n共 {len(sheet_rows)} 張影像、{len(names)} 位標註者。")
    print(f"匿名影像：{img_dir}/（{'複製' if copy_images else 'symlink'}）")
    print(f"對照表：{key_path}")
    print("\n⚠️  標註者請只開 sheet_*.csv 與 images/，")
    print("    不要開對照表、不要開原始資料夾、不要看深度圖或分割疊圖。")
    print("    level 欄只填 L0 / L1 / L2，看不懂的填 notes，不要留空。")


# ---------------------------------------------------------------- kappa
def cohen_kappa(a, b):
    """兩位標註者的 Cohen's kappa。a、b 為等長的等級序列。"""
    n = len(a)
    if n == 0:
        return float("nan")
    po = sum(1 for x, y in zip(a, b) if x == y) / n
    ca, cb = Counter(a), Counter(b)
    pe = sum((ca[c] / n) * (cb[c] / n) for c in LEVELS)
    return 1.0 if pe == 1 else (po - pe) / (1 - pe)


def fleiss_kappa(table):
    """三位以上的 Fleiss' kappa。table = [[各等級的計票], ...]，每列總和相同。"""
    N = len(table)
    n = sum(table[0])
    if N == 0 or n < 2:
        return float("nan")
    P = [(sum(c * c for c in row) - n) / (n * (n - 1)) for row in table]
    P_bar = sum(P) / N
    p = [sum(row[j] for row in table) / (N * n) for j in range(len(LEVELS))]
    P_e = sum(x * x for x in p)
    return 1.0 if P_e == 1 else (P_bar - P_e) / (1 - P_e)


def interpret(k):
    if k != k:
        return "無法計算"
    for lo, txt in ((0.80, "Almost perfect"), (0.60, "Substantial"),
                    (0.40, "Moderate"), (0.20, "Fair"), (0.0, "Slight")):
        if k >= lo:
            return txt
    return "Poor（比隨機還差）"


# ---------------------------------------------------------------- 計分
def score(out_dir):
    key_path = os.path.join(out_dir, KEY_NAME)
    if not os.path.isfile(key_path):
        sys.exit(f"[錯誤] 找不到對照表 {key_path}，請先產生表單。")
    with open(key_path, newline="", encoding="utf-8-sig") as f:
        key = {r["item_id"]: r for r in csv.DictReader(f)}

    sheets = {}
    for fn in sorted(os.listdir(out_dir)):
        if fn.startswith("sheet_") and fn.endswith(".csv"):
            name = fn[len("sheet_"):-len(".csv")]
            with open(os.path.join(out_dir, fn), newline="", encoding="utf-8-sig") as f:
                sheets[name] = {r["item_id"]: (r.get("level") or "").strip().upper()
                                for r in csv.DictReader(f)}
    if len(sheets) < 2:
        sys.exit("[錯誤] 至少需要兩份填好的 sheet_*.csv 才能算一致性。")

    # --- 完整性檢查：任何一格空白或非法都不能算 ---
    bad = defaultdict(list)
    for name, s in sheets.items():
        for item, lv in s.items():
            if lv not in LEVELS:
                bad[name].append(f"{item}={lv or '(空白)'}")
    if bad:
        print("[錯誤] 以下標註未完成或格式錯誤，補齊後再計分：\n")
        for name, items in bad.items():
            print(f"  sheet_{name}: {len(items)} 筆 — {items[:10]}"
                  + (" ..." if len(items) > 10 else ""))
        sys.exit(1)

    names = sorted(sheets)
    items = sorted(set.intersection(*(set(s) for s in sheets.values())))
    print(f"標註者：{', '.join(names)}   共同影像：{len(items)} 張\n")

    # --- 兩兩 Cohen's kappa ---
    print("兩兩一致性（Cohen's kappa）")
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a = [sheets[names[i]][it] for it in items]
            b = [sheets[names[j]][it] for it in items]
            k = cohen_kappa(a, b)
            print(f"  {names[i]} vs {names[j]}: kappa = {k:.3f}  ({interpret(k)})")

    # --- Fleiss' kappa ---
    if len(names) >= 3:
        table = [[sum(1 for n in names if sheets[n][it] == c) for c in LEVELS]
                 for it in items]
        k = fleiss_kappa(table)
        print(f"\n整體一致性（Fleiss' kappa, n={len(names)}）: "
              f"kappa = {k:.3f}  ({interpret(k)})")
        if k < 0.40:
            print("  ⚠️  低於 0.40 — 規則有嚴重問題，不可直接使用。"
                  "依規則手冊 B.4：改規則後全部重標。")

    # --- 多數決定案 + 爭議清單 ---
    gt_path = os.path.join(out_dir, "ground_truth.csv")
    unanimous = split = tie = 0
    disputes, mismatches = [], []
    with open(gt_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["item_id", "filename_color", "pair_id", "object",
                    "expected_level", "gt_level", "agreement", "votes"])
        for it in items:
            votes = [sheets[n][it] for n in names]
            cnt = Counter(votes)
            top, top_n = cnt.most_common(1)[0]
            if top_n == len(names):
                agree, unanimous = "unanimous", unanimous + 1
            elif top_n > len(names) - top_n:
                agree, split = "majority", split + 1
                disputes.append((it, votes))
            else:
                agree, tie = "TIE-需開會", tie + 1
                disputes.append((it, votes))
            k = key.get(it, {})
            w.writerow([it, k.get("filename_color", ""), k.get("pair_id", ""),
                        k.get("object", ""), k.get("expected_level", ""),
                        top, agree, "/".join(votes)])
            if k.get("expected_level") and k["expected_level"] != top:
                mismatches.append((it, k.get("object", ""), k["expected_level"], top))

    print(f"\n定案：全體一致 {unanimous} / 多數決 {split} / 平手需開會 {tie}")
    print(f"ground truth 已寫出：{gt_path}")

    if disputes:
        print(f"\n有分歧的 {len(disputes)} 張（先看這些）：")
        for it, votes in disputes[:15]:
            k = key.get(it, {})
            print(f"  {it}  {k.get('object','?'):<16} {k.get('distance_m','?')}m  →  "
                  f"{'/'.join(votes)}")
        if len(disputes) > 15:
            print(f"  ... 另外 {len(disputes) - 15} 張見 {gt_path}")

    if mismatches:
        print(f"\n與拍攝時 expected_level 不符的 {len(mismatches)} 張："
              "\n（這不是錯誤 — 依 PLAN，這本身就是要寫進論文的發現）")
        for it, obj, exp, got in mismatches[:15]:
            print(f"  {it}  {obj:<16} 預期 {exp} → 標成 {got}")
        if len(mismatches) > 15:
            print(f"  ... 另外 {len(mismatches) - 15} 張")


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description="三級處置標註表單產生器 / kappa 計分",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--meta", nargs="+",
                    help="metadata.csv 路徑；省略則自動抓 paper_data/ 下所有 session")
    ap.add_argument("--annotators", type=int, default=3, help="標註者人數（預設 3）")
    ap.add_argument("--names", nargs="+", help="標註者代號（預設 A B C...）")
    ap.add_argument("--out", default=DEFAULT_OUT, help=f"輸出目錄（預設 {DEFAULT_OUT}）")
    ap.add_argument("--seed", type=int, default=20260731, help="打亂順序的亂數種子")
    ap.add_argument("--copy", action="store_true", help="複製影像而非建 symlink")
    ap.add_argument("--score", action="store_true", help="計算 kappa 並定案")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    if args.score:
        score(args.out)
    else:
        names = args.names or [chr(ord("A") + i) for i in range(args.annotators)]
        make_sheets(args.meta or discover_meta(), names, args.out, args.seed, args.copy)


if __name__ == "__main__":
    main()
