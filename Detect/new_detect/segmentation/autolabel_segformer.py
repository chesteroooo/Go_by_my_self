#!/usr/bin/env python3
# =============================================================================
# autolabel_segformer.py — turn a folder of road images into a YOLOv8-seg
# dataset (pre-labeled by a pretrained SegFormer) ready for CORRECTION in
# Roboflow / CVAT. Works on Linux and Windows.
#
# Pipeline:
#   1) find + subsample ~N diverse frames from YOUR image folder
#      (grouped by sub-folder so multiple capture sessions each contribute),
#   2) run SegFormer-Cityscapes and map its classes to target classes,
#   3) convert each class mask to polygons -> YOLOv8-seg .txt labels,
#   4) emit images/ + labels/ + data.yaml (+ optional viz/ for QC).
#
# Class mapping (Cityscapes trainId -> target class); EDIT to fit your task:
#   road(0)->road, sidewalk(1)->sidewalk, vegetation(8)+terrain(9)->grass.
#   Anything not in the map is left UNLABELED (background) on purpose.
#
# ---- Install deps (CPU; same commands on Linux & Windows) --------------------
#   pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
#   pip install transformers pillow opencv-python numpy
#
# ---- Usage ------------------------------------------------------------------
#   python autolabel_segformer.py --data /path/to/images --out /path/to/dataset
#   python autolabel_segformer.py --data ./my_images --n 300 --viz
#   python autolabel_segformer.py --data ./my_images --limit 3 --viz   # quick test
#   (Linux may use `python3`. CPU: SegFormer-b4 ~5-15 s/frame.)
# =============================================================================
import os
import sys
import argparse
import shutil
import numpy as np
import cv2

# Cityscapes trainId -> target class id.  EDIT THIS for a different task.
CITY_TO_TARGET = {0: 1, 1: 2, 8: 0, 9: 0}
TARGET_NAMES = {0: "grass", 1: "road", 2: "sidewalk"}
VIZ_COLOR = {0: (0, 180, 0), 1: (0, 0, 210), 2: (230, 0, 230)}  # BGR
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def find_image_groups(data_dir):
    """Group images by their containing folder (so each capture session/subdir
    contributes to the sample). Works for a flat folder or nested folders,
    case-insensitively, on Linux and Windows."""
    groups = {}
    for root, _dirs, files in os.walk(data_dir):
        imgs = sorted(os.path.join(root, f) for f in files
                      if os.path.splitext(f)[1].lower() in IMG_EXTS)
        if imgs:
            groups[root] = imgs
    return groups


def subsample(groups, n, per_group_min=6):
    total = sum(len(v) for v in groups.values()) or 1
    picks = []
    for _grp, imgs in sorted(groups.items()):
        quota = min(len(imgs), max(per_group_min, round(n * len(imgs) / total)))
        stride = max(1, len(imgs) // quota)
        picks += imgs[::stride][:quota]
    return picks


def mask_to_polys(mask, min_area):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        if cv2.contourArea(c) < min_area:
            continue
        approx = cv2.approxPolyDP(c, 0.002 * cv2.arcLength(c, True), True).reshape(-1, 2)
        if len(approx) >= 3:
            out.append(approx)
    return out


def main():
    ap = argparse.ArgumentParser(description="SegFormer auto-labeler -> YOLOv8-seg dataset")
    ap.add_argument("--data", required=True,
                    help="folder of your images (searched recursively)")
    ap.add_argument("--out", default="autolabel_dataset",
                    help="output dataset folder (default: ./autolabel_dataset)")
    ap.add_argument("--n", type=int, default=300, help="target number of frames")
    ap.add_argument("--model", default="nvidia/segformer-b4-finetuned-cityscapes-1024-1024")
    ap.add_argument("--min-area-frac", type=float, default=0.004,
                    help="drop polygons smaller than this fraction of the image")
    ap.add_argument("--limit", type=int, default=0,
                    help="cap total frames (0 = no cap; use a small value to test)")
    ap.add_argument("--viz", action="store_true",
                    help="also save polygon overlays to viz/ for quality-check")
    args = ap.parse_args()

    if not os.path.isdir(args.data):
        sys.exit(f"--data folder not found: {args.data}")
    groups = find_image_groups(args.data)
    if not groups:
        sys.exit(f"no images ({', '.join(sorted(IMG_EXTS))}) found under: {args.data}")

    try:
        import torch
        from PIL import Image
        from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
    except ImportError as e:
        sys.exit(f"missing dependency ({e}).\ninstall (CPU):\n"
                 "  pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu\n"
                 "  pip install transformers pillow opencv-python numpy")
    torch.set_num_threads(os.cpu_count() or 4)

    frames = subsample(groups, args.n)
    if args.limit:
        frames = frames[:args.limit]
    print(f"found {sum(len(v) for v in groups.values())} images in {len(groups)} folder(s); "
          f"processing {len(frames)}")

    img_dir = os.path.join(args.out, "images")
    lbl_dir = os.path.join(args.out, "labels")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)
    viz_dir = os.path.join(args.out, "viz")
    if args.viz:
        os.makedirs(viz_dir, exist_ok=True)

    print(f"loading {args.model} (first run downloads it)...")
    proc = SegformerImageProcessor.from_pretrained(args.model)
    model = SegformerForSemanticSegmentation.from_pretrained(args.model).eval()

    seen = set()
    for i, path in enumerate(frames):
        img = Image.open(path).convert("RGB")
        W, H = img.size
        with torch.no_grad():
            logits = model(**proc(images=img, return_tensors="pt")).logits
        seg = torch.nn.functional.interpolate(
            logits, (H, W), mode="bilinear", align_corners=False).argmax(1)[0].numpy()

        lines = []
        overlay = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR) if args.viz else None
        for city_id, tgt in CITY_TO_TARGET.items():
            m = (seg == city_id).astype(np.uint8) * 255
            if not m.any():
                continue
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
            for poly in mask_to_polys(m, args.min_area_frac * W * H):
                norm = poly.astype(float)
                norm[:, 0] /= W
                norm[:, 1] /= H
                lines.append(f"{tgt} " + " ".join(f"{v:.5f}" for v in norm.flatten()))
                if overlay is not None:
                    cv2.polylines(overlay, [poly], True, VIZ_COLOR[tgt], 2)

        # unique output name even if two folders hold the same filename
        stem = os.path.splitext(os.path.basename(path))[0]
        name = stem
        k = 1
        while name in seen:
            name = f"{stem}_{k}"
            k += 1
        seen.add(name)

        shutil.copy(path, os.path.join(img_dir, name + ".jpg"))
        with open(os.path.join(lbl_dir, name + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        if overlay is not None:
            cv2.imwrite(os.path.join(viz_dir, name + ".png"), overlay)
        if (i + 1) % 25 == 0 or i == len(frames) - 1:
            print(f"  {i + 1}/{len(frames)}")

    out_abs = os.path.abspath(args.out).replace(os.sep, "/")   # forward slashes: YAML-safe on Windows
    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        f.write("# YOLOv8-seg dataset (auto-labeled by SegFormer; correct in Roboflow/CVAT)\n")
        f.write(f"path: {out_abs}\ntrain: images\nval: images\n")
        f.write(f"nc: {len(TARGET_NAMES)}\nnames:\n")
        for k in sorted(TARGET_NAMES):
            f.write(f"  {k}: {TARGET_NAMES[k]}\n")
    print(f"\ndataset -> {os.path.abspath(args.out)}")
    print(f"  images/  labels/  data.yaml" + ("  viz/" if args.viz else ""))
    print(f"  classes: {TARGET_NAMES}")


if __name__ == "__main__":
    main()
