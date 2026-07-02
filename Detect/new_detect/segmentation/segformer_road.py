#!/usr/bin/env python3
# =============================================================================
# segformer_road.py — run a PRETRAINED SegFormer (Cityscapes) on real road
# frames to see how well an off-the-shelf model segments your brick bike-lane
# WITHOUT any training. Saves a colorized overlay + reports class coverage.
#
# Cityscapes classes include: road, sidewalk, terrain(grass), vegetation(trees),
# building, sky, person, bicycle, motorcycle — a strong match for your scene.
#
# Usage:
#   python3 segformer_road.py IMG [IMG ...]
#   python3 segformer_road.py --model nvidia/segformer-b2-finetuned-cityscapes-1024-1024 IMG
# First run downloads the model (~15 MB for b0) to ~/.cache/huggingface.
# CPU-only is fine (a few seconds/frame).
# =============================================================================
import os, sys, glob
import numpy as np

OUTDIR = os.path.expanduser("~/aurora_bags/segformer_out")
MODEL = "nvidia/segformer-b0-finetuned-cityscapes-1024-1024"

# Cityscapes 19-class palette (index -> BGR for cv2)
PALETTE = [
    (128, 64, 128), (232, 35, 244), (70, 70, 70), (156, 102, 102), (153, 153, 190),
    (153, 153, 153), (30, 170, 250), (0, 220, 220), (35, 142, 107), (152, 251, 152),
    (180, 130, 70), (60, 20, 220), (0, 0, 255), (142, 0, 0), (70, 0, 0),
    (100, 60, 0), (100, 80, 0), (230, 0, 0), (32, 11, 119),
]


def main():
    args = sys.argv[1:]
    model_name = MODEL
    if args and args[0] == "--model":
        model_name = args[1]; args = args[2:]
    if not args:
        # default: one frame per training session
        args = [sorted(glob.glob(os.path.expanduser(
            "~/Go_by_my_self/Detect/new_detect/train_data/*/*.jpg")))[0]]
    try:
        import torch, cv2
        from PIL import Image
        from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
    except ImportError as e:
        sys.exit(f"missing dep ({e}). install: pip3 install --user torch torchvision "
                 f"transformers pillow --index-url https://download.pytorch.org/whl/cpu")

    os.makedirs(OUTDIR, exist_ok=True)
    print(f"loading {model_name} (first run downloads it)...")
    proc = SegformerImageProcessor.from_pretrained(model_name)
    model = SegformerForSemanticSegmentation.from_pretrained(model_name).eval()
    id2label = model.config.id2label

    for path in args:
        img = Image.open(path).convert("RGB")
        W, H = img.size
        inputs = proc(images=img, return_tensors="pt")
        with torch.no_grad():
            logits = model(**inputs).logits
        up = torch.nn.functional.interpolate(logits, size=(H, W),
                                             mode="bilinear", align_corners=False)
        seg = up.argmax(1)[0].cpu().numpy().astype(np.int32)

        # colorized mask + overlay
        color = np.zeros((H, W, 3), np.uint8)
        for cid in np.unique(seg):
            color[seg == cid] = PALETTE[cid % len(PALETTE)]
        bgr = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        overlay = cv2.addWeighted(bgr, 0.55, color, 0.45, 0)

        # draw a legend (class name + colour swatch) for the classes present
        present = sorted(np.unique(seg), key=lambda c: -(seg == c).sum())
        y0 = 20
        for cid in present[:9]:
            col = tuple(int(v) for v in PALETTE[cid % len(PALETTE)])
            cv2.rectangle(overlay, (6, y0 - 12), (24, y0 + 2), col, -1)
            cv2.rectangle(overlay, (6, y0 - 12), (24, y0 + 2), (255, 255, 255), 1)
            cv2.putText(overlay, id2label[int(cid)], (30, y0),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(overlay, id2label[int(cid)], (30, y0),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            y0 += 20

        tag = "-".join(p for p in model_name.split("/")[-1].split("-")
                       if p in ("b0", "b1", "b2", "b3", "b4", "b5", "cityscapes", "ade"))
        base = os.path.splitext(os.path.basename(path))[0]
        outp = os.path.join(OUTDIR, f"{base}_{tag or 'segformer'}.png")
        cv2.imwrite(outp, overlay)

        ids, counts = np.unique(seg, return_counts=True)
        print(f"\n{path}  ->  {outp}")
        for cid, c in sorted(zip(ids, counts), key=lambda z: -z[1]):
            print(f"  {id2label[int(cid)]:14s} {100.0*c/seg.size:5.1f}%")


if __name__ == "__main__":
    main()
