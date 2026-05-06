#!/usr/bin/env python3
"""Reference: run SAM2 Hiera-Tiny **encoder.onnx** + **decoder.onnx** with a **box prompt**.

This matches the files next to this script (see ``config.yaml``). Some third-party
C++ samples use a different packaging (e.g. ``*_preprocess.onnx`` + one ``*.onnx``);
your split (image → encoder, prompts+embeddings → decoder) is the standard one.

Pipeline
--------
1. Preprocess RGB: resize to 1024×1024, SAM-style mean/std (same family as Meta SAM).
2. ``encoder``: ``image`` [1,3,1024,1024] → ``image_embed``, ``high_res_feats_*``.
3. ``decoder``: box as two corners with labels **2** (top-left) and **3** (bottom-right)
   in **1024×1024 pixel coordinates** (float), plus empty mask prompt.
4. Output ``masks`` is [1, 3, 256, 256] logits — three hypotheses. Pick best by
   ``iou_predictions`` (highest IoU score), then sigmoid → binary mask, upsample
   to the original image size for centroid / visualization.

C++ / ONNX Runtime
------------------
Use the same tensor names and dtypes (float32). Link the macOS universal tarball you
extracted: see ``onnxruntime_cpp_hint.txt`` in this folder.

Usage
-----
  python3 run_sam2_box_onnx.py --image path.jpg --box 100 80 400 350
  python3 run_sam2_box_onnx.py --image path.jpg   # default: center 40%% box

Requires: ``pip install onnxruntime opencv-python pyyaml`` (or use repo ``.venv_onnx``).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import yaml


def _repo_sam_dir() -> Path:
    return Path(__file__).resolve().parent


def load_config(cfg_path: Path | None) -> dict:
    p = cfg_path or (_repo_sam_dir() / "config.yaml")
    with open(p, encoding="utf-8") as f:
        return yaml.safe_load(f)


def preprocess_sam_bgr(
    bgr: np.ndarray,
    input_hw: tuple[int, int] = (1024, 1024),
) -> tuple[np.ndarray, tuple[int, int]]:
    """Return NCHW float32 for encoder and original (H, W)."""
    h0, w0 = bgr.shape[:2]
    pixel_mean = np.array([123.675, 116.28, 103.53], dtype=np.float32)
    pixel_std = np.array([58.395, 57.12, 57.375], dtype=np.float32)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32)
    x = cv2.resize(rgb, input_hw, interpolation=cv2.INTER_LINEAR)
    x = (x - pixel_mean) / pixel_std
    x = np.transpose(x, (2, 0, 1))[np.newaxis].astype(np.float32)
    return x, (h0, w0)


def box_xyxy_to_decoder_inputs(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    orig_hw: tuple[int, int],
    model_hw: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray]:
    """Map original-image box corners to 1024-space point_coords / point_labels."""
    h0, w0 = orig_hw
    mh, mw = model_hw
    sx = mw / float(w0)
    sy = mh / float(h0)
    coords = np.array(
        [[[x1 * sx, y1 * sy], [x2 * sx, y2 * sy]]],
        dtype=np.float32,
    )
    # SAM / SAM2 ONNX box prompt convention (corner labels).
    labels = np.array([[2.0, 3.0]], dtype=np.float32)
    return coords, labels


def run_encoder(
    sess: ort.InferenceSession,
    image_nchw: np.ndarray,
) -> dict[str, np.ndarray]:
    inp = sess.get_inputs()[0].name
    outs = sess.run(None, {inp: image_nchw})
    names = [o.name for o in sess.get_outputs()]
    return dict(zip(names, outs))


def run_decoder_box(
    sess: ort.InferenceSession,
    enc: dict[str, np.ndarray],
    point_coords: np.ndarray,
    point_labels: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    mask_input = np.zeros((point_coords.shape[0], 1, 256, 256), dtype=np.float32)
    has_mask_input = np.zeros((point_coords.shape[0],), dtype=np.float32)
    feeds = {
        "image_embed": enc["image_embed"],
        "high_res_feats_0": enc["high_res_feats_0"],
        "high_res_feats_1": enc["high_res_feats_1"],
        "point_coords": point_coords,
        "point_labels": point_labels,
        "mask_input": mask_input,
        "has_mask_input": has_mask_input,
    }
    masks, iou_pred = sess.run(None, feeds)
    return masks, iou_pred


def main() -> None:
    ap = argparse.ArgumentParser(description="SAM2 Hiera-Tiny ONNX box segmentation (reference).")
    ap.add_argument("--image", type=Path, required=True)
    ap.add_argument("--box", type=float, nargs=4, metavar=("X1", "Y1", "X2", "Y2"))
    ap.add_argument("--config", type=Path, default=None, help="YAML with encoder/decoder paths.")
    ap.add_argument("--out-mask", type=Path, default=Path("/tmp/sam2_mask.png"))
    ap.add_argument("--out-overlay", type=Path, default=Path("/tmp/sam2_overlay.jpg"))
    args = ap.parse_args()

    cfg = load_config(args.config)
    here = _repo_sam_dir()
    enc_path = here / cfg["encoder_model_path"]
    dec_path = here / cfg["decoder_model_path"]
    input_size = int(cfg.get("input_size", 1024))
    model_hw = (input_size, input_size)

    bgr = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if bgr is None:
        raise SystemExit(f"Could not read image: {args.image}")

    h0, w0 = bgr.shape[:2]
    if args.box is None:
        mx, my = w0 * 0.3, h0 * 0.3
        x1, y1 = mx, my
        x2, y2 = w0 - mx, h0 - my
    else:
        x1, y1, x2, y2 = args.box

    image_nchw, orig_hw = preprocess_sam_bgr(bgr, input_hw=model_hw)

    providers = ["CPUExecutionProvider"]
    enc_sess = ort.InferenceSession(str(enc_path), providers=providers)
    dec_sess = ort.InferenceSession(str(dec_path), providers=providers)

    enc_out = run_encoder(enc_sess, image_nchw)
    pc, pl = box_xyxy_to_decoder_inputs(x1, y1, x2, y2, orig_hw, model_hw)
    masks, iou = run_decoder_box(dec_sess, enc_out, pc, pl)

    # masks: [1, 3, 256, 256] logits; iou: [1, 3]
    best = int(np.argmax(iou[0]))
    logits = masks[0, best]
    prob = 1.0 / (1.0 + np.exp(-logits))
    mask_small = (prob > 0.5).astype(np.uint8) * 255
    mask_full = cv2.resize(mask_small, (w0, h0), interpolation=cv2.INTER_LINEAR)
    mask_full = (mask_full > 127).astype(np.uint8) * 255

    ys, xs = np.where(mask_full > 0)
    if len(xs) == 0:
        print("Empty mask — try another box or check preprocessing.")
    else:
        cx, cy = float(xs.mean()), float(ys.mean())
        print(f"centroid_xy original image: ({cx:.1f}, {cy:.1f})  (for depth at full res)")
        print(f"best_mask_index={best}  iou_scores={np.round(iou[0], 4)}")

    cv2.imwrite(str(args.out_mask), mask_full)
    overlay = bgr.copy()
    overlay[mask_full > 0] = (0.4 * overlay[mask_full > 0] + 0.6 * np.array([0, 255, 0])).astype(
        np.uint8
    )
    cv2.imwrite(str(args.out_overlay), overlay)
    print(f"wrote {args.out_mask} and {args.out_overlay}")


if __name__ == "__main__":
    main()
