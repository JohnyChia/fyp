#!/usr/bin/env python3
"""Export one frozen model/split to the canonical prediction contract.

Supports YOLO26n, YOLO26n-P2, RF-DETR Nano, and D-FINE Nano.  It performs no
accuracy scoring and never selects a model.  Output is create-only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PHASE2A = ROOT / "experiment/analysis/dev_validation_audit/phase2a"
sys.path.insert(0, str(PHASE2A))
from unified_eval.adapters import adapt_envelope  # noqa: E402
from unified_eval.contract import CANONICAL_CLASSES, image_specs_from_coco, write_prediction_set  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""): digest.update(block)
    return digest.hexdigest()


def family(row: dict) -> str:
    explicit = row.get("model_family")
    if explicit in {"yolo26n", "yolo26n_p2", "rfdetr_n", "dfine_n"}: return explicit
    name = str(row.get("architecture", "")).lower().replace("-", "_").replace(" ", "_")
    if "dfine" in name or "d_fine" in name: return "dfine_n"
    if "rf" in name and "detr" in name: return "rfdetr_n"
    if "p2" in name: return "yolo26n_p2"
    if "yolo" in name: return "yolo26n"
    raise ValueError(f"unsupported architecture: {row.get('architecture')!r}")


def checkpoint(row: dict, registry: Path) -> Path:
    path = Path(row["checkpoint"])
    if not path.is_absolute(): path = registry.parent / path
    path = path.resolve(strict=True)
    if path.stat().st_size != row["checkpoint_bytes"] or sha256(path) != row["checkpoint_sha256"]:
        raise ValueError("checkpoint size/hash differs from registry")
    return path


def canonical_rows(image_id: int, boxes, scores, labels, width: int, height: int) -> list[dict]:
    values = []
    for box, score, label in zip(boxes, scores, labels):
        coords = [float(x) for x in box]; confidence = float(score); class_id = int(label)
        if not all(math.isfinite(x) for x in coords + [confidence]): raise RuntimeError("non-finite model output")
        if confidence < 0.001: continue
        if class_id not in range(6): raise RuntimeError(f"noncanonical class id {class_id}")
        x1, y1, x2, y2 = coords
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise RuntimeError(f"out-of-bounds/nonpositive box for image {image_id}: {coords}")
        values.append({"image_id": image_id, "class_id": class_id, "confidence": confidence,
                       "x1": x1, "y1": y1, "x2": x2, "y2": y2})
    values.sort(key=lambda x: (-x["confidence"], x["class_id"], x["x1"], x["y1"], x["x2"], x["y2"]))
    return values[:300]


def load_backend(model_family: str, ckpt: Path, row: dict, device: str):
    if model_family.startswith("yolo"):
        from ultralytics import YOLO
        return YOLO(str(ckpt), task="detect")
    if model_family == "rfdetr_n":
        # Transformers 5.x probes torch.accelerator during import, while the
        # pinned local inference torch (2.5.x) predates that namespace. RF-DETR
        # itself does not require an accelerator object for CPU inference.
        import torch
        if not hasattr(torch, "accelerator"):
            class _AcceleratorCompat:
                @staticmethod
                def current_accelerator():
                    return None
            torch.accelerator = _AcceleratorCompat()  # type: ignore[attr-defined]
        from rfdetr import RFDETR
        return RFDETR.from_checkpoint(str(ckpt), device=device, trust_checkpoint=True)
    import torch
    import torch.nn as nn
    dfine_root = Path(row.get("dfine_source_root", HERE.parent / "durian_v4_2_8run_ready/colab/third_party/D-FINE"))
    if not dfine_root.is_absolute(): dfine_root = registry_parent(row) / dfine_root
    dfine_root = dfine_root.resolve(strict=True)
    config_value = row.get("effective_config")
    if not config_value: raise ValueError("D-FINE registry row requires effective_config")
    config = Path(config_value)
    if not config.is_absolute(): config = registry_parent(row) / config
    config = config.resolve(strict=True)
    if sha256(config) != row.get("effective_config_sha256"): raise ValueError("D-FINE effective config hash mismatch")
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=dfine_root, check=True, text=True, capture_output=True).stdout.strip()
    if revision != row.get("dfine_source_commit"): raise ValueError("D-FINE source revision mismatch")
    sys.path.insert(0, str(dfine_root))
    from src.core import YAMLConfig
    cfg = YAMLConfig(str(config), resume=str(ckpt))
    if "HGNetv2" in cfg.yaml_cfg: cfg.yaml_cfg["HGNetv2"]["pretrained"] = False
    state_doc = torch.load(ckpt, map_location="cpu", weights_only=False)
    state = state_doc["ema"]["module"] if "ema" in state_doc else state_doc["model"]
    cfg.model.load_state_dict(state, strict=True)
    class Model(nn.Module):
        def __init__(self):
            super().__init__(); self.model = cfg.model.deploy(); self.postprocessor = cfg.postprocessor.deploy()
        def forward(self, images, sizes): return self.postprocessor(self.model(images), sizes)
    return Model().to(device).eval()


def registry_parent(row: dict) -> Path:
    return Path(row["_registry_parent"])


def infer(model, model_family: str, image_path: Path, image: dict, device: str):
    if model_family.startswith("yolo"):
        result = model.predict(source=str(image_path), imgsz=640, conf=0.001, iou=0.70, max_det=300,
                               device=device, agnostic_nms=False, augment=False, rect=True, verbose=False,
                               save=False, stream=False)[0]
        return (result.boxes.xyxy.detach().cpu().numpy(), result.boxes.conf.detach().cpu().numpy(),
                result.boxes.cls.detach().cpu().numpy(), {"background": 0, "invalid_geometry": 0})
    if model_family == "rfdetr_n":
        from PIL import Image
        with Image.open(image_path) as opened:
            result = model.predict(opened.convert("RGB"), threshold=0.001, shape=(384, 384), include_source_image=False)
        # RF-DETR's six-class checkpoints have a 7-logit classification head;
        # slot 6 is the no-object/background channel. Some checkpoints can
        # surface that slot through the public prediction API at a low export
        # floor, so remove it explicitly while preserving semantic IDs 0..5.
        boxes = np.asarray(result.xyxy); scores = np.asarray(result.confidence); labels = np.asarray(result.class_id)
        semantic = labels < len(CANONICAL_CLASSES)
        finite = np.isfinite(boxes).all(axis=1) & np.isfinite(scores)
        geometry = ((boxes[:, 0] >= 0) & (boxes[:, 1] >= 0) &
                    (boxes[:, 2] <= image["width"]) & (boxes[:, 3] <= image["height"]) &
                    (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1]))
        keep = semantic & finite & geometry
        rejected = {"background": int((~semantic).sum()),
                    "invalid_geometry": int((semantic & ~(finite & geometry)).sum())}
        return boxes[keep], scores[keep], labels[keep], rejected
    import torch
    import torchvision.transforms as transforms
    from PIL import Image
    with Image.open(image_path) as opened:
        rgb = opened.convert("RGB"); tensor = transforms.Compose([transforms.Resize((640, 640)), transforms.ToTensor()])(rgb).unsqueeze(0).to(device)
    size = torch.tensor([[image["width"], image["height"]]], device=device)
    with torch.inference_mode(): labels, boxes, scores = model(tensor, size)
    return (boxes[0].detach().cpu().numpy(), scores[0].detach().cpu().numpy(),
            labels[0].detach().cpu().numpy(), {"background": 0, "invalid_geometry": 0})


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values); return ordered[max(1, math.ceil(q * len(ordered))) - 1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True); parser.add_argument("--run-id", required=True)
    parser.add_argument("--split", required=True); parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--image-root", type=Path, required=True); parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args(); registry = args.registry.resolve(); output = args.output_dir.resolve()
    if output.exists(): raise FileExistsError(f"refusing to overwrite {output}")
    document = json.loads(registry.read_text(encoding="utf-8")); matches = [x for x in document.get("runs", []) if x.get("run_id") == args.run_id]
    if len(matches) != 1: raise ValueError("run-id must appear exactly once in registry")
    row = dict(matches[0]); row["_registry_parent"] = str(registry.parent)
    if row.get("class_order") != list(CANONICAL_CLASSES): raise ValueError("registry class order mismatch")
    ckpt = checkpoint(row, registry); model_family = family(row)
    gt_path = args.ground_truth.resolve(strict=True); image_root = args.image_root.resolve(strict=True)
    gt = json.loads(gt_path.read_text(encoding="utf-8")); images = sorted(gt.get("images", []), key=lambda x: x["id"])
    specs = image_specs_from_coco(gt)
    for image in images:
        path = image_root / image["file_name"]
        if not path.is_file(): raise FileNotFoundError(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}-", dir=output.parent))
    started = time.perf_counter()
    try:
        load_started = time.perf_counter(); model = load_backend(model_family, ckpt, row, args.device); load_s = time.perf_counter() - load_started
        all_rows, latencies = [], []
        rejected = {"background": 0, "invalid_geometry": 0}
        for image in images:
            tick = time.perf_counter(); boxes, scores, labels, rejected_now = infer(model, model_family, image_root / image["file_name"], image, args.device)
            latencies.append(time.perf_counter() - tick)
            for key in rejected: rejected[key] += rejected_now[key]
            all_rows.extend(canonical_rows(image["id"], boxes, scores, labels, image["width"], image["height"]))
        envelope = {"schema_version": 1, "model_family": model_family, "coordinate_space": "original_image_pixels",
                    "class_id_space": "canonical_0_based", "class_names": list(CANONICAL_CLASSES),
                    "processed_image_ids": [x["id"] for x in images], "predictions": all_rows}
        adapted = adapt_envelope(envelope, specs)
        write_prediction_set(staging / "predictions.csv", staging / "coverage.json", adapted)
        timing = {"images": len(images), "checkpoint_load_s": load_s, "device": args.device,
                  "latency_ms": {"mean": statistics.fmean(latencies) * 1000, "median": statistics.median(latencies) * 1000,
                                 "p90_nearest_rank": percentile(latencies, .9) * 1000},
                  "elapsed_s": time.perf_counter() - started}
        (staging / "timing.json").write_text(json.dumps(timing, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        receipt = {"schema_version": "durian.canonical_prediction_export.v1", "status": "PASS", "run_id": args.run_id,
                   "split": args.split, "model_family": model_family, "checkpoint_sha256": row["checkpoint_sha256"],
                   "ground_truth_sha256": sha256(gt_path), "image_count": len(images), "prediction_count": len(adapted.predictions),
                   "rejected_background_count": rejected["background"],
                   "rejected_invalid_geometry_count": rejected["invalid_geometry"],
                   "predictions_sha256": sha256(staging / "predictions.csv"), "coverage_sha256": sha256(staging / "coverage.json"),
                   "inference_floor": 0.001, "per_image_cap": 300, "accuracy_metrics_computed": False, "sealed_test_accessed": False}
        receipt["exporter_sha256"] = sha256(Path(__file__))
        (staging / "completion_receipt.json").write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(staging, output)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True); raise
    print(json.dumps({"status": "PASS", "output": str(output), "images": len(images), "predictions": len(all_rows)}, indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())
