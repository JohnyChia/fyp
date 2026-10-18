#!/usr/bin/env python3
"""Exact-SHA Run04 GPU latency/VRAM and output-validity audit."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import sys
import time
from pathlib import Path

from PIL import Image


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
FAST = ROOT / "experiment/analysis/dev_validation_audit/fyp_fast_track"
REGISTRY = HERE / "post_training_evidence/v42_corrected_comparison_20261018/FINAL_MODEL_REGISTRY_V42.json"
MANIFEST = FAST / "consistency_input_manifest.json"
PROTOCOL = HERE / "post_training_evidence/v42_corrected_comparison_20261018/common_comparison_v1/V42_CONSISTENCY_PROTOCOL.json"
EXPECTED_SHA = "04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5"
THRESHOLD = 0.45
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(FAST))
from export_canonical_predictions import checkpoint, family, infer, load_backend, sha256  # noqa: E402
from run_consistency import transform  # noqa: E402
from run_six_model_runtime_audit import canonical  # noqa: E402


def create_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def file_sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(output)

    import torch
    if not torch.cuda.is_available() or torch.cuda.device_count() < 1:
        raise RuntimeError("CUDA unavailable: record BLOCKED rather than running this audit")
    registry = json.loads(REGISTRY.read_text(encoding="utf-8"))
    row = dict(next(item for item in registry["runs"] if item["run_id"] == "Run04"))
    if row["checkpoint_sha256"] != EXPECTED_SHA:
        raise RuntimeError("registry does not bind the required Run04 SHA")
    row["_registry_parent"] = str(REGISTRY.parent)
    ckpt = checkpoint(row, REGISTRY)
    if file_sha(ckpt) != EXPECTED_SHA:
        raise RuntimeError("checkpoint SHA mismatch")
    model_family = family(row)
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    if protocol["operating_thresholds"]["Run04"] != THRESHOLD:
        raise RuntimeError("frozen Run04 threshold mismatch")
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    cuda_device = torch.device("cuda:0")
    torch.cuda.empty_cache()
    # Some WSL2/PyTorch builds reject a bare integer in the memory-stat API
    # even though allocation on cuda:0 succeeds.
    torch.cuda.reset_peak_memory_stats()
    model = load_backend(model_family, ckpt, row, "cuda")
    first = manifest["real_samples"][0]
    with Image.open(first["path"]) as opened:
        infer(model, model_family, Path(first["path"]), {
            "id": 0, "width": opened.width, "height": opened.height}, "cuda")
    torch.cuda.synchronize()

    latencies = []
    counters = {"non_finite": 0, "coordinate": 0, "background": 0}
    prediction_count = 0
    for sample in manifest["real_samples"]:
        path = Path(sample["path"])
        if file_sha(path) != sample["sha256"]:
            raise RuntimeError(f"input hash mismatch: {path}")
        with Image.open(path) as opened:
            base = opened.convert("RGB")
        conditions = [(f"baseline_{index}", base) for index in range(1, 4)]
        conditions += [
            (item["id"], transform(base, item["id"])[0])
            for item in protocol["controlled_real_test"]["transforms"]
        ]
        for condition, image in conditions:
            temporary = output.parent / f".Run04-gpu-{condition}.png"
            image.save(temporary)
            try:
                torch.cuda.synchronize()
                started = time.perf_counter_ns()
                boxes, scores, labels, rejected = infer(
                    model, model_family, temporary,
                    {"id": 0, "width": image.width, "height": image.height}, "cuda")
                torch.cuda.synchronize()
                latencies.append((time.perf_counter_ns() - started) / 1_000_000.0)
            finally:
                temporary.unlink(missing_ok=True)
            rows, nf, cv, bg = canonical(
                boxes, scores, labels, rejected, THRESHOLD, image.width, image.height)
            counters["non_finite"] += nf
            counters["coordinate"] += cv
            counters["background"] += bg
            prediction_count += len(rows)

    ordered = sorted(latencies)
    p90 = ordered[max(1, math.ceil(0.9 * len(ordered))) - 1]
    device = torch.cuda.get_device_properties(cuda_device)
    receipt = {
        "schema_version": "durian.run04_v42_gpu_resource_audit.v1",
        "status": "PASS" if counters["non_finite"] == 0 and counters["coordinate"] == 0 else "FAIL",
        "run_id": "Run04",
        "checkpoint_sha256": EXPECTED_SHA,
        "threshold": THRESHOLD,
        "device_name": device.name,
        "device_total_memory_mb": device.total_memory / 1024 ** 2,
        "torch_version": torch.__version__,
        "torch_cuda_version": torch.version.cuda,
        "sample_count": len(latencies),
        "gpu_latency_ms": {
            "mean": statistics.fmean(latencies),
            "median": statistics.median(latencies),
            "p90": p90,
            "maximum": max(latencies),
        },
        "peak_vram_allocated_mb": torch.cuda.max_memory_allocated() / 1024 ** 2,
        "peak_vram_reserved_mb": torch.cuda.max_memory_reserved() / 1024 ** 2,
        "prediction_count_at_frozen_threshold": prediction_count,
        "non_finite_output_count": counters["non_finite"],
        "coordinate_violation_count": counters["coordinate"],
        "rejected_background_count": counters["background"],
        "registry_sha256": sha256(REGISTRY),
        "protocol_sha256": sha256(PROTOCOL),
        "input_manifest_sha256": sha256(MANIFEST),
        "sealed_test_accessed": False,
    }
    create_json(output / "GPU_RESOURCE_RECEIPT.json", receipt)
    print(json.dumps(receipt, indent=2))
    return 0 if receipt["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
