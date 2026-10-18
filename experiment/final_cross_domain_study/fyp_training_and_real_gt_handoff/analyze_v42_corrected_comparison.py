#!/usr/bin/env python3
"""Calibration, error, domain-shift and clustered uncertainty for V4.2 comparison."""
from __future__ import annotations

import argparse
import csv
import json
import random
from collections import Counter
from pathlib import Path

from analyze_six_model_deployment_evidence import (
    CLASSES,
    counts_and_calibration,
    load_predictions,
    macro_f1,
    percentile,
    sha256,
)


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
GT = ROOT / "experiment/final_cross_domain_study/durian_v4_2_8run_ready/datasets/coco/synthetic_only/synthetic_dev/_annotations.coco.json"
GT_SHA = "d841970d5d7227ed830cfec72fb8605f0cd488b3fd14469891da5c4133c6f703"
IDENTITY = ROOT / "experiment/analysis/dev_validation_audit/phase2a/manifests/synthetic_dev_identity.json"
REAL_GT = ROOT / "experiment/analysis/dev_validation_audit/ai_assisted_fyp_reference/REAL_DEV_AI_ASSISTED_FYP_REFERENCE.coco.json"
RUNS = tuple(f"Run{i:02d}" for i in range(1, 7))


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    root, output = args.comparison_root.resolve(), args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)

    if sha256(GT) != GT_SHA:
        raise RuntimeError("V4.2 Synthetic Dev identity mismatch")
    gt, real_gt = json.loads(GT.read_text()), json.loads(REAL_GT.read_text())
    identity = json.loads(IDENTITY.read_text())
    image_to_tree = {int(row["image_id"]): row["physical_tree_id"] for row in identity["records"]}
    if len(image_to_tree) != 288 or len(set(image_to_tree.values())) != 36:
        raise RuntimeError("physical-tree grouping mismatch")

    common_path = root / "common_comparison_v1/comparison.json"
    overlay_path = root / "fyp_ai_reference_comparison_v1/comparison.json"
    threshold_path = root / "common_comparison_v1/threshold_receipt.json"
    registry_path = root / "FINAL_MODEL_REGISTRY_V42.json"
    common = {row["run_id"]: row for row in json.loads(common_path.read_text())["leaderboard"]}
    overlay = {row["run_id"]: row for row in json.loads(overlay_path.read_text())["leaderboard"]}
    thresholds = {row["run_id"]: row["threshold"] for row in json.loads(threshold_path.read_text())["selections"]}
    if set(common) != set(RUNS) or set(overlay) != set(RUNS) or set(thresholds) != set(RUNS):
        raise RuntimeError("six-run comparison coverage mismatch")

    per_run, summary = {}, []
    for run_id in RUNS:
        threshold = thresholds[run_id]
        synthetic_path = root / f"predictions/{run_id}/synthetic_dev/predictions.csv"
        real_path = root / f"predictions/{run_id}/real_dev/predictions.csv"
        synthetic = load_predictions(synthetic_path, threshold)
        real = load_predictions(real_path, threshold)
        per_image, counts, ece, bins = counts_and_calibration(gt, synthetic)
        _, real_counts, _, _ = counts_and_calibration(real_gt, real)
        per_run[run_id] = per_image

        run_output = output / run_id
        run_output.mkdir()
        calibration = {
            "schema_version": "durian.v42_calibration_receipt.v1",
            "status": "PASS", "run_id": run_id,
            "checkpoint_sha256": common[run_id]["checkpoint_sha256"],
            "calibration_ece": ece,
            "method": "10 equal-width confidence bins; class-aware greedy IoU>=0.50 correctness; frozen threshold; maxDets=100",
            "cohort": "V4_2_SYNTHETIC_DEV_288", "ground_truth_sha256": GT_SHA,
            "predictions_sha256": sha256(synthetic_path), "bins": bins,
            "sealed_test_accessed": False,
        }
        write_json(run_output / "calibration_receipt.json", calibration)

        class_rows = []
        for class_id, name in enumerate(CLASSES):
            tp, fp, fn = counts[class_id]
            rtp, rfp, rfn = real_counts[class_id]
            class_rows.append({
                "class_id": class_id, "class_name": name,
                "synthetic_tp": tp, "synthetic_fp": fp, "synthetic_fn": fn,
                "synthetic_recall_iou50": tp / (tp + fn) if tp + fn else None,
                "ai_real_reference_tp": rtp, "ai_real_reference_fp": rfp,
                "ai_real_reference_fn": rfn,
                "ai_real_reference_recall_iou50": rtp / (rtp + rfn) if rtp + rfn else None,
            })
        with (run_output / "error_by_class.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=class_rows[0].keys(), lineterminator="\n")
            writer.writeheader(); writer.writerows(class_rows)

        synthetic_map = common[run_id]["synthetic_dev_map50_95"]
        real_map = overlay[run_id]["fyp_ai_reference_moderate_high_map50_95"]
        summary.append({
            "run_id": run_id, "threshold": threshold,
            "synthetic_map50_95": synthetic_map,
            "ai_real_reference_moderate_high_map50_95": real_map,
            "absolute_domain_drop": synthetic_map - real_map,
            "real_to_synthetic_ratio": real_map / synthetic_map if synthetic_map else None,
            "calibration_ece": ece,
            "synthetic_false_negatives": sum(row[2] for row in counts),
            "ai_real_reference_false_negatives": sum(row[2] for row in real_counts),
        })

    trees = sorted(set(image_to_tree.values()))
    rng = random.Random(20261018)
    replicates = 2000
    values = {run_id: [] for run_id in RUNS}
    for _ in range(replicates):
        selected = Counter(trees[rng.randrange(len(trees))] for _ in trees)
        for run_id in RUNS:
            aggregate = [[0, 0, 0] for _ in CLASSES]
            for image_id, counts in per_run[run_id].items():
                multiple = selected[image_to_tree[image_id]]
                for class_id in range(len(CLASSES)):
                    for index in range(3):
                        aggregate[class_id][index] += counts[class_id][index] * multiple
            values[run_id].append(macro_f1(aggregate))

    comparisons = []
    for candidate, baseline, label in (
        ("Run02", "Run01", "Run02_minus_Run01"),
        ("Run04", "Run03", "Run04_minus_Run03"),
        ("Run06", "Run05", "Run06_minus_Run05"),
        ("Run04", "Run02", "Run04_minus_Run02"),
    ):
        delta = [a - b for a, b in zip(values[candidate], values[baseline])]
        comparisons.append({
            "label": label, "candidate": candidate, "baseline": baseline,
            "bootstrap_mean_delta_macro_f1": sum(delta) / len(delta),
            "ci95_low": percentile(delta, .025), "ci95_high": percentile(delta, .975),
            "probability_delta_gt_zero": sum(value > 0 for value in delta) / len(delta),
        })
    write_json(output / "uncertainty_receipt.json", {
        "status": "PASS",
        "metric": "frozen-threshold macro-F1 at class-aware IoU 0.50, maxDets=100",
        "method": "paired percentile bootstrap over 36 physical-tree clusters",
        "replicates": replicates, "seed": 20261018,
        "ground_truth_sha256": GT_SHA, "comparisons": comparisons,
        "limitations": [
            "Synthetic Dev sampling uncertainty only; excludes training-seed, real-domain and annotation uncertainty.",
            "This is macro-F1 uncertainty, not COCO mAP uncertainty.",
            "V4.1 Synthetic versus V4.2 Hybrid deltas are not single-variable causal effects.",
        ],
        "sealed_test_accessed": False,
    })
    with (output / "domain_shift_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary[0].keys(), lineterminator="\n")
        writer.writeheader(); writer.writerows(summary)
    write_json(output / "analysis_receipt.json", {
        "schema_version": "durian.v42_six_model_analysis_receipt.v1",
        "status": "PASS", "registry_sha256": sha256(registry_path),
        "common_comparison_sha256": sha256(common_path),
        "overlay_comparison_sha256": sha256(overlay_path),
        "synthetic_ground_truth_sha256": GT_SHA,
        "ai_reference_sha256": sha256(REAL_GT),
        "ai_reference_is_independent_expert_gt": False,
        "sealed_test_accessed": False,
    })
    print(json.dumps({"status": "PASS", "output": str(output), "runs": 6}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
