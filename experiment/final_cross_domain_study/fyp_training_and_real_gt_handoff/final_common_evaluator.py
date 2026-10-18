#!/usr/bin/env python3
"""Hash-bound, model-agnostic final comparison over canonical predictions.

The tool never runs model inference and never accesses sealed-test data.  All
backends must first export the Phase2A canonical CSV + coverage sidecar.  Real
scoring is impossible without an independently verified eligibility receipt.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
PHASE2A = WORKSPACE / "experiment/analysis/dev_validation_audit/phase2a"
V42 = HERE.parent / "durian_v4_2_8run_ready"
SYNTHETIC_VAL_GT = V42 / "datasets/coco/synthetic_only/synthetic_val/_annotations.coco.json"
SYNTHETIC_DEV_GT = V42 / "datasets/coco/synthetic_only/synthetic_dev/_annotations.coco.json"
EXPECTED_VAL_SHA = "9c3af6d86b305d8317d359b07d558cb47fe257dcfdef6de325905b28b638e69d"
EXPECTED_DEV_SHA = "d841970d5d7227ed830cfec72fb8605f0cd488b3fd14469891da5c4133c6f703"
RUN_IDS = tuple(f"Run{i:02d}" for i in range(1, 7))
CLASSES = (
    "leaf_algal", "leaf_blight", "leaf_colletotrichum",
    "leaf_healthy", "leaf_phomopsis", "leaf_rhizoctonia",
)
THRESHOLDS = tuple(round(value / 100, 2) for value in range(5, 100, 5))


def runtime_identity() -> dict[str, str]:
    return {"python": sys.version.split()[0], "pycocotools": importlib.metadata.version("pycocotools"),
            "numpy": importlib.metadata.version("numpy")}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def import_evaluator():
    sys.path.insert(0, str(PHASE2A))
    from unified_eval.coco_evaluator import evaluate, operating_point_metrics, write_reports
    from unified_eval.contract import image_specs_from_coco, read_prediction_set
    return evaluate, operating_point_metrics, write_reports, image_specs_from_coco, read_prediction_set


def verify_synthetic_identities() -> None:
    actual_val, actual_dev = sha256(SYNTHETIC_VAL_GT), sha256(SYNTHETIC_DEV_GT)
    if actual_val != EXPECTED_VAL_SHA or actual_dev != EXPECTED_DEV_SHA:
        raise RuntimeError(f"V4.2 Synthetic GT identity mismatch: val={actual_val}, dev={actual_dev}")


def validate_registry(path: Path) -> list[dict[str, Any]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    rows = document.get("runs")
    if not isinstance(rows, list) or tuple(row.get("run_id") for row in rows) != RUN_IDS:
        raise ValueError("registry runs must be ordered exactly Run01..Run06")
    required = {
        "run_id", "architecture", "condition", "fairness_label", "checkpoint",
        "checkpoint_bytes", "checkpoint_sha256", "selected_epoch", "ema_status",
        "training_dataset_manifest_sha256", "config_sha256", "class_order",
        "native_input_size", "checkpoint_selection_rule", "training_complete_receipt_sha256",
    }
    for row in rows:
        missing = sorted(required - set(row))
        if missing:
            raise ValueError(f"{row.get('run_id')}: missing registry fields {missing}")
        if tuple(row["class_order"]) != CLASSES:
            raise ValueError(f"{row['run_id']}: canonical class order mismatch")
        if row["checkpoint_selection_rule"] not in ("best_validation_map50_95", "best validation mAP50-95"):
            raise ValueError(f"{row['run_id']}: invalid checkpoint selection rule")
        checkpoint = Path(row["checkpoint"])
        if not checkpoint.is_absolute():
            checkpoint = path.parent / checkpoint
        if not checkpoint.is_file():
            raise FileNotFoundError(f"{row['run_id']}: missing checkpoint {checkpoint}")
        if checkpoint.stat().st_size != row["checkpoint_bytes"] or sha256(checkpoint) != row["checkpoint_sha256"]:
            raise ValueError(f"{row['run_id']}: checkpoint byte/hash mismatch")
    return rows


def validate_real_receipt(path: Path) -> Path:
    receipt = json.loads(path.read_text(encoding="utf-8"))
    expected = {
        "status": "PASS", "independent_object_level_verification": True,
        "geometry_qc_pass": True, "source_family_isolation_pass": True,
    }
    for key, wanted in expected.items():
        if receipt.get(key) != wanted:
            raise PermissionError(f"Real scoring blocked: {key} must equal {wanted!r}")
    gt = Path(receipt["ground_truth_path"])
    if not gt.is_absolute():
        gt = path.parent / gt
    gt = gt.resolve(strict=True)
    if "sealed" in str(gt).lower() or "08_sealed_test" in gt.parts:
        raise PermissionError("sealed-test paths are categorically forbidden")
    if sha256(gt) != receipt.get("ground_truth_sha256"):
        raise ValueError("eligible Real GT hash mismatch")
    return gt


def validate_evaluator_receipt(path: Path) -> dict[str, Any]:
    receipt = json.loads(path.read_text(encoding="utf-8"))
    expected = {
        "status": "FROZEN",
        "semantic_class_ids": list(range(6)),
        "coco_category_ids": list(range(1, 7)),
        "max_detections_per_image": 100,
        "threshold_source": "synthetic_val_only",
        "prediction_cache_required": True,
        "synthetic_val_ground_truth_sha256": EXPECTED_VAL_SHA,
        "synthetic_dev_ground_truth_sha256": EXPECTED_DEV_SHA,
        "real_scoring_gate": "independent_object_level_gt_eligibility_receipt_required",
    }
    for key, wanted in expected.items():
        if receipt.get(key) != wanted:
            raise ValueError(f"invalid common evaluator receipt: {key} must equal {wanted!r}")
    implementation = receipt.get("implementation", {})
    current = {
        "orchestrator_sha256": sha256(Path(__file__)),
        "contract_sha256": sha256(PHASE2A / "unified_eval/contract.py"),
        "coco_evaluator_sha256": sha256(PHASE2A / "unified_eval/coco_evaluator.py"),
    }
    if implementation != current:
        raise ValueError("common evaluator implementation hash mismatch; freeze a new receipt")
    if receipt.get("runtime") != runtime_identity():
        raise ValueError("common evaluator runtime identity mismatch; use the frozen Phase2A environment")
    return receipt


def prediction_set(root: Path, run_id: str, split: str, gt: Path):
    _, _, _, image_specs_from_coco, read_prediction_set = import_evaluator()
    document = json.loads(gt.read_text(encoding="utf-8"))
    specs = image_specs_from_coco(document)
    directory = root / run_id / split
    predictions = directory / "predictions.csv"
    coverage = directory / "coverage.json"
    result = read_prediction_set(predictions, coverage, specs)
    return result, predictions, coverage


def compatible_gt(original: Path, temporary_root: Path) -> Path:
    """Add optional COCO info in a temporary copy for pycocotools 2.0.10."""
    document = json.loads(original.read_text(encoding="utf-8"))
    document.setdefault("info", {})
    target = temporary_root / f"{original.parent.name}.json"
    write_json(target, document)
    return target


def freeze(args: argparse.Namespace) -> None:
    verify_synthetic_identities()
    output = args.output.resolve()
    if output.exists() and not args.replace:
        raise FileExistsError(f"refusing to overwrite {output}")
    # Import proves that the pinned CPU evaluator dependencies are usable.
    import_evaluator()
    receipt = {
        "schema_version": "durian.common_evaluator.v1",
        "status": "FROZEN",
        "semantic_class_ids": list(range(6)),
        "coco_category_ids": list(range(1, 7)),
        "canonical_classes": list(CLASSES),
        "coordinates": "original_image_pixels",
        "confidence_export_floor": 0.001,
        "max_detections_per_image": 100,
        "threshold_source": "synthetic_val_only",
        "threshold_candidates": list(THRESHOLDS),
        "threshold_objective": "unweighted_six_class_macro_f1_iou_0.50",
        "threshold_tie_break": "highest_threshold",
        "prediction_cache_required": True,
        "synthetic_val_ground_truth_sha256": EXPECTED_VAL_SHA,
        "synthetic_dev_ground_truth_sha256": EXPECTED_DEV_SHA,
        "real_scoring_gate": "independent_object_level_gt_eligibility_receipt_required",
        "sealed_test_accessed": False,
        "runtime": runtime_identity(),
        "implementation": {
            "orchestrator_sha256": sha256(Path(__file__)),
            "contract_sha256": sha256(PHASE2A / "unified_eval/contract.py"),
            "coco_evaluator_sha256": sha256(PHASE2A / "unified_eval/coco_evaluator.py"),
        },
    }
    write_json(output, receipt)
    print(json.dumps({"status": "FROZEN", "output": str(output)}, indent=2))


def compare(args: argparse.Namespace) -> None:
    verify_synthetic_identities()
    validate_evaluator_receipt(args.evaluator_receipt.resolve())
    rows = validate_registry(args.registry.resolve())
    prediction_root = args.prediction_root.resolve()
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite {output}")
    real_gt = validate_real_receipt(args.real_receipt.resolve()) if args.real_receipt else None
    evaluate, operating_point_metrics, write_reports, image_specs_from_coco, _ = import_evaluator()
    temporary = Path(tempfile.mkdtemp(prefix="durian-common-eval-"))
    staging = output.with_name(f".{output.name}.staging")
    if staging.exists():
        raise FileExistsError(f"stale staging directory exists: {staging}")
    staging.mkdir(parents=True)
    try:
        val_document = json.loads(SYNTHETIC_VAL_GT.read_text(encoding="utf-8"))
        threshold_rows, leaderboard = [], []
        for row in rows:
            run_id = row["run_id"]
            val_set, val_csv, _ = prediction_set(prediction_root, run_id, "synthetic_val", SYNTHETIC_VAL_GT)
            table = [operating_point_metrics(val_document, val_set.predictions,
                                             confidence_threshold=value, max_detections_per_image=100)
                     for value in THRESHOLDS]
            defined = [item for item in table if item["macro"]["f1"] is not None]
            if not defined:
                raise RuntimeError(f"{run_id}: no defined six-class macro-F1 threshold")
            selected = max(defined, key=lambda item: (item["macro"]["f1"], item["confidence_threshold"]))
            threshold_rows.append({"run_id": run_id, "checkpoint_sha256": row["checkpoint_sha256"],
                                   "prediction_sha256": sha256(val_csv),
                                   "threshold": selected["confidence_threshold"],
                                   "validation_macro_f1": selected["macro"]["f1"]})

            dev_set, dev_csv, dev_coverage = prediction_set(prediction_root, run_id, "synthetic_dev", SYNTHETIC_DEV_GT)
            dev_report = evaluate(compatible_gt(SYNTHETIC_DEV_GT, temporary), dev_set,
                                  confidence_thresholds=(selected["confidence_threshold"],))
            dev_report["dataset"].update(dataset_id="V4_2_SYNTHETIC_DEV_288",
                                         ground_truth_sha256=EXPECTED_DEV_SHA)
            dev_report["prediction_cache"] = {"csv_sha256": sha256(dev_csv),
                                               "coverage_sha256": sha256(dev_coverage)}
            write_reports(dev_report, staging / "evaluations" / run_id / "synthetic_dev")
            result = {"run_id": run_id, "architecture": row["architecture"],
                      "condition": row["condition"], "fairness_label": row["fairness_label"],
                      "checkpoint_sha256": row["checkpoint_sha256"],
                      "threshold": selected["confidence_threshold"],
                      "synthetic_dev_map50_95": dev_report["coco"]["map50_95"],
                      "real_dev_map50_95": None}
            if real_gt:
                real_set, real_csv, real_coverage = prediction_set(prediction_root, run_id, "real_dev", real_gt)
                real_report = evaluate(compatible_gt(real_gt, temporary), real_set,
                                       confidence_thresholds=(selected["confidence_threshold"],))
                real_report["dataset"].update(dataset_id="ELIGIBLE_REAL_DEV",
                                               ground_truth_sha256=sha256(real_gt))
                real_report["prediction_cache"] = {"csv_sha256": sha256(real_csv),
                                                    "coverage_sha256": sha256(real_coverage)}
                write_reports(real_report, staging / "evaluations" / run_id / "real_dev")
                result["real_dev_map50_95"] = real_report["coco"]["map50_95"]
            leaderboard.append(result)

        write_json(staging / "threshold_receipt.json", {
            "status": "FROZEN", "source": "V4_2_SYNTHETIC_VAL_288",
            "ground_truth_sha256": EXPECTED_VAL_SHA, "selections": threshold_rows,
            "real_predictions_or_metrics_accessed": False,
        })
        leaderboard.sort(key=lambda item: item["synthetic_dev_map50_95"], reverse=True)
        write_json(staging / "comparison.json", {
            "status": "PASS", "run_count": 6, "leaderboard": leaderboard,
            "winner": None, "deployment_run": None,
            "automatic_selection_disabled": True,
            "real_scoring_performed": real_gt is not None,
            "sealed_test_accessed": False,
            "decision": "REQUIRES_FROZEN_TRANSPARENT_DEPLOYMENT_DECISION_MATRIX",
        })
        with (staging / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(leaderboard[0]))
            writer.writeheader(); writer.writerows(leaderboard)
        receipt = {"status": "PASS", "registry_sha256": sha256(args.registry.resolve()),
                   "common_evaluator_receipt_sha256": sha256(args.evaluator_receipt.resolve()),
                   "real_gt_receipt_sha256": sha256(args.real_receipt.resolve()) if args.real_receipt else None,
                   "comparison_sha256": sha256(staging / "comparison.json"),
                   "threshold_receipt_sha256": sha256(staging / "threshold_receipt.json"),
                   "automatic_winner_selected": False, "sealed_test_accessed": False}
        write_json(staging / "completion_receipt.json", receipt)
        os.replace(staging, output)
        print(json.dumps({"status": "PASS", "output": str(output),
                          "real_scoring_performed": real_gt is not None}, indent=2))
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    finally:
        shutil.rmtree(temporary, ignore_errors=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    freeze_parser = commands.add_parser("freeze")
    freeze_parser.add_argument("--output", type=Path, required=True)
    freeze_parser.add_argument("--replace", action="store_true")
    freeze_parser.set_defaults(func=freeze)
    compare_parser = commands.add_parser("compare")
    compare_parser.add_argument("--registry", type=Path, required=True)
    compare_parser.add_argument("--evaluator-receipt", type=Path, required=True)
    compare_parser.add_argument("--prediction-root", type=Path, required=True)
    compare_parser.add_argument("--output-dir", type=Path, required=True)
    compare_parser.add_argument("--real-receipt", type=Path)
    compare_parser.set_defaults(func=compare)
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
