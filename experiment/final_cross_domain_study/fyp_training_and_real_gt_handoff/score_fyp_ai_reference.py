#!/usr/bin/env python3
"""Add clearly labelled AI-assisted Real reference scores to a base comparison.

Run the frozen common evaluator without --real-receipt first. This create-only
overlay reuses its Synthetic-Val thresholds and never represents its Real
scores as independent expert ground truth.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import tempfile
from pathlib import Path

import final_common_evaluator as common


def load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict): raise ValueError(f"expected JSON object: {path}")
    return value


def validate_reference_receipt(path: Path) -> tuple[dict, Path]:
    receipt = load(path)
    expected = {
        "status": "PASS_WITH_DECLARED_LIMITATIONS",
        "independent_object_level_verification": False,
        "qualified_expert_review": False,
        "eligible_for_REAL_GT_ELIGIBILITY_RECEIPT": False,
    }
    for key, wanted in expected.items():
        if receipt.get(key) != wanted: raise PermissionError(f"invalid FYP reference receipt: {key} must equal {wanted!r}")
    gt = Path(receipt["coco_path"])
    if not gt.is_absolute(): gt = path.parent / gt
    gt = gt.resolve(strict=True)
    if "sealed" in str(gt).lower() or "08_sealed_test" in gt.parts: raise PermissionError("sealed-test paths are forbidden")
    if common.sha256(gt) != receipt.get("coco_sha256"): raise ValueError("FYP reference COCO hash mismatch")
    return receipt, gt


def freeze(args: argparse.Namespace) -> int:
    output = args.output.resolve()
    if output.exists() and not args.replace: raise FileExistsError(f"refusing to overwrite {output}")
    reference_path = args.reference_receipt.resolve()
    common_path = args.base_evaluator_receipt.resolve()
    policy_path = args.policy.resolve()
    reference, _ = validate_reference_receipt(reference_path)
    common.validate_evaluator_receipt(common_path)
    policy = load(policy_path)
    required = {
        "status": "FROZEN", "final_selection_requires_eligible_real_scoring": False,
        "proposal_ai_assisted_reference_required": True,
        "proposal_ai_assisted_reference_is_independent_expert_gt": False,
        "sealed_test_accessed": False,
    }
    for key, wanted in required.items():
        if policy.get(key) != wanted: raise ValueError(f"invalid FYP policy: {key} must equal {wanted!r}")
    common.write_json(output, {
        "schema_version": "durian.fyp_ai_reference_evaluator.v1",
        "status": "FROZEN",
        "implementation_sha256": common.sha256(Path(__file__)),
        "base_common_evaluator_receipt_sha256": common.sha256(common_path),
        "reference_receipt_sha256": common.sha256(reference_path),
        "reference_coco_sha256": reference["coco_sha256"],
        "inference_manifest_sha256": reference["inference_manifest_sha256"],
        "proposal_policy_sha256": common.sha256(policy_path),
        "threshold_source": "synthetic_val_only",
        "independent_expert_gt": False,
        "sealed_test_accessed": False,
    })
    print(json.dumps({"status": "FROZEN", "output": str(output)}, indent=2))
    return 0


def validate_overlay_receipt(path: Path, reference_path: Path, common_path: Path, policy_path: Path) -> dict:
    receipt = load(path)
    expected = {
        "status": "FROZEN",
        "implementation_sha256": common.sha256(Path(__file__)),
        "base_common_evaluator_receipt_sha256": common.sha256(common_path),
        "reference_receipt_sha256": common.sha256(reference_path),
        "proposal_policy_sha256": common.sha256(policy_path),
        "threshold_source": "synthetic_val_only",
        "independent_expert_gt": False,
        "sealed_test_accessed": False,
    }
    for key, wanted in expected.items():
        if receipt.get(key) != wanted: raise ValueError(f"invalid FYP evaluator receipt: {key} must equal {wanted!r}")
    return receipt


def moderate_high_ground_truth(source: Path, destination: Path) -> tuple[Path, int]:
    document = load(source)
    annotations = [item for item in document.get("annotations", [])
                   if item.get("fyp_reference_confidence") in {"HIGH", "MODERATE"}]
    if len(annotations) != 86:
        raise ValueError(f"expected 86 HIGH/MODERATE reference objects, got {len(annotations)}")
    filtered = dict(document)
    filtered["annotations"] = annotations
    filtered["info"] = {**document.get("info", {}),
                        "confidence_scope": "HIGH_AND_MODERATE_ONLY",
                        "excluded_low_confidence_objects": 39}
    common.write_json(destination, filtered)
    return destination, len(annotations)


def score(args: argparse.Namespace) -> int:
    base_dir = args.base_comparison_dir.resolve(); output = args.output_dir.resolve()
    if output.exists(): raise FileExistsError(f"refusing to overwrite {output}")
    reference_path = args.reference_receipt.resolve()
    common_path = args.base_evaluator_receipt.resolve()
    policy_path = args.policy.resolve()
    common.validate_evaluator_receipt(common_path)
    overlay = validate_overlay_receipt(args.overlay_evaluator_receipt.resolve(), reference_path, common_path, policy_path)
    base = load(base_dir / "comparison.json"); thresholds = load(base_dir / "threshold_receipt.json")
    if base.get("status") != "PASS" or base.get("run_count") != 6: raise ValueError("base comparison must be a complete six-run PASS")
    if base.get("real_scoring_performed") is not False: raise ValueError("use the Synthetic-only base comparison for the FYP overlay")
    receipt, gt = validate_reference_receipt(reference_path)
    if receipt["coco_sha256"] != overlay.get("reference_coco_sha256") or receipt["inference_manifest_sha256"] != overlay.get("inference_manifest_sha256"):
        raise ValueError("FYP reference artifacts drifted after evaluator freeze")
    threshold_by_run = {x["run_id"]: x["threshold"] for x in thresholds.get("selections", [])}
    rows = {x["run_id"]: dict(x) for x in base.get("leaderboard", [])}
    if set(rows) != set(common.RUN_IDS) or set(threshold_by_run) != set(common.RUN_IDS): raise ValueError("base run/threshold coverage mismatch")

    evaluate, _, write_reports, _, _ = common.import_evaluator()
    temporary = Path(tempfile.mkdtemp(prefix="durian-fyp-reference-eval-"))
    staging = output.with_name(f".{output.name}.staging")
    if staging.exists(): raise FileExistsError(f"stale staging directory exists: {staging}")
    staging.mkdir(parents=True)
    try:
        moderate_high_gt, moderate_high_count = moderate_high_ground_truth(gt, temporary / "moderate_high_reference.json")
        for run_id in common.RUN_IDS:
            prediction_set, csv_path, coverage_path = common.prediction_set(args.prediction_root.resolve(), run_id, "real_dev", gt)
            report = evaluate(common.compatible_gt(gt, temporary), prediction_set,
                              confidence_thresholds=(threshold_by_run[run_id],))
            report["dataset"].update(dataset_id="AI_ASSISTED_FYP_REAL_REFERENCE_58",
                                     ground_truth_sha256=common.sha256(gt),
                                     independent_expert_gt=False,
                                     reporting_label="AI_ASSISTED_REFERENCE_DIAGNOSTIC")
            report["prediction_cache"] = {"csv_sha256": common.sha256(csv_path),
                                          "coverage_sha256": common.sha256(coverage_path)}
            write_reports(report, staging / "evaluations" / run_id / "fyp_ai_real_reference_all")
            rows[run_id]["fyp_ai_reference_map50_95"] = report["coco"]["map50_95"]
            moderate_report = evaluate(common.compatible_gt(moderate_high_gt, temporary), prediction_set,
                                       confidence_thresholds=(threshold_by_run[run_id],))
            moderate_report["dataset"].update(dataset_id="AI_ASSISTED_FYP_REAL_REFERENCE_58_MODERATE_HIGH",
                                              ground_truth_sha256=common.sha256(moderate_high_gt),
                                              independent_expert_gt=False,
                                              confidence_scope="HIGH_AND_MODERATE_ONLY",
                                              reporting_label="AI_ASSISTED_REFERENCE_DIAGNOSTIC")
            moderate_report["prediction_cache"] = {"csv_sha256": common.sha256(csv_path),
                                                    "coverage_sha256": common.sha256(coverage_path)}
            write_reports(moderate_report, staging / "evaluations" / run_id / "fyp_ai_real_reference_moderate_high")
            rows[run_id]["fyp_ai_reference_moderate_high_map50_95"] = moderate_report["coco"]["map50_95"]
        leaderboard = [rows[x] for x in common.RUN_IDS]
        leaderboard.sort(key=lambda x: (x["synthetic_dev_map50_95"], x["fyp_ai_reference_moderate_high_map50_95"]), reverse=True)
        comparison = {
            "status": "PASS_WITH_DECLARED_LIMITATIONS", "run_count": 6, "leaderboard": leaderboard,
            "real_scoring_performed": False, "proposal_ai_reference_scoring_performed": True,
            "proposal_ai_reference_is_independent_expert_gt": False,
            "proposal_ai_reference_all_object_count": receipt["counts"]["objects"],
            "proposal_ai_reference_moderate_high_object_count": moderate_high_count,
            "proposal_ai_reference_low_confidence_excluded_from_selection": True,
            "reference_receipt_sha256": common.sha256(reference_path),
            "overlay_evaluator_receipt_sha256": common.sha256(args.overlay_evaluator_receipt.resolve()),
            "proposal_policy_sha256": common.sha256(policy_path),
            "winner": None, "automatic_selection_disabled": True, "sealed_test_accessed": False,
            "decision": "USE_ONLY_WITH_FYP_PROPOSAL_DEPLOYMENT_DECISION_POLICY",
        }
        common.write_json(staging / "comparison.json", comparison)
        common.write_json(staging / "completion_receipt.json", {
            "status": "PASS_WITH_DECLARED_LIMITATIONS", "base_comparison_sha256": common.sha256(base_dir / "comparison.json"),
            "comparison_sha256": common.sha256(staging / "comparison.json"),
            "reference_receipt_sha256": common.sha256(reference_path),
            "reference_coco_sha256": receipt["coco_sha256"], "independent_expert_gt": False,
            "selection_reference_scope": "HIGH_AND_MODERATE_ONLY",
            "selection_reference_object_count": moderate_high_count,
            "overlay_evaluator_receipt_sha256": common.sha256(args.overlay_evaluator_receipt.resolve()),
            "proposal_policy_sha256": common.sha256(policy_path),
            "sealed_test_accessed": False,
        })
        os.replace(staging, output)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True); raise
    finally:
        shutil.rmtree(temporary, ignore_errors=True)
    print(json.dumps({"status": "PASS_WITH_DECLARED_LIMITATIONS", "output": str(output), "runs": 6}, indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    freeze_parser = commands.add_parser("freeze")
    freeze_parser.add_argument("--reference-receipt", type=Path, required=True)
    freeze_parser.add_argument("--base-evaluator-receipt", type=Path, required=True)
    freeze_parser.add_argument("--policy", type=Path, required=True)
    freeze_parser.add_argument("--output", type=Path, required=True)
    freeze_parser.add_argument("--replace", action="store_true")
    freeze_parser.set_defaults(func=freeze)
    score_parser = commands.add_parser("score")
    score_parser.add_argument("--base-comparison-dir", type=Path, required=True)
    score_parser.add_argument("--prediction-root", type=Path, required=True)
    score_parser.add_argument("--reference-receipt", type=Path, required=True)
    score_parser.add_argument("--base-evaluator-receipt", type=Path, required=True)
    score_parser.add_argument("--overlay-evaluator-receipt", type=Path, required=True)
    score_parser.add_argument("--policy", type=Path, required=True)
    score_parser.add_argument("--output-dir", type=Path, required=True)
    score_parser.set_defaults(func=score)
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__": raise SystemExit(main())
