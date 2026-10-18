#!/usr/bin/env python3
"""Create a hash-bound V4.2 six-model registry and pre-score policy.

This is intentionally separate from the historical V4.1 registry.  It does
not run inference, inspect sealed-test data, or alter model checkpoints.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
TRAINING = Path("/home/johny/durian_training")
OLD_REGISTRY = HERE / "post_training_evidence/FINAL_MODEL_REGISTRY.json"
V42 = HERE.parent / "durian_v4_2_8run_ready"
CORRECTION = V42 / "manifests/SOURCE_FAMILY_CORRECTION_V4_2.json"
CLASSES = [
    "leaf_algal", "leaf_blight", "leaf_colletotrichum",
    "leaf_healthy", "leaf_phomopsis", "leaf_rhizoctonia",
]
EXPECTED_BRANCH_HEAD = "d673a31b09543fb4e7aca4bb10bb22b6bfc76bd1"
EXPECTED_CORRECTION_SHA = "b634760affc11ce33f4dcbda5904aa4c482480824a3650b2d70b9add45a062bd"
EXPECTED_ARCHIVE_SHA = "19923e8d08acab1b66de2c0e3b94a56abaabf7051e8d462e929070daf3958834"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def require_file(path: Path, expected_sha: str | None = None) -> dict:
    path = path.resolve(strict=True)
    actual = sha256(path)
    if expected_sha is not None and actual != expected_sha:
        raise RuntimeError(f"SHA-256 mismatch: {path}: {actual} != {expected_sha}")
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": actual}


def yolo_history(path: Path) -> tuple[int, int, float]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    epochs = [int(float(row["epoch"])) for row in rows]
    if epochs != list(range(1, 201)):
        raise RuntimeError(f"non-contiguous E1-E200 history: {path}")
    metric = "metrics/mAP50-95(B)"
    best = max(rows, key=lambda row: float(row[metric]))
    return len(rows), int(float(best["epoch"])), float(best[metric])


def embedded_yolo_identity(path: Path) -> dict:
    from ultralytics import YOLO

    model = YOLO(str(path), task="detect")
    names = [model.names[index] for index in sorted(model.names)]
    if names != CLASSES:
        raise RuntimeError(f"class mapping mismatch in {path}: {names}")
    args = dict(model.ckpt.get("train_args", {}))
    return {
        "task": model.task,
        "class_order": names,
        "embedded_seed": args.get("seed"),
        "embedded_epochs": args.get("epochs"),
        "embedded_imgsz": args.get("imgsz"),
        "embedded_batch": args.get("batch"),
        "embedded_nbs": args.get("nbs"),
        "embedded_patience": args.get("patience"),
        "embedded_dataset_path": args.get("data"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite {output}")

    branch = subprocess.run(
        ["git", "branch", "--show-current"], cwd=TRAINING, check=True,
        text=True, capture_output=True,
    ).stdout.strip()
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=TRAINING, check=True,
        text=True, capture_output=True,
    ).stdout.strip()
    status = subprocess.run(
        ["git", "status", "--porcelain"], cwd=TRAINING, check=True,
        text=True, capture_output=True,
    ).stdout
    if branch != "model-synthetic-and-hybrid" or head != EXPECTED_BRANCH_HEAD or status:
        raise RuntimeError(
            f"training checkout identity/state mismatch: branch={branch} head={head} dirty={bool(status)}"
        )

    if sha256(CORRECTION) != EXPECTED_CORRECTION_SHA:
        raise RuntimeError("authoritative V4.2 source-family correction manifest mismatch")
    correction = json.loads(CORRECTION.read_text(encoding="utf-8"))
    counts = correction["counts"]["hybrid"]
    if counts["train"]["images"] != 1425 or counts["validation_combined"]["images"] != 399:
        raise RuntimeError("V4.2 corrected Hybrid counts mismatch")

    definitions = {
        "Run02": {
            "architecture": "YOLO26n", "model_family": "yolo26n",
            "checkpoint": TRAINING / "models_8run/model2_yolo26n_hybrid/best.pt",
            "checkpoint_sha256": "85cb301538b2c8ccb06b714337391698771afa569efce93c448415e845b428fd",
            "config": TRAINING / "models_8run/model2_yolo26n_hybrid/results/args.yaml",
            "receipt": TRAINING / "models_8run/model2_yolo26n_hybrid/results/e200_backup_manifest.json",
            "history": TRAINING / "models_8run/model2_yolo26n_hybrid/results/results.csv",
            "fairness_label": "V4_2_HYBRID_EFFECTIVE_CONFIG_DEVIATES_FROM_FROZEN_NBS_AND_PATIENCE",
            "initializer_verification": "NOT_VERIFIED_FROM_PUBLISHED_RUN_ARTIFACTS",
            "global_best_verification": True,
            "ema_status": "NOT_APPLICABLE", "native_input_size": 640,
        },
        "Run04": {
            "architecture": "YOLO26n-P2", "model_family": "yolo26n_p2",
            "checkpoint": TRAINING / "models_8run/model4_yolo26n_p2_hybrid/best.pt",
            "checkpoint_sha256": "04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5",
            "config": TRAINING / "models_8run/model4_yolo26n_p2_hybrid/args.yaml",
            "receipt": TRAINING / "models_8run/model4_yolo26n_p2_hybrid/FINAL_AUDIT.json",
            "history": TRAINING / "models_8run/model4_yolo26n_p2_hybrid/results.csv",
            "fairness_label": "V4_2_HYBRID_EFFECTIVE_CONFIG_DOCUMENTED",
            "initializer_verification": "NOT_VERIFIED_FROM_PUBLISHED_RUN_ARTIFACTS",
            "global_best_verification": True,
            "ema_status": "NOT_APPLICABLE", "native_input_size": 640,
        },
        "Run06": {
            "architecture": "RF-DETR Nano", "model_family": "rfdetr_n",
            "checkpoint": TRAINING / "models_8run/model6_rfdetr_n_hybrid/weights/checkpoint_best_ema.pth",
            "checkpoint_sha256": "b333edab9b69d7c85c6405fa468ad767c820adbe12825bcdf661dc5aa069c266",
            "config": V42 / "configs/hybrid/rfdetr_n.json",
            "receipt": TRAINING / "models_8run/model6_rfdetr_n_hybrid/results/JOHNY_E200_AUDIT.json",
            "fairness_label": "V4_2_HYBRID_LAST_RUNTIME_STAGE_BEST_GLOBAL_BEST_UNVERIFIED",
            "initializer_verification": "README_CLAIM_ONLY_NO_MACHINE_BOUND_START_RECEIPT",
            "global_best_verification": False,
            "ema_status": "EMA", "native_input_size": 384,
            "selected_epoch": "E196_E200_RUNTIME_STAGE_BEST_GLOBAL_EPOCH_UNKNOWN",
        },
    }

    audit_runs = {}
    new_rows = {}
    for run_id, definition in definitions.items():
        checkpoint = require_file(definition["checkpoint"], definition["checkpoint_sha256"])
        config = require_file(definition["config"])
        receipt = require_file(definition["receipt"])
        evidence = {
            "checkpoint": checkpoint, "effective_config": config,
            "training_completion_receipt": receipt,
            "initializer_verification": definition["initializer_verification"],
            "global_best_checkpoint_verified": definition["global_best_verification"],
        }
        if run_id in {"Run02", "Run04"}:
            rows, epoch, value = yolo_history(definition["history"])
            evidence.update(
                history=require_file(definition["history"]),
                history_rows=rows, best_epoch=epoch, best_native_val_map50_95=value,
                embedded_checkpoint_identity=embedded_yolo_identity(definition["checkpoint"]),
            )
            definition["selected_epoch"] = epoch
        audit_runs[run_id] = evidence
        new_rows[run_id] = {
            "run_id": run_id,
            "architecture": definition["architecture"],
            "model_family": definition["model_family"],
            "condition": "hybrid_v4_2_corrected",
            "fairness_label": definition["fairness_label"],
            "checkpoint": checkpoint["path"],
            "checkpoint_bytes": checkpoint["bytes"],
            "checkpoint_sha256": checkpoint["sha256"],
            "selected_epoch": definition["selected_epoch"],
            "ema_status": definition["ema_status"],
            "receipt_role": "V4_2_TRAINING_COMPLETION",
            "native_input_size": definition["native_input_size"],
            "checkpoint_selection_rule": "best_validation_map50_95",
            "training_dataset_manifest_sha256": EXPECTED_CORRECTION_SHA,
            "training_dataset_archive_sha256": EXPECTED_ARCHIVE_SHA,
            "config_sha256": config["sha256"],
            "class_order": CLASSES,
            "training_complete_receipt_sha256": receipt["sha256"],
            "initializer_verification": definition["initializer_verification"],
            "global_best_checkpoint_verified": definition["global_best_verification"],
        }

    old = json.loads(OLD_REGISTRY.read_text(encoding="utf-8"))
    retained = {row["run_id"]: row for row in old["runs"] if row["run_id"] in {"Run01", "Run03", "Run05"}}
    for row in retained.values():
        require_file(Path(row["checkpoint"]), row["checkpoint_sha256"])
        row["fairness_label"] = "V4_1_SYNTHETIC_RETAINED_REEVALUATED_ON_COMMON_V4_2_COHORTS"
        row["dataset_version_contrast_limitation"] = True

    registry_rows = []
    for number in range(1, 7):
        run_id = f"Run{number:02d}"
        registry_rows.append(new_rows.get(run_id, retained.get(run_id)))
    if any(row is None for row in registry_rows):
        raise RuntimeError("incomplete six-run registry")

    output.mkdir(parents=True)
    registry = {
        "schema_version": "durian.v42_corrected_six_model_registry.v1",
        "status": "FROZEN_WITH_DECLARED_LINEAGE_LIMITATIONS",
        "checkpoint_selection_rule": "best_validation_map50_95",
        "source_branch": branch,
        "source_branch_commit": head,
        "v42_source_family_correction_sha256": EXPECTED_CORRECTION_SHA,
        "v42_training_archive_sha256": EXPECTED_ARCHIVE_SHA,
        "sealed_test_accessed": False,
        "runs": registry_rows,
    }
    write_json(output / "FINAL_MODEL_REGISTRY_V42.json", registry)

    audit = {
        "schema_version": "durian.v42_artifact_intake_audit.v1",
        "status": "PASS_WITH_DECLARED_LIMITATIONS",
        "verified_branch": branch,
        "verified_commit": head,
        "v42_source_family_correction": require_file(CORRECTION, EXPECTED_CORRECTION_SHA),
        "hybrid_train_images": 1425,
        "hybrid_validation_images": 399,
        "runs": audit_runs,
        "scientific_limitations": [
            "Run01/03/05 are retained V4.1 Synthetic models while Run02/04/06 use corrected V4.2 Hybrid; training-condition contrasts are not single-variable causal estimates.",
            "Run02 effective nbs=64 and patience=200 differ from the canonical frozen nbs=16 and patience=40 recipe.",
            "Run04 effective patience=0 differs from the canonical frozen patience=40 recipe, although all 200 epochs completed.",
            "Published Run02/Run04 artifacts do not machine-bind the original initializer SHA-256.",
            "Run06 has only E196-E200 metrics and its published EMA is not verified as the global E1-E200 best checkpoint.",
            "The AI-assisted Real reference is not independent expert ground truth.",
        ],
        "sealed_test_accessed": False,
    }
    write_json(output / "ARTIFACT_INTEGRITY_AUDIT.json", audit)

    policy = {
        "schema_version": "durian.v42_engineering_deployment_policy.v1",
        "status": "FROZEN",
        "frozen_before_new_v42_common_evaluation": True,
        "frozen_before_sealed_test": True,
        "automatic_hidden_weighted_score": False,
        "final_selection_requires_eligible_real_scoring": False,
        "proposal_ai_assisted_reference_required": True,
        "proposal_ai_assisted_reference_is_independent_expert_gt": False,
        "candidate_runs": [f"Run{i:02d}" for i in range(1, 7)],
        "primary_selection_runs": [f"Run{i:02d}" for i in range(1, 7)],
        "eligibility": {
            "ros2_end_to_end_pass": True,
            "non_finite_output_count_max": 0,
            "coordinate_violation_count_max": 0,
            "repeated_inference_mismatch_count_max": 0,
        },
        "lexicographic_order": [
            {"metric": "synthetic_dev_map50_95", "direction": "descending"},
            {"metric": "fyp_ai_reference_moderate_high_map50_95", "direction": "descending"},
            {"metric": "perturbation_retention", "direction": "descending"},
            {"metric": "calibration_ece", "direction": "ascending"},
            {"metric": "cpu_p90_latency_ms", "direction": "ascending"},
            {"metric": "peak_rss_mb", "direction": "ascending"},
        ],
        "interpretation": "Engineering deployment ranking across the six exact checkpoints. It is not a clean causal estimate of Hybrid versus Synthetic training because dataset versions and effective recipes differ. AI-assisted Real reference scores are secondary diagnostics, not expert-verified Real accuracy.",
        "tie_policy": "An exact declared-metric tie remains unresolved.",
        "sealed_test_accessed": False,
    }
    write_json(output / "V42_ENGINEERING_DEPLOYMENT_POLICY.json", policy)
    print(json.dumps({"status": "PREPARED", "output": str(output), "runs": 6}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
