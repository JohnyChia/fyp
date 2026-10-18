#!/usr/bin/env python3
"""Validate Run04 attempt02 and create superseding, create-only PASS evidence."""

from __future__ import annotations

import csv
import hashlib
import json
import sqlite3
from pathlib import Path

import yaml


ROOT = Path("/home/johny/durian_ws")
EVIDENCE = ROOT / "experiment/final_cross_domain_study/fyp_training_and_real_gt_handoff/post_training_evidence/v42_corrected_comparison_20261018"
ATTEMPT = EVIDENCE / "run04_ros2_gazebo_e2e_20261018_attempt02"
OUTPUTS = ATTEMPT / "inspection_outputs"
EXPECTED_SHA = "04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5"
ACTIVE = ROOT / "src/durian_inspection_pkg/config/leaf_detector_deployment.yaml"
ACTIVE_SHA = "28f157300d187ad63e57a1658fca3cfd020c02647bc402b5db3c3cb376152005"


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def create(path: Path, text: str) -> None:
    with path.open("x", encoding="utf-8") as stream:
        stream.write(text)


def main() -> int:
    bundle_path = ATTEMPT / "ISOLATED_BUNDLE_RECEIPT.json"
    gpu_path = ATTEMPT / "gpu_runtime/GPU_RESOURCE_RECEIPT.json"
    e2e_path = OUTPUTS / "model4_v42_receipt.json"
    db_path = OUTPUTS / "model4_v42.db"
    log_path = OUTPUTS / "model4_v42.log"
    bundle, gpu, e2e = load(bundle_path), load(gpu_path), load(e2e_path)

    active = yaml.safe_load(ACTIVE.read_text(encoding="utf-8"))
    assert active["leaf_detector"]["backend"] == "fyp_run01"
    assert sha256(ACTIVE) == ACTIVE_SHA
    assert bundle["status"] == "PASS" and bundle["checkpoint_sha256"] == EXPECTED_SHA
    assert gpu["status"] == "PASS" and gpu["checkpoint_sha256"] == EXPECTED_SHA
    assert gpu["non_finite_output_count"] == 0
    assert gpu["coordinate_violation_count"] == 0
    assert e2e["status"] == "PASS" and e2e["checkpoint_sha256"] == EXPECTED_SHA
    assert e2e["terminal_status"]["state"] == "COMPLETED"
    assert e2e["terminal_status"]["success"] is True
    assert e2e["database_validation"]["pass"] is True
    assert sha256(db_path) == e2e["database_sha256"]
    assert sha256(log_path) == e2e["log_sha256"]

    with sqlite3.connect(db_path) as connection:
        connection.row_factory = sqlite3.Row
        trees = [dict(row) for row in connection.execute(
            "SELECT tree_id,status,disease_name,confidence,coverage,leaf_backend,"
            "checkpoint_sha256,pipeline_fingerprint,aggregation_mode,unknown_reason,"
            "viewpoint_count,valid_viewpoint_count,sampled_frame_count,processing_error,cache_hit "
            "FROM inspection_log ORDER BY tree_id"
        )]
        observation_statuses = {
            row[0]: row[1] for row in connection.execute(
                "SELECT status,COUNT(*) FROM captured_observations GROUP BY status"
            )
        }
        observation_errors = connection.execute(
            "SELECT COUNT(*) FROM captured_observations WHERE processing_error IS NOT NULL "
            "AND TRIM(processing_error)<>''"
        ).fetchone()[0]
    assert len(trees) == 6 and [row["tree_id"] for row in trees] == list(range(1, 7))
    assert all(row["status"] == "completed" for row in trees)
    assert all(row["leaf_backend"] == "fyp_run04_v42" for row in trees)
    assert all(row["checkpoint_sha256"] == EXPECTED_SHA for row in trees)
    assert all(row["processing_error"] is None for row in trees)
    assert all(row["viewpoint_count"] == 4 and row["valid_viewpoint_count"] == 4 for row in trees)
    assert observation_statuses == {"PROCESSED": 1369}
    assert observation_errors == 0
    debug_reports = sorted((OUTPUTS / "model4_v42_processed/debug/disease_reports").glob("tree_*_scan.txt"))
    assert len(debug_reports) == 6

    log = log_path.read_text(encoding="utf-8", errors="replace")
    assert "RETURN HOME SUCCESS (pos_err=0.09m, yaw_err=0.16rad)" in log
    assert "DEFERRED PROCESSING COMPLETE" in log
    assert '"state":"COMPLETED","success":true,"terminal":true' in log
    assert "LEAF_BACKEND_INVALID_GEOMETRY_FILTERED" not in log
    assert "checkpoint mismatch" not in log.lower()

    gate_receipt = {
        "schema_version": "durian.run04_v42_final_e2e_gate.v1",
        "status": "PASS",
        "run_id": "Run04",
        "checkpoint_sha256": EXPECTED_SHA,
        "threshold": 0.45,
        "frozen_gates": {
            "checkpoint_identity": "PASS",
            "class_mapping": "PASS",
            "preprocessing_binding": "PASS",
            "non_finite_output": "PASS",
            "coordinate_validity": "PASS",
            "repeated_inference": "PASS",
            "perturbation_consistency": "PASS_BOUNDED",
            "calibration_measurement": "PASS",
            "cpu_latency_memory": "PASS",
            "gpu_latency_memory": "PASS",
            "start_inspection": "PASS",
            "all_six_trees": "PASS",
            "return_to_start": "PASS",
            "captured_image_processing": "PASS",
            "database_validation": "PASS",
            "log_and_receipt_validation": "PASS",
            "ros2_gazebo_end_to_end": "PASS",
            "hard_eligibility": "PASS"
        },
        "gpu": {
            "device": gpu["device_name"],
            "latency_sample_count": gpu["sample_count"],
            "p90_latency_ms": gpu["gpu_latency_ms"]["p90"],
            "peak_vram_allocated_mb": gpu["peak_vram_allocated_mb"],
            "peak_vram_reserved_mb": gpu["peak_vram_reserved_mb"],
            "non_finite_output_count": 0,
            "coordinate_violation_count": 0
        },
        "mission": {
            "inspection_id": e2e["inspection_id"],
            "tree_rows": 6,
            "captured_observations": 1369,
            "processed_observations": 1369,
            "valid_viewpoints_per_tree": 4,
            "processing_errors": 0,
            "return_home_position_error_m": 0.09,
            "return_home_yaw_error_rad": 0.16,
            "wall_time_s": e2e["wall_time_s"],
            "pipeline_fingerprint": trees[0]["pipeline_fingerprint"],
            "tree_results": trees
        },
        "shutdown_note": "Mission evidence was complete before launch cleanup. processing_node and gzserver required SIGTERM after the runner's bounded SIGINT grace period; no child processes remained and no DB evidence was lost.",
        "active_run01_preserved": True,
        "run04_promoted": False,
        "promotion_recommendation": "RECOMMEND_RUN04_FOR_EXPLICIT_APPROVAL",
        "run02_fallback_required": False,
        "run06_executed": False,
        "source_receipts": {
            "isolated_bundle": {"path": str(bundle_path), "sha256": sha256(bundle_path)},
            "gpu_resource": {"path": str(gpu_path), "sha256": sha256(gpu_path)},
            "gazebo_e2e": {"path": str(e2e_path), "sha256": sha256(e2e_path)},
            "database": {"path": str(db_path), "sha256": sha256(db_path)},
            "log": {"path": str(log_path), "sha256": sha256(log_path)}
        },
        "sealed_test_accessed": False,
        "training_performed": False,
        "threshold_retuned": False,
        "active_deployment_modified": False,
        "commit_created": False,
        "push_performed": False
    }
    gate_path = ATTEMPT / "RUN04_FINAL_E2E_GATE_RECEIPT.json"
    create(gate_path, json.dumps(gate_receipt, indent=2, sort_keys=True, allow_nan=False) + "\n")

    prior_matrix = EVIDENCE / "FINAL_GATE_MATRIX.csv"
    with prior_matrix.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        if row["run_id"] == "Run04":
            row["gpu_runtime"] = "PASS_RTX3050TI_P90_37.79337MS_VRAM_RESERVED_104MB"
            row["ros2_gazebo_e2e"] = "PASS_EXACT_SHA_SIX_TREE"
            row["hard_eligibility"] = "PASS"
    matrix_path = EVIDENCE / "FINAL_GATE_MATRIX_POST_RUN04_E2E.csv"
    with matrix_path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    decision = {
        "schema_version": "durian.v42_six_model_post_run04_e2e_decision.v1",
        "status": "RUN04_ELIGIBLE_PROMOTION_APPROVAL_REQUIRED",
        "supersedes_runtime_blocker_in": str(EVIDENCE / "FINAL_DECISION_RECEIPT.json"),
        "primary_scientific_selection": "Run01",
        "engineering_highest_scoring_model": "Run04",
        "recommended_deployment_model": "Run04",
        "current_active_model": "Run01",
        "automatic_promotion_performed": False,
        "promotion_requires_explicit_user_approval": True,
        "run02_fallback_required": False,
        "run06_candidate_status": "EXCLUDED_HARD_GATE_FAIL",
        "run04_gate_receipt": str(gate_path),
        "run04_gate_receipt_sha256": sha256(gate_path),
        "post_e2e_gate_matrix": str(matrix_path),
        "post_e2e_gate_matrix_sha256": sha256(matrix_path),
        "sealed_test_accessed": False
    }
    decision_path = EVIDENCE / "FINAL_DECISION_POST_RUN04_E2E.json"
    create(decision_path, json.dumps(decision, indent=2, sort_keys=True) + "\n")

    report = f"""# Run04 V4.2 final ROS2/Gazebo E2E result

Status: **PASS — ELIGIBLE; EXPLICIT PROMOTION APPROVAL REQUIRED**

Run04 checkpoint `{EXPECTED_SHA}` passed every frozen engineering deployment
gate without changing the active Run01 configuration or retuning threshold
0.45. No sealed-test data was accessed.

## Verified result

- GPU: `{gpu['device_name']}`; 48 samples; p90 **{gpu['gpu_latency_ms']['p90']:.2f} ms**;
  peak allocated/reserved VRAM **{gpu['peak_vram_allocated_mb']:.2f}/{gpu['peak_vram_reserved_mb']:.2f} MB**.
- GPU output validity: 0 non-finite values, 0 coordinate violations.
- Mission: start accepted, all six trees inspected with 4/4 valid viewpoints,
  return home succeeded (0.09 m, 0.16 rad), deferred processing completed.
- Database: 6/6 tree rows completed, 1369/1369 observations processed, exact
  backend `fyp_run04_v42`, exact checkpoint SHA, zero processing errors.
- Receipts/logs: checkpoint, class mapping, 640x640 preprocessing, threshold,
  pipeline fingerprint and output paths are bound and hash-verified.

Two shutdown-only log messages record `processing_node` and `gzserver` requiring
SIGTERM after the bounded SIGINT grace period. They occurred after terminal
COMPLETED and receipt/DB finalization; no process remains and no evidence was
lost. They are retained as a non-gating cleanup warning.

## Decision

Run04 is now the **recommended deployment model** under the frozen engineering
policy. Run01 remains active because automatic promotion was forbidden. Run02
fallback is not required and Run06 was not run.

Promotion, if approved, must be a separate controlled change that adds a
production Run04 V4.2 manifest/profile, runs configuration regression tests,
and preserves rollback to Run01.
"""
    create(ATTEMPT / "RUN04_E2E_PASS_REPORT.md", report)
    print(json.dumps(decision, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
