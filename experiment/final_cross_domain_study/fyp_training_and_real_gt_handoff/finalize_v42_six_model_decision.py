#!/usr/bin/env python3
"""Consolidate the verified V4.2 six-model evidence without re-evaluation.

This script is deliberately limited to reading existing receipts and writing a
decision matrix/report under the V4.2 evidence directory.  It never opens a
sealed-test cohort, changes a checkpoint, or changes the active ROS2 profile.
"""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path


WORKSPACE = Path("/home/johny/durian_ws")
HANDOFF = WORKSPACE / "experiment/final_cross_domain_study/fyp_training_and_real_gt_handoff"
ROOT = HANDOFF / "post_training_evidence/v42_corrected_comparison_20261018"
OLD_AUDIT = HANDOFF / "post_training_evidence/deployment_audit_20261013"


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> None:
    registry_path = ROOT / "FINAL_MODEL_REGISTRY_V42.json"
    integrity_path = ROOT / "ARTIFACT_INTEGRITY_AUDIT.json"
    policy_path = ROOT / "V42_ENGINEERING_DEPLOYMENT_POLICY.json"
    common_path = ROOT / "common_comparison_v1/comparison.json"
    real_path = ROOT / "fyp_ai_reference_comparison_v1/comparison.json"
    domain_path = ROOT / "analysis/domain_shift_summary.csv"
    uncertainty_path = ROOT / "analysis/uncertainty_receipt.json"
    prior_status_path = OLD_AUDIT / "POST_FIX_CURRENT_DEPLOYMENT_STATUS.json"

    registry = load_json(registry_path)
    integrity = load_json(integrity_path)
    policy = load_json(policy_path)
    common = load_json(common_path)
    real = load_json(real_path)
    uncertainty = load_json(uncertainty_path)
    prior_status = load_json(prior_status_path)

    assert policy["sealed_test_accessed"] is False
    assert common["sealed_test_accessed"] is False
    assert real["sealed_test_accessed"] is False
    assert uncertainty["sealed_test_accessed"] is False
    assert integrity["verified_commit"] == "d673a31b09543fb4e7aca4bb10bb22b6bfc76bd1"

    common_by_run = {row["run_id"]: row for row in common["leaderboard"]}
    real_by_run = {row["run_id"]: row for row in real["leaderboard"]}
    with domain_path.open(newline="", encoding="utf-8") as stream:
        domain_by_run = {row["run_id"]: row for row in csv.DictReader(stream)}

    old_gate_path = OLD_AUDIT / "gate_matrix.csv"
    with old_gate_path.open(newline="", encoding="utf-8") as stream:
        old_gate = {row["run_id"]: row for row in csv.DictReader(stream)}

    rows = []
    for run_id in [f"Run{i:02d}" for i in range(1, 7)]:
        common_row = common_by_run[run_id]
        real_row = real_by_run[run_id]
        domain_row = domain_by_run[run_id]

        if run_id in {"Run02", "Run04", "Run06"}:
            runtime_dir = ROOT / "deployment_audit" / run_id / (
                "runtime_serial" if run_id in {"Run02", "Run04"} else "runtime"
            )
            consistency = load_json(runtime_dir / "consistency_receipt.json")
            resource = load_json(runtime_dir / "resource_receipt.json")
            checkpoint_identity = (
                "FAIL_GLOBAL_BEST_UNVERIFIED"
                if run_id == "Run06"
                else "PASS_WITH_TRAINING_RECEIPT_LIMITATIONS"
            )
            non_finite = "PASS" if resource["non_finite_output_count"] == 0 else "FAIL"
            coordinate = "PASS" if resource["coordinate_violation_count"] == 0 else "FAIL"
            repeated = (
                "PASS" if consistency["repeated_inference_mismatch_count"] == 0 else "FAIL"
            )
            perturbation = float(consistency["perturbation_retention"])
            cpu_p90 = float(resource["cpu_p90_latency_ms"])
            peak_rss = float(resource["peak_rss_mb"])
            gpu_gate = "BLOCKED_OS_CUDA_UNAVAILABLE"
            ros2_gate = "BLOCKED_NEW_SHA_NOT_E2E_TESTED"
        else:
            prior = old_gate[run_id]
            checkpoint_identity = "PASS_RETAINED_EXACT_SHA"
            non_finite = prior["non_finite_output"]
            coordinate = prior["coordinate_validity"]
            repeated = prior["repeated_inference"]
            perturbation = float(prior["perturbation_retention"])
            cpu_p90 = float(prior["cpu_p90_latency_ms"])
            peak_rss = float(prior["peak_rss_mb"])
            gpu_gate = "PASS_HISTORICAL_TESTED_WORKSTATION_SUMMARY"
            ros2_gate = "PASS_HISTORICAL_TESTED_WORKSTATION_SUMMARY"

        if run_id == "Run06" or "FAIL" in {non_finite, coordinate, repeated}:
            eligibility = "FAIL"
        elif ros2_gate.startswith("BLOCKED"):
            eligibility = "BLOCKED"
        else:
            eligibility = "PASS"

        rows.append(
            {
                "run_id": run_id,
                "architecture": common_row["architecture"],
                "training_condition": common_row["condition"],
                "checkpoint_sha256": common_row["checkpoint_sha256"],
                "threshold": common_row["threshold"],
                "synthetic_dev_map50_95": common_row["synthetic_dev_map50_95"],
                "ai_real_moderate_high_map50_95": real_row[
                    "fyp_ai_reference_moderate_high_map50_95"
                ],
                "calibration_ece": float(domain_row["calibration_ece"]),
                "perturbation_retention": perturbation,
                "cpu_p90_latency_ms": cpu_p90,
                "peak_rss_mb": peak_rss,
                "checkpoint_identity": checkpoint_identity,
                "non_finite_output": non_finite,
                "coordinate_validity": coordinate,
                "repeated_inference": repeated,
                "gpu_runtime": gpu_gate,
                "ros2_gazebo_e2e": ros2_gate,
                "hard_eligibility": eligibility,
            }
        )

    csv_path = ROOT / "FINAL_GATE_MATRIX.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    source_receipts = {
        str(path.relative_to(WORKSPACE)): sha256(path)
        for path in [
            registry_path,
            integrity_path,
            policy_path,
            common_path,
            real_path,
            domain_path,
            uncertainty_path,
            prior_status_path,
            old_gate_path,
        ]
    }
    raw_e2e_root = (
        HANDOFF
        / "post_training_evidence/gazebo_six_model_inspections_20261013"
        / "SIX_MODEL_INSPECTION_SUMMARY.json"
    )
    decision = {
        "schema_version": "durian.v42_six_model_final_decision.v1",
        "status": "COMPARISON_COMPLETE_DEPLOYMENT_SWITCH_BLOCKED",
        "primary_scientific_selection": {
            "run_id": "Run01",
            "meaning": "Original frozen V4.1 Synthetic-control primary scope",
        },
        "engineering_highest_scoring_model": {
            "run_id": "Run04",
            "status": "PROVISIONAL_PENDING_EXACT_SHA_ROS2_GAZEBO_E2E",
        },
        "ai_assisted_real_reference_leader": {
            "run_id": "Run02",
            "margin_over_run04_map50_95": real_by_run["Run02"][
                "fyp_ai_reference_moderate_high_map50_95"
            ]
            - real_by_run["Run04"]["fyp_ai_reference_moderate_high_map50_95"],
            "ground_truth_status": "NOT_INDEPENDENT_EXPERT_GT",
        },
        "current_deployable_model": {
            "run_id": "Run01",
            "status": "GO_ONLY_ON_PREVIOUSLY_TESTED_WORKSTATION",
        },
        "deployment_switch_decision": "NO_GO_BLOCKED",
        "deployment_switch_target": "Run04",
        "blockers": [
            "Run04 exact V4.2 checkpoint SHA has not passed actual CUDA ROS2/Gazebo E2E.",
            "CUDA/NVML was unavailable during this audit, so GPU latency and VRAM are not measured.",
            "Run04 published artifacts do not machine-bind the original initializer SHA-256.",
            "The prior raw Gazebo six-model summary referenced by the historical status file is absent at its recorded relative path.",
        ],
        "explicit_exclusions": {
            "Run06": [
                "Four coordinate-validity violations",
                "Global E1-E200 best checkpoint not verifiable from published artifacts",
            ]
        },
        "raw_historical_e2e_summary_present": raw_e2e_root.exists(),
        "active_ros2_profile_unchanged": "fyp_run01",
        "sealed_test_accessed": False,
        "training_performed": False,
        "threshold_retuned_on_dev": False,
        "verification": {
            "ros2_package_pytest": "PASS_61",
            "comparison_pipeline_unittest": "PASS_16_IN_PINNED_PHASE2A_ENVIRONMENT",
            "git_diff_check": "PASS",
            "note": "Recorded from commands executed on 2026-10-18; this consolidator does not rerun tests.",
        },
        "source_receipts_sha256": source_receipts,
        "gate_matrix_sha256": sha256(csv_path),
    }
    decision_path = ROOT / "FINAL_DECISION_RECEIPT.json"
    write_json(decision_path, decision)

    # A dependency-free SVG overview; numerical labels are sourced from the
    # already-frozen comparison receipts above.
    svg_path = ROOT / "six_model_cross_domain_comparison.svg"
    width, height = 980, 460
    x0, chart_w, bar_h = 205, 700, 22
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:DejaVu Sans,Arial,sans-serif;fill:#1f2937}.title{font-size:20px;font-weight:700}.label{font-size:13px}.value{font-size:12px}.note{font-size:11px;fill:#4b5563}</style>',
        '<text x="30" y="34" class="title">Six-model permitted-cohort comparison (mAP50-95)</text>',
        '<text x="30" y="55" class="note">Blue: Synthetic/Gazebo Dev; orange: AI-assisted Real HIGH/MODERATE (not expert GT)</text>',
    ]
    for index, row in enumerate(rows):
        y = 88 + index * 56
        syn = float(row["synthetic_dev_map50_95"])
        real_score = float(row["ai_real_moderate_high_map50_95"])
        parts.extend(
            [
                f'<text x="30" y="{y + 17}" class="label">{row["run_id"]} {row["architecture"]}</text>',
                f'<rect x="{x0}" y="{y}" width="{chart_w * syn:.2f}" height="{bar_h}" rx="2" fill="#2563eb"/>',
                f'<text x="{x0 + chart_w * syn + 7:.2f}" y="{y + 16}" class="value">{syn:.4f}</text>',
                f'<rect x="{x0}" y="{y + 25}" width="{chart_w * real_score:.2f}" height="{bar_h}" rx="2" fill="#f97316"/>',
                f'<text x="{x0 + chart_w * real_score + 7:.2f}" y="{y + 41}" class="value">{real_score:.4f}</text>',
            ]
        )
    parts.extend(
        [
            '<text x="30" y="438" class="note">Run04 is the engineering leader; deployment switch remains BLOCKED pending exact-SHA CUDA ROS2/Gazebo E2E.</text>',
            '</svg>',
        ]
    )
    svg_path.write_text("\n".join(parts) + "\n", encoding="utf-8")

    report_path = ROOT / "V42_SIX_MODEL_FINAL_COMPARISON.md"
    report_path.write_text(
        """# V4.2 corrected six-model comparison and deployment decision

Status: **COMPARISON COMPLETE; RUN04 DEPLOYMENT SWITCH BLOCKED**

No model was retrained, no checkpoint or dataset was modified, thresholds were
selected only on the common Synthetic Val cohort, and no sealed-test outcome was
accessed. The V4.2 Hybrid lineage is bound to correction-manifest SHA-256
`b634760affc11ce33f4dcbda5904aa4c482480824a3650b2d70b9add45a062bd`
and Git commit `d673a31b09543fb4e7aca4bb10bb22b6bfc76bd1`.

## Decision

- **Primary scientific selection:** Run01. This preserves the original frozen
  V4.1 Synthetic-control selection scope; it does not claim that Run01 is the
  strongest corrected engineering model.
- **Engineering highest-scoring model:** Run04 (V4.2 Hybrid YOLO26n-P2),
  provisional. It leads Synthetic/Gazebo Dev at **0.5613**, is within **0.0045**
  of the AI-assisted Real-reference leader, has **0.9667** perturbation
  retention, and passed CPU output/repeat/coordinate checks.
- **AI-assisted Real-reference leader:** Run02 at **0.2454**, versus Run04 at
  **0.2409**. This 58-image reference is not independent expert ground truth,
  so it is a secondary diagnostic and cannot override the frozen
  Synthetic/Gazebo-first policy.
- **Current deployable model:** Run01 on the previously tested workstation.
- **Switching deployment to Run04:** **NO-GO / BLOCKED**, until the exact Run04
  SHA passes actual CUDA ROS2/Gazebo end-to-end execution and GPU resource
  measurement. The active ROS2 profile remains Run01.

## Common-cohort results

| Run | Training condition | Threshold | Synthetic/Gazebo Dev mAP50-95 | AI Real HIGH/MOD mAP50-95 | ECE | Perturbation retention | CPU p90 ms | RSS MB | Eligibility |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| Run01 | V4.1 Synthetic | 0.40 | 0.5193 | 0.0091 | 0.0807 | 0.5000 | 77.75 | 601.53 | PASS, historical tested-workstation receipt |
| Run02 | V4.2 Hybrid | 0.40 | 0.5455 | **0.2454** | 0.0832 | 0.9143 | 90.77 | 622.26 | BLOCKED: new-SHA E2E |
| Run03 | V4.1 Synthetic | 0.45 | 0.5133 | 0.0236 | 0.0738 | 0.4429 | 121.62 | 634.08 | PASS, historical tested-workstation receipt |
| Run04 | V4.2 Hybrid | 0.45 | **0.5613** | 0.2409 | 0.0832 | **0.9667** | 121.10 | 666.19 | BLOCKED: new-SHA E2E |
| Run05 | V4.1 Synthetic | 0.45 | 0.4956 | 0.0619 | 0.0669 | 0.7882 | 239.67 | 1026.41 | PASS, historical tested-workstation receipt |
| Run06 | V4.2 Hybrid | 0.50 | 0.5003 | 0.2403 | 0.0655 | 1.0000 | 217.67 | 1031.79 | **FAIL** |

## Hard gates and integrity

Run02 and Run04 produced no non-finite outputs, no invalid coordinates and no
repeat mismatch in the new CPU audit. Run06 is explicitly excluded: it produced
four coordinate violations, and its published EMA checkpoint is not proven to
be the global best over E1-E200. No silent promotion is allowed.

Run02 completed 200 epochs and its global-best checkpoint is identifiable, but
its effective `nbs=64` and `patience=200` differ from the canonical recipe.
Run04 completed 200 epochs and its global-best checkpoint is identifiable, but
its effective `patience=0` differs from the canonical recipe. Neither published
YOLO artifact set machine-binds its starting initializer SHA-256. These facts do
not invalidate the observed inference scores, but they prevent claiming a clean
single-variable causal comparison of V4.1 Synthetic versus V4.2 Hybrid.

The ROS2 loader requires `device_policy: cuda_required`. CUDA/NVML was
unavailable during this audit; therefore GPU latency, VRAM, and actual new-weight
ROS2/Gazebo E2E are **BLOCKED**, not PASS. The historical status JSON reports a
tested-workstation pass for the older exact checkpoints, but its referenced raw
Gazebo summary is no longer present at the recorded relative path; this evidence
gap is declared rather than hidden.

## Reliability, domain shift and uncertainty

Run04's weakest Synthetic Dev class is `leaf_blight` (AP50-95 **0.5348**).
Its small-object AP is **0.1557**, higher than Run02 (**0.1321**), while small
objects remain the main weakness. On the AI-assisted reference, Run04 recall is
lowest for `leaf_rhizoctonia` (0.1905) and `leaf_blight` (0.2353).

The paired 2,000-replicate physical-tree bootstrap on Synthetic Dev macro-F1
gives Run04 minus Run02 = **+0.00975**, 95% CI **[+0.00402, +0.01600]**.
This quantifies only Synthetic Dev sampling uncertainty; it excludes training
seed, real-domain and annotation uncertainty and is not a COCO-mAP interval.

All six models show substantial domain shift. Run04's AI-reference-to-Synthetic
ratio is 0.429, so neither its Gazebo score nor the AI-assisted reference alone
supports a claim of production real-world accuracy.

## Verification

- ROS2 package tests: **61 passed**.
- Frozen comparison/tooling tests: **16 passed** in the pinned Phase2A
  environment (`Python 3.10.12`, `pycocotools 2.0.10`, `numpy 2.2.6`).
- `git diff --check`: **PASS**.
- The workspace already contained unrelated modified/untracked ROS2 and receipt
  files; none was reset, deleted, committed or pushed by this comparison.

## Required next action

When CUDA is available, run the exact Run04 checkpoint
`04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5`
through the existing ROS2 start-inspection, six-tree return, image-processing,
database/log receipt flow. Verify manifest binding, class order, preprocessing,
GPU latency/VRAM and zero invalid geometry. If every frozen hard gate passes,
Run04 becomes the recommended deployment model; otherwise keep Run01 and record
the failed gate explicitly.
""",
        encoding="utf-8",
    )

    print(report_path)
    print(decision_path)
    print(csv_path)
    print(svg_path)


if __name__ == "__main__":
    main()
