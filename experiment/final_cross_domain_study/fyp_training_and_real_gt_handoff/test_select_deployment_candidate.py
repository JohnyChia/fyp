#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import unittest
from pathlib import Path

from select_deployment_candidate import RUN_IDS, select

HERE = Path(__file__).resolve().parent


def fixtures():
    comparison_rows, deployment_rows = [], []
    for index, run_id in enumerate(RUN_IDS, 1):
        checkpoint = f"hash-{run_id}"
        comparison_rows.append({
            "run_id": run_id, "checkpoint_sha256": checkpoint,
            "real_dev_map50_95": 0.50 + index / 100,
            "synthetic_dev_map50_95": 0.60 + index / 100,
        })
        deployment_rows.append({
            "run_id": run_id, "checkpoint_sha256": checkpoint,
            "ros2_end_to_end_pass": True, "non_finite_output_count": 0,
            "coordinate_violation_count": 0, "repeated_inference_mismatch_count": 0,
            "perturbation_retention": 0.90, "calibration_ece": 0.05,
            "cpu_p90_latency_ms": 100 - index, "peak_rss_mb": 500 + index,
        })
    comparison = {"run_count": 6, "real_scoring_performed": True, "leaderboard": comparison_rows,
                  "sealed_test_accessed": False, "automatic_selection_disabled": True}
    deployment = {"status": "PASS", "runs": deployment_rows, "sealed_test_accessed": False}
    policy = json.loads((HERE / "FINAL_DEPLOYMENT_DECISION_POLICY.json").read_text())
    return comparison, deployment, policy


class DeploymentSelectionTests(unittest.TestCase):
    def test_best_real_metric_is_selected(self):
        comparison, deployment, policy = fixtures()
        result = select(comparison, deployment, policy)
        self.assertEqual(result["status"], "SELECTED")
        self.assertEqual(result["selected_run"], "Run06")

    def test_ros2_failure_excludes_top_accuracy_model(self):
        comparison, deployment, policy = fixtures()
        deployment["runs"][-1]["ros2_end_to_end_pass"] = False
        result = select(comparison, deployment, policy)
        self.assertEqual(result["selected_run"], "Run05")
        self.assertEqual(result["excluded"], [{"run_id": "Run06", "reasons": ["ROS2_END_TO_END_FAIL"]}])

    def test_no_real_scoring_fails_closed(self):
        comparison, deployment, policy = fixtures()
        comparison["real_scoring_performed"] = False
        with self.assertRaisesRegex(ValueError, "eligible Real scoring"):
            select(comparison, deployment, policy)

    def test_checkpoint_mismatch_fails(self):
        comparison, deployment, policy = fixtures()
        deployment["runs"][0]["checkpoint_sha256"] = "wrong"
        with self.assertRaisesRegex(ValueError, "checkpoint identity"):
            select(comparison, deployment, policy)

    def test_explicit_fyp_proposal_policy_accepts_declared_reference(self):
        comparison, deployment, _ = fixtures()
        comparison.update({"real_scoring_performed": False,
                           "proposal_ai_reference_scoring_performed": True,
                           "proposal_ai_reference_is_independent_expert_gt": False})
        for index, row in enumerate(comparison["leaderboard"], 1):
            row["fyp_ai_reference_moderate_high_map50_95"] = 0.40 + index / 100
        policy = json.loads((HERE / "FYP_PROPOSAL_DEPLOYMENT_DECISION_POLICY.json").read_text())
        result = select(comparison, deployment, policy)
        self.assertEqual(result["selected_run"], "Run05")
        self.assertEqual(result["selection_scope"], "FYP_PROPOSAL_AI_ASSISTED_REFERENCE")

    def test_fyp_policy_rejects_missing_reference_scores(self):
        comparison, deployment, _ = fixtures()
        comparison["real_scoring_performed"] = False
        policy = json.loads((HERE / "FYP_PROPOSAL_DEPLOYMENT_DECISION_POLICY.json").read_text())
        with self.assertRaisesRegex(ValueError, "AI-assisted reference"):
            select(comparison, deployment, policy)

    def test_duplicate_run_rows_fail_closed(self):
        comparison, deployment, policy = fixtures()
        deployment["runs"][-1] = dict(deployment["runs"][0])
        with self.assertRaisesRegex(ValueError, "six unique rows"):
            select(comparison, deployment, policy)

    def test_non_finite_metric_fails_closed(self):
        comparison, deployment, policy = fixtures()
        comparison["leaderboard"][-1]["real_dev_map50_95"] = math.nan
        with self.assertRaisesRegex(ValueError, "missing numeric decision metric"):
            select(comparison, deployment, policy)


if __name__ == "__main__":
    unittest.main()
