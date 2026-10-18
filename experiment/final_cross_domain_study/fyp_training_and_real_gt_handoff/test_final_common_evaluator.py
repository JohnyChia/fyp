#!/usr/bin/env python3
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import final_common_evaluator as target


class FinalCommonEvaluatorTests(unittest.TestCase):
    def test_v42_synthetic_hashes_are_frozen(self):
        target.verify_synthetic_identities()

    def test_run_ids_and_threshold_grid(self):
        self.assertEqual(target.RUN_IDS, tuple(f"Run{i:02d}" for i in range(1, 7)))
        self.assertEqual(target.THRESHOLDS[0], 0.05)
        self.assertEqual(target.THRESHOLDS[-1], 0.95)
        self.assertEqual(len(target.THRESHOLDS), 19)
        self.assertEqual(target.runtime_identity(), {"python": "3.10.12", "pycocotools": "2.0.10", "numpy": "2.2.6"})

    def test_pending_real_receipt_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "receipt.json"
            path.write_text(json.dumps({"status": "PENDING_HUMAN_VERIFICATION"}))
            with self.assertRaisesRegex(PermissionError, "Real scoring blocked"):
                target.validate_real_receipt(path)

    def test_sealed_path_is_rejected_before_scoring(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            gt = root / "sealed_real_test.json"
            gt.write_text("{}")
            receipt = root / "receipt.json"
            receipt.write_text(json.dumps({
                "status": "PASS", "independent_object_level_verification": True,
                "geometry_qc_pass": True, "source_family_isolation_pass": True,
                "ground_truth_path": str(gt), "ground_truth_sha256": target.sha256(gt),
            }))
            with self.assertRaisesRegex(PermissionError, "sealed-test"):
                target.validate_real_receipt(receipt)

if __name__ == "__main__":
    unittest.main()
