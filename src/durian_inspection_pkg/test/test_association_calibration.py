from pathlib import Path
import sys

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from association_calibration import CalibrationRecord, calibrate_association


def _record(leaf, observation, view, xyz):
    return CalibrationRecord(leaf, observation, view, 7, xyz, 3, 1.0)


POLICY = {
    "version": "test-v1", "status": "UNFROZEN",
    "center_patch_radius_px": 2, "minimum_valid_depth_samples": 3,
    "maximum_match_distance_m": 0.12,
    "evidence_rule": "distinct_view_support_then_mean_confidence",
    "aggregation_mode": "unique_leaf_count",
}


def test_known_identity_metrics_choose_separating_distance():
    rows = (
        _record("A", "a1", "v1", (0.00, 0, 0)),
        _record("A", "a2", "v2", (0.02, 0, 0)),
        _record("B", "b1", "v1", (0.50, 0, 0)),
        _record("B", "b2", "v2", (0.53, 0, 0)),
    )
    report = calibrate_association(rows, POLICY, 2, 2, 2, 0.1)
    assert report["eligible_to_freeze"]
    assert report["selected"]["precision"] == 1.0
    assert report["selected"]["recall"] == 1.0
    assert report["selected"]["f1"] == 1.0
    assert report["selected"]["false_merge_rate"] == 0.0
    assert report["selected"]["false_split_rate"] == 0.0
    assert report["production_associator_metrics"]["f1"] == 1.0


def test_insufficient_or_invalid_depth_corpus_cannot_freeze():
    rows = (
        _record("A", "a1", "v1", None),
        _record("A", "a2", "v2", (0.02, 0, 0)),
        _record("B", "b1", "v1", (0.50, 0, 0)),
    )
    report = calibrate_association(rows, POLICY, 3, 2, 2, 0.1)
    assert not report["eligible_to_freeze"]
    assert report["invalid_depth_rate"] > 0.1
    assert report["eligibility_errors"]


def test_calibration_selection_does_not_accept_tree_class_ground_truth():
    signature = calibrate_association.__annotations__
    assert "dominant_class" not in signature
