import copy
import json
import math
from pathlib import Path
import sqlite3
import sys
import threading

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from canonical_leaf_classes import class_table_sha256
from checkpoint_manifest import load_checkpoint_manifest
from deployment_fingerprint import build_deployment_fingerprint
from experiment_spec import parse_experiment_spec
from leaf_association import (
    AssociationPolicy,
    AssociationPolicyUnfrozen,
    Detection3D,
    aggregate_tree,
    associate_unique_leaves,
    robust_center_depth,
    transform_point,
)
from leaf_deployment_config import load_deployment_config
from leaf_backend_runtime import LeafBackendError, LeafBackendSpec
from inspection_server import (
    navigation_watchdog_violation,
    recovery_completion_state,
)
from multi_frame_policy import (
    FrameDetectionEvidence,
    TemporalPolicy,
    aggregate_temporal_multi_view,
    sample_frames,
)
from multi_view_inspection import (
    MultiViewPolicy,
    NavigationSafetyPolicy,
    ViewCompletionTracker,
    recovery_motion_command,
)
from observation_store import connect
from world_gt_verifier import VerificationError, verify_world


def frozen_policy(distance=0.2):
    return AssociationPolicy.from_mapping({
        "version": "test-v1", "status": "FROZEN",
        "center_patch_radius_px": 1, "minimum_valid_depth_samples": 3,
        "maximum_match_distance_m": distance,
        "evidence_rule": "distinct_view_support_then_mean_confidence",
        "aggregation_mode": "unique_leaf_count",
    })


def temporal_policy(interval=1.0, maximum=10, minimum=1):
    return TemporalPolicy.from_mapping({
        "version": "TEMPORAL_MULTI_VIEW_SUPPORT_V1", "status": "FROZEN",
        "sampling_interval_sec": interval,
        "maximum_sampled_frames_per_viewpoint": maximum,
        "minimum_valid_frames_per_viewpoint": minimum,
        "viewpoint_weighting": "equal_one_vote",
    })


def frame(observation, view, timestamp, counts, confidences):
    return FrameDetectionEvidence(observation, view, timestamp, counts, confidences)


def test_dynamic_inventory_and_config_driven_viewpoints():
    mapping = {
        "version": "test", "enabled": True, "radius_m": 2.0,
        "angular_offsets_deg": [0, 120, 240], "capture_duration_sec": 1.0,
    }
    policy = MultiViewPolicy.from_mapping(mapping)
    first = policy.viewpoints(1.0, 2.0)
    moved = policy.viewpoints(9.0, -3.0)
    assert len(first) == 3
    assert (first[0].x, first[0].y) == pytest.approx((3.0, 2.0))
    assert (moved[0].x, moved[0].y) == pytest.approx((11.0, -3.0))
    for tree_ids in ((9, 3, 20), (12, 4, 99, 2, 8, 31)):
        tracker = ViewCompletionTracker(tree_ids, [row.viewpoint_id for row in first])
        for tree_id in tree_ids:
            for row in first:
                tracker.complete(tree_id, row.viewpoint_id)
            assert tracker.tree_complete(tree_id)
        assert tracker.inventory_complete
    changed = dict(mapping, angular_offsets_deg=[0, 90, 180, 270])
    assert len(MultiViewPolicy.from_mapping(changed).viewpoints(1, 2)) == 4


def test_completion_waits_for_every_required_view():
    tracker = ViewCompletionTracker([10, 40], ["a", "b"])
    tracker.complete(10, "a")
    tracker.complete(10, "b")
    tracker.complete(40, "a")
    assert tracker.tree_complete(10)
    assert not tracker.tree_complete(40)
    assert not tracker.inventory_complete
    tracker.complete(40, "b")
    assert tracker.inventory_complete


def test_watchdog_recovery_preserves_current_viewpoint_context():
    assert navigation_watchdog_violation(0.799, 0.8)
    assert not navigation_watchdog_violation(0.8, 0.8)
    assert recovery_completion_state("CALCULATE_GOAL", True, True) == "CALCULATE_GOAL"
    assert recovery_completion_state("CALCULATE_GOAL", False, True) == "FIND_TREE"
    assert recovery_completion_state("FIND_TREE", True, True) == "FIND_TREE"


def test_navigation_recovery_moves_away_until_configured_clearance():
    policy = NavigationSafetyPolicy.from_mapping({
        "watchdog_min_distance_m": 0.8,
        "goal_min_distance_m": 1.0,
        "recovery_clearance_m": 1.2,
        "max_recovery_attempts_per_leg": 3,
        "recovery_linear_speed_mps": 0.2,
        "recovery_angular_speed_radps": 0.4,
        "recovery_align_tolerance_rad": 0.15,
        "recovery_timeout_sec": 12.0,
    })
    # Robot east of the tree and facing west: rotate toward east first.
    linear, angular, complete = recovery_motion_command(
        0.5, 0.0, math.pi, 0.0, 0.0, policy
    )
    assert linear == 0.0
    assert abs(angular) == pytest.approx(0.4)
    assert not complete
    # Once facing away, forward motion increases rather than reduces clearance.
    assert recovery_motion_command(0.5, 0.0, 0.0, 0.0, 0.0, policy) == (
        0.2, 0.0, False
    )
    assert recovery_motion_command(1.2, 0.0, 0.0, 0.0, 0.0, policy) == (
        0.0, 0.0, True
    )


def test_navigation_safety_policy_rejects_unbounded_or_inverted_values():
    valid = {
        "watchdog_min_distance_m": 0.8,
        "goal_min_distance_m": 1.0,
        "recovery_clearance_m": 1.2,
        "max_recovery_attempts_per_leg": 3,
        "recovery_linear_speed_mps": 0.2,
        "recovery_angular_speed_radps": 0.4,
        "recovery_align_tolerance_rad": 0.15,
        "recovery_timeout_sec": 12.0,
    }
    assert NavigationSafetyPolicy.from_mapping(valid).max_recovery_attempts_per_leg == 3
    with pytest.raises(ValueError, match="positive integer"):
        NavigationSafetyPolicy.from_mapping(
            dict(valid, max_recovery_attempts_per_leg=0)
        )
    with pytest.raises(ValueError, match="watchdog < goal <= clearance"):
        NavigationSafetyPolicy.from_mapping(
            dict(valid, watchdog_min_distance_m=1.0)
        )


def test_temporal_sampling_and_frame_tie_breakers():
    rows = [
        frame("b", "v", 0.4, {"leaf_blight": 1}, {"leaf_blight": .7}),
        frame("a", "v", 0.0, {"leaf_algal": 2, "leaf_blight": 2},
              {"leaf_algal": 1.2, "leaf_blight": 1.8}),
        frame("c", "v", 1.1, {"leaf_algal": 1}, {"leaf_algal": .9}),
    ]
    sampled = sample_frames(rows, temporal_policy(interval=1.0))
    assert [row.observation_id for row in sampled] == ["a", "c"]
    result = aggregate_temporal_multi_view(rows, temporal_policy(interval=1.0), 1, 5, ["v"])
    # Frame a ties on count and is won by blight mean confidence; frame c votes algal.
    # Viewpoint tie then uses mean per-frame detection support, won by algal (3 vs 2).
    assert result.class_name == "leaf_algal"
    assert result.sampled_frame_count == 2


def test_temporal_views_have_equal_tree_weight_and_canonical_ties():
    rows = []
    for index in range(8):
        rows.append(frame(f"a{index}", "view_000", float(index),
                          {"leaf_algal": 4}, {"leaf_algal": 2.4}))
    rows.extend([
        frame("b", "view_001", 0.0, {"leaf_blight": 1}, {"leaf_blight": .6}),
        frame("c", "view_002", 0.0, {"leaf_blight": 1}, {"leaf_blight": .6}),
        frame("d", "view_003", 0.0, {"leaf_algal": 1}, {"leaf_algal": .6}),
    ])
    result = aggregate_temporal_multi_view(
        rows, temporal_policy(interval=0), 4, 35,
        ["view_000", "view_001", "view_002", "view_003"],
    )
    assert result.viewpoint_votes == {"leaf_algal": 2, "leaf_blight": 2}
    # Equal votes/support/confidence resolve by canonical class ID, not frame volume.
    assert result.class_name == "leaf_algal"


@pytest.mark.parametrize("roi,detections,expected", [
    (0, 0, "UNKNOWN_NO_ROI"),
    (2, 0, "UNKNOWN_NO_DETECTION"),
    (2, 3, "UNKNOWN_NO_VALID_EVIDENCE"),
])
def test_temporal_unknown_reason_semantics(roi, detections, expected):
    result = aggregate_temporal_multi_view([], temporal_policy(), roi, detections, ["view_000"])
    assert result.unknown_reason == expected


def test_unique_leaf_association_duplicate_separate_and_deterministic():
    p = frozen_policy()
    rows = [
        Detection3D("o2", "v2", 1, .7, (0.03, 0, 0)),
        Detection3D("o1", "v1", 1, .8, (0, 0, 0)),
        Detection3D("o1", "v1", 1, .8, (0, 0, 0)),
        Detection3D("o3", "v1", 3, .9, (1, 0, 0)),
    ]
    forward = associate_unique_leaves(rows, p)
    reverse = associate_unique_leaves(reversed(rows), p)
    assert len(forward) == 2
    assert [(h.centroid, len(h.evidence)) for h in forward] == [
        (h.centroid, len(h.evidence)) for h in reverse
    ]
    result = aggregate_tree(forward, roi_count=3, backend_detection_count=4)
    assert result.unique_counts == {"leaf_blight": 1, "leaf_healthy": 1}
    assert result.class_name == "leaf_healthy"  # confidence breaks the count tie


def test_class_disagreement_uses_distinct_view_support_not_repeat_volume():
    p = frozen_policy()
    rows = [
        Detection3D("a", "v1", 0, .99, (0, 0, 0)),
        Detection3D("b", "v1", 0, .98, (0.01, 0, 0)),
        Detection3D("c", "v2", 1, .70, (0.02, 0, 0)),
        Detection3D("d", "v3", 1, .70, (0.03, 0, 0)),
    ]
    result = aggregate_tree(associate_unique_leaves(rows, p), roi_count=1, backend_detection_count=4)
    assert result.class_name == "leaf_blight"


def test_two_adjacent_detections_in_one_frame_remain_separate_leaves():
    p = frozen_policy(distance=0.2)
    rows = [
        Detection3D("same_frame", "v1", 0, .8, (0, 0, 0)),
        Detection3D("same_frame", "v1", 0, .7, (0.05, 0, 0)),
    ]
    assert len(associate_unique_leaves(rows, p)) == 2


def test_depth_and_transform_fail_closed_inputs():
    np = pytest.importorskip("numpy")
    depth = np.full((5, 5), float("nan"), dtype=float)
    assert robust_center_depth(depth, (1, 1, 3, 3), frozen_policy(), .5, 8) is None
    depth[1:4, 1:4] = [
        [1.0, 1.1, 1.2], [1.0, 20.0, 1.2], [1.0, 1.1, 1.2]
    ]
    assert robust_center_depth(depth, (1, 1, 3, 3), frozen_policy(), .5, 8) == pytest.approx(1.1)
    with pytest.raises(KeyError):
        transform_point((0, 0, 1), {})


@pytest.mark.parametrize("roi,detections,expected", [
    (0, 0, "UNKNOWN_NO_ROI"),
    (2, 0, "UNKNOWN_NO_DETECTION"),
    (2, 3, "UNKNOWN_NO_VALID_EVIDENCE"),
])
def test_unknown_reason_semantics(roi, detections, expected):
    assert aggregate_tree([], roi_count=roi, backend_detection_count=detections).unknown_reason == expected


def test_unfrozen_association_refuses_production_decision():
    policy = copy.copy(frozen_policy())
    object.__setattr__(policy, "status", "UNFROZEN")
    with pytest.raises(AssociationPolicyUnfrozen):
        associate_unique_leaves([], policy)


def _fixture(tmp_path):
    textures = tmp_path / "textures"
    textures.mkdir()
    (textures / "leaf_healthy.png").write_bytes(b"texture")
    material = tmp_path / "leaves.material"
    material.write_text("material leaf_healthy { technique { pass { texture_unit { texture leaf_healthy.png } } } }", encoding="utf-8")
    world = tmp_path / "world.sdf"
    world.write_text("""<sdf version='1.6'><world name='default'>
      <model name='durian_tree_9'><pose>1 2 0 0 0 0</pose><link name='tree'>
        <visual name='leaf_phys_A'><pose>0 0 1 0 0 0</pose><material><script><name>leaf_healthy</name></script></material></visual>
      </link></model></world></sdf>""", encoding="utf-8")
    data = {
        "schema": "durian-controlled-experiment-gt", "version": 1,
        "ground_truth_definition": "largest unique physical leaf count",
        "world": {"path": str(world), "validate_sha256": False, "sha256": None},
        "classes": {"table_sha256": class_table_sha256()},
        "materials": {"script": str(material), "texture_directory": str(textures)},
        "trees": [{
            "id": 9, "model_name": "durian_tree_9", "pose": [1, 2, 0, 0, 0, 0],
            "dominant_class": "leaf_healthy", "total_leaves": 1,
            "expected_class_counts": {"leaf_healthy": 1},
            "intended_viewpoints": ["view_a"],
            "leaves": [{"id": "physical_A", "visual_name": "leaf_phys_A",
                        "class": "leaf_healthy", "material": "leaf_healthy",
                        "pose": [0, 0, 1, 0, 0, 0], "slot": "s1",
                        "sector": "north", "height_band": "mid"}],
        }],
    }
    return world, data


def test_world_gt_verifier_pass_and_failures(tmp_path):
    world, data = _fixture(tmp_path)
    verify_world(parse_experiment_spec(data))

    wrong_pose = copy.deepcopy(data)
    wrong_pose["trees"][0]["pose"][0] = 8
    with pytest.raises(VerificationError, match="wrong tree pose"):
        verify_world(parse_experiment_spec(wrong_pose))

    wrong_hash = copy.deepcopy(data)
    wrong_hash["world"].update(validate_sha256=True, sha256="0" * 64)
    with pytest.raises(VerificationError, match="world SHA256 mismatch"):
        verify_world(parse_experiment_spec(wrong_hash))

    wrong_class_hash = copy.deepcopy(data)
    wrong_class_hash["classes"]["table_sha256"] = "0" * 64
    with pytest.raises(VerificationError, match="class-table hash mismatch"):
        verify_world(parse_experiment_spec(wrong_class_hash))

    world.write_text(world.read_text().replace("leaf_healthy</name>", "leaf_blight</name>"), encoding="utf-8")
    with pytest.raises(VerificationError, match="material mismatch"):
        verify_world(parse_experiment_spec(data))


def test_manifest_duplicate_missing_and_extra_leaf_failures(tmp_path):
    _, data = _fixture(tmp_path)
    duplicate = copy.deepcopy(data)
    duplicate["trees"].append(copy.deepcopy(duplicate["trees"][0]))
    duplicate["trees"][1]["id"] = 10
    duplicate["trees"][1]["model_name"] = "durian_tree_10"
    with pytest.raises(ValueError, match="duplicate physical leaf id"):
        parse_experiment_spec(duplicate)
    missing = copy.deepcopy(data)
    missing["trees"][0]["leaves"] = []
    with pytest.raises(ValueError, match="leaf record count"):
        parse_experiment_spec(missing)
    extra = copy.deepcopy(data)
    extra["trees"][0]["leaves"].append(copy.deepcopy(extra["trees"][0]["leaves"][0]))
    with pytest.raises(ValueError, match="leaf record count"):
        parse_experiment_spec(extra)


def test_world_missing_extra_and_duplicate_visuals_fail(tmp_path):
    world, data = _fixture(tmp_path)
    original = world.read_text(encoding="utf-8")
    world.write_text(original.replace("<visual name='leaf_phys_A'>", "<visual name='other'>"), encoding="utf-8")
    with pytest.raises(VerificationError, match="leaf visual set mismatch"):
        verify_world(parse_experiment_spec(data))

    world.write_text(original.replace(
        "</link>",
        "<visual name='leaf_extra'><pose>0 0 1 0 0 0</pose><material><script><name>leaf_healthy</name></script></material></visual></link>"
    ), encoding="utf-8")
    with pytest.raises(VerificationError, match="leaf visual set mismatch"):
        verify_world(parse_experiment_spec(data))

    duplicate_visual = "<visual name='leaf_phys_A'><pose>0 0 1 0 0 0</pose><material><script><name>leaf_healthy</name></script></material></visual>"
    world.write_text(original.replace("</link>", duplicate_visual + "</link>"), encoding="utf-8")
    with pytest.raises(VerificationError, match="duplicate leaf/visual IDs"):
        verify_world(parse_experiment_spec(data))


def test_permuted_gt_assignment_cannot_change_prediction(tmp_path):
    hypotheses = associate_unique_leaves([
        Detection3D("o", "v", 3, .8, (0, 0, 0))
    ], frozen_policy())
    before = aggregate_tree(hypotheses, roi_count=1, backend_detection_count=1)
    _, data = _fixture(tmp_path)
    # Evaluation data is deliberately not passed to aggregate_tree.
    data["trees"][0]["dominant_class"] = "leaf_blight"
    after = aggregate_tree(hypotheses, roi_count=1, backend_detection_count=1)
    assert before == after


def test_db_migration_idempotent_and_concurrent(tmp_path):
    db = tmp_path / "migration.db"
    with sqlite3.connect(db) as conn:
        conn.execute("CREATE TABLE inspection_log (id INTEGER PRIMARY KEY)")
    failures = []
    def migrate():
        try:
            with connect(str(db)):
                pass
        except Exception as exc:  # pragma: no cover - asserted below
            failures.append(exc)
    threads = [threading.Thread(target=migrate) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert failures == []
    with connect(str(db)) as conn:
        columns = {row[1] for row in conn.execute("PRAGMA table_info(inspection_log)")}
        assert {
            "aggregation_mode", "unknown_reason", "processing_error",
            "viewpoint_count", "valid_viewpoint_count", "sampled_frame_count",
            "world_sha256", "experiment_version",
        } <= columns


def test_fingerprint_changes_with_all_result_affecting_policy():
    config_path = PACKAGE / "config" / "leaf_detector_deployment.yaml"
    config = load_deployment_config(config_path)
    profile = config["leaf_detector"]["profiles"][config["leaf_detector"]["backend"]]
    manifest = load_checkpoint_manifest(profile["manifest"])
    original = build_deployment_fingerprint(config, manifest)
    changed_view = copy.deepcopy(config)
    changed_view["inspection_policy"]["radius_m"] += 0.01
    assert build_deployment_fingerprint(changed_view, manifest) != original
    changed_association = copy.deepcopy(config)
    changed_association["multi_frame"]["same_leaf_association"]["maximum_match_distance_m"] += 0.01
    assert build_deployment_fingerprint(changed_association, manifest) != original
    changed_temporal = copy.deepcopy(config)
    changed_temporal["multi_frame"]["temporal_sampling"]["sampling_interval_sec"] += 0.01
    assert build_deployment_fingerprint(changed_temporal, manifest) != original
    changed_runtime = copy.deepcopy(config)
    changed_runtime["leaf_detector"]["profiles"][config["leaf_detector"]["backend"]][
        "runtime_options"
    ]["test_result_affecting_option"] = "changed"
    assert build_deployment_fingerprint(changed_runtime, manifest) != original


def test_deployment_switch_rejects_missing_or_incompatible_checkpoint(tmp_path):
    config_path = PACKAGE / "config" / "leaf_detector_deployment.yaml"
    config = load_deployment_config(config_path)
    config["leaf_detector"]["backend"] = "yolo26n"

    missing = copy.deepcopy(config)
    missing["leaf_detector"]["profiles"]["yolo26n"]["checkpoint"] = str(
        tmp_path / "missing.pt"
    )
    missing_path = tmp_path / "missing.yaml"
    missing_path.write_text(yaml.safe_dump(missing), encoding="utf-8")
    with pytest.raises(LeafBackendError, match="checkpoint paths differ"):
        LeafBackendSpec.from_config(missing_path)

    incompatible = copy.deepcopy(config)
    incompatible["leaf_detector"]["profiles"]["yolo26n"]["manifest"] = config[
        "leaf_detector"
    ]["profiles"]["rfdetr_n"]["manifest"]
    incompatible_path = tmp_path / "incompatible.yaml"
    incompatible_path.write_text(yaml.safe_dump(incompatible), encoding="utf-8")
    with pytest.raises(LeafBackendError, match="backend and manifest candidate differ"):
        LeafBackendSpec.from_config(incompatible_path)


def test_production_modules_do_not_import_ground_truth_or_dominant_labels():
    production = ["processing_node.py", "inspection_server.py", "vision_node.py",
                  "leaf_association.py", "multi_view_inspection.py"]
    for name in production:
        text = (PACKAGE / name).read_text(encoding="utf-8")
        assert "experiment_spec" not in text
        assert "dominant_class" not in text
