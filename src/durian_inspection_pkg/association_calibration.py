"""Known-identity calibration for the production 3-D leaf association policy.

The calibration corpus is deliberately independent of tree-class ground truth.
It labels physical leaf correspondence only; final disease correctness is never
used to select an association distance.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from math import sqrt
from pathlib import Path
from typing import Iterable, Mapping

from leaf_association import AssociationPolicy, Detection3D, associate_unique_leaves


@dataclass(frozen=True)
class CalibrationRecord:
    physical_leaf_id: str
    observation_id: str
    viewpoint_id: str
    tree_id: int
    tree_xyz: tuple[float, float, float] | None
    class_id: int
    confidence: float


def load_calibration_corpus(path: str | Path) -> tuple[CalibrationRecord, ...]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("schema") != "durian-leaf-association-calibration" or data.get("version") != 1:
        raise ValueError("unsupported association calibration schema/version")
    rows = []
    for index, item in enumerate(data.get("records", [])):
        leaf_id = item.get("physical_leaf_id")
        observation_id = item.get("observation_id")
        viewpoint_id = item.get("viewpoint_id")
        tree_id = item.get("tree_id")
        if not all(isinstance(value, str) and value for value in (leaf_id, observation_id, viewpoint_id)):
            raise ValueError(f"record {index}: physical leaf, observation, and viewpoint IDs are required")
        if not isinstance(tree_id, int) or isinstance(tree_id, bool):
            raise ValueError(f"record {index}: tree_id must be an integer")
        xyz = item.get("tree_xyz")
        if xyz is not None:
            if not isinstance(xyz, list) or len(xyz) != 3:
                raise ValueError(f"record {index}: tree_xyz must contain three values or null")
            xyz = tuple(float(value) for value in xyz)
        rows.append(CalibrationRecord(
            leaf_id, observation_id, viewpoint_id, tree_id, xyz,
            int(item.get("class_id", 0)), float(item.get("confidence", 1.0)),
        ))
    if not rows:
        raise ValueError("calibration corpus contains no records")
    return tuple(rows)


def _distance(left: CalibrationRecord, right: CalibrationRecord) -> float:
    return sqrt(sum((a-b)**2 for a, b in zip(left.tree_xyz, right.tree_xyz)))


def _percentile(values: list[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * probability
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = index - lower
    return ordered[lower] * (1 - fraction) + ordered[upper] * fraction


def calibrate_association(records: Iterable[CalibrationRecord], policy_template: Mapping[str, object],
                          minimum_leaf_identities: int,
                          minimum_same_leaf_pairs: int,
                          minimum_different_leaf_pairs: int,
                          maximum_invalid_depth_rate: float) -> dict[str, object]:
    rows = tuple(records)
    valid = [row for row in rows if row.tree_xyz is not None]
    invalid_rate = 1.0 - len(valid) / len(rows)
    leaf_ids = {row.physical_leaf_id for row in valid}
    same_distances, different_distances = [], []
    pairs = []
    for left_index, left in enumerate(valid):
        for right in valid[left_index + 1:]:
            if left.tree_id != right.tree_id or left.observation_id == right.observation_id:
                continue
            actual_same = left.physical_leaf_id == right.physical_leaf_id
            distance = _distance(left, right)
            (same_distances if actual_same else different_distances).append(distance)
            pairs.append((left, right, actual_same, distance))
    eligibility_errors = []
    if len(leaf_ids) < minimum_leaf_identities:
        eligibility_errors.append(f"physical leaf identities {len(leaf_ids)} < {minimum_leaf_identities}")
    if len(same_distances) < minimum_same_leaf_pairs:
        eligibility_errors.append(f"same-leaf pairs {len(same_distances)} < {minimum_same_leaf_pairs}")
    if len(different_distances) < minimum_different_leaf_pairs:
        eligibility_errors.append(f"different-leaf pairs {len(different_distances)} < {minimum_different_leaf_pairs}")
    if invalid_rate > maximum_invalid_depth_rate:
        eligibility_errors.append(
            f"invalid-depth rate {invalid_rate:.6f} > {maximum_invalid_depth_rate:.6f}"
        )
    candidate_distances = sorted({distance for *_, distance in pairs})
    candidates = []
    if candidate_distances:
        thresholds = [candidate_distances[0] / 2.0]
        thresholds.extend(
            (left + right) / 2.0
            for left, right in zip(candidate_distances, candidate_distances[1:])
        )
        thresholds.append(candidate_distances[-1] * 1.01)
        for threshold in thresholds:
            tp = sum(actual and distance <= threshold for *_, actual, distance in pairs)
            fp = sum(not actual and distance <= threshold for *_, actual, distance in pairs)
            fn = sum(actual and distance > threshold for *_, actual, distance in pairs)
            tn = sum(not actual and distance > threshold for *_, actual, distance in pairs)
            precision = tp / (tp + fp) if tp + fp else 0.0
            recall = tp / (tp + fn) if tp + fn else 0.0
            f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
            candidates.append({
                "distance_m": threshold, "true_positive": tp, "false_positive": fp,
                "false_negative": fn, "true_negative": tn, "precision": precision,
                "recall": recall, "f1": f1,
                "false_merge_rate": fp / (fp + tn) if fp + tn else 0.0,
                "false_split_rate": fn / (fn + tp) if fn + tp else 0.0,
            })
    selected = min(candidates, key=lambda row: (-row["f1"], row["distance_m"])) if candidates else None
    per_view = {}
    for viewpoint in sorted({row.viewpoint_id for row in rows}):
        view_rows = [row for row in rows if row.viewpoint_id == viewpoint]
        per_view[viewpoint] = {
            "records": len(view_rows),
            "valid_depth_records": sum(row.tree_xyz is not None for row in view_rows),
            "invalid_depth_rate": sum(row.tree_xyz is None for row in view_rows) / len(view_rows),
        }
    production_metrics = None
    if selected is not None:
        selected_policy = dict(policy_template)
        selected_policy["maximum_match_distance_m"] = selected["distance_m"]
        production_metrics = production_association_metrics(
            valid, AssociationPolicy.from_mapping(selected_policy)
        )
    result = {
        "schema": "durian-leaf-association-calibration-report",
        "version": 1,
        "selection_basis": "physical_leaf_identity_pair_correctness_only",
        "eligible_to_freeze": not eligibility_errors and selected is not None,
        "eligibility_errors": eligibility_errors,
        "record_count": len(rows),
        "physical_leaf_identity_count": len(leaf_ids),
        "invalid_depth_rate": invalid_rate,
        "same_leaf_distance_m": {
            "count": len(same_distances), "median": _percentile(same_distances, 0.5),
            "p95": _percentile(same_distances, 0.95), "maximum": max(same_distances, default=None),
        },
        "different_leaf_distance_m": {
            "count": len(different_distances), "minimum": min(different_distances, default=None),
            "p05": _percentile(different_distances, 0.05), "median": _percentile(different_distances, 0.5),
        },
        "selected": selected,
        "production_associator_metrics": production_metrics,
        "per_view": per_view,
        "policy_template": dict(policy_template),
    }
    return result


def production_association_metrics(records: Iterable[CalibrationRecord], policy: AssociationPolicy) -> dict[str, object]:
    """Measure pair correctness from the exact production clustering function."""
    grouped = {}
    for row in records:
        if row.tree_xyz is None:
            continue
        grouped.setdefault(row.tree_id, []).append(row)
    tp = fp = fn = tn = 0
    for tree_id, tree_rows in sorted(grouped.items()):
        detections = [Detection3D(
            row.observation_id, row.viewpoint_id, row.class_id,
            row.confidence, row.tree_xyz,
        ) for row in tree_rows]
        hypotheses = associate_unique_leaves(detections, policy, require_frozen=False)
        hypothesis_by_detection = {
            item: hypothesis.hypothesis_id
            for hypothesis in hypotheses for item in hypothesis.evidence
        }
        def hypothesis_id(row):
            return hypothesis_by_detection[Detection3D(
                row.observation_id, row.viewpoint_id, row.class_id,
                row.confidence, row.tree_xyz,
            )]
        for left_index, left in enumerate(tree_rows):
            for right in tree_rows[left_index + 1:]:
                if left.observation_id == right.observation_id:
                    continue
                actual = left.physical_leaf_id == right.physical_leaf_id
                predicted = hypothesis_id(left) == hypothesis_id(right)
                tp += int(actual and predicted)
                fp += int(not actual and predicted)
                fn += int(actual and not predicted)
                tn += int(not actual and not predicted)
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    return {
        "true_positive": tp, "false_positive": fp, "false_negative": fn,
        "true_negative": tn, "precision": precision, "recall": recall,
        "f1": 2 * precision * recall / (precision + recall) if precision + recall else 0.0,
        "false_merge_rate": fp / (fp + tn) if fp + tn else 0.0,
        "false_split_rate": fn / (fn + tp) if fn + tp else 0.0,
    }
