"""Framework-neutral RGB-D association and unique-leaf tree aggregation.

Association parameters are configuration, not model-specific branches.  A policy
whose status is not ``FROZEN`` is intentionally rejected by production callers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite, sqrt
from typing import Iterable, Mapping, Sequence

from canonical_leaf_classes import canonical_class_id, canonical_class_name
from unknown_outcomes import UnknownReason


class AssociationPolicyUnfrozen(RuntimeError):
    pass


@dataclass(frozen=True)
class AssociationPolicy:
    version: str
    status: str
    center_patch_radius_px: int
    minimum_valid_depth_samples: int
    maximum_match_distance_m: float
    evidence_rule: str
    aggregation_mode: str

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "AssociationPolicy":
        required = (
            "version", "status", "center_patch_radius_px",
            "minimum_valid_depth_samples", "maximum_match_distance_m",
            "evidence_rule", "aggregation_mode",
        )
        missing = [key for key in required if key not in value]
        if missing:
            raise ValueError(f"association policy missing fields: {missing}")
        radius = value["center_patch_radius_px"]
        samples = value["minimum_valid_depth_samples"]
        distance = value["maximum_match_distance_m"]
        if not isinstance(radius, int) or isinstance(radius, bool) or radius < 0:
            raise ValueError("center_patch_radius_px must be a non-negative integer")
        if not isinstance(samples, int) or isinstance(samples, bool) or samples <= 0:
            raise ValueError("minimum_valid_depth_samples must be a positive integer")
        if not isinstance(distance, (int, float)) or isinstance(distance, bool) or distance <= 0:
            raise ValueError("maximum_match_distance_m must be positive")
        if value["evidence_rule"] != "distinct_view_support_then_mean_confidence":
            raise ValueError("unsupported leaf evidence rule")
        if value["aggregation_mode"] != "unique_leaf_count":
            raise ValueError("unsupported primary aggregation mode")
        return cls(str(value["version"]), str(value["status"]), radius, samples,
                   float(distance), str(value["evidence_rule"]), str(value["aggregation_mode"]))

    def require_frozen(self) -> None:
        if self.status != "FROZEN":
            raise AssociationPolicyUnfrozen(
                f"association policy {self.version!r} is {self.status!r}; validation and freezing are required"
            )


@dataclass(frozen=True)
class Detection3D:
    observation_id: str
    viewpoint_id: str
    class_id: int
    confidence: float
    tree_xyz: tuple[float, float, float]

    def __post_init__(self):
        canonical_class_name(self.class_id)
        if not self.observation_id or not self.viewpoint_id:
            raise ValueError("observation and viewpoint IDs are required")
        if not isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError("confidence must be finite and in [0, 1]")
        if len(self.tree_xyz) != 3 or not all(isfinite(item) for item in self.tree_xyz):
            raise ValueError("tree_xyz must contain three finite values")


@dataclass
class LeafHypothesis:
    hypothesis_id: int
    centroid: tuple[float, float, float]
    evidence: list[Detection3D] = field(default_factory=list)


@dataclass(frozen=True)
class TreeAggregation:
    class_name: str | None
    confidence: float
    aggregation_mode: str
    unknown_reason: str | None
    unique_counts: Mapping[str, int]
    leaf_assignments: tuple[tuple[int, str, float], ...]


def robust_center_depth(depth, bbox_xyxy: Sequence[float], policy: AssociationPolicy,
                        minimum_depth_m: float, maximum_depth_m: float) -> float | None:
    """Return median valid center-patch depth; accepts NumPy-like 2-D arrays."""
    x1, y1, x2, y2 = map(float, bbox_xyxy)
    cx, cy = int(round((x1 + x2) / 2)), int(round((y1 + y2) / 2))
    height, width = depth.shape[:2]
    r = policy.center_patch_radius_px
    values = []
    for y in range(max(0, cy - r), min(height, cy + r + 1)):
        for x in range(max(0, cx - r), min(width, cx + r + 1)):
            value = float(depth[y, x])
            if isfinite(value) and minimum_depth_m <= value <= maximum_depth_m:
                values.append(value)
    if len(values) < policy.minimum_valid_depth_samples:
        return None
    values.sort()
    middle = len(values) // 2
    return values[middle] if len(values) % 2 else (values[middle - 1] + values[middle]) / 2.0


def project_pixel(u: float, v: float, depth_m: float, camera_k: Sequence[float]) -> tuple[float, float, float]:
    fx, fy, cx, cy = float(camera_k[0]), float(camera_k[4]), float(camera_k[2]), float(camera_k[5])
    if fx <= 0 or fy <= 0:
        raise ValueError("camera focal lengths must be positive")
    return ((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m)


def transform_point(point: Sequence[float], transform: Mapping[str, object]) -> tuple[float, float, float]:
    """Apply a geometry_msgs-style translation/quaternion transform."""
    t = transform["translation"]
    q = transform["rotation"]
    qx, qy, qz, qw = (float(q[key]) for key in ("x", "y", "z", "w"))
    norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm <= 0:
        raise ValueError("transform quaternion has zero norm")
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    x, y, z = map(float, point)
    # Quaternion-derived rotation matrix.
    rx = (1-2*(qy*qy+qz*qz))*x + 2*(qx*qy-qz*qw)*y + 2*(qx*qz+qy*qw)*z
    ry = 2*(qx*qy+qz*qw)*x + (1-2*(qx*qx+qz*qz))*y + 2*(qy*qz-qx*qw)*z
    rz = 2*(qx*qz-qy*qw)*x + 2*(qy*qz+qx*qw)*y + (1-2*(qx*qx+qy*qy))*z
    return rx + float(t["x"]), ry + float(t["y"]), rz + float(t["z"])


def associate_unique_leaves(detections: Iterable[Detection3D], policy: AssociationPolicy,
                            require_frozen: bool = True) -> tuple[LeafHypothesis, ...]:
    if require_frozen:
        policy.require_frozen()
    rows = sorted(detections, key=lambda row: (
        row.observation_id, row.viewpoint_id, row.tree_xyz, row.class_id, -row.confidence
    ))
    hypotheses: list[LeafHypothesis] = []
    seen = set()
    for row in rows:
        signature = (row.observation_id, row.class_id, row.tree_xyz)
        if signature in seen:
            continue
        seen.add(signature)
        candidates = []
        for hypothesis in hypotheses:
            # One physical hypothesis cannot consume two detections from the
            # same frame; this preserves adjacent leaves before cross-view matching.
            if any(item.observation_id == row.observation_id for item in hypothesis.evidence):
                continue
            distance = sqrt(sum((a-b)**2 for a, b in zip(row.tree_xyz, hypothesis.centroid)))
            if distance <= policy.maximum_match_distance_m:
                candidates.append((distance, hypothesis.hypothesis_id, hypothesis))
        if candidates:
            hypothesis = min(candidates)[2]
            hypothesis.evidence.append(row)
            points = [item.tree_xyz for item in hypothesis.evidence]
            hypothesis.centroid = tuple(sum(p[axis] for p in points) / len(points) for axis in range(3))
        else:
            hypotheses.append(LeafHypothesis(len(hypotheses), row.tree_xyz, [row]))
    return tuple(hypotheses)


def _assign_leaf(hypothesis: LeafHypothesis) -> tuple[str, float]:
    by_class_view: dict[int, dict[str, float]] = {}
    for row in hypothesis.evidence:
        per_view = by_class_view.setdefault(row.class_id, {})
        per_view[row.viewpoint_id] = max(per_view.get(row.viewpoint_id, 0.0), row.confidence)
    ranked = []
    for class_id, views in by_class_view.items():
        mean_confidence = sum(views.values()) / len(views)
        ranked.append((-len(views), -mean_confidence, class_id, mean_confidence))
    _, _, class_id, confidence = min(ranked)
    return canonical_class_name(class_id), confidence


def aggregate_tree(hypotheses: Iterable[LeafHypothesis], aggregation_mode: str = "unique_leaf_count",
                   roi_count: int = 0, backend_detection_count: int = 0) -> TreeAggregation:
    if aggregation_mode != "unique_leaf_count":
        raise ValueError("primary aggregation must be unique_leaf_count")
    leaves = list(hypotheses)
    if not leaves:
        reason = UnknownReason.NO_ROI if roi_count == 0 else (
            UnknownReason.NO_DETECTION if backend_detection_count == 0 else UnknownReason.NO_VALID_EVIDENCE
        )
        return TreeAggregation(None, 0.0, aggregation_mode, reason.value, {}, ())
    assignments = []
    counts: dict[str, int] = {}
    confidences: dict[str, list[float]] = {}
    for hypothesis in leaves:
        name, confidence = _assign_leaf(hypothesis)
        assignments.append((hypothesis.hypothesis_id, name, confidence))
        counts[name] = counts.get(name, 0) + 1
        confidences.setdefault(name, []).append(confidence)
    ranked = []
    for name, count in counts.items():
        mean_confidence = sum(confidences[name]) / len(confidences[name])
        ranked.append((-count, -mean_confidence, canonical_class_id(name), name, mean_confidence))
    _, _, _, winner, confidence = min(ranked)
    return TreeAggregation(winner, confidence, aggregation_mode, None, counts, tuple(assignments))
