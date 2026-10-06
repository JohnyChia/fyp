"""Backend-neutral temporal multi-view aggregation for production inference."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite
from typing import Iterable, Mapping
from canonical_leaf_classes import canonical_class_id
from unknown_outcomes import UnknownReason

TEMPORAL_MULTI_VIEW_SUPPORT_V1 = "TEMPORAL_MULTI_VIEW_SUPPORT_V1"


@dataclass(frozen=True)
class FrameDetectionEvidence:
    observation_id: str
    viewpoint_id: str
    timestamp: float
    class_counts: Mapping[str, int]
    class_confidence_sums: Mapping[str, float]

    def __post_init__(self):
        if not self.observation_id or not self.viewpoint_id or not isfinite(self.timestamp):
            raise ValueError("valid observation/viewpoint/timestamp required")
        if set(self.class_counts) != set(self.class_confidence_sums):
            raise ValueError("frame count/confidence class sets differ")
        for name, count in self.class_counts.items():
            canonical_class_id(name)
            confidence = float(self.class_confidence_sums[name])
            if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
                raise ValueError("frame class counts must be positive integers")
            if not isfinite(confidence) or not 0 <= confidence <= count:
                raise ValueError("frame confidence sum is invalid")


@dataclass(frozen=True)
class TemporalPolicy:
    version: str
    status: str
    sampling_interval_sec: float
    maximum_sampled_frames_per_viewpoint: int
    minimum_valid_frames_per_viewpoint: int
    viewpoint_weighting: str

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]):
        if value.get("version") != TEMPORAL_MULTI_VIEW_SUPPORT_V1:
            raise ValueError("unsupported temporal aggregation version")
        interval, maximum = value.get("sampling_interval_sec"), value.get("maximum_sampled_frames_per_viewpoint")
        minimum = value.get("minimum_valid_frames_per_viewpoint")
        if not isinstance(interval, (int, float)) or isinstance(interval, bool) or interval < 0:
            raise ValueError("sampling_interval_sec must be non-negative")
        if not isinstance(maximum, int) or isinstance(maximum, bool) or maximum <= 0:
            raise ValueError("maximum_sampled_frames_per_viewpoint must be positive")
        if not isinstance(minimum, int) or isinstance(minimum, bool) or not 0 < minimum <= maximum:
            raise ValueError("minimum_valid_frames_per_viewpoint must be within maximum")
        if value.get("viewpoint_weighting") != "equal_one_vote":
            raise ValueError("viewpoint_weighting must be equal_one_vote")
        if value.get("status") != "FROZEN":
            raise ValueError("production temporal policy must be FROZEN")
        return cls(str(value["version"]), str(value["status"]), float(interval), maximum,
                   minimum, str(value["viewpoint_weighting"]))


@dataclass(frozen=True)
class TemporalAggregation:
    class_name: str | None
    confidence: float
    unknown_reason: str | None
    viewpoint_count: int
    valid_viewpoint_count: int
    sampled_frame_count: int
    viewpoint_votes: Mapping[str, int]
    diagnostics: Mapping[str, object]


def _frame_dominant(frame):
    ranked = [(-count, -float(frame.class_confidence_sums[name]) / count,
               canonical_class_id(name), name)
              for name, count in frame.class_counts.items()]
    return min(ranked)[3] if ranked else None


def sample_frames(frames: Iterable[FrameDetectionEvidence], policy: TemporalPolicy):
    selected, last = [], None
    for frame in sorted(frames, key=lambda row: (row.timestamp, row.observation_id)):
        if last is not None and frame.timestamp - last < policy.sampling_interval_sec:
            continue
        selected.append(frame)
        last = frame.timestamp
        if len(selected) >= policy.maximum_sampled_frames_per_viewpoint:
            break
    return tuple(selected)


def aggregate_temporal_multi_view(frames: Iterable[FrameDetectionEvidence], policy: TemporalPolicy,
                                  roi_count: int, backend_detection_count: int,
                                  required_viewpoint_ids: Iterable[str] = ()):
    rows = tuple(frames)
    required = tuple(dict.fromkeys(required_viewpoint_ids))
    viewpoint_ids = required or tuple(sorted({row.viewpoint_id for row in rows}))
    results, total_sampled = {}, 0
    for viewpoint_id in viewpoint_ids:
        sampled = sample_frames((row for row in rows if row.viewpoint_id == viewpoint_id), policy)
        valid = [row for row in sampled if row.class_counts]
        total_sampled += len(sampled)
        if len(valid) < policy.minimum_valid_frames_per_viewpoint:
            results[viewpoint_id] = {"valid": False, "sampled_frames": len(sampled)}
            continue
        votes, support, conf, detections = {}, {}, {}, {}
        for frame in valid:
            dominant = _frame_dominant(frame)
            if dominant:
                votes[dominant] = votes.get(dominant, 0) + 1
            for name, count in frame.class_counts.items():
                support[name] = support.get(name, 0) + count
                conf[name] = conf.get(name, 0.0) + float(frame.class_confidence_sums[name])
                detections[name] = detections.get(name, 0) + count
        ranked = [(-count, -support[name] / len(valid), -conf[name] / detections[name],
                   canonical_class_id(name), name) for name, count in votes.items()]
        winner = min(ranked)[4]
        total_support = sum(support.values())
        results[viewpoint_id] = {
            "valid": True, "sampled_frames": len(sampled), "valid_frames": len(valid),
            "winner": winner, "frame_votes": votes,
            "normalized_support": {name: value / total_support for name, value in support.items()},
            "mean_confidence": {name: conf[name] / detections[name] for name in detections},
        }
    valid_views = [value for value in results.values() if value.get("valid")]
    if not valid_views:
        reason = UnknownReason.NO_ROI if roi_count == 0 else (UnknownReason.NO_DETECTION
                 if backend_detection_count == 0 else UnknownReason.NO_VALID_EVIDENCE)
        return TemporalAggregation(None, 0.0, reason.value, len(viewpoint_ids), 0,
                                   total_sampled, {}, {"viewpoints": results})
    view_votes, support, confidence = {}, {}, {}
    for view in valid_views:
        winner = view["winner"]
        view_votes[winner] = view_votes.get(winner, 0) + 1
        for name, value in view["normalized_support"].items():
            support.setdefault(name, []).append(value)
        for name, value in view["mean_confidence"].items():
            confidence.setdefault(name, []).append(value)
    ranked = []
    for name, votes in view_votes.items():
        normalized = sum(support.get(name, ())) / len(valid_views)
        mean_conf = sum(confidence.get(name, ())) / len(confidence.get(name, ()))
        ranked.append((-votes, -normalized, -mean_conf, canonical_class_id(name), name, mean_conf))
    winner = min(ranked)
    return TemporalAggregation(winner[4], winner[5], None, len(viewpoint_ids), len(valid_views),
                               total_sampled, view_votes, {"viewpoints": results})


def exact_repeated_evidence_pairs(evidence):
    """Legacy diagnostic retained; temporal production does not use bbox area."""
    rows, repeats = list(evidence), []
    for left_index, left in enumerate(rows):
        for right_index in range(left_index + 1, len(rows)):
            right = rows[right_index]
            if getattr(left, "observation_id", None) != getattr(right, "observation_id", None) and left == right:
                repeats.append((left_index, right_index))
    return repeats
