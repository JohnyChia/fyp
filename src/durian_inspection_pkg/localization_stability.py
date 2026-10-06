"""Generic continuity checks for an authoritative navigation pose stream."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping


def _positive_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ValueError(f"{label} must be positive")
    return float(value)


@dataclass(frozen=True)
class LocalizationGuardPolicy:
    enabled: bool
    source: str
    global_frame: str
    odom_frame: str
    max_translation_step_m: float
    max_yaw_step_rad: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    stationary_linear_speed_mps: float
    stationary_angular_speed_radps: float
    stationary_translation_drift_m: float
    stationary_yaw_drift_rad: float
    settling_interval_sec: float
    settling_timeout_sec: float
    required_consecutive_good_samples: int
    tf_unavailable_timeout_sec: float

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "LocalizationGuardPolicy":
        enabled = value.get("enabled")
        if not isinstance(enabled, bool):
            raise ValueError("localization_guard.enabled must be boolean")
        strings = {}
        for key in ("source", "global_frame", "odom_frame"):
            raw = value.get(key)
            if not isinstance(raw, str) or not raw.strip():
                raise ValueError(f"localization_guard.{key} must be a non-empty string")
            strings[key] = raw.strip()
        numbers = {}
        for key in (
            "max_translation_step_m",
            "max_yaw_step_rad",
            "max_linear_speed_mps",
            "max_angular_speed_radps",
            "stationary_linear_speed_mps",
            "stationary_angular_speed_radps",
            "stationary_translation_drift_m",
            "stationary_yaw_drift_rad",
            "settling_interval_sec",
            "settling_timeout_sec",
            "tf_unavailable_timeout_sec",
        ):
            numbers[key] = _positive_number(value.get(key), f"localization_guard.{key}")
        samples = value.get("required_consecutive_good_samples")
        if isinstance(samples, bool) or not isinstance(samples, int) or samples <= 0:
            raise ValueError(
                "localization_guard.required_consecutive_good_samples must be positive"
            )
        return cls(
            enabled=enabled,
            required_consecutive_good_samples=samples,
            **strings,
            **numbers,
        )


@dataclass(frozen=True)
class PoseSample:
    timestamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ContinuityResult:
    accepted: bool
    translation_delta_m: float
    yaw_delta_rad: float
    stationary: bool
    reason: str = ""
    elapsed_sec: float = 0.0
    allowed_translation_m: float = 0.0
    allowed_yaw_rad: float = 0.0


def angular_distance(first: float, second: float) -> float:
    return abs((second - first + math.pi) % (2.0 * math.pi) - math.pi)


def evaluate_pose_continuity(
    previous: PoseSample | None,
    current: PoseSample,
    linear_speed_mps: float,
    angular_speed_radps: float,
    policy: LocalizationGuardPolicy,
    previous_was_stationary: bool | None = None,
) -> ContinuityResult:
    """Reject discontinuities while allowing motion bounded by speed and time."""
    stationary = (
        abs(linear_speed_mps) <= policy.stationary_linear_speed_mps
        and abs(angular_speed_radps) <= policy.stationary_angular_speed_radps
    )
    if previous is None or not policy.enabled:
        return ContinuityResult(True, 0.0, 0.0, stationary)

    elapsed = max(0.0, current.timestamp_sec - previous.timestamp_sec)
    translation = math.hypot(current.x - previous.x, current.y - previous.y)
    yaw_delta = angular_distance(previous.yaw, current.yaw)
    # A sample taken just after the robot stops still spans part of the last
    # moving interval. Apply stationary drift limits only when both ends of
    # the sampled interval are known stationary.
    stationary_interval = stationary and previous_was_stationary is not False
    if stationary_interval:
        max_translation = policy.stationary_translation_drift_m
        max_yaw = policy.stationary_yaw_drift_rad
    else:
        max_translation = (
            policy.max_translation_step_m + policy.max_linear_speed_mps * elapsed
        )
        max_yaw = policy.max_yaw_step_rad + policy.max_angular_speed_radps * elapsed
    if translation > max_translation or yaw_delta > max_yaw:
        return ContinuityResult(
            False,
            translation,
            yaw_delta,
            stationary,
            "LOCALIZATION_POSE_JUMP",
            elapsed,
            max_translation,
            max_yaw,
        )
    return ContinuityResult(
        True,
        translation,
        yaw_delta,
        stationary,
        elapsed_sec=elapsed,
        allowed_translation_m=max_translation,
        allowed_yaw_rad=max_yaw,
    )


class StationaryStabilityGate:
    """Require consecutive stable stationary samples for a settling interval."""

    def __init__(self, policy: LocalizationGuardPolicy):
        self.policy = policy
        self.reset()

    def reset(self) -> None:
        self.started_at: float | None = None
        self.anchor: PoseSample | None = None
        self.good_samples = 0

    def observe(
        self,
        sample: PoseSample,
        linear_speed_mps: float,
        angular_speed_radps: float,
    ) -> bool:
        stationary = (
            abs(linear_speed_mps) <= self.policy.stationary_linear_speed_mps
            and abs(angular_speed_radps) <= self.policy.stationary_angular_speed_radps
        )
        if not stationary:
            self.reset()
            return False
        if self.anchor is None:
            self.anchor = sample
            self.started_at = sample.timestamp_sec
            self.good_samples = 1
            return False
        drift = math.hypot(sample.x - self.anchor.x, sample.y - self.anchor.y)
        yaw_drift = angular_distance(self.anchor.yaw, sample.yaw)
        if (
            drift > self.policy.stationary_translation_drift_m
            or yaw_drift > self.policy.stationary_yaw_drift_rad
        ):
            self.reset()
            return False
        self.good_samples += 1
        return (
            sample.timestamp_sec - float(self.started_at)
            >= self.policy.settling_interval_sec
            and self.good_samples >= self.policy.required_consecutive_good_samples
        )
