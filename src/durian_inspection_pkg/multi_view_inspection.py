"""Config-driven geometry and completion tracking for multi-view inspection."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping


@dataclass(frozen=True)
class Viewpoint:
    viewpoint_id: str
    index: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MultiViewPolicy:
    version: str
    enabled: bool
    radius_m: float
    angular_offsets_deg: tuple[float, ...]
    capture_duration_sec: float

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "MultiViewPolicy":
        offsets = value.get("angular_offsets_deg")
        if not isinstance(offsets, list) or not offsets:
            raise ValueError("inspection_policy.angular_offsets_deg must be a non-empty list")
        if not all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in offsets):
            raise ValueError("inspection viewpoint offsets must be numeric")
        radius = value.get("radius_m")
        duration = value.get("capture_duration_sec")
        if not isinstance(radius, (int, float)) or isinstance(radius, bool) or radius <= 0:
            raise ValueError("inspection_policy.radius_m must be positive")
        if not isinstance(duration, (int, float)) or isinstance(duration, bool) or duration <= 0:
            raise ValueError("inspection_policy.capture_duration_sec must be positive")
        version = value.get("version")
        if not isinstance(version, str) or not version.strip():
            raise ValueError("inspection_policy.version is required")
        enabled = value.get("enabled")
        if not isinstance(enabled, bool):
            raise ValueError("inspection_policy.enabled must be boolean")
        selected = offsets if enabled else offsets[:1]
        return cls(version, enabled, float(radius), tuple(map(float, selected)), float(duration))

    def viewpoints(self, tree_x: float, tree_y: float, base_angle_rad: float = 0.0) -> tuple[Viewpoint, ...]:
        rows = []
        for index, degrees in enumerate(self.angular_offsets_deg):
            angle = base_angle_rad + math.radians(degrees)
            x = float(tree_x) + self.radius_m * math.cos(angle)
            y = float(tree_y) + self.radius_m * math.sin(angle)
            yaw = math.atan2(float(tree_y) - y, float(tree_x) - x)
            rows.append(Viewpoint(f"view_{index:03d}", index, x, y, yaw))
        return tuple(rows)


@dataclass(frozen=True)
class NavigationSafetyPolicy:
    """Configuration for bounded near-tree navigation recovery."""

    watchdog_min_distance_m: float
    goal_min_distance_m: float
    recovery_clearance_m: float
    max_recovery_attempts_per_leg: int
    recovery_linear_speed_mps: float
    recovery_angular_speed_radps: float
    recovery_align_tolerance_rad: float
    recovery_timeout_sec: float

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "NavigationSafetyPolicy":
        required_positive = (
            "watchdog_min_distance_m",
            "goal_min_distance_m",
            "recovery_clearance_m",
            "recovery_linear_speed_mps",
            "recovery_angular_speed_radps",
            "recovery_align_tolerance_rad",
            "recovery_timeout_sec",
        )
        parsed: dict[str, float] = {}
        for key in required_positive:
            raw = value.get(key)
            if isinstance(raw, bool) or not isinstance(raw, (int, float)) or raw <= 0:
                raise ValueError(f"inspection_policy.navigation_safety.{key} must be positive")
            parsed[key] = float(raw)

        attempts = value.get("max_recovery_attempts_per_leg")
        if isinstance(attempts, bool) or not isinstance(attempts, int) or attempts <= 0:
            raise ValueError(
                "inspection_policy.navigation_safety.max_recovery_attempts_per_leg "
                "must be a positive integer"
            )
        if not (
            parsed["watchdog_min_distance_m"]
            < parsed["goal_min_distance_m"]
            <= parsed["recovery_clearance_m"]
        ):
            raise ValueError(
                "navigation safety distances must satisfy watchdog < goal <= clearance"
            )
        return cls(
            **parsed,
            max_recovery_attempts_per_leg=attempts,
        )


def recovery_motion_command(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    tree_x: float,
    tree_y: float,
    policy: NavigationSafetyPolicy,
) -> tuple[float, float, bool]:
    """Return linear/angular velocity that moves away from the target tree."""
    distance = math.hypot(robot_x - tree_x, robot_y - tree_y)
    if distance >= policy.recovery_clearance_m:
        return 0.0, 0.0, True

    away_yaw = math.atan2(robot_y - tree_y, robot_x - tree_x)
    yaw_error = (away_yaw - robot_yaw + math.pi) % (2.0 * math.pi) - math.pi
    if abs(yaw_error) > policy.recovery_align_tolerance_rad:
        angular = math.copysign(policy.recovery_angular_speed_radps, yaw_error)
        return 0.0, angular, False
    return policy.recovery_linear_speed_mps, 0.0, False


class ViewCompletionTracker:
    """Tracks arbitrary tree/view inventories without assuming numeric continuity."""

    def __init__(self, tree_ids: Iterable[int], required_view_ids: Iterable[str]):
        trees = tuple(tree_ids)
        views = tuple(required_view_ids)
        if not trees or len(set(trees)) != len(trees):
            raise ValueError("tree inventory must be non-empty and unique")
        if not views or len(set(views)) != len(views):
            raise ValueError("viewpoint inventory must be non-empty and unique")
        self._required = {(tree_id, view_id) for tree_id in trees for view_id in views}
        self._completed: set[tuple[int, str]] = set()

    def complete(self, tree_id: int, viewpoint_id: str) -> None:
        key = (tree_id, viewpoint_id)
        if key not in self._required:
            raise ValueError(f"unexpected tree/viewpoint work item: {key!r}")
        self._completed.add(key)

    def tree_complete(self, tree_id: int) -> bool:
        required = {item for item in self._required if item[0] == tree_id}
        return bool(required) and required <= self._completed

    @property
    def inventory_complete(self) -> bool:
        return self._required <= self._completed
