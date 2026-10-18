"""Structural validation for the active selectable leaf-backend config."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping, Sequence

from canonical_leaf_classes import CANONICAL_CLASS_NAMES


def _require_mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _validate_probability(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if not 0.0 <= result <= 1.0:
        raise ValueError(f"{label} must be in [0, 1]")
    return result


def validate_deployment_config(config: Mapping[str, object]) -> None:
    if config.get("schema_version") != 1:
        raise ValueError("unsupported leaf deployment config schema")
    leaf = _require_mapping(config.get("leaf_detector"), "leaf_detector")
    backend = leaf.get("backend")
    if not isinstance(backend, str) or not backend.strip():
        raise ValueError("leaf detector backend must be a non-empty string")
    profiles = _require_mapping(leaf.get("profiles"), "leaf_detector.profiles")
    if not profiles:
        raise ValueError("leaf_detector.profiles must not be empty")
    if backend not in profiles:
        raise ValueError("selected leaf detector backend has no configured profile")

    for name in profiles:
        if not isinstance(name, str) or not name.strip():
            raise ValueError("backend profile names must be non-empty strings")
        profile = _require_mapping(profiles[name], f"profiles.{name}")
        checkpoint = profile.get("checkpoint")
        manifest = profile.get("manifest")
        if not isinstance(checkpoint, str) or not Path(checkpoint).is_absolute():
            raise ValueError(f"profiles.{name}.checkpoint must be an absolute path")
        if not isinstance(manifest, str) or not Path(manifest).is_absolute():
            raise ValueError(f"profiles.{name}.manifest must be an absolute path")
        if profile.get("framework") not in {"ultralytics", "rfdetr", "dfine"}:
            raise ValueError(
                f"profiles.{name}.framework must be ultralytics, rfdetr, or dfine"
            )

        confidence = _require_mapping(profile.get("confidence"), f"profiles.{name}.confidence")
        _validate_probability(confidence.get("model"), f"profiles.{name}.confidence.model")
        per_class = _require_mapping(confidence.get("per_class"), f"profiles.{name}.confidence.per_class")
        for class_name, value in per_class.items():
            if class_name not in CANONICAL_CLASS_NAMES:
                raise ValueError(f"unknown per-class threshold name: {class_name!r}")
            _validate_probability(value, f"profiles.{name}.confidence.per_class.{class_name}")

        native_resolution = profile.get("native_resolution")
        if not isinstance(native_resolution, list) or len(native_resolution) != 2 or not all(
            isinstance(value, int) and not isinstance(value, bool) and value > 0
            for value in native_resolution
        ):
            raise ValueError(
                f"profiles.{name}.native_resolution must contain two positive integers"
            )
        if not isinstance(profile.get("adapter_version"), str) or not profile[
            "adapter_version"
        ].strip():
            raise ValueError(f"profiles.{name}.adapter_version must be a non-empty string")
        if not isinstance(profile.get("resize_policy"), str) or not profile[
            "resize_policy"
        ].strip():
            raise ValueError(f"profiles.{name}.resize_policy must be a non-empty string")
        launcher = _require_mapping(profile.get("launcher"), f"profiles.{name}.launcher")
        command = launcher.get("command")
        if not isinstance(command, list) or not command or not all(
            isinstance(part, str) and part for part in command
        ):
            raise ValueError(f"profiles.{name}.launcher.command must be a non-empty argv list")

        environment = _require_mapping(
            launcher.get("environment", {}),
            f"profiles.{name}.launcher.environment",
        )
        for key, value in environment.items():
            if not isinstance(key, str) or not key.strip():
                raise ValueError(
                    f"profiles.{name}.launcher.environment keys must be non-empty strings"
                )
            if key == "PYTHONPATH":
                raise ValueError(
                    f"profiles.{name}.launcher.environment must not define PYTHONPATH; "
                    "use launcher.python_path"
                )
            if not isinstance(value, str):
                raise ValueError(
                    f"profiles.{name}.launcher.environment.{key} must be a string"
                )

        python_path = launcher.get("python_path")
        if (
            not isinstance(python_path, Sequence)
            or isinstance(python_path, (str, bytes))
            or not python_path
        ):
            raise ValueError(
                f"profiles.{name}.launcher.python_path must be a non-empty sequence"
            )
        for index, entry in enumerate(python_path):
            if not isinstance(entry, str) or not entry.strip():
                raise ValueError(
                    f"profiles.{name}.launcher.python_path[{index}] must be a non-empty string"
                )
            if not Path(entry).is_absolute():
                raise ValueError(
                    f"profiles.{name}.launcher.python_path[{index}] must be an absolute path"
                )

        working_directory = launcher.get("working_directory")
        if working_directory is not None and (
            not isinstance(working_directory, str) or not working_directory.strip()
        ):
            raise ValueError(
                f"profiles.{name}.launcher.working_directory must be a non-empty string"
            )
        if working_directory is not None and not Path(working_directory).is_absolute():
            raise ValueError(
                f"profiles.{name}.launcher.working_directory must be an absolute path"
            )
        runtime_options = profile.get("runtime_options", {})
        runtime_options = _require_mapping(
            runtime_options, f"profiles.{name}.runtime_options"
        )
        device_policy = runtime_options.get("device_policy", "cuda_required")
        if device_policy != "cuda_required":
            raise ValueError(
                f"profiles.{name}.runtime_options.device_policy must be "
                "cuda_required; no CPU fallback has been validated"
            )
        expected_commit = runtime_options.get("expected_source_commit")
        if expected_commit is not None and (
            not isinstance(expected_commit, str)
            or len(expected_commit) != 40
            or any(character not in "0123456789abcdef" for character in expected_commit)
        ):
            raise ValueError(
                f"profiles.{name}.runtime_options.expected_source_commit "
                "must be a lowercase 40-character Git commit"
            )
        model_config = runtime_options.get("model_config")
        if model_config is not None and (
            not isinstance(model_config, str) or not model_config.strip()
        ):
            raise ValueError(
                f"profiles.{name}.runtime_options.model_config must be a non-empty string"
            )

    inference = _require_mapping(leaf.get("inference"), "leaf_detector.inference")
    if not isinstance(inference.get("full_roi"), bool):
        raise ValueError("full_roi must be boolean")
    if not isinstance(inference.get("tile_enabled"), bool):
        raise ValueError("tile_enabled must be boolean")
    tile_size = inference.get("tile_size")
    if not isinstance(tile_size, list) or len(tile_size) != 2 or not all(
        isinstance(value, int) and not isinstance(value, bool) and value > 0
        for value in tile_size
    ):
        raise ValueError("tile_size must contain two positive integers")
    overlap = _validate_probability(inference.get("overlap"), "leaf_detector.inference.overlap")
    if overlap >= 1.0:
        raise ValueError("tile overlap must be less than 1")

    deadline = _require_mapping(leaf.get("deadline"), "leaf_detector.deadline")
    if not isinstance(deadline.get("startup_ms"), int) or deadline["startup_ms"] <= 0:
        raise ValueError("startup deadline must be a positive integer")
    if not isinstance(deadline.get("request_ms"), int) or deadline["request_ms"] <= 0:
        raise ValueError("request deadline must be a positive integer")
    _require_mapping(leaf.get("merge"), "leaf_detector.merge")

    aggregation = _require_mapping(config.get("multi_frame"), "multi_frame")
    for key in (
        "minimum_observations",
        "temporal_sampling",
        "same_leaf_association",
        "insufficient_evidence_outcome",
    ):
        if key not in aggregation:
            raise ValueError(f"multi_frame.{key} is required")
    from leaf_association import AssociationPolicy
    from multi_frame_policy import TemporalPolicy, TEMPORAL_MULTI_VIEW_SUPPORT_V1
    from multi_view_inspection import MultiViewPolicy, NavigationSafetyPolicy

    AssociationPolicy.from_mapping(
        _require_mapping(
            aggregation.get("same_leaf_association"),
            "multi_frame.same_leaf_association",
        )
    )
    if aggregation.get("primary_aggregation") != TEMPORAL_MULTI_VIEW_SUPPORT_V1:
        raise ValueError("multi_frame.primary_aggregation must select the frozen temporal policy")
    TemporalPolicy.from_mapping(
        _require_mapping(aggregation.get("temporal_sampling"), "multi_frame.temporal_sampling")
    )
    fallback = _require_mapping(aggregation.get("fallback"), "multi_frame.fallback")
    if not isinstance(fallback.get("enabled"), bool):
        raise ValueError("multi_frame.fallback.enabled must be boolean")
    if fallback.get("mode") != "temporally_sampled_frame_support":
        raise ValueError("multi_frame.fallback.mode is unsupported")
    inspection_policy = _require_mapping(
        config.get("inspection_policy"), "inspection_policy"
    )
    view_policy = MultiViewPolicy.from_mapping(inspection_policy)
    navigation_safety = NavigationSafetyPolicy.from_mapping(
        _require_mapping(
            inspection_policy.get("navigation_safety"),
            "inspection_policy.navigation_safety",
        )
    )
    if navigation_safety.recovery_clearance_m > view_policy.radius_m:
        raise ValueError(
            "navigation recovery clearance must not exceed the inspection radius"
        )
    from localization_stability import LocalizationGuardPolicy
    LocalizationGuardPolicy.from_mapping(
        _require_mapping(config.get("localization_guard"), "localization_guard")
    )
    experiment = _require_mapping(config.get("experiment_context"), "experiment_context")
    if not isinstance(experiment.get("version"), str) or not experiment["version"]:
        raise ValueError("experiment_context.version is required")
    if not isinstance(experiment.get("world_path"), str) or not Path(experiment["world_path"]).is_absolute():
        raise ValueError("experiment_context.world_path must be absolute")


def load_deployment_config(path: str | Path) -> dict[str, object]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load leaf deployment config") from exc
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    validate_deployment_config(data)
    return data
