"""Deterministic cache fingerprints for result-affecting pipeline inputs."""

from __future__ import annotations

import hashlib
import json
from typing import Mapping

from checkpoint_manifest import validate_checkpoint_manifest
from leaf_deployment_config import validate_deployment_config


RESULT_AFFECTING_KEYS = (
    "checkpoint_sha256",
    "backend_name",
    "framework_adapter_version",
    "backend_runtime_options",
    "class_table_sha256",
    "confidence_threshold",
    "per_class_thresholds",
    "native_resize_policy",
    "tile_configuration",
    "merge_configuration",
    "frame_sampling_configuration",
    "multi_view_policy",
    "association_policy",
    "aggregation_policy",
)


def build_pipeline_fingerprint(inputs: Mapping[str, object]) -> str:
    missing = [key for key in RESULT_AFFECTING_KEYS if key not in inputs]
    if missing:
        raise ValueError(f"missing result-affecting fingerprint fields: {missing}")
    payload = dict(inputs)
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def deployment_fingerprint_inputs(
    config: Mapping[str, object], manifest: Mapping[str, object]
) -> dict[str, object]:
    """Derive cache inputs from the selected profile and checkpoint manifest."""
    validate_deployment_config(config)
    validate_checkpoint_manifest(manifest)

    leaf = config["leaf_detector"]
    backend = leaf["backend"]
    profile = leaf["profiles"][backend]
    if manifest["candidate"] != backend:
        raise ValueError("selected backend and checkpoint manifest candidate differ")
    if manifest["absolute_path"] != profile["checkpoint"]:
        raise ValueError("selected checkpoint path and manifest path differ")

    confidence = profile["confidence"]
    checkpoint_identity = {
        "status": manifest["status"],
        "sha256": manifest["sha256"],
        "file_size_bytes": manifest["file_size_bytes"],
    }
    return {
        "checkpoint_sha256": checkpoint_identity,
        "backend_name": backend,
        "framework_adapter_version": profile["adapter_version"],
        "backend_runtime_options": profile.get("runtime_options", {}),
        "class_table_sha256": manifest["class_table_sha256"],
        "confidence_threshold": confidence["model"],
        "per_class_thresholds": confidence["per_class"],
        "native_resize_policy": {
            "resolution": profile["native_resolution"],
            "policy": profile["resize_policy"],
        },
        "tile_configuration": leaf["inference"],
        "merge_configuration": leaf["merge"],
        "frame_sampling_configuration": config["multi_frame"],
        "multi_view_policy": config["inspection_policy"],
        "association_policy": config["multi_frame"]["same_leaf_association"],
        "aggregation_policy": {
            "primary": config["multi_frame"]["primary_aggregation"],
            "temporal": config["multi_frame"]["temporal_sampling"],
            "fallback": config["multi_frame"]["fallback"],
        },
    }


def build_deployment_fingerprint(
    config: Mapping[str, object], manifest: Mapping[str, object]
) -> str:
    return build_pipeline_fingerprint(
        deployment_fingerprint_inputs(config, manifest)
    )
