#!/usr/bin/env python3
"""Create, never overwrite, an isolated Run04 V4.2 ROS2 deployment bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml


EXPECTED_SHA = "04fa8dc9fa7df9bb238ab42469dee857caedb1c36d2d02bdf252bb9e831bc6f5"
CHECKPOINT = Path("/home/johny/durian_training/models_8run/model4_yolo26n_p2_hybrid/best.pt")
ACTIVE = Path("/home/johny/durian_ws/src/durian_inspection_pkg/config/leaf_detector_deployment.yaml")
PYTHON = "/home/johny/durian_ws/experiment/analysis/dev_validation_audit/phase2c/.venv/bin/python"
PYTHON_PATH = "/home/johny/durian_ws/src/durian_inspection_pkg"
CLASS_NAMES = (
    "leaf_algal", "leaf_blight", "leaf_colletotrichum",
    "leaf_healthy", "leaf_phomopsis", "leaf_rhizoctonia",
)
CLASS_TABLE_SHA = "35143075880cba0d446cb16045a559657b14d5b984c209ea96d39bc00f180642"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def create(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(text)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(f"create-only output already exists: {output}")
    if not CHECKPOINT.is_file() or sha256(CHECKPOINT) != EXPECTED_SHA:
        raise RuntimeError("Run04 V4.2 checkpoint identity mismatch")

    active_sha_before = sha256(ACTIVE)
    config = yaml.safe_load(ACTIVE.read_text(encoding="utf-8"))
    if config["leaf_detector"]["backend"] != "fyp_run01":
        raise RuntimeError("active deployment is no longer Run01")

    manifest_path = output / "RUN04_V42_ISOLATED_MANIFEST.json"
    config_path = output / "RUN04_V42_ISOLATED_DEPLOYMENT.yaml"
    manifest = {
        "schema_version": 1,
        "candidate": "fyp_run04_v42",
        "status": "AVAILABLE",
        "deployment_status": "BLOCKER",
        "filename": CHECKPOINT.name,
        "absolute_path": str(CHECKPOINT),
        "sha256": EXPECTED_SHA,
        "file_size_bytes": CHECKPOINT.stat().st_size,
        "architecture": "Ultralytics YOLO26n-P2",
        "framework": "ultralytics",
        "framework_version": "8.4.152",
        "source_experiment": "formal six-model engineering comparison Run04 V4.2 corrected Hybrid",
        "native_inference_resolution": [640, 640],
        "canonical_class_order": [
            {"id": index, "name": name} for index, name in enumerate(CLASS_NAMES)
        ],
        "class_table_sha256": CLASS_TABLE_SHA,
        "checkpoint_class_order_verification": "PASS",
        "checkpoint_evidence": {"foreground_classes": 6, "registry_role": "V4_2_ENGINEERING_CANDIDATE"},
        "selected_validation_threshold": 0.45,
        "runtime_smoke_verification": "PENDING_EXACT_SHA_CUDA_ROS2_GAZEBO_E2E",
        "blocker": "Do not promote until the isolated exact-SHA E2E receipt passes every frozen gate."
    }
    profile = {
        "framework": "ultralytics",
        "checkpoint": str(CHECKPOINT),
        "manifest": str(manifest_path),
        "confidence": {"model": 0.45, "per_class": {}},
        "native_resolution": [640, 640],
        "resize_policy": "framework_native_rect_original_pixel_inverse_map",
        "adapter_version": "frozen-common-evaluator-v1",
        "runtime_options": {"device_policy": "cuda_required"},
        "launcher": {
            "command": [PYTHON, "-m", "leaf_backend_runtime"],
            "environment": {"PYTHONNOUSERSITE": "1"},
            "python_path": [PYTHON_PATH],
        },
    }
    config["leaf_detector"]["profiles"]["fyp_run04_v42"] = profile
    config["leaf_detector"]["backend"] = "fyp_run04_v42"

    create(manifest_path, json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    create(config_path, yaml.safe_dump(config, sort_keys=False))

    import sys
    sys.path.insert(0, PYTHON_PATH)
    from leaf_backend_runtime import LeafBackendSpec
    spec = LeafBackendSpec.from_config(config_path)
    if spec.checkpoint_sha256 != EXPECTED_SHA or spec.threshold != 0.45:
        raise RuntimeError("isolated config did not bind the frozen candidate")
    if sha256(ACTIVE) != active_sha_before:
        raise RuntimeError("active Run01 configuration changed unexpectedly")

    receipt = {
        "schema_version": "durian.run04_v42_isolated_bundle.v1",
        "status": "PASS",
        "active_config_path": str(ACTIVE),
        "active_config_sha256_before_after": active_sha_before,
        "active_backend_preserved": "fyp_run01",
        "isolated_backend": spec.name,
        "checkpoint_sha256": spec.checkpoint_sha256,
        "threshold": spec.threshold,
        "manifest": str(manifest_path),
        "manifest_sha256": sha256(manifest_path),
        "deployment_config": str(config_path),
        "deployment_config_sha256": sha256(config_path),
        "sealed_test_accessed": False,
    }
    receipt_path = output / "ISOLATED_BUNDLE_RECEIPT.json"
    create(receipt_path, json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    print(json.dumps(receipt, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
