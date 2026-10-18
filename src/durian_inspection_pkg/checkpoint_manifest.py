"""Checkpoint provenance helpers; no framework/model loading occurs here."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Mapping

from canonical_leaf_classes import class_table_sha256, validate_class_table


AVAILABLE = "AVAILABLE"
BLOCKER = "BLOCKER"
PASS = "PASS"
NOT_PROVEN = "NOT PROVEN"


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def checkpoint_identity(path: str | Path) -> dict[str, object]:
    checkpoint = Path(path).expanduser().resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    return {
        "filename": checkpoint.name,
        "absolute_path": str(checkpoint),
        "sha256": sha256_file(checkpoint),
        "file_size_bytes": checkpoint.stat().st_size,
    }


def validate_checkpoint_manifest(
    manifest: Mapping[str, object],
    *,
    require_available: bool = False,
    require_deployment_ready: bool = False,
) -> None:
    candidate = manifest.get("candidate")
    if not isinstance(candidate, str) or not candidate.strip():
        raise ValueError("checkpoint candidate must be a non-empty string")
    status = manifest.get("status")
    if status not in {AVAILABLE, BLOCKER}:
        raise ValueError("manifest status must be AVAILABLE or BLOCKER")

    validate_class_table(manifest.get("canonical_class_order"))
    if manifest.get("class_table_sha256") != class_table_sha256():
        raise ValueError("manifest class-table hash is incorrect")

    threshold = manifest.get("selected_validation_threshold")
    if isinstance(threshold, bool) or not isinstance(threshold, (int, float)):
        raise ValueError("selected validation threshold must be numeric")
    if not 0.0 <= float(threshold) <= 1.0:
        raise ValueError("selected validation threshold must be in [0, 1]")

    if require_available and status != AVAILABLE:
        raise ValueError(f"{candidate} checkpoint remains unavailable")

    deployment_status = manifest.get("deployment_status")
    if deployment_status is not None and deployment_status not in {PASS, BLOCKER}:
        raise ValueError("deployment status must be PASS or BLOCKER")
    class_verification = manifest.get("checkpoint_class_order_verification")
    if class_verification not in {PASS, NOT_PROVEN}:
        raise ValueError("checkpoint class-order verification must be PASS or NOT PROVEN")

    if status == BLOCKER:
        if manifest.get("sha256") is not None or manifest.get("file_size_bytes") is not None:
            raise ValueError("unavailable checkpoint must not claim a hash or size")
        if require_deployment_ready:
            raise ValueError(f"{candidate} checkpoint is not deployment-ready")
        return

    path = Path(str(manifest.get("absolute_path", ""))).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    if path.name != manifest.get("filename"):
        raise ValueError("manifest filename does not match absolute path")
    if path.stat().st_size != manifest.get("file_size_bytes"):
        raise ValueError("checkpoint file size does not match manifest")
    if sha256_file(path) != manifest.get("sha256"):
        raise ValueError("checkpoint SHA-256 does not match manifest")
    if require_deployment_ready:
        if deployment_status != PASS:
            raise ValueError(f"{candidate} checkpoint is not deployment-ready")
        if class_verification != PASS:
            raise ValueError("available checkpoint class order has not passed verification")


def load_checkpoint_manifest(path: str | Path) -> dict[str, object]:
    manifest = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_checkpoint_manifest(manifest)
    return manifest
