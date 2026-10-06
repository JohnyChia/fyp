#!/usr/bin/env python3
"""Safe development-readiness checks that never traverse sealed-test data."""

from __future__ import annotations

import ast
import copy
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/durian_inspection_pkg"
CONFIG = PACKAGE / "config/leaf_detector_deployment.yaml"
FORMAL_ROOT = (
    ROOT / "experiment/final_cross_domain_study/durian_v4_1_8run_final"
)
BACKENDS = ("yolo26n", "yolo26n_p2", "rfdetr_n", "dfine_n")
CLASSES = (
    "leaf_algal",
    "leaf_blight",
    "leaf_colletotrichum",
    "leaf_healthy",
    "leaf_phomopsis",
    "leaf_rhizoctonia",
)

sys.path.insert(0, str(PACKAGE))


class Validation:
    def __init__(self) -> None:
        self.failures: list[str] = []
        self.warnings: list[str] = []

    def check(self, condition: bool, label: str, detail: str = "") -> None:
        status = "PASS" if condition else "FAIL"
        suffix = f" — {detail}" if detail else ""
        print(f"{label}: {status}{suffix}")
        if not condition:
            self.failures.append(label)

    def warn(self, label: str, detail: str) -> None:
        print(f"{label}: BLOCKED_ENVIRONMENT — {detail}")
        self.warnings.append(label)


def validate() -> int:
    from canonical_leaf_classes import CANONICAL_CLASS_NAMES
    from leaf_backend_runtime import LeafBackendSpec
    from leaf_deployment_config import load_deployment_config
    from observation_store import connect

    result = Validation()
    print("FYP DEVELOPMENT VALIDATOR (sealed-test-safe)")

    result.check((ROOT / "src/durian_message/package.xml").is_file(), "ROS_MESSAGE_PACKAGE")
    result.check((PACKAGE / "package.xml").is_file(), "ROS_INSPECTION_PACKAGE")
    result.check(CONFIG.is_file(), "DEPLOYMENT_CONFIG_PRESENT")
    result.check(tuple(CANONICAL_CLASS_NAMES) == CLASSES, "CANONICAL_CLASS_ORDER")

    config = load_deployment_config(CONFIG)
    profiles = config["leaf_detector"]["profiles"]
    result.check(tuple(profiles) == BACKENDS, "FOUR_BACKEND_DEFINITIONS")
    for backend in BACKENDS:
        selected = copy.deepcopy(config)
        selected["leaf_detector"]["backend"] = backend
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as stream:
            yaml.safe_dump(selected, stream, sort_keys=False)
            temporary_config = Path(stream.name)
        try:
            spec = LeafBackendSpec.from_config(temporary_config)
            launcher_ok = Path(spec.command[0]).is_file() and all(
                Path(entry).is_dir() for entry in spec.python_path
            )
            result.check(
                spec.name == backend and launcher_ok,
                f"BACKEND_{backend.upper()}_IDENTITY_HASH_RUNTIME",
                f"framework={spec.framework} sha256={spec.checkpoint_sha256[:12]}…",
            )
            result.check(
                spec.runtime_options.get("device_policy", "cuda_required")
                == "cuda_required",
                f"BACKEND_{backend.upper()}_DEVICE_POLICY",
            )
        except Exception as exc:
            result.check(False, f"BACKEND_{backend.upper()}_IDENTITY_HASH_RUNTIME", str(exc))
        finally:
            temporary_config.unlink(missing_ok=True)

    dfine_source = Path(profiles["dfine_n"]["launcher"]["working_directory"])
    expected_commit = profiles["dfine_n"]["runtime_options"]["expected_source_commit"]
    try:
        actual_commit = subprocess.run(
            ["git", "-C", str(dfine_source), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        result.check(actual_commit == expected_commit, "DFINE_SOURCE_IDENTITY")
    except Exception as exc:
        result.check(False, "DFINE_SOURCE_IDENTITY", str(exc))

    with tempfile.TemporaryDirectory(prefix="durian_db_validation_") as temp_dir:
        db_path = Path(temp_dir) / "legacy.db"
        legacy = sqlite3.connect(db_path)
        legacy.execute(
            "CREATE TABLE inspection_log "
            "(id INTEGER PRIMARY KEY, tree_x REAL, tree_y REAL, status TEXT)"
        )
        legacy.commit()
        legacy.close()
        try:
            with connect(str(db_path)) as connection:
                columns = {
                    row[1]
                    for row in connection.execute("PRAGMA table_info(inspection_log)")
                }
            with connect(str(db_path)):
                pass
            required = {
                "inspection_id", "tree_id", "processing_error", "leaf_backend",
                "checkpoint_sha256", "pipeline_fingerprint", "completed_at",
            }
            result.check(required <= columns, "DATABASE_MIGRATION_IDEMPOTENCE")
        except Exception as exc:
            result.check(False, "DATABASE_MIGRATION_IDEMPOTENCE", str(exc))

    syntax_errors = []
    for source in sorted(PACKAGE.rglob("*.py")):
        try:
            ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        except (OSError, SyntaxError) as exc:
            syntax_errors.append(f"{source}: {exc}")
    result.check(not syntax_errors, "PYTHON_SYNTAX", syntax_errors[0] if syntax_errors else "")

    formal_report = FORMAL_ROOT / "FINAL_VALIDATION_REPORT.json"
    sealed_guard = FORMAL_ROOT / "sealed_test_guard.json"
    try:
        report = json.loads(formal_report.read_text(encoding="utf-8"))
        guard = json.loads(sealed_guard.read_text(encoding="utf-8"))
        formal_ok = report.get("status") == "PASS" and report.get("formal_training_started") is False
        sealed_ok = (
            guard.get("sealed_test_accessed") is False
            and guard.get("sealed_test_included") is False
        )
        result.check(formal_ok, "FROZEN_FORMAL_PACKAGE_DECLARATION")
        result.check(sealed_ok, "SEALED_TEST_GUARD")
    except Exception as exc:
        result.check(False, "FROZEN_FORMAL_PACKAGE_DECLARATION", str(exc))
        result.check(False, "SEALED_TEST_GUARD", str(exc))

    gpu = subprocess.run(
        ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
        capture_output=True,
        text=True,
    )
    if gpu.returncode == 0 and gpu.stdout.strip():
        print(f"CUDA_RUNTIME: PASS — {gpu.stdout.strip().splitlines()[0]}")
    else:
        result.warn("CUDA_RUNTIME", "GPU unavailable; native inference smoke not claimed")

    print(f"DEVELOPMENT_FAILURES: {len(result.failures)}")
    print(f"ENVIRONMENT_BLOCKERS: {len(result.warnings)}")
    print("FINAL_STATUS: " + ("DEVELOPMENT_READY" if not result.failures else "NO_GO_FIXES_REQUIRED"))
    return 0 if not result.failures else 1


if __name__ == "__main__":
    raise SystemExit(validate())
