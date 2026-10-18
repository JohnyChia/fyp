#!/usr/bin/env python3
"""Run the frozen CPU audit logic against an explicitly supplied registry.

The historical audit module remains unchanged.  This wrapper replaces only
its registry and operating-threshold inputs, which are supplied by the new
V4.2 comparison receipts.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import run_six_model_runtime_audit as audit


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--threshold-receipt", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    registry = args.registry.resolve(strict=True)
    threshold_receipt = args.threshold_receipt.resolve(strict=True)
    receipt = json.loads(threshold_receipt.read_text(encoding="utf-8"))
    thresholds = {row["run_id"]: row["threshold"] for row in receipt.get("selections", [])}
    if set(thresholds) != {f"Run{i:02d}" for i in range(1, 7)}:
        raise ValueError("threshold receipt must contain Run01..Run06 exactly once")
    if receipt.get("source") != "V4_2_SYNTHETIC_VAL_288":
        raise ValueError("thresholds must originate from V4.2 Synthetic Val")

    source_protocol = audit.PROTOCOL.resolve(strict=True)
    protocol = json.loads(source_protocol.read_text(encoding="utf-8"))
    protocol["operating_thresholds"] = thresholds
    protocol["derived_from_protocol_sha256"] = sha256(source_protocol)
    protocol["v42_threshold_receipt_sha256"] = sha256(threshold_receipt)
    protocol["sealed_test_accessed"] = False
    derived_protocol = threshold_receipt.parent / "V42_CONSISTENCY_PROTOCOL.json"
    encoded = json.dumps(protocol, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if derived_protocol.exists():
        if derived_protocol.read_text(encoding="utf-8") != encoded:
            raise RuntimeError("existing V4.2 consistency protocol differs")
    else:
        derived_protocol.write_text(encoded, encoding="utf-8")

    audit.REGISTRY = registry
    audit.THRESHOLDS = thresholds
    audit.PROTOCOL = derived_protocol
    args.output_dir.resolve().parent.mkdir(parents=True, exist_ok=True)
    sys.argv = [sys.argv[0], "--run-id", args.run_id, "--output-dir", str(args.output_dir)]
    return audit.main()


if __name__ == "__main__":
    raise SystemExit(main())
