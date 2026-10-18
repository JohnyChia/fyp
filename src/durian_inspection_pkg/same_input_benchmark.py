"""Read-only same-input replay through the production ProcessingNode core."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import time
from typing import Iterable, Mapping

from observation_store import connect


RESULT_COLUMNS = (
    "backend", "checkpoint_sha256", "pipeline_fingerprint", "corpus_sha256",
    "ordered_manifest_sha256", "association_policy_version", "tree_id",
    "aggregation_mode", "prediction", "confidence", "unknown_reason",
    "inference_seconds", "evidence_json",
)


def _sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_observation_manifest_read_only(source_db: str | Path, inspection_id: str):
    """Freeze order and hash every corpus file without writing the source DB."""
    uri = f"file:{Path(source_db).resolve()}?mode=ro&immutable=1"
    conn = sqlite3.connect(uri, uri=True)
    try:
        conn.execute("PRAGMA query_only=ON")
        columns = {row[1] for row in conn.execute("PRAGMA table_info(captured_observations)")}
        viewpoint_id = "viewpoint_id" if "viewpoint_id" in columns else "NULL AS viewpoint_id"
        viewpoint_index = "viewpoint_index" if "viewpoint_index" in columns else "NULL AS viewpoint_index"
        rows = conn.execute(
            f"""SELECT observation_id, tree_id, {viewpoint_id}, {viewpoint_index},
                       rgb_path, depth_path, metadata_path, rgb_sha256, depth_sha256,
                       observation_sha256, context_sha256, rgb_timestamp, depth_timestamp
                FROM captured_observations WHERE inspection_id=?
                ORDER BY tree_id, viewpoint_index, rgb_timestamp, id""", (inspection_id,)
        ).fetchall()
    finally:
        conn.close()
    if not rows:
        raise ValueError(f"no observations for inspection_id={inspection_id}")
    keys = ("observation_id", "tree_id", "viewpoint_id", "viewpoint_index",
            "rgb_path", "depth_path", "metadata_path", "rgb_sha256", "depth_sha256",
            "observation_sha256", "context_sha256", "rgb_timestamp", "depth_timestamp")
    manifest, corpus_digest = [], hashlib.sha256()
    for row in rows:
        item = dict(zip(keys, row))
        hashes = {}
        for kind in ("rgb", "depth", "metadata"):
            file_path = item.get(f"{kind}_path")
            if file_path:
                hashes[kind] = _sha256_file(file_path)
                corpus_digest.update(kind.encode("ascii") + hashes[kind].encode("ascii"))
        item["file_sha256"] = hashes
        manifest.append(item)
    payload = json.dumps(manifest, sort_keys=True, separators=(",", ":"), allow_nan=False)
    ordered_hash = hashlib.sha256(payload.encode()).hexdigest()
    corpus_digest.update(ordered_hash.encode("ascii"))
    return tuple(manifest), ordered_hash, corpus_digest.hexdigest()


def source_snapshot(source_db: str | Path, manifest: Iterable[Mapping[str, object]]) -> str:
    """Hash source DB sidecars and all corpus files for before/after proof."""
    digest, base = hashlib.sha256(), Path(source_db).resolve()
    for path in (base, Path(str(base) + "-wal"), Path(str(base) + "-shm")):
        if path.exists():
            digest.update(path.name.encode() + _sha256_file(path).encode())
    paths = sorted({str(row[key]) for row in manifest
                    for key in ("rgb_path", "depth_path", "metadata_path") if row.get(key)})
    for path in paths:
        digest.update(path.encode() + _sha256_file(path).encode())
    return digest.hexdigest()


def initialize_benchmark_db(path: str | Path) -> None:
    with sqlite3.connect(path) as conn:
        conn.execute("""CREATE TABLE IF NOT EXISTS benchmark_results (
            id INTEGER PRIMARY KEY AUTOINCREMENT, backend TEXT NOT NULL,
            checkpoint_sha256 TEXT NOT NULL, pipeline_fingerprint TEXT NOT NULL,
            corpus_sha256 TEXT NOT NULL, ordered_manifest_sha256 TEXT NOT NULL,
            association_policy_version TEXT NOT NULL, tree_id INTEGER NOT NULL,
            aggregation_mode TEXT NOT NULL, prediction TEXT NOT NULL,
            confidence REAL NOT NULL, unknown_reason TEXT,
            inference_seconds REAL NOT NULL, evidence_json TEXT NOT NULL,
            UNIQUE(backend, pipeline_fingerprint, ordered_manifest_sha256, tree_id))""")


def write_results(path: str | Path, rows: Iterable[Mapping[str, object]]) -> None:
    initialize_benchmark_db(path)
    values = [tuple(json.dumps(row.get("evidence", {}), sort_keys=True)
                    if key == "evidence_json" else row.get(key) for key in RESULT_COLUMNS)
              for row in rows]
    with sqlite3.connect(path) as conn:
        conn.executemany(
            f"INSERT OR REPLACE INTO benchmark_results ({','.join(RESULT_COLUMNS)}) "
            f"VALUES ({','.join('?' for _ in RESULT_COLUMNS)})", values)


def stage_production_database(source_db: str | Path, inspection_id: str,
                              manifest: Iterable[Mapping[str, object]], stage_db: str | Path) -> None:
    """Copy rows to an independent DB; corpus files remain read-only inputs."""
    source = sqlite3.connect(f"file:{Path(source_db).resolve()}?mode=ro&immutable=1", uri=True)
    source.execute("PRAGMA query_only=ON")
    try:
        raw_trees = source.execute(
            "SELECT tree_id,tree_x,tree_y,distance,scan_time,timestamp FROM inspection_log "
            "WHERE inspection_id=? ORDER BY id", (inspection_id,)).fetchall()
    finally:
        source.close()
    trees = {}
    for tree_id, *values in raw_trees:
        trees.setdefault(int(tree_id), tuple(values))
    missing = sorted({int(row["tree_id"]) for row in manifest} - set(trees))
    if missing:
        raise ValueError(f"inspection_log lacks tree coordinates for IDs: {missing}")
    with connect(str(stage_db)) as target:
        for tree_id, values in sorted(trees.items()):
            target.execute("""INSERT INTO inspection_log
                (tree_x,tree_y,distance,scan_time,status,timestamp,inspection_id,tree_id,cache_hit)
                VALUES (?,?,?,?,'processing',?,?,?,0)""", (*values, inspection_id, tree_id))
        for row in manifest:
            target.execute("""INSERT INTO captured_observations
                (observation_id,inspection_id,tree_id,viewpoint_id,viewpoint_index,
                 rgb_path,depth_path,metadata_path,rgb_sha256,depth_sha256,
                 observation_sha256,context_sha256,rgb_timestamp,depth_timestamp,status)
                VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,'CAPTURED')""",
                (row["observation_id"], inspection_id, row["tree_id"], row["viewpoint_id"],
                 row["viewpoint_index"], row["rgb_path"], row["depth_path"],
                 row["metadata_path"], row["rgb_sha256"], row["depth_sha256"],
                 row["observation_sha256"], row["context_sha256"],
                 row["rgb_timestamp"], row["depth_timestamp"]))


def assert_identical_backend_inputs(rows, expected_backends) -> None:
    rows, expected = list(rows), set(expected_backends)
    if {row["backend"] for row in rows} != expected:
        raise RuntimeError("benchmark backend result set is incomplete")
    if len({row["corpus_sha256"] for row in rows}) != 1 or len({row["ordered_manifest_sha256"] for row in rows}) != 1:
        raise RuntimeError("backend passes did not use identical frozen input")


def run_production_backend(backend: str, config_path: str | Path,
                           stage_db: str | Path, processed_root: str | Path,
                           inspection_id: str, corpus_sha256: str,
                           ordered_manifest_sha256: str):
    """Invoke the exact production `_process_tree_observations` implementation."""
    import rclpy
    from processing_node import ProcessingNode
    args = ["--ros-args", "-p", f"database_path:={Path(stage_db).resolve()}",
            "-p", f"processed_root:={Path(processed_root).resolve()}",
            "-p", f"leaf_deployment_config_path:={Path(config_path).resolve()}"]
    rclpy.init(args=args)
    node = None
    try:
        node = ProcessingNode()
        if node.leaf_backend_spec.name != backend:
            raise RuntimeError("benchmark backend differs from production profile")
        node.load_all_models()
        if not node.models_ready:
            raise RuntimeError(f"production models failed to load for {backend}")
        with connect(str(stage_db)) as conn:
            trees = conn.execute("SELECT tree_id,tree_x,tree_y FROM inspection_log "
                                 "WHERE inspection_id=? ORDER BY id", (inspection_id,)).fetchall()
        timings = {}
        for tree_id, tree_x, tree_y in trees:
            started = time.perf_counter()
            node._process_tree_observations(inspection_id, int(tree_id), float(tree_x), float(tree_y))
            timings[int(tree_id)] = time.perf_counter() - started
        with connect(str(stage_db)) as conn:
            results = conn.execute("SELECT tree_id,disease_name,confidence,aggregation_mode,"
                                   "unknown_reason,processing_error FROM inspection_log "
                                   "WHERE inspection_id=? ORDER BY id", (inspection_id,)).fetchall()
            evidence = {int(tree_id): json.loads(payload) for tree_id, payload in
                        conn.execute("SELECT source_tree_id,evidence_json FROM tree_processing_cache")}
        output = []
        for tree_id, prediction, confidence, mode, unknown, error in results:
            if error:
                raise RuntimeError(f"production processing failed for tree {tree_id}: {error}")
            output.append({"backend": backend,
                "checkpoint_sha256": node.leaf_backend_spec.checkpoint_sha256,
                "pipeline_fingerprint": node.pipeline_fingerprint,
                "corpus_sha256": corpus_sha256,
                "ordered_manifest_sha256": ordered_manifest_sha256,
                "association_policy_version": node.association_policy.version,
                "tree_id": int(tree_id), "aggregation_mode": mode,
                "prediction": prediction, "confidence": float(confidence),
                "unknown_reason": unknown, "inference_seconds": timings[int(tree_id)],
                "evidence": evidence.get(int(tree_id), {})})
        return output
    finally:
        if node is not None:
            if node.leaf_backend is not None:
                node.leaf_backend.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def selected_backend_config(base_config: Mapping[str, object], backend: str):
    config = copy.deepcopy(base_config)
    if backend not in config["leaf_detector"]["profiles"]:
        raise ValueError(f"unknown configured backend: {backend}")
    config["leaf_detector"]["backend"] = backend
    return config
