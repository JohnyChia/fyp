import inspect
from pathlib import Path
import sqlite3
import sys

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from observation_store import connect
from same_input_benchmark import (
    assert_identical_backend_inputs, load_observation_manifest_read_only,
    run_production_backend, source_snapshot, stage_production_database,
    write_results,
)


def _source_fixture(tmp_path):
    source = tmp_path / "capture.db"
    rgb, depth, metadata = tmp_path / "rgb", tmp_path / "depth", tmp_path / "meta"
    rgb.write_bytes(b"rgb")
    depth.write_bytes(b"depth")
    metadata.write_text("{}", encoding="utf-8")
    with connect(str(source)) as conn:
        conn.execute("""INSERT INTO inspection_log
            (tree_x,tree_y,status,inspection_id,tree_id)
            VALUES (1.0,2.0,'completed','i1',44)""")
        conn.execute("""INSERT INTO captured_observations
            (observation_id,inspection_id,tree_id,viewpoint_id,viewpoint_index,
             rgb_path,depth_path,metadata_path,rgb_sha256,depth_sha256,
             observation_sha256,rgb_timestamp,status)
            VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)""",
            ("o1", "i1", 44, "view_000", 0, str(rgb), str(depth), str(metadata),
             "rgbsha", "depthsha", "obssha", 1.0, "CAPTURED"))
    return source


def test_read_only_manifest_and_staging_do_not_mutate_source(tmp_path):
    source = _source_fixture(tmp_path)
    manifest, ordered_hash, corpus_hash = load_observation_manifest_read_only(source, "i1")
    before = source_snapshot(source, manifest)
    stage = tmp_path / "stage.db"
    stage_production_database(source, "i1", manifest, stage)
    after = source_snapshot(source, manifest)
    assert before == after
    assert len(ordered_hash) == len(corpus_hash) == 64
    with sqlite3.connect(source) as conn:
        assert conn.execute("SELECT status FROM captured_observations").fetchone()[0] == "CAPTURED"
    with sqlite3.connect(stage) as conn:
        assert conn.execute("SELECT status FROM captured_observations").fetchone()[0] == "CAPTURED"
        assert conn.execute("SELECT status FROM inspection_log").fetchone()[0] == "processing"


def test_four_pass_identity_gate_and_independent_results(tmp_path):
    rows = []
    for backend in ("a", "b", "c", "d"):
        rows.append({
            "backend": backend, "checkpoint_sha256": backend,
            "pipeline_fingerprint": f"fp-{backend}", "corpus_sha256": "corpus",
            "ordered_manifest_sha256": "ordered", "association_policy_version": "v1",
            "tree_id": 44, "aggregation_mode": "unique_leaf_count",
            "prediction": "Unknown", "confidence": 0.0,
            "unknown_reason": "UNKNOWN_NO_VALID_EVIDENCE",
            "inference_seconds": 0.1, "evidence": {},
        })
    assert_identical_backend_inputs(rows, ("a", "b", "c", "d"))
    result = tmp_path / "results.db"
    write_results(result, rows)
    with sqlite3.connect(result) as conn:
        assert conn.execute("SELECT COUNT(DISTINCT corpus_sha256) FROM benchmark_results").fetchone()[0] == 1
        assert conn.execute("SELECT COUNT(DISTINCT backend) FROM benchmark_results").fetchone()[0] == 4


def test_benchmark_calls_production_processing_core_without_algorithm_copy():
    source = inspect.getsource(run_production_backend)
    assert "node._process_tree_observations" in source
    assert "make_leaf_crop" not in source
    assert "associate_unique_leaves" not in source

