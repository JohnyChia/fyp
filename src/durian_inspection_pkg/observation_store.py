"""SQLite schema and filesystem helpers for deferred vision processing."""

import os
import sqlite3
import time

SCHEMA = """
CREATE TABLE IF NOT EXISTS captured_observations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    observation_id TEXT NOT NULL UNIQUE,
    inspection_id TEXT NOT NULL,
    tree_id INTEGER NOT NULL,
    viewpoint_id TEXT,
    viewpoint_index INTEGER,
    rgb_path TEXT NOT NULL,
    depth_path TEXT,
    metadata_path TEXT NOT NULL,
    rgb_sha256 TEXT NOT NULL,
    depth_sha256 TEXT,
    observation_sha256 TEXT NOT NULL,
    context_sha256 TEXT,
    duplicate_of TEXT,
    rgb_timestamp REAL NOT NULL,
    depth_timestamp REAL,
    status TEXT NOT NULL DEFAULT 'CAPTURED',
    processing_error TEXT,
    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
    processed_at TEXT
);
CREATE INDEX IF NOT EXISTS idx_observation_pending
    ON captured_observations(inspection_id, tree_id, status, rgb_timestamp);
CREATE INDEX IF NOT EXISTS idx_observation_hash
    ON captured_observations(observation_sha256);

CREATE TABLE IF NOT EXISTS tree_processing_cache (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    batch_sha256 TEXT NOT NULL,
    pipeline_fingerprint TEXT NOT NULL,
    source_inspection_id TEXT NOT NULL,
    source_tree_id INTEGER NOT NULL,
    disease_name TEXT NOT NULL,
    coverage REAL NOT NULL,
    confidence REAL NOT NULL,
    priority TEXT NOT NULL,
    remedy TEXT NOT NULL,
    evidence_json TEXT NOT NULL,
    aggregation_mode TEXT,
    unknown_reason TEXT,
    viewpoint_count INTEGER,
    valid_viewpoint_count INTEGER,
    sampled_frame_count INTEGER,
    world_sha256 TEXT,
    experiment_version TEXT,
    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(batch_sha256, pipeline_fingerprint)
);
"""


def missing_required_capture_transforms(metadata):
    """Return transform names that deferred processing must never query live."""
    transforms = metadata.get("transforms", {}) if isinstance(metadata, dict) else {}
    required = ("odom_from_camera", "odom_from_map")
    return tuple(name for name in required if not transforms.get(name))


def connect(db_path):
    os.makedirs(os.path.dirname(os.path.abspath(db_path)), exist_ok=True)
    conn = sqlite3.connect(db_path, timeout=30.0)
    conn.execute("PRAGMA busy_timeout=30000")
    for attempt in range(20):
        try:
            conn.execute("PRAGMA journal_mode=WAL")
            break
        except sqlite3.OperationalError as exc:
            if "locked" not in str(exc).lower() or attempt == 19:
                conn.close()
                raise
            time.sleep(0.05)
    conn.execute("PRAGMA foreign_keys=ON")
    conn.executescript(SCHEMA)
    # Serialize schema inspection and ALTER TABLE operations. Without the
    # immediate transaction, two ROS processes can both observe a missing
    # column and race to add it.
    try:
        conn.execute("BEGIN IMMEDIATE")
        ensure_inspection_columns(conn)
        ensure_observation_columns(conn)
        conn.commit()
    except Exception:
        conn.rollback()
        conn.close()
        raise
    return conn


def ensure_inspection_columns(conn):
    """Migrate the existing Flutter-facing table without dropping user data."""
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS inspection_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            tree_x REAL, tree_y REAL, distance REAL,
            scan_time REAL, status TEXT, timestamp DATETIME,
            disease_name TEXT, coverage REAL, confidence REAL,
            priority TEXT, remedy TEXT
        )
        """
    )
    columns = {row[1] for row in conn.execute("PRAGMA table_info(inspection_log)")}
    additions = {
        "inspection_id": "TEXT",
        "tree_id": "INTEGER",
        "processing_error": "TEXT",
        "cache_hit": "INTEGER NOT NULL DEFAULT 0",
        "cache_source_inspection_id": "TEXT",
        "completed_at": "DATETIME",
        "leaf_backend": "TEXT",
        "checkpoint_sha256": "TEXT",
        "pipeline_fingerprint": "TEXT",
        "aggregation_mode": "TEXT",
        "unknown_reason": "TEXT",
        "viewpoint_count": "INTEGER",
        "valid_viewpoint_count": "INTEGER",
        "sampled_frame_count": "INTEGER",
        "world_sha256": "TEXT",
        "experiment_version": "TEXT",
    }
    for name, declaration in additions.items():
        if name not in columns:
            conn.execute(f"ALTER TABLE inspection_log ADD COLUMN {name} {declaration}")
    conn.execute(
        "CREATE INDEX IF NOT EXISTS idx_inspection_tree "
        "ON inspection_log(inspection_id, tree_id)"
    )


def ensure_observation_columns(conn):
    columns = {row[1] for row in conn.execute("PRAGMA table_info(captured_observations)")}
    for name, declaration in {
        "context_sha256": "TEXT",
        "duplicate_of": "TEXT",
        "viewpoint_id": "TEXT",
        "viewpoint_index": "INTEGER",
    }.items():
        if name not in columns:
            conn.execute(f"ALTER TABLE captured_observations ADD COLUMN {name} {declaration}")
    cache_columns = {
        row[1] for row in conn.execute("PRAGMA table_info(tree_processing_cache)")
    }
    for name, declaration in {
        "aggregation_mode": "TEXT",
        "unknown_reason": "TEXT",
        "viewpoint_count": "INTEGER",
        "valid_viewpoint_count": "INTEGER",
        "sampled_frame_count": "INTEGER",
        "world_sha256": "TEXT",
        "experiment_version": "TEXT",
    }.items():
        if name not in cache_columns:
            conn.execute(f"ALTER TABLE tree_processing_cache ADD COLUMN {name} {declaration}")
