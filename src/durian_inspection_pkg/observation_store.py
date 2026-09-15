"""SQLite schema and filesystem helpers for deferred vision processing."""

import os
import sqlite3

SCHEMA = """
CREATE TABLE IF NOT EXISTS captured_observations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    observation_id TEXT NOT NULL UNIQUE,
    inspection_id TEXT NOT NULL,
    tree_id INTEGER NOT NULL,
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
    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(batch_sha256, pipeline_fingerprint)
);
"""


def connect(db_path):
    os.makedirs(os.path.dirname(os.path.abspath(db_path)), exist_ok=True)
    conn = sqlite3.connect(db_path, timeout=30.0)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    conn.execute("PRAGMA busy_timeout=30000")
    conn.executescript(SCHEMA)
    ensure_inspection_columns(conn)
    ensure_observation_columns(conn)
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
    }
    for name, declaration in additions.items():
        if name not in columns:
            conn.execute(f"ALTER TABLE inspection_log ADD COLUMN {name} {declaration}")
    conn.execute(
        "CREATE INDEX IF NOT EXISTS idx_inspection_tree "
        "ON inspection_log(inspection_id, tree_id)"
    )
    conn.commit()


def ensure_observation_columns(conn):
    columns = {row[1] for row in conn.execute("PRAGMA table_info(captured_observations)")}
    for name, declaration in {
        "context_sha256": "TEXT",
        "duplicate_of": "TEXT",
    }.items():
        if name not in columns:
            conn.execute(f"ALTER TABLE captured_observations ADD COLUMN {name} {declaration}")
    conn.commit()
