from pathlib import Path
from types import SimpleNamespace
import math
import sys

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from leaf_backend_runtime import (
    InvalidDetectionGeometry,
    LeafBackendClient,
    LeafBackendError,
    LeafBackendUnavailable,
    LeafDetection,
    _FrameworkBackend,
)
from observation_store import missing_required_capture_transforms


def _spec():
    return SimpleNamespace(
        name="test_backend", framework="test", native_resolution=(640, 640)
    )


def test_leaf_detection_rejects_degenerate_and_nonfinite_geometry():
    common = {
        "class_id": 0,
        "class_name": "leaf_algal",
        "confidence": 0.5,
        "backend": "test_backend",
        "source": "test",
        "inference_size": (640, 640),
    }
    with pytest.raises(InvalidDetectionGeometry):
        LeafDetection(x1=2, y1=1, x2=2, y2=5, **common)
    with pytest.raises(InvalidDetectionGeometry):
        LeafDetection(x1=math.nan, y1=1, x2=2, y2=5, **common)


def test_framework_filter_drops_only_invalid_geometry():
    backend = _FrameworkBackend(_spec(), {})
    result = backend._validated_detections([
        ([1, 2, 5, 8], 0, 0.7),
        ([3, 4, 3, 9], 0, 0.8),
        ([math.nan, 1, 4, 8], 0, 0.9),
    ])
    assert len(result) == 1
    assert result[0].confidence == pytest.approx(0.7)


def test_framework_filter_does_not_hide_semantic_failures():
    backend = _FrameworkBackend(_spec(), {})
    with pytest.raises((LeafBackendError, ValueError, KeyError)):
        backend._validated_detections([([1, 2, 5, 8], 999, 0.7)])


def test_offline_processing_requires_capture_time_transforms():
    complete = {
        "transforms": {
            "odom_from_camera": {"translation": {}},
            "odom_from_map": {"translation": {}},
        }
    }
    assert missing_required_capture_transforms(complete) == ()
    assert missing_required_capture_transforms({}) == (
        "odom_from_camera",
        "odom_from_map",
    )
    assert missing_required_capture_transforms({
        "transforms": {"odom_from_map": {"translation": {}}}
    }) == ("odom_from_camera",)


def test_capture_uses_bounded_tf_fallback_and_rejects_missing_tf():
    vision = (PACKAGE / "vision_node.py").read_text()
    assert 'declare_parameter("tf_latest_fallback_max_age_sec", 1.5)' in vision
    assert '"mode": "bounded_latest"' in vision
    assert "required capture transform unavailable; observation rejected" in vision


def test_offline_processing_never_looks_up_missing_historical_tf():
    processing = (PACKAGE / "processing_node.py").read_text()
    assert "SKIPPED_TF_UNAVAILABLE" in processing
    assert "frame excluded without querying live TF" in processing
    assert "diagnostic_logging_enabled" in processing
    assert "diagnostic_artifacts_enabled" in processing


def test_ros_shutdown_paths_are_idempotent():
    for name in ("bridge_node.py", "processing_node.py", "map_publisher.py"):
        source = (PACKAGE / name).read_text()
        assert "if rclpy.ok():" in source
        assert "rclpy.shutdown()" in source


def test_leaf_backend_client_requests_worker_shutdown_before_termination():
    """Regression guard for the D-FINE launch-shutdown timeout investigation."""
    class Input:
        def __init__(self):
            self.messages = []

        def write(self, value):
            self.messages.append(value)

        def flush(self):
            pass

    class Process:
        def __init__(self):
            self.stdin = Input()
            self.wait_calls = []
            self.terminated = False

        def poll(self):
            return None

        def wait(self, timeout):
            self.wait_calls.append(timeout)
            return 0

        def terminate(self):
            self.terminated = True

    client = LeafBackendClient.__new__(LeafBackendClient)
    process = Process()
    client._process = process
    client.close()
    assert client._process is None
    assert process.stdin.messages == ['{"operation":"shutdown"}\n']
    assert process.wait_calls == [2.0]
    assert not process.terminated


def test_leaf_backend_start_closes_worker_after_failed_handshake(monkeypatch):
    client = LeafBackendClient.__new__(LeafBackendClient)
    client.spec = SimpleNamespace(
        name="test_backend",
        command=("python",),
        working_directory=None,
        startup_timeout_seconds=0.1,
    )
    client.config_path = "/tmp/test.yaml"
    client._process = None
    client._lock = __import__("threading").Lock()
    monkeypatch.setattr(client, "_validate_launcher_paths", lambda: None)
    monkeypatch.setattr(client, "_build_worker_environment", lambda: {})
    monkeypatch.setattr("leaf_backend_runtime.shutil.which", lambda _: "/usr/bin/python")
    process = SimpleNamespace()
    monkeypatch.setattr("leaf_backend_runtime.subprocess.Popen", lambda *a, **k: process)
    monkeypatch.setattr(
        client, "_read_message", lambda *_: (_ for _ in ()).throw(
            LeafBackendUnavailable("startup timeout")
        )
    )
    closed = []
    monkeypatch.setattr(client, "close", lambda: (closed.append(True), setattr(client, "_process", None)))

    with pytest.raises(LeafBackendUnavailable, match="startup timeout"):
        client.start()
    assert closed == [True]
    assert client._process is None


def test_leaf_backend_infer_invalidates_desynchronized_worker(monkeypatch):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")

    class Input:
        def write(self, value):
            self.value = value

        def flush(self):
            pass

    client = LeafBackendClient.__new__(LeafBackendClient)
    client.spec = SimpleNamespace(
        name="test_backend", threshold=0.1, per_class_thresholds={}
    )
    client._process = SimpleNamespace(stdin=Input())
    client._lock = __import__("threading").Lock()
    monkeypatch.setattr(
        client,
        "_read_message",
        lambda *_: {"ok": True, "request_id": "stale", "detections": []},
    )
    closed = []
    monkeypatch.setattr(client, "close", lambda: (closed.append(True), setattr(client, "_process", None)))

    with pytest.raises(LeafBackendError, match="response ID mismatch"):
        client.infer(np.zeros((4, 4, 3), dtype=np.uint8))
    assert closed == [True]
    assert client._process is None
