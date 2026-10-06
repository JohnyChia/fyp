"""Configuration-selected leaf inference backends and isolated worker protocol.

This module intentionally imports framework packages only inside the selected
backend.  It can therefore be installed in the ROS environment for the client
contract and in separate framework environments for worker execution.
"""

from __future__ import annotations

import argparse
import base64
import contextlib
import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path
import selectors
import shutil
import subprocess
import sys
import threading
import uuid
from typing import Mapping, Sequence

from canonical_leaf_classes import canonical_class_name
from checkpoint_manifest import load_checkpoint_manifest
from leaf_deployment_config import load_deployment_config


_IMPORT_SHADOW_NAMES = frozenset(
    {"torch", "torchvision", "transformers", "rfdetr", "ultralytics"}
)


class LeafBackendError(RuntimeError):
    """Base error for a selected leaf backend."""


class LeafBackendUnavailable(LeafBackendError):
    """Raised when the selected backend cannot run in its configured environment."""


class InvalidDetectionGeometry(LeafBackendError):
    """One framework prediction has non-finite or non-positive-area geometry."""


@dataclass(frozen=True)
class LeafDetection:
    x1: float
    y1: float
    x2: float
    y2: float
    class_id: int
    class_name: str
    confidence: float
    backend: str
    source: str
    inference_size: tuple[int, int]

    def __post_init__(self) -> None:
        expected_name = canonical_class_name(self.class_id)
        if self.class_name != expected_name:
            raise LeafBackendError(
                f"class ID/name mismatch: {self.class_id} != {self.class_name!r}"
            )
        if not 0.0 <= float(self.confidence) <= 1.0:
            raise LeafBackendError("detection confidence is outside [0, 1]")
        coordinates = (self.x1, self.y1, self.x2, self.y2)
        if not all(math.isfinite(float(value)) for value in coordinates):
            raise InvalidDetectionGeometry("detection has a non-finite bounding box")
        if self.x2 <= self.x1 or self.y2 <= self.y1:
            raise InvalidDetectionGeometry("detection has an invalid bounding box")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "LeafDetection":
        size = value.get("inference_size")
        if not isinstance(size, Sequence) or isinstance(size, (str, bytes)) or len(size) != 2:
            raise LeafBackendError("invalid detection inference_size")
        return cls(
            x1=float(value["x1"]),
            y1=float(value["y1"]),
            x2=float(value["x2"]),
            y2=float(value["y2"]),
            class_id=int(value["class_id"]),
            class_name=str(value["class_name"]),
            confidence=float(value["confidence"]),
            backend=str(value["backend"]),
            source=str(value["source"]),
            inference_size=(int(size[0]), int(size[1])),
        )


@dataclass(frozen=True)
class LeafBackendSpec:
    name: str
    framework: str
    checkpoint: str
    checkpoint_sha256: str
    manifest_path: str
    threshold: float
    per_class_thresholds: Mapping[str, float]
    native_resolution: tuple[int, int]
    resize_policy: str
    adapter_version: str
    command: tuple[str, ...]
    environment: Mapping[str, str]
    python_path: tuple[str, ...]
    working_directory: str | None
    startup_timeout_seconds: float
    request_timeout_seconds: float
    runtime_options: Mapping[str, object]

    @classmethod
    def from_config(cls, config_path: str | Path) -> "LeafBackendSpec":
        path = Path(config_path).expanduser().resolve()
        config = load_deployment_config(path)
        leaf = config["leaf_detector"]
        name = leaf["backend"]
        profile = leaf["profiles"][name]
        manifest = load_checkpoint_manifest(profile["manifest"])
        if manifest["candidate"] != name:
            raise LeafBackendError("selected backend and manifest candidate differ")
        checkpoint = str(Path(profile["checkpoint"]).expanduser().resolve())
        if checkpoint != str(Path(manifest["absolute_path"]).expanduser().resolve()):
            raise LeafBackendError("profile and manifest checkpoint paths differ")
        configured_framework = str(profile["framework"]).strip().lower()
        manifest_framework = str(manifest["framework"]).strip().lower()
        aliases = {"d-fine": "dfine", "ultralytics": "ultralytics", "rfdetr": "rfdetr"}
        if aliases.get(manifest_framework, manifest_framework) != configured_framework:
            raise LeafBackendError("profile and manifest frameworks differ")
        launcher = profile["launcher"]
        confidence = profile["confidence"]
        if float(confidence["model"]) != float(manifest["selected_validation_threshold"]):
            raise LeafBackendError(
                "profile threshold and checkpoint manifest threshold differ"
            )
        runtime_options = dict(profile.get("runtime_options", {}))
        expected_commit = runtime_options.get("expected_source_commit")
        if expected_commit is not None and manifest.get("framework_version") != (
            f"git:{expected_commit}"
        ):
            raise LeafBackendError(
                "configured framework source commit and manifest version differ"
            )
        return cls(
            name=name,
            framework=configured_framework,
            checkpoint=checkpoint,
            checkpoint_sha256=str(manifest["sha256"]),
            manifest_path=str(Path(profile["manifest"]).expanduser().resolve()),
            threshold=float(confidence["model"]),
            per_class_thresholds={
                str(key): float(value)
                for key, value in confidence["per_class"].items()
            },
            native_resolution=tuple(int(v) for v in profile["native_resolution"]),
            resize_policy=str(profile["resize_policy"]),
            adapter_version=str(profile["adapter_version"]),
            command=tuple(str(part) for part in launcher["command"]),
            environment={
                str(key): str(value)
                for key, value in launcher.get("environment", {}).items()
            },
            python_path=tuple(str(entry) for entry in launcher["python_path"]),
            working_directory=(
                str(Path(launcher["working_directory"]).expanduser())
                if launcher.get("working_directory")
                else None
            ),
            startup_timeout_seconds=float(leaf["deadline"]["startup_ms"]) / 1000.0,
            request_timeout_seconds=float(leaf["deadline"]["request_ms"]) / 1000.0,
            runtime_options=runtime_options,
        )


class LeafBackendClient:
    """Synchronous JSON-lines client for a configured isolated backend worker."""

    def __init__(self, config_path: str | Path):
        self.config_path = str(Path(config_path).expanduser().resolve())
        self.spec = LeafBackendSpec.from_config(self.config_path)
        self._process: subprocess.Popen[str] | None = None
        self._lock = threading.Lock()

    def _build_worker_environment(self) -> dict[str, str]:
        worker_env = os.environ.copy()
        worker_env.update(self.spec.environment)
        worker_env["PYTHONPATH"] = os.pathsep.join(self.spec.python_path)
        worker_env["PYTHONNOUSERSITE"] = "1"
        return worker_env

    def _validate_launcher_paths(self) -> None:
        missing_python_paths = [
            entry for entry in self.spec.python_path if not Path(entry).is_dir()
        ]
        if missing_python_paths:
            raise LeafBackendUnavailable(
                f"backend={self.spec.name} configured PYTHONPATH directory is unavailable: "
                f"{missing_python_paths[0]}"
            )

        working_directory = self.spec.working_directory
        if working_directory is None:
            return
        working_path = Path(working_directory)
        if not working_path.is_dir():
            raise LeafBackendUnavailable(
                f"backend={self.spec.name} working directory is unavailable: "
                f"{working_directory}"
            )
        collisions = _find_import_shadow_collisions(working_path)
        if collisions:
            raise LeafBackendUnavailable(
                f"backend={self.spec.name} working directory contains import-shadowing "
                f"module/package: {collisions[0]}"
            )

    @property
    def threshold(self) -> float:
        return self.spec.threshold

    def _read_message(self, timeout_seconds: float | None = None) -> dict[str, object]:
        process = self._process
        if process is None or process.stdout is None:
            raise LeafBackendUnavailable("leaf backend worker is not running")
        timeout = (
            self.spec.request_timeout_seconds
            if timeout_seconds is None
            else timeout_seconds
        )
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        try:
            events = selector.select(timeout)
        finally:
            selector.close()
        if not events:
            raise LeafBackendUnavailable(
                f"backend={self.spec.name} exceeded the configured response deadline"
            )
        line = process.stdout.readline()
        if not line:
            code = process.poll()
            raise LeafBackendUnavailable(
                f"backend={self.spec.name} worker exited unexpectedly (code={code})"
            )
        try:
            message = json.loads(line)
        except json.JSONDecodeError as exc:
            raise LeafBackendError("leaf backend returned invalid protocol JSON") from exc
        if not isinstance(message, dict):
            raise LeafBackendError("leaf backend returned a non-object protocol message")
        return message

    def start(self) -> None:
        if self._process is not None:
            return
        executable = self.spec.command[0]
        if os.path.isabs(executable):
            available = Path(executable).is_file() and os.access(executable, os.X_OK)
        else:
            available = shutil.which(executable) is not None
        if not available:
            raise LeafBackendUnavailable(
                f"backend={self.spec.name} configured runtime executable is unavailable: "
                f"{executable}"
            )
        self._validate_launcher_paths()
        command = [
            *self.spec.command,
            "--worker",
            "--config",
            self.config_path,
        ]
        self._process = subprocess.Popen(
            command,
            cwd=self.spec.working_directory,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            bufsize=1,
            env=self._build_worker_environment(),
        )
        try:
            message = self._read_message(self.spec.startup_timeout_seconds)
            if not message.get("ok"):
                error = str(message.get("error", "backend initialization failed"))
                raise LeafBackendUnavailable(error)
            if message.get("backend") != self.spec.name:
                raise LeafBackendError("worker selected a different backend")
        except Exception:
            # A failed handshake leaves no trustworthy protocol boundary.  In
            # particular, a late startup response must never be consumed as
            # the response to a later inference request.
            self.close()
            raise

    def infer(self, image) -> list[LeafDetection]:
        if self._process is None:
            # Restart only on a later request.  The request that observed a
            # timeout/crash still fails closed and is never replayed.
            self.start()
        try:
            import cv2
        except ImportError as exc:
            raise LeafBackendUnavailable("OpenCV is required by the ROS backend client") from exc
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            raise LeafBackendError("could not encode leaf ROI for backend transport")
        request_id = uuid.uuid4().hex
        request = {
            "operation": "infer",
            "request_id": request_id,
            "image_png": base64.b64encode(encoded.tobytes()).decode("ascii"),
        }
        try:
            with self._lock:
                process = self._process
                if process is None or process.stdin is None:
                    raise LeafBackendUnavailable("leaf backend worker is not running")
                process.stdin.write(json.dumps(request, separators=(",", ":")) + "\n")
                process.stdin.flush()
                response = self._read_message()
            if response.get("request_id") != request_id:
                raise LeafBackendError("leaf backend response ID mismatch")
            if not response.get("ok"):
                raise LeafBackendError(
                    str(response.get("error", "leaf inference failed"))
                )
            values = response.get("detections")
            if not isinstance(values, list):
                raise LeafBackendError("leaf backend detections must be a list")
        except Exception:
            # Timeout, worker death, OOM reports, malformed replies and ID
            # mismatches all invalidate the stream.  Closing prevents stale
            # replies from being mistaken for a future result and releases
            # framework/GPU state before the next-request restart.
            self.close()
            raise
        detections = [LeafDetection.from_mapping(value) for value in values]
        return [
            detection
            for detection in detections
            if detection.confidence
            >= self.spec.per_class_thresholds.get(
                detection.class_name, self.spec.threshold
            )
        ]

    def close(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        if process.poll() is None and process.stdin is not None:
            try:
                process.stdin.write('{"operation":"shutdown"}\n')
                process.stdin.flush()
                process.wait(timeout=2.0)
            except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()


def _find_import_shadow_collisions(working_directory: Path) -> tuple[str, ...]:
    """Return dangerous top-level import names present in a worker cwd."""
    collisions = set()
    for entry in working_directory.iterdir():
        if entry.is_dir() and entry.name in _IMPORT_SHADOW_NAMES:
            collisions.add(entry.name)
            continue
        if not entry.is_file():
            continue
        for name in _IMPORT_SHADOW_NAMES:
            if entry.name == f"{name}.py" or (
                entry.name.startswith(f"{name}.")
                and entry.suffix in {".pyc", ".so", ".pyd"}
            ):
                collisions.add(name)
    return tuple(sorted(collisions))


class _FrameworkBackend:
    def __init__(self, spec: LeafBackendSpec, manifest: Mapping[str, object]):
        self.spec = spec
        self.manifest = manifest

    def infer(self, image) -> list[LeafDetection]:
        raise NotImplementedError

    def _detection(self, box, class_id, confidence) -> LeafDetection:
        class_id = int(class_id)
        return LeafDetection(
            x1=float(box[0]),
            y1=float(box[1]),
            x2=float(box[2]),
            y2=float(box[3]),
            class_id=class_id,
            class_name=canonical_class_name(class_id),
            confidence=float(confidence),
            backend=self.spec.name,
            source=self.spec.framework,
            inference_size=self.spec.native_resolution,
        )

    def _validated_detections(self, candidates) -> list[LeafDetection]:
        """Drop only malformed geometry; retain fail-closed semantic checks."""
        accepted = []
        rejected_geometry = 0
        for box, class_id, confidence in candidates:
            try:
                accepted.append(self._detection(box, class_id, confidence))
            except InvalidDetectionGeometry:
                rejected_geometry += 1
        if rejected_geometry:
            print(
                "LEAF_BACKEND_INVALID_GEOMETRY_FILTERED: "
                f"backend={self.spec.name} count={rejected_geometry}",
                file=sys.stderr,
            )
        return accepted


class _UltralyticsBackend(_FrameworkBackend):
    def __init__(self, spec, manifest):
        super().__init__(spec, manifest)
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise LeafBackendUnavailable("Ultralytics runtime is unavailable") from exc
        self.model = YOLO(spec.checkpoint)

    def infer(self, image) -> list[LeafDetection]:
        result = self.model(
            image,
            verbose=False,
            conf=self.spec.threshold,
            imgsz=max(self.spec.native_resolution),
        )[0]
        return self._validated_detections(
            (
                box.xyxy[0].tolist(),
                box.cls[0].item(),
                box.conf[0].item(),
            )
            for box in result.boxes
        )


class _RFDETRBackend(_FrameworkBackend):
    def __init__(self, spec, manifest):
        super().__init__(spec, manifest)
        if manifest.get("runtime_output_indexing_verification") != "PASS":
            raise LeafBackendUnavailable(
                "RF-DETR runtime class IDs remain unverified; framework-native smoke test required"
            )
        try:
            from rfdetr import RFDETRNano
        except ImportError as exc:
            raise LeafBackendUnavailable("RF-DETR runtime is unavailable") from exc
        self.model = RFDETRNano(
            pretrain_weights=spec.checkpoint,
            resolution=max(spec.native_resolution),
        )

    def infer(self, image) -> list[LeafDetection]:
        rgb = image[:, :, ::-1].copy()
        detections = self.model.predict(rgb, threshold=self.spec.threshold)
        return self._validated_detections(
            (box, class_id, confidence)
            for box, class_id, confidence in zip(
                detections.xyxy, detections.class_id, detections.confidence
            )
        )


class _DFINEBackend(_FrameworkBackend):
    def __init__(self, spec, manifest):
        super().__init__(spec, manifest)
        if manifest.get("runtime_deploy_label_mapping_verification") != "PASS":
            raise LeafBackendUnavailable("D-FINE deploy label mapping is not verified")
        model_config = spec.runtime_options.get("model_config")
        if not isinstance(model_config, str) or not Path(model_config).is_file():
            raise LeafBackendUnavailable(
                "D-FINE runtime model_config is unavailable in the configured environment"
            )
        try:
            import torch
            from src.core import YAMLConfig
        except ImportError as exc:
            raise LeafBackendUnavailable("D-FINE runtime/source is unavailable") from exc
        self.torch = torch
        config = YAMLConfig(model_config, resume=spec.checkpoint)
        checkpoint = torch.load(spec.checkpoint, map_location="cpu", weights_only=False)
        state = checkpoint.get("ema", checkpoint.get("model", checkpoint))
        if isinstance(state, Mapping) and "module" in state:
            state = state["module"]
        config.model.load_state_dict(state)
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = config.model.deploy().to(self.device).eval()
        self.postprocessor = config.postprocessor.deploy().to(self.device).eval()

    def infer(self, image) -> list[LeafDetection]:
        torch = self.torch
        import numpy as np
        rgb = np.ascontiguousarray(image[:, :, ::-1])
        tensor = torch.from_numpy(rgb).permute(2, 0, 1).float().div(255.0).unsqueeze(0)
        tensor = torch.nn.functional.interpolate(
            tensor,
            size=self.spec.native_resolution,
            mode="bilinear",
            align_corners=False,
        ).to(self.device)
        original_size = torch.tensor(
            [[image.shape[1], image.shape[0]]], device=self.device
        )
        with torch.no_grad():
            labels, boxes, scores = self.postprocessor(self.model(tensor), original_size)
        return self._validated_detections(
            (box.tolist(), class_id.item(), score.item())
            for box, class_id, score in zip(boxes[0], labels[0], scores[0])
            if float(score.item()) >= self.spec.threshold
        )


def _create_framework_backend(spec: LeafBackendSpec) -> _FrameworkBackend:
    manifest = load_checkpoint_manifest(spec.manifest_path)
    if spec.runtime_options.get("device_policy", "cuda_required") == "cuda_required":
        try:
            import torch
        except ImportError as exc:
            raise LeafBackendUnavailable(
                f"backend={spec.name} cannot verify required CUDA runtime"
            ) from exc
        if not torch.cuda.is_available():
            raise LeafBackendUnavailable(
                f"backend={spec.name} requires CUDA; CPU fallback is not authorized"
            )
    expected_commit = spec.runtime_options.get("expected_source_commit")
    if expected_commit:
        if not spec.working_directory:
            raise LeafBackendUnavailable(
                f"backend={spec.name} requires a configured framework working directory"
            )
        try:
            actual_commit = subprocess.run(
                ["git", "-C", spec.working_directory, "rev-parse", "HEAD"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        except (OSError, subprocess.CalledProcessError) as exc:
            raise LeafBackendUnavailable(
                f"backend={spec.name} framework source commit cannot be verified"
            ) from exc
        if actual_commit != expected_commit:
            raise LeafBackendUnavailable(
                f"backend={spec.name} framework source commit does not match configuration"
            )
    implementations = {
        "ultralytics": _UltralyticsBackend,
        "rfdetr": _RFDETRBackend,
        "dfine": _DFINEBackend,
    }
    implementation = implementations.get(spec.framework)
    if implementation is None:
        raise LeafBackendUnavailable(f"unsupported leaf framework: {spec.framework}")
    return implementation(spec, manifest)


def _write_protocol(message: Mapping[str, object]) -> None:
    sys.stdout.write(json.dumps(message, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def run_worker(config_path: str) -> int:
    try:
        spec = LeafBackendSpec.from_config(config_path)
        with contextlib.redirect_stdout(sys.stderr):
            backend = _create_framework_backend(spec)
        _write_protocol({"ok": True, "backend": spec.name, "framework": spec.framework})
    except Exception as exc:
        _write_protocol({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
        return 2

    for line in sys.stdin:
        request_id = None
        try:
            request = json.loads(line)
            request_id = request.get("request_id")
            if request.get("operation") == "shutdown":
                return 0
            if request.get("operation") != "infer":
                raise LeafBackendError("unsupported worker operation")
            import cv2
            import numpy as np
            encoded = base64.b64decode(request["image_png"], validate=True)
            image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise LeafBackendError("worker could not decode image")
            with contextlib.redirect_stdout(sys.stderr):
                detections = backend.infer(image)
            _write_protocol({
                "ok": True,
                "request_id": request_id,
                "detections": [asdict(detection) for detection in detections],
            })
        except Exception as exc:
            _write_protocol({
                "ok": False,
                "request_id": request_id,
                "error": f"{type(exc).__name__}: {exc}",
            })
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Selectable leaf backend worker")
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("--config", required=True)
    args = parser.parse_args(argv)
    if not args.worker:
        parser.error("only worker mode is supported")
    return run_worker(args.config)


if __name__ == "__main__":
    raise SystemExit(main())
