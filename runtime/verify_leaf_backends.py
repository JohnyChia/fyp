#!/usr/bin/env python3
"""Read-only native and central-worker verification for all leaf backends."""

from __future__ import annotations

import argparse
import copy
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any


ROOT = Path("/home/johny/durian_ws")
PACKAGE_DIR = ROOT / "src/durian_inspection_pkg"
sys.path.insert(0, str(PACKAGE_DIR))
from canonical_leaf_classes import CANONICAL_CLASS_NAMES

CONFIG_PATH = PACKAGE_DIR / "config/leaf_detector_deployment.yaml"
IMAGE_PATH = (
    ROOT
    / "experiment/datasets/domains/real/unlabelled_source/photos/real_source_0000.jpg"
)
BACKENDS = ("yolo26n", "yolo26n_p2", "rfdetr_n", "dfine_n")
CANONICAL_NAMES = CANONICAL_CLASS_NAMES
SHARED_SITE = ROOT / ".venv/lib/python3.10/site-packages"
OVERLAYS = (
    ROOT / "runtime/leaf_backend_deps/rfdetr",
    ROOT / "runtime/leaf_backend_deps/dfine",
)
DFINE_SOURCE = (
    ROOT
    / "runtime/leaf_backend_sources"
    / "dfine-956d1709314c2c6a4df6f34de232054578a7449f"
)
DFINE_COMMIT = "956d1709314c2c6a4df6f34de232054578a7449f"


def _add_package_path() -> None:
    package_path = str(PACKAGE_DIR)
    if package_path not in sys.path:
        sys.path.insert(0, package_path)


def _valid_detection(box: Any, score: Any, class_id: Any) -> int:
    values = [float(value) for value in box]
    score_value = float(score)
    class_value = int(class_id)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite bounding box")
    if len(values) != 4 or values[2] <= values[0] or values[3] <= values[1]:
        raise ValueError("invalid bounding-box geometry")
    if not math.isfinite(score_value) or not 0.0 <= score_value <= 1.0:
        raise ValueError("invalid confidence score")
    if not 0 <= class_value < len(CANONICAL_NAMES):
        raise ValueError("class ID outside canonical range")
    return class_value


def _native_yolo(spec: Any, image: Any) -> dict[str, Any]:
    import importlib.metadata as metadata
    import torch
    import ultralytics
    from ultralytics import YOLO

    if metadata.version("ultralytics") != "8.4.61":
        raise RuntimeError("unexpected Ultralytics runtime version")
    if SHARED_SITE not in Path(ultralytics.__file__).resolve().parents:
        raise RuntimeError("Ultralytics did not load from the shared environment")

    model = YOLO(spec.checkpoint)
    result = model(
        image,
        device=0,
        verbose=False,
        conf=spec.threshold,
        imgsz=max(spec.native_resolution),
    )[0]
    ids = [
        _valid_detection(
            box.xyxy[0].tolist(), box.conf[0].item(), box.cls[0].item()
        )
        for box in result.boxes
    ]
    if not next(model.model.parameters()).is_cuda:
        raise RuntimeError("YOLO model is not on CUDA")
    del result, model
    gc.collect()
    torch.cuda.empty_cache()
    return {"ids": ids, "torch_file": torch.__file__}


def _native_rfdetr(spec: Any, image: Any) -> dict[str, Any]:
    import importlib.metadata as metadata
    import cv2
    import rfdetr
    import torch
    import transformers
    from rfdetr import RFDETRNano

    overlay = OVERLAYS[0]
    if metadata.version("rfdetr") != "1.11.0":
        raise RuntimeError("unexpected RF-DETR runtime version")
    if Path(rfdetr.__file__).resolve().is_relative_to(overlay) is False:
        raise RuntimeError("RF-DETR did not load from its overlay")
    if transformers.__version__ != "5.17.0":
        raise RuntimeError("unexpected RF-DETR Transformers version")
    if Path(transformers.__file__).resolve().is_relative_to(overlay) is False:
        raise RuntimeError("RF-DETR Transformers did not load from its overlay")

    model = RFDETRNano(
        pretrain_weights=spec.checkpoint,
        resolution=max(spec.native_resolution),
    )
    if list(model.class_names) != list(CANONICAL_NAMES):
        raise RuntimeError("RF-DETR checkpoint class names are not canonical")
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    detections = model.predict(
        rgb,
        threshold=spec.threshold,
        include_source_image=False,
    )
    ids = [
        _valid_detection(box, score, class_id)
        for box, score, class_id in zip(
            detections.xyxy,
            detections.confidence,
            detections.class_id,
        )
    ]
    if not ids:
        raise RuntimeError("RF-DETR returned no detections on the frozen diagnostic image")
    if model.model.device.type != "cuda":
        raise RuntimeError("RF-DETR model is not on CUDA")
    for class_id in ids:
        if model.class_names[class_id] != CANONICAL_NAMES[class_id]:
            raise RuntimeError("RF-DETR raw class ID needs an unauthorized offset")
    del detections, model
    gc.collect()
    torch.cuda.empty_cache()
    return {
        "ids": ids,
        "torch_file": torch.__file__,
        "transformers_version": transformers.__version__,
        "transformers_file": transformers.__file__,
    }


def _native_dfine(spec: Any, image: Any) -> dict[str, Any]:
    import numpy as np
    import torch
    import transformers
    from src.core import YAMLConfig

    overlay = OVERLAYS[1]
    if transformers.__version__ != "4.57.1":
        raise RuntimeError("unexpected D-FINE Transformers version")
    if Path(transformers.__file__).resolve().is_relative_to(overlay) is False:
        raise RuntimeError("D-FINE Transformers did not load from its overlay")

    model_config = str(spec.runtime_options["model_config"])
    config = YAMLConfig(model_config, resume=spec.checkpoint)
    if config.yaml_cfg.get("num_classes") != len(CANONICAL_NAMES):
        raise RuntimeError("D-FINE runtime num_classes differs from canonical table")
    if config.yaml_cfg.get("remap_mscoco_category") is not True:
        raise RuntimeError("D-FINE runtime remap_mscoco_category is not true")
    checkpoint = torch.load(spec.checkpoint, map_location="cpu", weights_only=False)
    state = checkpoint.get("ema", checkpoint.get("model", checkpoint))
    if isinstance(state, dict) and "module" in state:
        state = state["module"]
    config.model.load_state_dict(state, strict=True)
    device = torch.device("cuda:0")
    model = config.model.deploy().to(device).eval()
    postprocessor = config.postprocessor.deploy().to(device).eval()
    rgb = np.ascontiguousarray(image[:, :, ::-1])
    tensor = (
        torch.from_numpy(rgb)
        .permute(2, 0, 1)
        .float()
        .div(255.0)
        .unsqueeze(0)
    )
    tensor = torch.nn.functional.interpolate(
        tensor,
        size=spec.native_resolution,
        mode="bilinear",
        align_corners=False,
    ).to(device)
    original_size = torch.tensor(
        [[image.shape[1], image.shape[0]]], device=device
    )
    with torch.no_grad():
        labels, boxes, scores = postprocessor(model(tensor), original_size)
    ids = [
        _valid_detection(box.tolist(), score.item(), label.item())
        for label, box, score in zip(labels[0], boxes[0], scores[0])
        if float(score.item()) >= spec.threshold
    ]
    if not ids:
        raise RuntimeError("D-FINE returned no detections on the frozen diagnostic image")
    if not next(model.parameters()).is_cuda:
        raise RuntimeError("D-FINE model is not on CUDA")
    del tensor, labels, boxes, scores, postprocessor, model, config
    gc.collect()
    torch.cuda.empty_cache()
    return {
        "ids": ids,
        "torch_file": torch.__file__,
        "transformers_version": transformers.__version__,
        "transformers_file": transformers.__file__,
        "remap_mscoco_category": True,
    }


def _native_main(config_path: Path) -> int:
    _add_package_path()
    import cv2
    import torch
    from leaf_backend_runtime import LeafBackendSpec

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    spec = LeafBackendSpec.from_config(config_path)
    image = cv2.imread(str(IMAGE_PATH))
    if image is None:
        raise RuntimeError(f"diagnostic image is unreadable: {IMAGE_PATH}")
    if spec.framework == "ultralytics":
        evidence = _native_yolo(spec, image)
    elif spec.framework == "rfdetr":
        evidence = _native_rfdetr(spec, image)
    elif spec.framework == "dfine":
        evidence = _native_dfine(spec, image)
    else:
        raise RuntimeError(f"unsupported framework: {spec.framework}")
    evidence.update(
        {
            "backend": spec.name,
            "import": True,
            "load": True,
            "infer": True,
            "class_ids": True,
        }
    )
    print(json.dumps(evidence, sort_keys=True))
    return 0


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _no_torch_duplication() -> bool:
    forbidden = ("torch", "torchvision", "triton", "nvidia", "cuda-toolkit")
    for overlay in OVERLAYS:
        for entry in overlay.iterdir():
            normalized = entry.name.lower().replace("_", "-")
            if any(
                normalized == name
                or normalized.startswith(f"{name}-")
                for name in forbidden
            ):
                return False
    return True


def _run_native(spec: Any, config_path: Path) -> dict[str, Any]:
    environment = os.environ.copy()
    environment.update(spec.environment)
    environment["PYTHONPATH"] = os.pathsep.join(spec.python_path)
    environment["PYTHONNOUSERSITE"] = "1"
    completed = subprocess.run(
        [
            str(ROOT / ".venv/bin/python"),
            "-B",
            str(Path(__file__).resolve()),
            "--native-config",
            str(config_path),
        ],
        cwd=spec.working_directory,
        env=environment,
        text=True,
        capture_output=True,
        timeout=120,
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or completed.stdout.strip())
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    return json.loads(lines[-1])


def _ros_package_ready() -> bool:
    setup_text = (PACKAGE_DIR / "setup.py").read_text(encoding="utf-8")
    if "'leaf_backend_runtime'" not in setup_text:
        return False
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(PACKAGE_DIR), environment.get("PYTHONPATH", "")]
    ).rstrip(os.pathsep)
    check = subprocess.run(
        [
            "/usr/bin/python3",
            "-B",
            "-c",
            (
                "import processing_node; "
                "from leaf_backend_runtime import LeafBackendSpec; "
                f"LeafBackendSpec.from_config({str(CONFIG_PATH)!r})"
            ),
        ],
        env=environment,
        text=True,
        capture_output=True,
        timeout=30,
    )
    return check.returncode == 0


def _main() -> int:
    _add_package_path()
    import cv2
    import torch
    import yaml
    from leaf_backend_runtime import LeafBackendClient, LeafBackendSpec

    base_config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    image = cv2.imread(str(IMAGE_PATH))
    if image is None:
        raise RuntimeError(f"diagnostic image is unreadable: {IMAGE_PATH}")

    rows: dict[str, dict[str, bool]] = {
        backend: {
            "config": False,
            "import": False,
            "load": False,
            "infer": False,
            "class_ids": False,
            "worker": False,
        }
        for backend in BACKENDS
    }
    native_evidence: dict[str, dict[str, Any]] = {}
    checkpoint_hashes = True

    for backend in BACKENDS:
        selected = copy.deepcopy(base_config)
        selected["leaf_detector"]["backend"] = backend
        temporary_path: Path | None = None
        client: LeafBackendClient | None = None
        try:
            with tempfile.NamedTemporaryFile(
                "w", suffix=".yaml", dir="/tmp", delete=False
            ) as stream:
                yaml.safe_dump(selected, stream, sort_keys=False)
                temporary_path = Path(stream.name)
            spec = LeafBackendSpec.from_config(temporary_path)
            rows[backend]["config"] = spec.name == backend
            manifest = json.loads(Path(spec.manifest_path).read_text(encoding="utf-8"))
            checkpoint_hashes &= _sha256(Path(spec.checkpoint)) == manifest["sha256"]
            evidence = _run_native(spec, temporary_path)
            native_evidence[backend] = evidence
            for key in ("import", "load", "infer", "class_ids"):
                rows[backend][key] = bool(evidence[key])

            client = LeafBackendClient(temporary_path)
            client.start()
            detections = client.infer(image)
            for detection in detections:
                if detection.backend != backend:
                    raise RuntimeError("worker returned a different backend identity")
                if detection.class_name != CANONICAL_NAMES[detection.class_id]:
                    raise RuntimeError("worker returned a non-canonical class mapping")
                _valid_detection(
                    (detection.x1, detection.y1, detection.x2, detection.y2),
                    detection.confidence,
                    detection.class_id,
                )
            rows[backend]["worker"] = True
        except Exception as exc:
            print(f"{backend}: {type(exc).__name__}: {exc}", file=sys.stderr)
        finally:
            if client is not None:
                client.close()
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    torch_path = Path(torch.__file__).resolve()
    shared_stack = torch.cuda.is_available() and SHARED_SITE in torch_path.parents
    no_duplication = _no_torch_duplication()
    rfdetr_manifest = json.loads(
        (ROOT / "models/leaf_candidates/manifests/rfdetr_n.json").read_text()
    )
    rfdetr_indexing = (
        rfdetr_manifest.get("runtime_output_indexing_verification") == "PASS"
        and bool(native_evidence.get("rfdetr_n", {}).get("ids"))
    )
    dfine_indexing = (
        native_evidence.get("dfine_n", {}).get("remap_mscoco_category") is True
        and bool(native_evidence.get("dfine_n", {}).get("ids"))
    )
    source_head = subprocess.check_output(
        ["git", "-C", str(DFINE_SOURCE), "rev-parse", "HEAD"], text=True
    ).strip()
    ros_ready = _ros_package_ready() and source_head == DFINE_COMMIT

    print("BACKEND       CONFIG  IMPORT  LOAD  INFER  CLASS_IDS  WORKER  RESULT")
    for backend in BACKENDS:
        values = rows[backend]
        result = all(values.values())
        print(
            f"{backend:<13} "
            f"{'PASS' if values['config'] else 'FAIL':<7} "
            f"{'PASS' if values['import'] else 'FAIL':<7} "
            f"{'PASS' if values['load'] else 'FAIL':<5} "
            f"{'PASS' if values['infer'] else 'FAIL':<6} "
            f"{'PASS' if values['class_ids'] else 'FAIL':<10} "
            f"{'PASS' if values['worker'] else 'FAIL':<7} "
            f"{'PASS' if result else 'FAIL'}"
        )

    checks = {
        "CUDA_SHARED_STACK": shared_stack,
        "NO_TORCH_DUPLICATION": no_duplication,
        "RFDETR_INDEXING": rfdetr_indexing,
        "DFINE_INDEXING": dfine_indexing,
        "CHECKPOINT_HASHES": checkpoint_hashes,
        "ROS_PACKAGE_READY": ros_ready,
    }
    for label, passed in checks.items():
        print(f"{label:<24} {'PASS' if passed else 'FAIL'}")
    return 0 if all(all(row.values()) for row in rows.values()) and all(checks.values()) else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-config", type=Path)
    arguments = parser.parse_args()
    try:
        if arguments.native_config is not None:
            raise SystemExit(_native_main(arguments.native_config))
        raise SystemExit(_main())
    except Exception as error:
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr)
        raise SystemExit(1)
