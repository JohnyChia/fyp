"""Fail-closed SDF world to experiment-manifest verifier."""

from __future__ import annotations

from collections import Counter
import hashlib
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from canonical_leaf_classes import CANONICAL_CLASS_NAMES, canonical_class_id, class_table_sha256
from experiment_spec import ExperimentSpec, load_experiment_spec


class VerificationError(RuntimeError):
    pass


def _fail(message: str) -> None:
    raise VerificationError(message)


def _numbers(text: str | None, subject: str) -> tuple[float, ...]:
    try:
        values = tuple(float(item) for item in (text or "").split())
    except ValueError:
        _fail(f"{subject}: pose contains a non-numeric value")
    if len(values) != 6:
        _fail(f"{subject}: expected six pose values, found {len(values)}")
    return values


def _same_pose(left, right, tolerance=1e-9):
    return all(abs(a-b) <= tolerance for a, b in zip(left, right))


def _material_catalog(script_path: Path) -> dict[str, str]:
    if not script_path.is_file():
        _fail(f"material script does not exist: {script_path}")
    text = script_path.read_text(encoding="utf-8")
    catalog = {}
    pattern = re.compile(r"material\s+(\S+)\s*\{.*?texture\s+(\S+)", re.DOTALL)
    for name, texture in pattern.findall(text):
        catalog[name] = texture
    return catalog


def verify_world(spec: ExperimentSpec, world_path: str | Path | None = None) -> None:
    path = Path(world_path or spec.world_path)
    if not path.is_file():
        _fail(f"world file does not exist: {path}")
    if spec.validate_world_hash:
        actual_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual_hash != spec.world_sha256:
            _fail(f"world SHA256 mismatch: expected {spec.world_sha256}, found {actual_hash}")
    actual_class_hash = class_table_sha256()
    if actual_class_hash != spec.class_table_sha256:
        _fail(f"canonical class-table hash mismatch: expected {spec.class_table_sha256}, found {actual_class_hash}")
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        _fail(f"world XML cannot be parsed: {exc}")
    models = {model.get("name"): model for model in root.findall(".//world/model")}
    expected_names = {tree.model_name for tree in spec.trees}
    design = spec.controlled_design or {}
    if design:
        expected_tree_count = int(design.get("expected_tree_count", 0))
        if expected_tree_count <= 0 or len(spec.trees) != expected_tree_count:
            _fail(
                "manifest tree count differs from controlled_design: "
                f"expected {expected_tree_count}, found {len(spec.trees)}"
            )
    tree_like_names = set()
    for name, model in models.items():
        materials_in_model = {
            visual.findtext("./material/script/name")
            for visual in model.findall("./link/visual")
        }
        if any(material in CANONICAL_CLASS_NAMES for material in materials_in_model):
            tree_like_names.add(name)
    if tree_like_names != expected_names:
        _fail(f"world tree model set mismatch: expected {sorted(expected_names)}, found {sorted(tree_like_names)}")
    script = Path(spec.material_script)
    textures = Path(spec.texture_directory)
    catalog = _material_catalog(script)
    world_leaf_ids = set()
    for tree in spec.trees:
        model = models.get(tree.model_name)
        if model is None:
            _fail(f"missing tree model: {tree.model_name}")
        suffix = tree.model_name.rsplit("_", 1)[-1]
        if suffix.isdigit() and int(suffix) != tree.tree_id:
            _fail(
                f"{tree.model_name}: world model ID differs from manifest tree ID {tree.tree_id}"
            )
        if not _same_pose(_numbers(model.findtext("pose"), tree.model_name), tree.pose):
            _fail(f"wrong tree pose for {tree.model_name}: expected {tree.pose}")
        visual_rows = model.findall("./link/visual")
        visual_names = [visual.get("name") for visual in visual_rows]
        duplicate_names = sorted({name for name in visual_names if visual_names.count(name) > 1})
        if duplicate_names:
            _fail(f"{tree.model_name}: duplicate leaf/visual IDs: {duplicate_names}")
        visuals = {visual.get("name"): visual for visual in visual_rows}
        expected_visuals = {leaf.visual_name for leaf in tree.leaves}
        leaf_visuals = {
            name for name, visual in visuals.items()
            if name in expected_visuals
            or visual.findtext("./material/script/name") in CANONICAL_CLASS_NAMES
        }
        if leaf_visuals != expected_visuals:
            missing = sorted(expected_visuals - leaf_visuals)
            extra = sorted(leaf_visuals - expected_visuals)
            _fail(f"{tree.model_name}: leaf visual set mismatch; missing={missing}, extra={extra}")
        counts = Counter()
        for leaf in tree.leaves:
            if leaf.leaf_id in world_leaf_ids:
                _fail(f"duplicate physical leaf ID across world: {leaf.leaf_id}")
            world_leaf_ids.add(leaf.leaf_id)
            visual = visuals[leaf.visual_name]
            actual_material = visual.findtext("./material/script/name")
            if actual_material != leaf.material:
                _fail(f"leaf {leaf.leaf_id}: material mismatch; expected {leaf.material}, found {actual_material}")
            if actual_material != leaf.class_name:
                _fail(f"leaf {leaf.leaf_id}: visual class {leaf.class_name} differs from Gazebo material {actual_material}")
            canonical_class_id(actual_material)
            if not _same_pose(_numbers(visual.findtext("pose"), f"leaf {leaf.leaf_id}"), leaf.pose):
                _fail(f"leaf {leaf.leaf_id}: pose differs from manifest")
            texture = catalog.get(actual_material)
            if not texture:
                _fail(f"leaf {leaf.leaf_id}: material {actual_material} is not defined in {script}")
            if not (textures / texture).is_file():
                _fail(f"leaf {leaf.leaf_id}: texture does not exist: {textures / texture}")
            counts[actual_material] += 1
        if dict(counts) != dict(tree.expected_counts):
            _fail(f"{tree.model_name}: world class counts mismatch; expected {dict(tree.expected_counts)}, found {dict(counts)}")
        if sum(counts.values()) != tree.total_leaves:
            _fail(f"{tree.model_name}: total leaf count mismatch; expected {tree.total_leaves}, found {sum(counts.values())}")
        leaders = [name for name, count in counts.items() if count == max(counts.values())]
        if leaders != [tree.dominant_class]:
            _fail(f"{tree.model_name}: physical-count dominant class mismatch; expected {tree.dominant_class}, found {leaders}")
        if design:
            leaves_per_tree = int(design.get("leaves_per_tree", 0))
            sector_count = int(design.get("sector_count", 0))
            dominant_per_sector = int(design.get("dominant_per_sector", 0))
            minority_per_sector = int(design.get("each_minority_per_sector", 0))
            if tree.total_leaves != leaves_per_tree or sector_count <= 0:
                _fail(f"{tree.model_name}: controlled leaf total/sector definition mismatch")
            expected_classes = set(CANONICAL_CLASS_NAMES)
            if set(counts) != expected_classes or any(counts[name] <= 0 for name in expected_classes):
                _fail(f"{tree.model_name}: every canonical class must be physically present")
            sectors = {}
            for leaf in tree.leaves:
                sectors.setdefault(leaf.sector, Counter())[leaf.class_name] += 1
            if len(sectors) != sector_count:
                _fail(f"{tree.model_name}: expected {sector_count} balanced sectors, found {len(sectors)}")
            for sector, sector_counts in sectors.items():
                if sector_counts[tree.dominant_class] != dominant_per_sector:
                    _fail(f"{tree.model_name}/{sector}: dominant count is not balanced")
                for name in expected_classes - {tree.dominant_class}:
                    if sector_counts[name] != minority_per_sector:
                        _fail(f"{tree.model_name}/{sector}: minority count is not balanced")


def verify_manifest(manifest_path: str | Path, world_path: str | Path | None = None) -> None:
    verify_world(load_experiment_spec(manifest_path), world_path)
