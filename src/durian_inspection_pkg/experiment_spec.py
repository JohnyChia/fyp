"""Strict parser for versioned experiment ground-truth manifests.

This module is evaluation infrastructure.  Production inference modules must not
import it; that one-way boundary prevents expected labels leaking into results.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from canonical_leaf_classes import canonical_class_id


@dataclass(frozen=True)
class PhysicalLeaf:
    leaf_id: str
    visual_name: str
    class_name: str
    material: str
    pose: tuple[float, ...]
    slot: str
    sector: str
    height_band: str


@dataclass(frozen=True)
class ExperimentTree:
    tree_id: int
    model_name: str
    pose: tuple[float, ...]
    dominant_class: str
    total_leaves: int
    expected_counts: Mapping[str, int]
    intended_viewpoints: tuple[str, ...]
    leaves: tuple[PhysicalLeaf, ...]


@dataclass(frozen=True)
class ExperimentSpec:
    schema: str
    version: int
    world_path: str
    world_sha256: str | None
    validate_world_hash: bool
    class_table_sha256: str
    ground_truth_definition: str
    material_script: str
    texture_directory: str
    controlled_design: Mapping[str, object] | None
    trees: tuple[ExperimentTree, ...]


def _pose(value, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{field} must contain [x, y, z, roll, pitch, yaw]")
    return tuple(float(item) for item in value)


def parse_experiment_spec(data: Mapping[str, object]) -> ExperimentSpec:
    if not isinstance(data, Mapping):
        raise ValueError("experiment manifest must be a mapping")
    if data.get("schema") != "durian-controlled-experiment-gt":
        raise ValueError("unsupported experiment manifest schema")
    if data.get("version") != 1:
        raise ValueError("unsupported experiment manifest version")
    world = data.get("world")
    classes = data.get("classes")
    materials = data.get("materials")
    trees_data = data.get("trees")
    if not all(isinstance(item, Mapping) for item in (world, classes, materials)):
        raise ValueError("world, classes, and materials mappings are required")
    if not isinstance(trees_data, list) or not trees_data:
        raise ValueError("trees must be a non-empty list")
    trees = []
    tree_ids = set()
    model_names = set()
    global_leaf_ids = set()
    for item in trees_data:
        if not isinstance(item, Mapping):
            raise ValueError("each tree must be a mapping")
        tree_id = item.get("id")
        model_name = item.get("model_name")
        if not isinstance(tree_id, int) or isinstance(tree_id, bool):
            raise ValueError("tree id must be an integer")
        if tree_id in tree_ids:
            raise ValueError(f"duplicate tree id in manifest: {tree_id}")
        if not isinstance(model_name, str) or not model_name:
            raise ValueError("tree model_name is required")
        if model_name in model_names:
            raise ValueError(f"duplicate tree model name in manifest: {model_name}")
        dominant = item.get("dominant_class")
        canonical_class_id(dominant)
        counts = item.get("expected_class_counts")
        if not isinstance(counts, Mapping) or not counts:
            raise ValueError(f"tree {tree_id}: expected_class_counts is required")
        parsed_counts = {}
        for name, count in counts.items():
            canonical_class_id(name)
            if not isinstance(count, int) or isinstance(count, bool) or count < 0:
                raise ValueError(f"tree {tree_id}: class counts must be non-negative integers")
            parsed_counts[name] = count
        total = item.get("total_leaves")
        if not isinstance(total, int) or isinstance(total, bool) or total <= 0:
            raise ValueError(f"tree {tree_id}: total_leaves must be positive")
        if sum(parsed_counts.values()) != total:
            raise ValueError(f"tree {tree_id}: expected class counts do not sum to total_leaves")
        viewpoints = item.get("intended_viewpoints")
        if not isinstance(viewpoints, list) or not viewpoints or len(set(viewpoints)) != len(viewpoints):
            raise ValueError(f"tree {tree_id}: intended_viewpoints must be non-empty and unique")
        leaf_rows = item.get("leaves")
        if not isinstance(leaf_rows, list) or len(leaf_rows) != total:
            raise ValueError(f"tree {tree_id}: leaf record count differs from total_leaves")
        leaves = []
        for leaf in leaf_rows:
            if not isinstance(leaf, Mapping):
                raise ValueError(f"tree {tree_id}: leaf must be a mapping")
            leaf_id = leaf.get("id")
            if not isinstance(leaf_id, str) or not leaf_id:
                raise ValueError(f"tree {tree_id}: leaf id is required")
            if leaf_id in global_leaf_ids:
                raise ValueError(f"duplicate physical leaf id in manifest: {leaf_id}")
            class_name = leaf.get("class")
            canonical_class_id(class_name)
            required_text = ("visual_name", "material", "slot", "sector", "height_band")
            if any(not isinstance(leaf.get(key), str) or not leaf.get(key) for key in required_text):
                raise ValueError(f"leaf {leaf_id}: visual/material/slot/sector/height_band are required")
            leaves.append(PhysicalLeaf(
                leaf_id, leaf["visual_name"], class_name, leaf["material"],
                _pose(leaf.get("pose"), f"leaf {leaf_id}.pose"), leaf["slot"],
                leaf["sector"], leaf["height_band"],
            ))
            global_leaf_ids.add(leaf_id)
        observed_counts = {name: 0 for name in parsed_counts}
        for leaf in leaves:
            observed_counts[leaf.class_name] = observed_counts.get(leaf.class_name, 0) + 1
        if observed_counts != parsed_counts:
            raise ValueError(f"tree {tree_id}: leaf records do not match expected_class_counts")
        max_count = max(parsed_counts.values())
        leaders = [name for name, count in parsed_counts.items() if count == max_count]
        if leaders != [dominant]:
            raise ValueError(f"tree {tree_id}: dominant_class is not the unique count maximum")
        tree_ids.add(tree_id)
        model_names.add(model_name)
        trees.append(ExperimentTree(tree_id, model_name, _pose(item.get("pose"), f"tree {tree_id}.pose"),
                                    dominant, total, parsed_counts, tuple(viewpoints), tuple(leaves)))
    hash_value = world.get("sha256")
    hash_enabled = world.get("validate_sha256", False)
    if not isinstance(hash_enabled, bool):
        raise ValueError("world.validate_sha256 must be boolean")
    if hash_enabled and (not isinstance(hash_value, str) or len(hash_value) != 64):
        raise ValueError("world.sha256 must be provided when hash validation is enabled")
    design = data.get("controlled_design")
    if design is not None and not isinstance(design, Mapping):
        raise ValueError("controlled_design must be a mapping when present")
    return ExperimentSpec(
        str(data["schema"]), int(data["version"]), str(world.get("path", "")),
        hash_value, hash_enabled, str(classes.get("table_sha256", "")),
        str(data.get("ground_truth_definition", "")), str(materials.get("script", "")),
        str(materials.get("texture_directory", "")), design, tuple(trees),
    )


def load_experiment_spec(path: str | Path) -> ExperimentSpec:
    import yaml
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return parse_experiment_spec(data)
