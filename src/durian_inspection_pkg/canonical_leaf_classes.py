"""Strict canonical leaf-disease class contract for deployment boundaries."""

from __future__ import annotations

import hashlib
import json
from types import MappingProxyType
from typing import Mapping, Sequence


CANONICAL_CLASS_NAMES = (
    "leaf_algal",
    "leaf_blight",
    "leaf_colletotrichum",
    "leaf_healthy",
    "leaf_phomopsis",
    "leaf_rhizoctonia",
)
CANONICAL_ID_TO_NAME = MappingProxyType(dict(enumerate(CANONICAL_CLASS_NAMES)))
CANONICAL_NAME_TO_ID = MappingProxyType(
    {name: class_id for class_id, name in CANONICAL_ID_TO_NAME.items()}
)


def canonical_class_table() -> list[dict[str, object]]:
    """Return a JSON-safe copy of the ordered deployment class table."""
    return [
        {"id": class_id, "name": name}
        for class_id, name in CANONICAL_ID_TO_NAME.items()
    ]


def _normalise_table(table: object) -> list[dict[str, object]]:
    if isinstance(table, Mapping):
        rows = [{"id": key, "name": value} for key, value in table.items()]
    elif isinstance(table, Sequence) and not isinstance(table, (str, bytes)):
        rows = []
        for class_id, value in enumerate(table):
            if isinstance(value, str):
                rows.append({"id": class_id, "name": value})
            elif isinstance(value, Mapping):
                rows.append({"id": value.get("id"), "name": value.get("name")})
            else:
                raise ValueError("class table rows must be names or {id, name} mappings")
    else:
        raise ValueError("class table must be an ordered sequence or mapping")

    for row in rows:
        if isinstance(row["id"], bool) or not isinstance(row["id"], int):
            raise ValueError("class IDs must be integers, not booleans or strings")
        if not isinstance(row["name"], str):
            raise ValueError("class names must be strings")
    return rows


def validate_class_table(table: object) -> tuple[str, ...]:
    """Accept only the exact zero-based canonical table in canonical order."""
    rows = _normalise_table(table)
    expected = canonical_class_table()
    if rows != expected:
        raise ValueError(
            "class table must exactly match canonical IDs 0..5 and names in order; "
            f"expected {expected!r}, received {rows!r}"
        )
    return CANONICAL_CLASS_NAMES


def canonical_class_id(name: str) -> int:
    """Resolve an exact canonical name; aliases and fuzzy matches are rejected."""
    try:
        return CANONICAL_NAME_TO_ID[name]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"unknown canonical leaf class name: {name!r}") from exc


def canonical_class_name(class_id: int) -> str:
    """Resolve an exact zero-based canonical ID; one-based IDs are rejected."""
    if isinstance(class_id, bool):
        raise ValueError("class ID must be an integer, not a boolean")
    try:
        return CANONICAL_ID_TO_NAME[class_id]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"unknown canonical leaf class ID: {class_id!r}") from exc


def class_table_sha256(table: object | None = None) -> str:
    """Hash the canonical JSON representation of a validated ordered table."""
    if table is not None:
        validate_class_table(table)
    payload = json.dumps(
        canonical_class_table(), sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()
