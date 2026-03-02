"""Shared protocol namespace loader for cores/plugins."""

import json
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace
from typing import Any

PROTOCOLS_DIR = Path(__file__).resolve().parent
REGISTRY_PATH = PROTOCOLS_DIR / "registry.json"


def _load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _build_namespace(tree: dict[str, Any]) -> SimpleNamespace:
    values: dict[str, Any] = {}
    for key, value in tree.items():
        if type(value) is dict and "Fields" not in value and "Key" not in value:
            values[key] = _build_namespace(value)
            continue
        values[key] = key
    return SimpleNamespace(**values)


def _flatten_leaf_paths(tree: dict[str, Any]) -> list[tuple[str, ...]]:
    leaves: list[tuple[str, ...]] = []
    for key, value in tree.items():
        if type(value) is dict and "Fields" not in value and "Key" not in value:
            for sub_path in _flatten_leaf_paths(value):
                leaves.append((key,) + sub_path)
            continue
        leaves.append((key,))
    return leaves


@lru_cache(maxsize=None)
def _load_schema(domain: str) -> dict[str, Any]:
    registry = _load_json(REGISTRY_PATH)
    protocol_entries = registry["Protocols"]
    target_name = domain.upper()
    for entry in protocol_entries:
        if str(entry["Name"]).upper() == target_name:
            schema_path = PROTOCOLS_DIR / str(entry["SchemaPath"])
            return _load_json(schema_path)
    raise RuntimeError(f"protocol not found domain={domain}")


@lru_cache(maxsize=None)
def load_protocol_namespace(domain: str) -> SimpleNamespace:
    schema = _load_schema(domain)
    messages = schema["Messages"]
    state_tree = messages["State"]
    action_tree = messages["Action"]
    event_tree = messages["Event"]

    query_to_state: dict[str, str] = {}
    for leaf_path in _flatten_leaf_paths(state_tree):
        token = leaf_path[-1]
        if token in query_to_state:
            raise RuntimeError(f"duplicate state token for query derivation token={token}")
        query_to_state[token] = token

    return SimpleNamespace(
        State=_build_namespace(state_tree),
        Action=_build_namespace(action_tree),
        Event=_build_namespace(event_tree),
        Query=_build_namespace(state_tree),
        QueryToState=query_to_state,
    )
