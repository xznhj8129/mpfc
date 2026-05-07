"""YAML protocol namespace loader.

Usage:
    from protocols.namespace_loader import load_protocol_namespace

    UAV = load_protocol_namespace("UAV")
    print(UAV.Action.Flight.Arm)
    print(UAV.Enums.FCAutopilotType.PX4)
"""

from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import yaml

PROTOCOLS_DIR = Path(__file__).resolve().parent
SCHEMA_FORMAT_PATH = PROTOCOLS_DIR / "schema_format.yaml"


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _build_enum_namespace(enums: dict[str, list[str]]) -> SimpleNamespace:
    values: dict[str, Any] = {}
    for enum_name, members in enums.items():
        values[enum_name] = SimpleNamespace(**{member: member for member in members})
    return SimpleNamespace(**values)


def _first_message_key(structs: dict[str, Any], struct_name: str) -> str:
    struct = structs[struct_name]
    if "key" in struct:
        return str(struct["key"])

    variants = struct["variants"]
    if type(variants) is list:
        return _first_message_key(structs, variants[0])

    for child_value in variants.values():
        if type(child_value) is dict and "key" in child_value:
            return str(child_value["key"])
        if type(child_value) is dict and "variants" in child_value:
            inline_struct_name = str(child_value["variants"][0])
            return _first_message_key(structs, inline_struct_name)
        if type(child_value) is str:
            return _first_message_key(structs, child_value)

    raise RuntimeError(f"message key not found struct={struct_name}")


def _key_token(key: str, index: int) -> str:
    return key.split(".", 4)[index]


def _build_namespace(structs: dict[str, Any], struct_name: str, token_index: int) -> SimpleNamespace:
    struct = structs[struct_name]
    variants = struct["variants"]
    values: dict[str, Any] = {}

    if type(variants) is list:
        for child_struct_name in variants:
            child_name = _key_token(_first_message_key(structs, child_struct_name), token_index)
            values[child_name] = _build_namespace(structs, child_struct_name, token_index + 1)
        return SimpleNamespace(**values)

    for child_name, child_value in variants.items():
        if type(child_value) is dict and "key" in child_value:
            values[child_name] = child_name
            continue

        if type(child_value) is str:
            child_struct = structs[child_value]
            if "variants" in child_struct:
                nested_name = _key_token(_first_message_key(structs, child_value), token_index)
                values[nested_name] = _build_namespace(structs, child_value, token_index + 1)
                continue
            values[child_value] = child_value
            continue

        raise RuntimeError(f"unsupported struct variant struct={struct_name} child={child_name}")

    return SimpleNamespace(**values)


def _flatten_leaf_tokens(structs: dict[str, Any], struct_name: str) -> list[tuple[str, ...]]:
    struct = structs[struct_name]
    if "key" in struct:
        return [(struct_name,)]

    variants = struct["variants"]
    leaves: list[tuple[str, ...]] = []

    if type(variants) is list:
        for child_struct_name in variants:
            child_name = _key_token(_first_message_key(structs, child_struct_name), 2)
            for sub_path in _flatten_leaf_tokens(structs, child_struct_name):
                leaves.append((child_name,) + sub_path)
        return leaves

    for child_name, child_value in variants.items():
        if type(child_value) is dict and "key" in child_value:
            leaves.append((child_name,))
            continue

        if type(child_value) is str:
            child_struct = structs[child_value]
            if "variants" in child_struct:
                nested_name = _key_token(_first_message_key(structs, child_value), 3)
                for sub_path in _flatten_leaf_tokens(structs, child_value):
                    leaves.append((nested_name,) + sub_path)
                continue
            leaves.append((child_value,))
            continue

        raise RuntimeError(f"unsupported struct variant struct={struct_name} child={child_name}")

    return leaves


@lru_cache(maxsize=None)
def _load_schema(domain: str) -> dict[str, Any]:
    target_name = domain.upper()
    for schema_path in sorted(PROTOCOLS_DIR.glob("*.yaml")):
        if schema_path == SCHEMA_FORMAT_PATH:
            continue
        schema = _load_yaml(schema_path)
        if type(schema) is dict and str(schema["name"]).upper() == target_name:
            return schema
    raise RuntimeError(f"protocol not found domain={domain}")


@lru_cache(maxsize=None)
def load_protocol_namespace(domain: str) -> SimpleNamespace:
    schema = _load_schema(domain)
    enums = schema.get("enums", {})
    structs = schema["structs"]

    state_namespace = _build_namespace(structs, "State", 2)
    query_to_state: dict[str, str] = {}
    for leaf_path in _flatten_leaf_tokens(structs, "State"):
        token = leaf_path[-1]
        if token in query_to_state:
            raise RuntimeError(f"duplicate state token for query derivation token={token}")
        query_to_state[token] = token

    return SimpleNamespace(
        Enums=_build_enum_namespace(enums),
        State=state_namespace,
        Action=_build_namespace(structs, "Action", 2),
        Event=_build_namespace(structs, "Event", 2),
        Query=state_namespace,
        QueryToState=query_to_state,
    )
