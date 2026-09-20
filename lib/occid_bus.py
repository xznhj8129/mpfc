"""OCCID transport helpers for the MPFC MQTT runtime.

The local MQTT bus intentionally stays JSON-readable for debugging and
inspection. OCCID models are rendered directly into JSON-compatible fields with
small model identity tags; OCCID's MsgPack ``encode()`` remains available for
binary transports that actually need a compact wire representation.
"""

from __future__ import annotations

import base64
from enum import Enum, IntEnum as StdIntEnum
from typing import Any

import occid


OCCID_MODEL_KEY = "_occid_model"
OCCID_MODEL_ID_KEY = "_occid_model_id"
OCCID_BYTES_KEY = "_bytes_b64"
OCCID_META_KEYS = {OCCID_MODEL_KEY, OCCID_MODEL_ID_KEY}


def is_occid_model(value: Any) -> bool:
    return isinstance(value, (occid.OCCIDModel, occid.OCCIDValue))


def _to_bus_value(value: Any) -> Any:
    if is_occid_model(value):
        return _pack_occid_model(value)
    if isinstance(value, StdIntEnum):
        return value.name
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, bytes):
        return {OCCID_BYTES_KEY: base64.b64encode(value).decode("ascii")}
    if type(value) == dict:
        return {key: _to_bus_value(item) for key, item in value.items()}
    if type(value) in (list, tuple):
        return [_to_bus_value(item) for item in value]
    return value


def _pack_occid_model(model: Any) -> dict[str, Any]:
    model_type = type(model)
    model_id = occid.OCCID_MODEL_ID_BY_CLASS.get(model_type)
    if model_id is None:
        raise ValueError(f"OCCID model has no registered model id type={model_type.__name__}")
    payload = {
        OCCID_MODEL_KEY: model_type.__name__,
        OCCID_MODEL_ID_KEY: int(model_id),
    }
    if isinstance(model, occid.OCCIDValue):
        payload["value"] = _to_bus_value(model.root)
        return payload
    payload.update(
        {
            field_name: _to_bus_value(getattr(model, field_name))
            for field_name in model_type.model_fields
        }
    )
    return payload


def pack_occid(model: Any) -> dict[str, Any]:
    """Render an OCCID model as inspectable JSON-compatible bus data."""
    if not is_occid_model(model):
        raise TypeError(f"expected OCCID model, got {type(model).__name__}")
    return _pack_occid_model(model)


def _bus_model_to_wire(payload: dict[str, Any]) -> tuple[type, dict[str, Any]]:
    if not OCCID_META_KEYS.issubset(payload):
        raise ValueError("invalid OCCID bus payload: missing model metadata")

    model_id = int(payload[OCCID_MODEL_ID_KEY])
    model_type = occid.OCCID_MODEL_BY_ID.get(model_id)
    if model_type is None:
        raise ValueError(f"unknown OCCID model id {model_id}")

    model_name = str(payload[OCCID_MODEL_KEY])
    if model_name != model_type.__name__:
        raise ValueError(
            f"OCCID model name/id mismatch name={model_name} id={model_id} "
            f"expected={model_type.__name__}"
        )

    if issubclass(model_type, occid.OCCIDValue):
        return model_type, {"value": _bus_to_wire(payload.get("value"))}

    fields = {
        key: _bus_to_wire(value)
        for key, value in payload.items()
        if key not in OCCID_META_KEYS
    }
    return model_type, fields


def _ordinal_fields(model_type: type, fields: dict[str, Any]) -> dict[int, Any]:
    """Map named bus fields onto the compiled numeric wire ordinals."""
    names = tuple(model_type.model_fields)
    return {
        names.index(name): value
        for name, value in fields.items()
        if name in names
    }


def _bus_to_wire(value: Any) -> Any:
    if type(value) == dict:
        if set(value) == {OCCID_BYTES_KEY}:
            return base64.b64decode(value[OCCID_BYTES_KEY], validate=True)
        if OCCID_META_KEYS.issubset(value):
            model_type, fields = _bus_model_to_wire(value)
            model_id = occid.OCCID_MODEL_ID_BY_CLASS[model_type]
            if issubclass(model_type, occid.OCCIDValue):
                return [model_id, fields["value"]]
            return [model_id, _ordinal_fields(model_type, fields)]
        return {key: _bus_to_wire(item) for key, item in value.items()}
    if type(value) == list:
        return [_bus_to_wire(item) for item in value]
    return value


def unpack_occid(payload: Any) -> Any:
    """Validate and reconstruct an OCCID model from its readable bus form."""
    if type(payload) is not dict:
        raise ValueError(f"invalid OCCID bus payload type={type(payload).__name__}")
    model_type, fields = _bus_model_to_wire(payload)
    if issubclass(model_type, occid.OCCIDValue):
        return model_type._from_wire_value(fields["value"])
    return model_type._from_wire_fields(_ordinal_fields(model_type, fields))


def get_occid_state(state: dict[str, Any], key: str, expected_type: type | tuple[type, ...] | None = None) -> Any:
    payload = state.get(key)
    if payload is None:
        return None
    model = unpack_occid(payload)
    if expected_type is not None and not isinstance(model, expected_type):
        raise TypeError(
            f"unexpected OCCID state key={key} expected={expected_type} actual={type(model).__name__}"
        )
    return model


def _next_request_id(router: Any) -> str:
    router.request_counter += 1
    return f"req-{router.request_counter}"


def send_occid_request(router: Any, request_topic: str, model: Any) -> str:
    """Send any OCCID model to a correlated plugin request endpoint."""
    if not is_occid_model(model):
        raise TypeError(f"expected OCCID model, got {type(model).__name__}")
    request_id = _next_request_id(router)
    payload = {"request_id": request_id, "model": pack_occid(model)}
    from lib.common import build_envelope

    router.publish(request_topic, build_envelope(router.client_id, request_topic, payload))
    return request_id


def decode_occid_request(request: dict[str, Any]) -> tuple[str, Any]:
    request_id = str(request["request_id"])
    return request_id, unpack_occid(request["model"])


def send_occid_command(router: Any, request_topic: str, command: Any) -> str:
    if not occid.is_a(command, occid.Command):
        raise TypeError(f"expected OCCID Command, got {type(command).__name__}")
    request_id = _next_request_id(router)
    payload = {"request_id": request_id, "command": pack_occid(command)}
    from lib.common import build_envelope

    router.publish(request_topic, build_envelope(router.client_id, request_topic, payload))
    return request_id


def decode_occid_command(request: dict[str, Any]) -> tuple[str, Any]:
    request_id = str(request["request_id"])
    command = unpack_occid(request["command"])
    if not occid.is_a(command, occid.Command):
        raise TypeError(f"request payload is not OCCID Command actual={type(command).__name__}")
    return request_id, command


def send_occid_input(router: Any, input_topic: str, input_model: Any) -> None:
    """Publish one latest-value OCCID Input sample without request/response correlation."""
    if not occid.is_a(input_model, occid.Input):
        raise TypeError(f"expected OCCID Input, got {type(input_model).__name__}")
    from lib.common import build_envelope

    router.publish(input_topic, build_envelope(router.client_id, input_topic, pack_occid(input_model)))


def decode_occid_input(payload: Any) -> Any:
    model = unpack_occid(payload)
    if not occid.is_a(model, occid.Input):
        raise TypeError(f"input payload is not OCCID Input actual={type(model).__name__}")
    return model
