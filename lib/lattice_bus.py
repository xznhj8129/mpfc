"""Node-local bus transport for Lattice documents and MAVLink records.

MPFC's private MQTT bus stays JSON-readable.  Lattice entity/task documents
are carried as aliased JSON (ENG-1) using the official ``anduril`` SDK types;
vehicle commands and direct-control samples are carried as MAVLink-shaped
records from :mod:`lib.mavlink_models`.  Plugin RPC stays node-local and is
unchanged apart from the payload types.
"""
from __future__ import annotations

from typing import Any, Type, TypeVar

from anduril.core import UniversalBaseModel

from lib.mavlink_models import MavlinkRecord, pack_record, unpack_record

T = TypeVar("T", bound=UniversalBaseModel)


def is_lattice_model(value: Any) -> bool:
    """True for official anduril-lattice-sdk document types."""
    return isinstance(value, UniversalBaseModel)


def pack_lattice(model: UniversalBaseModel) -> dict[str, Any]:
    """Render a Lattice document as inspectable JSON-compatible bus data."""
    if not is_lattice_model(model):
        raise TypeError(f"expected Lattice document, got {type(model).__name__}")
    return model.model_dump(mode="json", by_alias=True, exclude_none=True)


def unpack_lattice(payload: Any, expected_type: Type[T]) -> T:
    """Validate a bus document back into its Lattice SDK type."""
    if type(payload) is not dict:
        raise ValueError(f"invalid Lattice bus payload type={type(payload).__name__}")
    try:
        document = expected_type.model_validate(payload)
    except Exception as exc:
        raise ValueError(
            f"invalid Lattice {expected_type.__name__} document: {exc}"
        ) from exc
    return document


def entity_document(entity: UniversalBaseModel) -> dict[str, Any]:
    """Canonical aliased JSON form of a Lattice Entity."""
    return pack_lattice(entity)


def task_document(task: UniversalBaseModel) -> dict[str, Any]:
    """Canonical aliased JSON form of a Lattice Task."""
    return pack_lattice(task)


def get_lattice_state(
    state: dict[str, Any],
    key: str,
    expected_type: Type[T],
) -> T | None:
    payload = state.get(key)
    if payload is None:
        return None
    return unpack_lattice(payload, expected_type)


def get_record_state(
    state: dict[str, Any],
    key: str,
    expected_type: Type[T] | None = None,
) -> MavlinkRecord | None:
    payload = state.get(key)
    if payload is None:
        return None
    return unpack_record(payload, expected_type)


def _next_request_id(router: Any) -> str:
    router.request_counter += 1
    return f"req-{router.request_counter}"


def send_request(router: Any, request_topic: str, model: UniversalBaseModel) -> str:
    """Send any Lattice document to a correlated plugin request endpoint."""
    request_id = _next_request_id(router)
    payload = {"request_id": request_id, "model": pack_lattice(model)}
    from lib.common import build_envelope

    router.publish(request_topic, build_envelope(router.client_id, request_topic, payload))
    return request_id


def decode_request(request: dict[str, Any], expected_type: Type[T]) -> tuple[str, T]:
    request_id = str(request["request_id"])
    return request_id, unpack_lattice(request["model"], expected_type)


def send_json_request(router: Any, request_topic: str, data: dict[str, Any]) -> str:
    """Send a plain local JSON payload to a correlated request endpoint."""
    request_id = _next_request_id(router)
    payload = {"request_id": request_id, "payload": data}
    from lib.common import build_envelope

    router.publish(request_topic, build_envelope(router.client_id, request_topic, payload))
    return request_id


def decode_json_request(request: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    request_id = str(request["request_id"])
    payload = request.get("payload")
    if type(payload) is not dict:
        raise ValueError("request payload must be an object")
    return request_id, payload


def send_command(router: Any, request_topic: str, command: MavlinkRecord) -> str:
    """Send one MAVLink-shaped command to a correlated endpoint."""
    request_id = _next_request_id(router)
    payload = {"request_id": request_id, "command": pack_record(command)}
    from lib.common import build_envelope

    router.publish(request_topic, build_envelope(router.client_id, request_topic, payload))
    return request_id


def decode_command(request: dict[str, Any]) -> tuple[str, MavlinkRecord]:
    request_id = str(request["request_id"])
    return request_id, unpack_record(request["command"])


def send_input(router: Any, input_topic: str, sample: MavlinkRecord) -> None:
    """Publish one latest-value control sample without request/response correlation."""
    from lib.common import build_envelope

    router.publish(input_topic, build_envelope(router.client_id, input_topic, pack_record(sample)))


def decode_input(payload: Any) -> MavlinkRecord:
    return unpack_record(payload)
