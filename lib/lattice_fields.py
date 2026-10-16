"""Field access for native SDK models and protobuf Any contents."""
from typing import Any

def _field(node: Any, *names: str) -> Any:  # Read a field from SDK models, extra fields, or dicts.
    if node is None:
        return None
    for name in names:
        if isinstance(node, dict):
            value = node.get(name)
        else:
            value = getattr(node, name, None)
            if value is None and hasattr(node, "model_extra"):
                value = (node.model_extra or {}).get(name)
        if value is not None:
            return value
    return None


def _float_field(node: Any, *names: str) -> float | None:  # Read an optional float field.
    value = _field(node, *names)
    if value is None:
        return None
    return float(value)


