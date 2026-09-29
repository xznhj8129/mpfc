"""Shared UAV compatibility helpers.

Signed normalized-control/PWM conversion is pure MPFC-local math (see
:mod:`lib.interop_common`).  These wrappers preserve the existing helper names
for current callers.
"""

from lib.interop_common import normalized_to_pwm, pwm_to_normalized


def scale_float_pwm(value: float, pwm_low: int, pwm_high: int) -> int:
    return normalized_to_pwm(value, pwm_low, pwm_high)


def scale_pwm_float(value: float, pwm_low: int, pwm_high: int) -> float:
    return pwm_to_normalized(value, pwm_low, pwm_high)


def scale_aux_pwm_float(value: float, pwm_mid: int, pwm_low: int, pwm_high: int) -> int:
    if value < pwm_low or value > pwm_high:
        raise ValueError("value must be within PWM bounds")
    return 0 if value < pwm_mid else 1


def build_control_fields(roll: float, pitch: float, yaw: float, throttle: float, aux: list[float] | None = None) -> dict:
    fields = {
        "Roll": roll,
        "Pitch": pitch,
        "Yaw": yaw,
        "Throttle": throttle,
    }
    if aux:
        fields["Aux"] = aux
    return fields


def merge_control_fields(base: dict, override: dict) -> dict:
    merged = dict(base)
    for key in ("Roll", "Pitch", "Yaw", "Throttle"):
        if key in override:
            merged[key] = override[key]
    if "Aux" in override:
        base_aux = list(base.get("Aux") or [])
        override_aux = list(override["Aux"] or [])
        if len(base_aux) < len(override_aux):
            base_aux.extend([0.0] * (len(override_aux) - len(base_aux)))
        for index, value in enumerate(override_aux):
            if value is None:
                continue
            base_aux[index] = value
        if base_aux:
            merged["Aux"] = base_aux
    return merged
