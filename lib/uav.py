"""Shared UAV helpers."""


def scale_float_pwm(value: float, pwm_low: int, pwm_high: int) -> int:
    if value < -1.0 or value > 1.0:
        raise ValueError("value must be between -1.0 and 1.0")
    scaled_value = pwm_low + ((value + 1.0) / 2.0) * (pwm_high - pwm_low)
    return int(scaled_value)


def scale_pwm_float(value: float, pwm_low: int, pwm_high: int) -> float:
    if value < pwm_low or value > pwm_high:
        raise ValueError("value must be within PWM bounds")
    normalized = 2.0 * ((value - pwm_low) / (pwm_high - pwm_low)) - 1.0
    return round(normalized, 3)


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
