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
