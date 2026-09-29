"""Deterministic representation conversion helpers (MPFC-local).

Local conversion helpers for MPFC's node bus: pure math only, no I/O,
no endpoint discovery, no sequencing.  These helpers feed the MAVLink-shaped
records used across MPFC's private bus.
"""
from __future__ import annotations

import math


def require_finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def degrees_to_radians(value: float, name: str = "angle_deg") -> float:
    return math.radians(require_finite(value, name))


def radians_to_degrees(value: float, name: str = "angle_rad") -> float:
    return math.degrees(require_finite(value, name))


def fru_to_frd_vector(x: float, y: float, z_up: float) -> tuple[float, float, float]:
    """Convert a body Forward-Right-Up vector to Forward-Right-Down."""
    return (
        require_finite(x, "x"),
        require_finite(y, "y"),
        -require_finite(z_up, "z_up"),
    )


def pwm_to_normalized(value_us: float, pwm_min_us: float, pwm_max_us: float) -> float:
    """Convert PWM bounds to the normalized control range [-1, 1]."""
    value = require_finite(value_us, "value_us")
    low = require_finite(pwm_min_us, "pwm_min_us")
    high = require_finite(pwm_max_us, "pwm_max_us")
    if high <= low:
        raise ValueError("pwm_max_us must be greater than pwm_min_us")
    if value < low or value > high:
        raise ValueError(f"PWM value {value} outside [{low}, {high}]")
    return 2.0 * ((value - low) / (high - low)) - 1.0


def normalized_to_pwm(value: float, pwm_min_us: float, pwm_max_us: float) -> int:
    """Convert normalized control input [-1, 1] to rounded PWM."""
    normalized = require_finite(value, "value")
    low = require_finite(pwm_min_us, "pwm_min_us")
    high = require_finite(pwm_max_us, "pwm_max_us")
    if high <= low:
        raise ValueError("pwm_max_us must be greater than pwm_min_us")
    if normalized < -1.0 or normalized > 1.0:
        raise ValueError(f"normalized value {normalized} outside [-1, 1]")
    return int(round(low + ((normalized + 1.0) / 2.0) * (high - low)))
