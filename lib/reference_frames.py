#!/usr/bin/env python3
"""
Reference-frame conversion helpers.
Usage:
    from lib.reference_frames import fru_to_frd_vector, up_to_ned_down
"""

import math
from typing import Any, Dict

FRAME_FRD = "FRD"
SQRT_HALF = math.sqrt(0.5)


def up_to_ned_down(value: float | None) -> float | None:
    if value is None:
        return None
    return -value


def ned_down_to_up(value: float | None) -> float | None:
    if value is None:
        return None
    return -value


def fru_to_frd_vector(x: float, y: float, z_up: float) -> tuple[float, float, float]:
    return x, y, -z_up


def rc_dict_to_aetr(values: Dict[str, Any]) -> list[Any]:
    return [
        values.get("roll"),
        values.get("pitch"),
        values.get("throttle"),
        values.get("yaw"),
    ]


def rc_rpyt_to_aetr(roll: float, pitch: float, yaw: float, throttle: float) -> list[float]:
    return [roll, pitch, throttle, yaw]


def _quaternion_product(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    """Hamilton product ``a * b`` (rotation ``a`` applied after rotation ``b``)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def attitude_enu_quaternion(
    roll_rad: float, pitch_rad: float, yaw_rad: float
) -> tuple[float, float, float, float]:
    """Convert FRD/NED euler attitude to the Lattice body-FLU -> ENU quaternion.

    MAVLink euler angles build ``q(FRD->NED) = q_z(yaw) * q_y(pitch) * q_x(roll)``.
    Lattice ``attitude_enu`` is the body FLU frame expressed in ENU, so compose
    ``q(NED->ENU) * q(FRD->NED) * q(FLU->FRD)``:

    * ``q(FLU->FRD)`` is 180 deg about body x (forward),
    * ``q(NED->ENU)`` is 180 deg about the horizontal axis bisecting north/east.

    Returns ``(w, x, y, z)``.
    """
    roll = float(roll_rad)
    pitch = float(pitch_rad)
    yaw = float(yaw_rad)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    ned_frd = (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    enu_ned = (0.0, SQRT_HALF, SQRT_HALF, 0.0)
    frd_flu = (0.0, 1.0, 0.0, 0.0)
    return _quaternion_product(_quaternion_product(enu_ned, ned_frd), frd_flu)
