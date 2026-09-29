"""Endpoint command semantics that stay local to MPFC.

Command identity comes from MAVLink (``MAV_CMD_*`` in
:mod:`lib.mavlink_models`, sourced from pymavlink).  This module holds the
small routing vocabulary MPFC still needs on top of MAVLink: direct-control
process names, the takeoff-altitude parameter id, and the standard flight-mode
words the adapters translate into vehicle-native modes.  None of it is a
shared model; it is endpoint mechanics (LOCAL).
"""
from __future__ import annotations

DIRECT_CONTROL_ATTITUDE = "ATTITUDE_THRUST"
DIRECT_CONTROL_MANUAL = "MANUAL_AXIS"

# Standard autopilot parameter for takeoff altitude (ArduPilot/PX4 name).
PARAM_TAKEOFF_ALTITUDE = "MIS_TAKEOFF_ALT"

STANDARD_MODES = (
    "POSITION_HOLD",
    "MISSION",
    "ALTITUDE_HOLD",
    "CRUISE",
    "SAFE_RECOVERY",
    "LAND",
    "TAKEOFF",
    "ORBIT",
    "EXTERNAL_CONTROL",
    "NON_STANDARD",
)


def standard_mode_name(value: object) -> str:
    name = str(getattr(value, "name", value)).strip().upper()
    if name not in STANDARD_MODES:
        raise ValueError(f"unknown standard flight mode {value!r}")
    return name
