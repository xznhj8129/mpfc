"""Deterministic MAVSDK <-> MAVLink-shaped record conversions (MPFC-local).

This module does not connect to MAVSDK and does not choose which MAVSDK action
or offboard operation to call.  Callers select the operation; these helpers only
convert the selected operation's typed values.
"""
from __future__ import annotations

from dataclasses import dataclass

from pymavlink import mavutil

from lib.interop_common import degrees_to_radians, radians_to_degrees, require_finite
from lib.mavlink_models import (
    AngularVelocity,
    Attitude,
    AttitudeTarget,
    GlobalPositionInt,
    RepositionCommand,
)


@dataclass(frozen=True)
class MavsdkAttitudeFields:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    thrust_value: float


@dataclass(frozen=True)
class MavsdkPositionFields:
    latitude_deg: float
    longitude_deg: float
    absolute_altitude_m: float
    relative_altitude_m: float


@dataclass(frozen=True)
class MavsdkGotoFields:
    latitude_deg: float
    longitude_deg: float
    absolute_altitude_m: float
    yaw_deg: float


def standard_mode_from_native_name(native_name: str) -> str:
    name = str(native_name).upper()
    mapping = {
        "HOLD": "POSITION_HOLD",
        "POSCTL": "POSITION_HOLD",
        "POSHOLD": "POSITION_HOLD",
        "LOITER": "POSITION_HOLD",
        "ORBIT": "ORBIT",
        "CRUISE": "CRUISE",
        "ALTCTL": "ALTITUDE_HOLD",
        "ALT_HOLD": "ALTITUDE_HOLD",
        "RETURN_TO_LAUNCH": "SAFE_RECOVERY",
        "RTL": "SAFE_RECOVERY",
        "RTH": "SAFE_RECOVERY",
        "MISSION": "MISSION",
        "AUTO_MISSION": "MISSION",
        "LAND": "LAND",
        "LANDING": "LAND",
        "TAKEOFF": "TAKEOFF",
        "OFFBOARD": "EXTERNAL_CONTROL",
        "GUIDED": "EXTERNAL_CONTROL",
    }
    return mapping.get(name, "NON_STANDARD")


def gnss_fix_type_from_native_value(native_value: int) -> int:
    mapping = {
        0: mavutil.mavlink.GPS_FIX_TYPE_NO_GPS,
        1: mavutil.mavlink.GPS_FIX_TYPE_NO_FIX,
        2: mavutil.mavlink.GPS_FIX_TYPE_2D_FIX,
        3: mavutil.mavlink.GPS_FIX_TYPE_3D_FIX,
        4: mavutil.mavlink.GPS_FIX_TYPE_DGPS,
        5: mavutil.mavlink.GPS_FIX_TYPE_RTK_FLOAT,
        6: mavutil.mavlink.GPS_FIX_TYPE_RTK_FIXED,
    }
    return mapping.get(int(native_value), mavutil.mavlink.GPS_FIX_TYPE_NO_GPS)


def attitude_from_euler_degrees(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Attitude:
    return Attitude(
        roll_rad=degrees_to_radians(roll_deg, "roll_deg"),
        pitch_rad=degrees_to_radians(pitch_deg, "pitch_deg"),
        yaw_rad=degrees_to_radians(yaw_deg, "yaw_deg"),
        rollspeed_rad_s=0.0,
        pitchspeed_rad_s=0.0,
        yawspeed_rad_s=0.0,
    )


def angular_velocity_from_body_rates(
    roll_rad_s: float,
    pitch_rad_s: float,
    yaw_rad_s: float,
) -> AngularVelocity:
    return AngularVelocity(
        x_rad_s=require_finite(roll_rad_s, "roll_rad_s"),
        y_rad_s=require_finite(pitch_rad_s, "pitch_rad_s"),
        z_rad_s=require_finite(yaw_rad_s, "yaw_rad_s"),
    )


def position_to_location_record(fields: MavsdkPositionFields) -> GlobalPositionInt:
    return GlobalPositionInt(
        latitude_deg=require_finite(fields.latitude_deg, "latitude_deg"),
        longitude_deg=require_finite(fields.longitude_deg, "longitude_deg"),
        altitude_m=require_finite(fields.absolute_altitude_m, "absolute_altitude_m"),
        relative_altitude_m=require_finite(
            fields.relative_altitude_m, "relative_altitude_m"
        ),
        vx_m_s=0.0,
        vy_m_s=0.0,
        vz_m_s=0.0,
        heading_deg=0.0,
    )


def goto_command_to_fields(
    command: RepositionCommand,
    *,
    current_absolute_altitude_m: float | None = None,
    current_relative_altitude_m: float | None = None,
    current_yaw_rad: float | None = None,
) -> MavsdkGotoFields:
    """Convert a reposition command to MAVSDK goto_location fields.

    MAVSDK's ``goto_location`` accepts absolute sea-level altitude.  Relative
    targets are converted with the caller's current absolute and relative
    altitude samples; the caller still owns connection state, retries, and
    execution policy.
    """
    latitude_deg = require_finite(command.latitude_deg, "latitude_deg")
    longitude_deg = require_finite(command.longitude_deg, "longitude_deg")
    target_altitude_m = require_finite(command.altitude_m, "altitude_m")

    reference = str(command.altitude_reference)
    if reference in ("asl", "amsl"):
        absolute_altitude_m = target_altitude_m
    elif reference == "relative":
        if current_absolute_altitude_m is None or current_relative_altitude_m is None:
            raise ValueError(
                "relative MAVSDK goto requires current absolute and relative altitude"
            )
        current_absolute = require_finite(
            current_absolute_altitude_m, "current_absolute_altitude_m"
        )
        current_relative = require_finite(
            current_relative_altitude_m, "current_relative_altitude_m"
        )
        absolute_altitude_m = current_absolute + (target_altitude_m - current_relative)
    else:
        raise ValueError(f"unsupported MAVSDK goto altitude reference {reference}")

    if command.yaw_rad is not None:
        yaw_rad = command.yaw_rad
    elif current_yaw_rad is not None:
        yaw_rad = current_yaw_rad
    else:
        yaw_rad = 0.0

    return MavsdkGotoFields(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        absolute_altitude_m=require_finite(absolute_altitude_m, "absolute_altitude_m"),
        yaw_deg=radians_to_degrees(yaw_rad, "yaw_rad"),
    )


def attitude_setpoint_to_fields(setpoint: AttitudeTarget) -> MavsdkAttitudeFields:
    thrust = require_finite(setpoint.thrust, "thrust")
    if thrust < 0.0 or thrust > 1.0:
        raise ValueError(f"thrust {thrust} outside [0, 1]")
    return MavsdkAttitudeFields(
        roll_deg=radians_to_degrees(setpoint.roll_rad, "roll_rad"),
        pitch_deg=radians_to_degrees(setpoint.pitch_rad, "pitch_rad"),
        yaw_deg=radians_to_degrees(setpoint.yaw_rad, "yaw_rad"),
        thrust_value=thrust,
    )
