"""Deterministic MSP/INAV -> MAVLink-shaped record conversions (MPFC-local).

  The caller owns MSP
transport, polling, mode activation, waypoint operations, arming sequences, and
recovery; these helpers only normalize protocol-native values into the records
used on MPFC's private bus.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

from pymavlink import mavutil

from lib.interop_common import degrees_to_radians, fru_to_frd_vector, pwm_to_normalized, require_finite
from lib.mavlink_models import (
    AngularVelocity,
    Attitude,
    ControlAxes,
    GlobalPositionInt,
    GpsRawInt,
)


@dataclass(frozen=True)
class InavGpsFields:
    latitude_deg: float
    longitude_deg: float
    absolute_altitude_m: float | None
    relative_altitude_m: float | None
    fix_name: str
    fix_code: int
    satellites_used: int
    ground_speed_m_s: float | None = None
    ground_course_deg: float | None = None
    hdop: float | None = None


def standard_mode_from_native_names(native_names: Sequence[str]) -> str:
    names = {str(name).upper().replace("_", " ") for name in native_names}
    if any(name in names for name in {"NAV POSHOLD", "POSHOLD", "LOITER"}):
        return "POSITION_HOLD"
    if any(name in names for name in {"RTH", "NAV RTH"}):
        return "SAFE_RECOVERY"
    if any(name in names for name in {"NAV WP", "MISSION"}):
        return "MISSION"
    if any(name in names for name in {"NAV LAND", "LAND"}):
        return "LAND"
    if any(name in names for name in {"NAV CRUISE", "CRUISE"}):
        return "CRUISE"
    if any(name in names for name in {"ALT HOLD", "ALTHOLD"}):
        return "ALTITUDE_HOLD"
    return "NON_STANDARD"


def gnss_fix_type_from_native_name(native_name: str) -> int:
    name = str(native_name).upper()
    mavlink = mavutil.mavlink
    if "RTK_FIXED" in name:
        return mavlink.GPS_FIX_TYPE_RTK_FIXED
    if "RTK_FLOAT" in name:
        return mavlink.GPS_FIX_TYPE_RTK_FLOAT
    if "DGPS" in name:
        return mavlink.GPS_FIX_TYPE_DGPS
    if "3D" in name:
        return mavlink.GPS_FIX_TYPE_3D_FIX
    if "2D" in name:
        return mavlink.GPS_FIX_TYPE_2D_FIX
    if "NO_FIX" in name or "NONE" in name:
        return mavlink.GPS_FIX_TYPE_NO_FIX
    return mavlink.GPS_FIX_TYPE_NO_GPS


def attitude_from_degrees(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Attitude:
    return Attitude(
        roll_rad=degrees_to_radians(roll_deg, "roll_deg"),
        pitch_rad=degrees_to_radians(pitch_deg, "pitch_deg"),
        yaw_rad=degrees_to_radians(yaw_deg, "yaw_deg"),
        rollspeed_rad_s=0.0,
        pitchspeed_rad_s=0.0,
        yawspeed_rad_s=0.0,
    )


def angular_velocity_from_fru_degrees_s(
    x_deg_s: float,
    y_deg_s: float,
    z_deg_s: float,
) -> AngularVelocity:
    x, y, z = fru_to_frd_vector(
        degrees_to_radians(x_deg_s, "x_deg_s"),
        degrees_to_radians(y_deg_s, "y_deg_s"),
        degrees_to_radians(z_deg_s, "z_deg_s"),
    )
    return AngularVelocity(x_rad_s=x, y_rad_s=y, z_rad_s=z)


def gps_records(fields: InavGpsFields) -> tuple[GlobalPositionInt, GpsRawInt]:
    absolute_altitude = (
        None
        if fields.absolute_altitude_m is None
        else require_finite(fields.absolute_altitude_m, "absolute_altitude_m")
    )
    relative_altitude = (
        None
        if fields.relative_altitude_m is None
        else require_finite(fields.relative_altitude_m, "relative_altitude_m")
    )
    latitude = require_finite(fields.latitude_deg, "latitude_deg")
    longitude = require_finite(fields.longitude_deg, "longitude_deg")
    speed = (
        None
        if fields.ground_speed_m_s is None
        else require_finite(fields.ground_speed_m_s, "ground_speed_m_s")
    )
    course = (
        None
        if fields.ground_course_deg is None
        else require_finite(fields.ground_course_deg, "ground_course_deg")
    )
    vx = vy = 0.0
    if speed is not None and course is not None:
        rad = degrees_to_radians(course, "ground_course_deg")
        vx = speed * math.sin(rad)
        vy = speed * math.cos(rad)
    location = GlobalPositionInt(
        latitude_deg=latitude,
        longitude_deg=longitude,
        altitude_m=0.0 if absolute_altitude is None else absolute_altitude,
        relative_altitude_m=0.0 if relative_altitude is None else relative_altitude,
        vx_m_s=vx,
        vy_m_s=vy,
        vz_m_s=0.0,
        heading_deg=0.0 if course is None else course,
    )
    gnss = GpsRawInt(
        fix_type=gnss_fix_type_from_native_name(fields.fix_name),
        satellites_visible=int(fields.satellites_used),
        latitude_deg=latitude,
        longitude_deg=longitude,
        altitude_m=0.0 if absolute_altitude is None else absolute_altitude,
        eph=None if fields.hdop is None else require_finite(fields.hdop, "hdop"),
        epv=None,
        velocity_m_s=speed,
        cog_deg=course,
    )
    return location, gnss


def rc_pwm_mapping_to_control_axes(
    channels: Mapping[str, float],
    *,
    roll_channel: str,
    pitch_channel: str,
    yaw_channel: str,
    throttle_channel: str,
    aux_channels: Sequence[str] = (),
    pwm_min_us: float,
    pwm_max_us: float,
) -> ControlAxes:
    return ControlAxes(
        roll=pwm_to_normalized(channels[roll_channel], pwm_min_us, pwm_max_us),
        pitch=pwm_to_normalized(channels[pitch_channel], pwm_min_us, pwm_max_us),
        yaw=pwm_to_normalized(channels[yaw_channel], pwm_min_us, pwm_max_us),
        throttle=pwm_to_normalized(channels[throttle_channel], pwm_min_us, pwm_max_us),
        aux=tuple(
            pwm_to_normalized(channels[channel], pwm_min_us, pwm_max_us)
            for channel in aux_channels
            if channel in channels
        ),
    )


def rc_sequence_to_control_axes(
    values: Sequence[float],
    *,
    pwm_min_us: float,
    pwm_max_us: float,
) -> ControlAxes:
    """Convert a raw PWM AETR sequence [roll, pitch, throttle, yaw, ...aux]."""
    if len(values) < 4:
        raise ValueError("RC sequence requires at least four AETR values")
    return ControlAxes(
        roll=pwm_to_normalized(values[0], pwm_min_us, pwm_max_us),
        pitch=pwm_to_normalized(values[1], pwm_min_us, pwm_max_us),
        yaw=pwm_to_normalized(values[3], pwm_min_us, pwm_max_us),
        throttle=pwm_to_normalized(values[2], pwm_min_us, pwm_max_us),
        aux=tuple(
            pwm_to_normalized(value, pwm_min_us, pwm_max_us) for value in values[4:]
        ),
    )
