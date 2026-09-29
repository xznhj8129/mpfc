"""MAVLink-shaped local records for MPFC's private node bus.

MPFC is a UAV node: the vehicle side of the system speaks MAVLink/MAVSDK
(DEC-1).  This module defines the small set of records that travel between
MPFC's own components (flight cores, the UAV controller, the endpoint
adapters) and mirrors the MAVLink message vocabulary where a standard message
exists.  Fields use the MAVLink names and units (metres, radians, IntEnum
values from ``pymavlink``); protocol-local facts that MAVLink does not carry
(hold-time readiness policy, receiver configuration, endpoint diagnostics)
stay explicit local records and are marked LOCAL.

Records are plain dataclasses so the node bus stays readable JSON.  Use
``pack_record``/``unpack_record`` to move them across the bus.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields, is_dataclass
from typing import Any, ClassVar, Dict, Tuple, Type, TypeVar

from pymavlink import mavutil

# MAVLink constants re-exported for adapters and flight cores.  Names come
# straight from the official pymavlink dialect so nothing is hand-written.
MAV_CMD_COMPONENT_ARM_DISARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
MAV_CMD_NAV_TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
MAV_CMD_NAV_LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
MAV_CMD_NAV_RETURN_TO_LAUNCH = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
MAV_CMD_NAV_LOITER_UNLIM = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
MAV_CMD_DO_REPOSITION = mavutil.mavlink.MAV_CMD_DO_REPOSITION

MAV_MODE_FLAG_SAFETY_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
MAV_PARAM_TYPE_REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

VALID_ALTITUDE_REFERENCES = ("relative", "hae", "agl", "asl", "amsl")


@dataclass(frozen=True)
class MavlinkRecord:
    """Base for bus-carried records; ``MESSAGE_NAME`` is the wire tag."""

    MESSAGE_NAME: ClassVar[str] = ""

    def to_bus(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"_mavlink_message": type(self).__name__}
        for entry in fields(self):
            payload[entry.name] = _to_bus_value(getattr(self, entry.name))
        return payload


R = TypeVar("R", bound=MavlinkRecord)
_RECORD_TYPES: Dict[str, Type[MavlinkRecord]] = {}


def record_type(name: str) -> Type[MavlinkRecord]:
    registered = _RECORD_TYPES.get(str(name))
    if registered is None:
        raise ValueError(f"unknown MAVLink record type {name!r}")
    return registered


def pack_record(record: MavlinkRecord) -> Dict[str, Any]:
    if not isinstance(record, MavlinkRecord):
        raise TypeError(f"expected MavlinkRecord, got {type(record).__name__}")
    return record.to_bus()


def unpack_record(payload: Any, expected_type: Type[R] | None = None) -> MavlinkRecord:
    if type(payload) is not dict:
        raise ValueError(f"invalid MAVLink record payload type={type(payload).__name__}")
    name = payload.get("_mavlink_message")
    if name is None:
        raise ValueError("invalid MAVLink record payload: missing _mavlink_message")
    record_class = record_type(str(name))
    accepted = {entry.name for entry in fields(record_class)}
    kwargs = {
        key: _from_bus_value(value)
        for key, value in payload.items()
        if key != "_mavlink_message" and key in accepted
    }
    record = record_class(**kwargs)  # type: ignore[arg-type]
    if expected_type is not None and not isinstance(record, expected_type):
        raise TypeError(
            f"unexpected MAVLink record expected={expected_type.__name__} "
            f"actual={type(record).__name__}"
        )
    return record


def _to_bus_value(value: Any) -> Any:
    if is_dataclass(value) and isinstance(value, MavlinkRecord):
        return value.to_bus()
    if isinstance(value, tuple):
        return [_to_bus_value(item) for item in value]
    if isinstance(value, list):
        return [_to_bus_value(item) for item in value]
    if type(value) is dict:
        return {key: _to_bus_value(item) for key, item in value.items()}
    return value


def _from_bus_value(value: Any) -> Any:
    if type(value) is dict and "_mavlink_message" in value:
        return unpack_record(value)
    if type(value) is list:
        return tuple(_from_bus_value(item) for item in value)
    return value


# ---------------------------------------------------------------------------
# Commands (vehicle command families, DEC-1)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CommandLong(MavlinkRecord):
    """MAVLink COMMAND_LONG: one MAV_CMD with seven parameters."""

    command: int
    target_system: int = 0
    target_component: int = 0
    confirmation: int = 0
    params: Tuple[float, float, float, float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


@dataclass(frozen=True)
class RepositionCommand(MavlinkRecord):
    """MAV_CMD_DO_REPOSITION with the altitude datum kept explicit.

    MAVLink carries the destination altitude without a datum field; ENG-13
    requires a truthful datum, so the datum travels beside the MAVLink values
    and adapters resolve it against the vehicle home position.
    """

    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    altitude_reference: str = "relative"
    yaw_rad: float | None = None
    ground_speed_m_s: float = 0.0


@dataclass(frozen=True)
class SetMode(MavlinkRecord):
    """MAVLink SET_MODE (base_mode + custom_mode).

    ``mode_name`` is local metadata: the standard-mode word an adapter
    resolves into the vehicle-native custom mode (MAVLink has no mode names).
    """

    base_mode: int
    custom_mode: int
    mode_name: str = ""


@dataclass(frozen=True)
class ParamSet(MavlinkRecord):
    """MAVLink PARAM_SET for autopilot parameters (e.g. MIS_TAKEOFF_ALT)."""

    param_id: str
    value: float
    param_type: int = MAV_PARAM_TYPE_REAL32


@dataclass(frozen=True)
class DirectControl(MavlinkRecord):
    """LOCAL direct-control process acquisition/release.

    MAVLink has no process handle; MAVSDK offboard/manual modes are started and
    stopped explicitly, so the adapter-local process state travels as this
    local record.
    """

    enabled: bool
    mode: str


@dataclass(frozen=True)
class AttitudeTarget(MavlinkRecord):
    """MAVLink SET_ATTITUDE_TARGET (body attitude + normalized thrust)."""

    type_mask: int
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    thrust: float


@dataclass(frozen=True)
class ManualControl(MavlinkRecord):
    """MAVLink MANUAL_CONTROL operator override axes [-1000, 1000] + buttons."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    r: float = 0.0
    buttons: int = 0


# ---------------------------------------------------------------------------
# Telemetry (MAVLink message vocabulary)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Heartbeat(MavlinkRecord):
    """MAVLink HEARTBEAT-derived vehicle state.

    ``mode_name``/``standard_mode`` are local vocabulary: the native mode word
    and its MPFC standard-mode mapping (MAVLink carries only custom_mode).
    """

    type: int
    autopilot: int
    base_mode: int
    custom_mode: int
    system_status: int
    mavlink_version: int = 3
    mode_name: str = ""
    standard_mode: str = "NON_STANDARD"

    @property
    def armed(self) -> bool:
        return bool(self.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)

    @property
    def custom_mode_enabled(self) -> bool:
        return bool(self.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)


@dataclass(frozen=True)
class GlobalPositionInt(MavlinkRecord):
    """MAVLink GLOBAL_POSITION_INT in engineering units."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    relative_altitude_m: float
    vx_m_s: float
    vy_m_s: float
    vz_m_s: float
    heading_deg: float


@dataclass(frozen=True)
class Attitude(MavlinkRecord):
    """MAVLink ATTITUDE."""

    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    rollspeed_rad_s: float
    pitchspeed_rad_s: float
    yawspeed_rad_s: float


@dataclass(frozen=True)
class AngularVelocity(MavlinkRecord):
    """Body angular velocity in FRD frame (rad/s)."""

    x_rad_s: float
    y_rad_s: float
    z_rad_s: float


@dataclass(frozen=True)
class Altitude(MavlinkRecord):
    """MAVLink ALTITUDE (altitude-only sources without a global position)."""

    altitude_monotonic: float | None = None
    altitude_amsl: float | None = None
    altitude_local: float | None = None
    altitude_relative: float | None = None
    altitude_terrain: float | None = None
    bottom_clearance: float | None = None


@dataclass(frozen=True)
class VehicleControl(MavlinkRecord):
    """Local aggregate: MAVLink HEARTBEAT plus MPFC readiness/validity policy."""

    heartbeat: Heartbeat
    readiness: Readiness
    navigation_validity: NavigationValidity


@dataclass(frozen=True)
class GpsRawInt(MavlinkRecord):
    """MAVLink GPS_RAW_INT."""
    fix_type: int
    satellites_visible: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    eph: float | None = None
    epv: float | None = None
    velocity_m_s: float | None = None
    cog_deg: float | None = None


@dataclass(frozen=True)
class BatteryStatus(MavlinkRecord):
    """MAVLink BATTERY_STATUS (first battery)."""

    voltage_v: float | None = None
    current_a: float | None = None
    remaining_pct: float | None = None
    temperature_degc: float | None = None
    consumed_mah: float | None = None
    consumed_wh: float | None = None


@dataclass(frozen=True)
class RcChannels(MavlinkRecord):
    """MAVLink RC_CHANNELS raw PWM values."""

    channel_count: int
    channels: Tuple[float, ...] = ()
    rssi: int | None = None


@dataclass(frozen=True)
class HighresImu(MavlinkRecord):
    """MAVLink HIGHRES_IMU (FRD body axes, m/s^2 and mG)."""

    xacc: float | None = None
    yacc: float | None = None
    zacc: float | None = None
    xgyro: float | None = None
    ygyro: float | None = None
    zgyro: float | None = None
    xmag: float | None = None
    ymag: float | None = None
    zmag: float | None = None
    abs_pressure: float | None = None
    diff_pressure: float | None = None
    pressure_alt: float | None = None
    temperature_degc: float | None = None


@dataclass(frozen=True)
class VfrHud(MavlinkRecord):
    """MAVLink VFR_HUD."""

    airspeed_m_s: float
    groundspeed_m_s: float
    heading_deg: float
    throttle_pct: float
    alt_m: float
    climb_m_s: float


@dataclass(frozen=True)
class NavControllerOutput(MavlinkRecord):
    """MAVLink NAV_CONTROLLER_OUTPUT."""

    nav_roll_deg: float
    nav_pitch_deg: float
    nav_bearing_deg: float
    target_bearing_deg: float
    wp_dist_m: float
    alt_error_m: float
    aspd_error_m_s: float
    xtrack_error_m: float


@dataclass(frozen=True)
class AutopilotVersion(MavlinkRecord):
    """MAVLink AUTOPILOT_VERSION diagnostic identity."""

    flight_sw_version: int = 0
    middleware_sw_version: int = 0
    os_sw_version: int = 0
    board_version: int = 0
    vendor_id: int = 0
    product_id: int = 0
    capabilities: int = 0
    uid: int = 0
    flight_sw_version_text: str = ""


@dataclass(frozen=True)
class MissionState(MavlinkRecord):
    """LOCAL view of the onboard autopilot mission (MISSION_CURRENT/COUNT)."""

    current_waypoint: int
    waypoint_count: int
    valid: bool = True


@dataclass(frozen=True)
class FirmwareVersion(MavlinkRecord):
    """LOCAL diagnostic identity/version for the endpoint firmware."""

    firmware: str
    version: str
    board: str = ""


@dataclass(frozen=True)
class RcChannelMapEntry(MavlinkRecord):
    """LOCAL RC receiver channel mapping entry (MSP/INAV configuration)."""

    axis: str
    source_channel: int
    output_channel: int | None = None
    label: str = ""


@dataclass(frozen=True)
class RcModeRange(MavlinkRecord):
    """LOCAL RC mode activation range."""

    mode_id: int | None
    mode_name: str
    channel: int
    pwm_start: float
    pwm_end: float


@dataclass(frozen=True)
class ReceiverConfig(MavlinkRecord):
    """LOCAL RC receiver configuration."""

    rx_min_usec: int = 1000
    rx_max_usec: int = 2000
    rx_center_usec: int = 1500


@dataclass(frozen=True)
class ControlChannelOverride(MavlinkRecord):
    """LOCAL per-aux-channel manual override sample."""

    channel_index: int
    value: float | None = None


@dataclass(frozen=True)
class SensorConfig(MavlinkRecord):
    """LOCAL flight sensor configuration reported by the endpoint."""

    accelerometer: str = ""
    barometer: str = ""
    magnetometer: str = ""
    airspeed: str = ""
    rangefinder: str = ""
    optical_flow: str = ""


@dataclass(frozen=True)
class RuntimeLoad(MavlinkRecord):
    """LOCAL endpoint runtime diagnostics (CPU/loop load)."""

    cycle_time_us: float | None = None
    cpu_load_pct: float | None = None


@dataclass(frozen=True)
class Readiness(MavlinkRecord):
    """LOCAL arm/takeoff readiness policy result.

    ``armable`` and ``ekf_using_gps`` are the raw adapter facts (MAVLink health
    checks); ``arm_ready``/``takeoff_ready`` are computed by uav_controller with
    its stabilization hold times.
    """

    armable: bool = False
    ekf_using_gps: bool = False
    arm_ready: bool = False
    takeoff_ready: bool = False
    problems: Tuple[str, ...] = ()


@dataclass(frozen=True)
class ControlAxes(MavlinkRecord):
    """LOCAL normalized control axes [-1, 1] used for RC/override merging."""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0
    aux: Tuple[float, ...] = ()


@dataclass(frozen=True)
class ControlOverride(MavlinkRecord):
    """LOCAL manual override sample (MAVLink MANUAL_CONTROL semantics).

    Axes are normalized [-1, 1]; ``aux`` carries per-channel overrides for
    endpoints that accept raw auxiliary channels.
    """

    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None
    throttle: float | None = None
    aux: Tuple[ControlChannelOverride, ...] = ()


@dataclass(frozen=True)
class RemoteControlState(MavlinkRecord):
    """LOCAL aggregate of RC telemetry, control output, and receiver config."""

    rc_telemetry: RcChannels | None
    control_output: ControlAxes
    control_override: ControlOverride | None
    receiver_config: ReceiverConfig
    channel_map: Tuple[RcChannelMapEntry, ...]
    mode_ranges: Tuple[RcModeRange, ...]


@dataclass(frozen=True)
class HomePosition(MavlinkRecord):
    """MAVLink HOME_POSITION."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float


@dataclass(frozen=True)
class NavigationValidity(MavlinkRecord):
    """LOCAL navigation validity derived from MAVLink GPS/health facts."""

    local_position_ok: bool = False
    global_position_ok: bool = False
    home_position_ok: bool = False
    problems: Tuple[str, ...] = ()


_TOKEN = object()


def __register_records() -> None:
    for record_class in list(globals().values()):
        if (
            isinstance(record_class, type)
            and issubclass(record_class, MavlinkRecord)
            and record_class is not MavlinkRecord
        ):
            _RECORD_TYPES[record_class.__name__] = record_class


__register_records()
