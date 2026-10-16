from __future__ import annotations

import math
import re
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from pymavlink import mavutil


def _finite_or_none(value: Any) -> float | None:
    number = float(value)
    return number if math.isfinite(number) else None


@dataclass(frozen=True)
class SourceSpec:
    raw: str
    connection: str
    baud: int | None
    serial: bool

    @classmethod
    def parse(cls, raw: str) -> "SourceSpec":
        value = raw.strip()
        if not value:
            raise ValueError("empty MAVLink source")

        if value.startswith("serial://"):
            value = value[len("serial://"):]

        if value.startswith("/dev/"):
            match = re.match(r"^(.*):(\d+)$", value)
            if match:
                return cls(raw=raw, connection=match.group(1), baud=int(match.group(2)), serial=True)
            return cls(raw=raw, connection=value, baud=None, serial=True)

        for scheme in ("udpin", "udpout", "tcp", "tcpin"):
            prefix = scheme + "://"
            if value.startswith(prefix):
                return cls(raw=raw, connection=scheme + ":" + value[len(prefix):], baud=None, serial=False)

        if value.startswith("udp://"):
            return cls(raw=raw, connection="udpin:" + value[len("udp://"):], baud=None, serial=False)

        return cls(raw=raw, connection=value, baud=None, serial=False)


@dataclass
class MavSourceBinding:
    """Persistent MAVLink system/component identity for one adapter process.

    A binding may be declared explicitly or discovered from the first qualifying
    autopilot heartbeat. Once set, it never changes during the process lifetime.
    Connection-specific telemetry belongs in MavSnapshot, not here.
    """

    sysid: int | None = None
    compid: int | None = None

    def __post_init__(self) -> None:
        if self.sysid is not None and not 1 <= int(self.sysid) <= 255:
            raise ValueError("MAVLink sysid must be between 1 and 255")
        if self.compid is not None and not 1 <= int(self.compid) <= 255:
            raise ValueError("MAVLink compid must be between 1 and 255")

    def bind(self, *, sysid: int, compid: int) -> None:
        if self.sysid is not None and self.sysid != sysid:
            raise ValueError(f"MAVLink sysid already bound to {self.sysid}, not {sysid}")
        if self.compid is not None and self.compid != compid:
            raise ValueError(f"MAVLink compid already bound to {self.compid}, not {compid}")
        if self.sysid is None:
            self.sysid = sysid
        if self.compid is None:
            self.compid = compid


@dataclass
class StatusText:
    severity: int
    text: str
    received_ts: float


@dataclass
class Battery:
    battery_id: int = 0
    voltage_v: float | None = None
    current_a: float | None = None
    consumed_mah: float | None = None
    remaining_pct: float | None = None
    temperature_c: float | None = None


@dataclass
class MavSnapshot:
    # This object is intentionally MAVLink-shaped. Protocol-specific facts stay
    # here and are translated at the MPFC bus boundary rather than entering it
    # as generic ints/floats/strings.
    binding: MavSourceBinding = field(default_factory=MavSourceBinding)
    connection_ready: bool = False

    sysid: int | None = None
    compid: int | None = None
    mav_type: int | None = None
    mav_type_name: str | None = None
    autopilot: int | None = None
    autopilot_name: str | None = None
    system_status: int | None = None
    system_status_name: str | None = None
    base_mode: int | None = None
    custom_mode: int | None = None
    mode_name: str | None = None
    armed: bool | None = None
    in_air: bool | None = None

    lat: float | None = None
    lon: float | None = None
    absolute_alt_m: float | None = None
    relative_alt_m: float | None = None
    local_x_m: float | None = None
    local_y_m: float | None = None
    local_z_m: float | None = None

    vx_m_s: float | None = None
    vy_m_s: float | None = None
    vz_m_s: float | None = None
    roll_rad: float | None = None
    pitch_rad: float | None = None
    yaw_rad: float | None = None
    roll_rate_rad_s: float | None = None
    pitch_rate_rad_s: float | None = None
    yaw_rate_rad_s: float | None = None
    heading_deg: float | None = None
    climb_rate_m_s: float | None = None
    airspeed_m_s: float | None = None
    groundspeed_m_s: float | None = None

    gnss_fix: int | None = None
    gnss_sats: int | None = None
    gnss_lat: float | None = None
    gnss_lon: float | None = None
    gnss_alt_m: float | None = None
    gnss_groundspeed_m_s: float | None = None
    gnss_course_deg: float | None = None
    gnss_hdop: float | None = None
    gnss_vdop: float | None = None
    gnss_hacc_m: float | None = None
    gnss_vacc_m: float | None = None
    gnss_yaw_deg: float | None = None

    battery: Battery = field(default_factory=Battery)
    batteries: dict[int, Battery] = field(default_factory=dict)

    sensor_present_mask: int | None = None
    sensor_enabled_mask: int | None = None
    sensor_health_mask: int | None = None
    estimator_flags: int | None = None
    estimator_bad_flags: int = 0

    # SYS_STATUS communication quality is an aggregate communication fact from
    # the vehicle. RADIO_STATUS counters are separate radio-link facts.
    comm_drop_rate_ratio: float | None = None
    comm_error_count: int | None = None
    radio_receive_errors: int | None = None
    radio_corrected_errors: int | None = None

    flight_sw_version: int | None = None
    middleware_sw_version: int | None = None
    os_sw_version: int | None = None
    board_version: int | None = None
    vendor_id: int | None = None
    product_id: int | None = None
    hardware_uid: int | None = None

    source_observation_ts: float | None = None
    source_time_basis: str | None = None
    last_received_ts: float | None = None
    last_received_monotonic: float | None = None
    status_text: deque[StatusText] = field(default_factory=lambda: deque(maxlen=8))

    @property
    def has_telemetry(self) -> bool:
        return self.last_received_ts is not None

    def is_fresh(self, now: float, stale_after: float) -> bool:
        return self.last_received_ts is not None and (now - self.last_received_ts) <= stale_after

    @staticmethod
    def _enum_name(group: str, value: int) -> str | None:
        try:
            return mavutil.mavlink.enums[group][value].name
        except Exception:
            return None

    def _source_time_from(self, message: Any) -> None:
        if message.get_type() == "SYSTEM_TIME":
            unix_us = int(getattr(message, "time_unix_usec", 0) or 0)
            if unix_us > 0:
                self.source_observation_ts = unix_us / 1_000_000.0
                self.source_time_basis = "UNIX"
                return

        if hasattr(message, "time_usec"):
            value = int(getattr(message, "time_usec", 0) or 0)
            if value > 0:
                self.source_observation_ts = value / 1_000_000.0
                self.source_time_basis = "UNIX" if value >= 1_000_000_000_000 else "BOOT"
                return

        if hasattr(message, "time_boot_ms"):
            value = int(getattr(message, "time_boot_ms", 0) or 0)
            if value > 0:
                self.source_observation_ts = value / 1000.0
                self.source_time_basis = "BOOT"

    def _qualifying_source_heartbeat(self, message: Any) -> bool:
        if message.get_type() != "HEARTBEAT":
            return False

        sysid = int(message.get_srcSystem())
        compid = int(message.get_srcComponent())
        if self.binding.sysid is not None and sysid != self.binding.sysid:
            return False
        if self.binding.compid is not None and compid != self.binding.compid:
            return False

        # A complete binding is already authoritative source selection. Discovery
        # heuristics only apply while one or both source IDs are still unknown.
        if self.binding.sysid is not None and self.binding.compid is not None:
            return True

        mav_type = int(getattr(message, "type", -1))
        autopilot = int(getattr(message, "autopilot", -1))
        if mav_type == getattr(mavutil.mavlink, "MAV_TYPE_GCS", 6):
            return False
        if autopilot == getattr(mavutil.mavlink, "MAV_AUTOPILOT_INVALID", 8):
            return False
        return True

    def _select_or_filter(self, message: Any) -> bool:
        sysid = int(message.get_srcSystem())
        compid = int(message.get_srcComponent())

        # Every connection generation must begin with the selected source's
        # heartbeat. This prevents a reconnect from making old observations fresh
        # and prevents another system/component on a multiplexed stream from
        # silently taking over the declared asset.
        if not self.connection_ready:
            if not self._qualifying_source_heartbeat(message):
                return False
            self.binding.bind(sysid=sysid, compid=compid)
            self.connection_ready = True
            return True

        return sysid == self.binding.sysid and compid == self.binding.compid

    def consume(self, message: Any, *, received_ts: float | None = None) -> bool:
        if message is None or message.get_type() == "BAD_DATA":
            return False
        if not self._select_or_filter(message):
            return False

        now = time.time() if received_ts is None else float(received_ts)
        self.last_received_ts = now
        self.last_received_monotonic = time.monotonic()
        self._source_time_from(message)

        kind = message.get_type()
        if kind == "HEARTBEAT":
            self.sysid = int(message.get_srcSystem())
            self.compid = int(message.get_srcComponent())
            self.mav_type = int(message.type)
            self.mav_type_name = self._enum_name("MAV_TYPE", self.mav_type)
            self.autopilot = int(message.autopilot)
            self.autopilot_name = self._enum_name("MAV_AUTOPILOT", self.autopilot)
            self.system_status = int(message.system_status)
            self.system_status_name = self._enum_name("MAV_STATE", self.system_status)
            self.base_mode = int(message.base_mode)
            self.custom_mode = int(message.custom_mode)
            self.armed = bool(self.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            try:
                mode = mavutil.mode_string_v10(message)
                self.mode_name = None if not mode or mode == "Mode(0x00000000)" else str(mode)
            except Exception:
                self.mode_name = None

        elif kind == "GLOBAL_POSITION_INT":
            self.lat = float(message.lat) / 1e7
            self.lon = float(message.lon) / 1e7
            self.absolute_alt_m = float(message.alt) / 1000.0
            self.relative_alt_m = float(message.relative_alt) / 1000.0
            self.vx_m_s = float(message.vx) / 100.0
            self.vy_m_s = float(message.vy) / 100.0
            self.vz_m_s = float(message.vz) / 100.0
            hdg = int(message.hdg)
            if hdg != 0xFFFF:
                self.heading_deg = hdg / 100.0

        elif kind == "LOCAL_POSITION_NED":
            self.local_x_m = float(message.x)
            self.local_y_m = float(message.y)
            self.local_z_m = float(message.z)
            self.vx_m_s = float(message.vx)
            self.vy_m_s = float(message.vy)
            self.vz_m_s = float(message.vz)

        elif kind == "ATTITUDE":
            self.roll_rad = float(message.roll)
            self.pitch_rad = float(message.pitch)
            self.yaw_rad = float(message.yaw)
            self.roll_rate_rad_s = float(message.rollspeed)
            self.pitch_rate_rad_s = float(message.pitchspeed)
            self.yaw_rate_rad_s = float(message.yawspeed)

        elif kind == "VFR_HUD":
            self.airspeed_m_s = _finite_or_none(message.airspeed)
            self.groundspeed_m_s = _finite_or_none(message.groundspeed)
            self.heading_deg = _finite_or_none(message.heading)
            self.climb_rate_m_s = _finite_or_none(message.climb)

        elif kind == "GPS_RAW_INT":
            self.gnss_fix = int(message.fix_type)
            sats = int(message.satellites_visible)
            self.gnss_sats = None if sats == 255 else sats
            if int(message.lat) or int(message.lon):
                self.gnss_lat = float(message.lat) / 1e7
                self.gnss_lon = float(message.lon) / 1e7
                self.gnss_alt_m = float(message.alt) / 1000.0
            vel = int(message.vel)
            self.gnss_groundspeed_m_s = None if vel == 0xFFFF else vel / 100.0
            cog = int(message.cog)
            self.gnss_course_deg = None if cog == 0xFFFF else cog / 100.0
            eph = int(message.eph)
            epv = int(message.epv)
            self.gnss_hdop = None if eph == 0xFFFF else eph / 100.0
            self.gnss_vdop = None if epv == 0xFFFF else epv / 100.0
            h_acc = int(getattr(message, "h_acc", 0xFFFFFFFF))
            v_acc = int(getattr(message, "v_acc", 0xFFFFFFFF))
            self.gnss_hacc_m = None if h_acc == 0xFFFFFFFF else h_acc / 1000.0
            self.gnss_vacc_m = None if v_acc == 0xFFFFFFFF else v_acc / 1000.0
            yaw = int(getattr(message, "yaw", 0) or 0)
            self.gnss_yaw_deg = None if yaw == 0 else (yaw % 36000) / 100.0

        elif kind == "SYS_STATUS":
            self.sensor_present_mask = int(message.onboard_control_sensors_present)
            self.sensor_enabled_mask = int(message.onboard_control_sensors_enabled)
            self.sensor_health_mask = int(message.onboard_control_sensors_health)
            remaining = int(message.battery_remaining)
            if remaining >= 0:
                self.battery.remaining_pct = float(remaining)
            voltage = int(message.voltage_battery)
            if voltage not in (0, 0xFFFF):
                self.battery.voltage_v = voltage / 1000.0
            current = int(message.current_battery)
            if current not in (-1, 0xFFFF):
                self.battery.current_a = current / 100.0
            # drop_rate_comm is centi-percent. stored as a 0..1 ratio.
            self.comm_drop_rate_ratio = int(message.drop_rate_comm) / 10000.0
            self.comm_error_count = int(message.errors_comm)

        elif kind == "BATTERY_STATUS":
            battery_id = int(message.id)
            battery = self.batteries.setdefault(battery_id, Battery(battery_id=battery_id))
            remaining = int(message.battery_remaining)
            if remaining >= 0:
                battery.remaining_pct = float(remaining)
            temperature = int(message.temperature)
            if temperature != 0x7FFF:
                battery.temperature_c = temperature / 100.0
            voltages = [int(v) for v in message.voltages if int(v) not in (0, 0xFFFF)]
            for value in getattr(message, "voltages_ext", ()) or ():
                value = int(value)
                if value not in (0, 0xFFFF):
                    voltages.append(value)
            if voltages:
                battery.voltage_v = sum(voltages) / 1000.0
            current = int(message.current_battery)
            if current not in (-1, 0x7FFF):
                battery.current_a = current / 100.0
            consumed = int(message.current_consumed)
            if consumed >= 0:
                battery.consumed_mah = float(consumed)
            if battery_id == 0:
                self.battery = battery

        elif kind == "RADIO_STATUS":
            # rssi/remrssi/noise/remnoise are device-dependent MAVLink values,
            # so they are deliberately not normalized into a signal quality.
            self.radio_receive_errors = int(message.rxerrors)
            self.radio_corrected_errors = int(message.fixed)

        elif kind == "EXTENDED_SYS_STATE":
            landed = int(message.landed_state)
            unknown = getattr(mavutil.mavlink, "MAV_LANDED_STATE_UNDEFINED", 0)
            if landed != unknown:
                self.in_air = landed == getattr(mavutil.mavlink, "MAV_LANDED_STATE_IN_AIR", 2)

        elif kind in ("ESTIMATOR_STATUS", "EKF_STATUS_REPORT"):
            flags = int(getattr(message, "flags", 0) or 0)
            self.estimator_flags = flags
            bad = 0
            for name in ("ESTIMATOR_GPS_GLITCH", "ESTIMATOR_ACCEL_ERROR"):
                bad |= int(getattr(mavutil.mavlink, name, 0) or 0)
            self.estimator_bad_flags = flags & bad

        elif kind == "AUTOPILOT_VERSION":
            self.flight_sw_version = int(getattr(message, "flight_sw_version", 0) or 0)
            self.middleware_sw_version = int(getattr(message, "middleware_sw_version", 0) or 0)
            self.os_sw_version = int(getattr(message, "os_sw_version", 0) or 0)
            self.board_version = int(getattr(message, "board_version", 0) or 0)
            self.vendor_id = int(getattr(message, "vendor_id", 0) or 0)
            self.product_id = int(getattr(message, "product_id", 0) or 0)
            self.hardware_uid = int(getattr(message, "uid", 0) or 0)

        elif kind == "STATUSTEXT":
            text = getattr(message, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            text = str(text).rstrip("\x00").strip()
            if text:
                self.status_text.append(
                    StatusText(severity=int(message.severity), text=text, received_ts=now)
                )

        return True
