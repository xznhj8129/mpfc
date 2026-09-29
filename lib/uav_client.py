"""Convenience API for MPFC flight cores over MAVLink-shaped records.

This is an SDK facade, not a second semantic protocol.  Convenience methods map
program operations onto MAVLink commands (DEC-1) and MAVLink-shaped state
records.  High-rate direct-control samples remain latest-value records sent on
the input topic.
"""

from __future__ import annotations

from typing import Any, Iterable

from lib.bus_topics import ANGULAR_VELOCITY, ATTITUDE, FLIGHT_CONTROL, LOCATION
from lib.common import build_request_topic, build_response_topic, build_state_topics, build_topic_base
from lib.lattice_bus import (
    decode_request,
    get_record_state,
    send_command,
    send_input,
)
from lib.mavlink_models import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_TAKEOFF,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    AngularVelocity,
    Attitude,
    AttitudeTarget,
    CommandLong,
    ControlOverride,
    DirectControl,
    GlobalPositionInt,
    MavlinkRecord,
    ParamSet,
    RepositionCommand,
    SetMode,
    VehicleControl,
)
from lib.provisioning import asset_id
from lib.uav_semantics import (
    DIRECT_CONTROL_ATTITUDE,
    DIRECT_CONTROL_MANUAL,
    PARAM_TAKEOFF_ALTITUDE,
    standard_mode_name,
)

import math


class UavCommandError(RuntimeError):
    pass


class UavClient:
    """Program-facing UAV API backed by MAVLink commands and state records."""

    DEFAULT_STATE_KEYS = (FLIGHT_CONTROL, LOCATION, ATTITUDE, ANGULAR_VELOCITY)
    IMMEDIATE_COMMAND_TYPES = (
        CommandLong,
        RepositionCommand,
        SetMode,
        ParamSet,
        DirectControl,
    )
    CONTROL_INPUT_TYPES = (AttitudeTarget, ControlOverride)

    def __init__(
        self,
        runtime: Any,
        interface: dict[str, Any],
        response_timeout_s: float,
        *,
        target_id: str | None = None,
    ) -> None:
        self.runtime = runtime
        self.interface_id = str(interface["id"])
        self.topic_ns = str(interface["topic_ns"])
        self.response_timeout_s = float(response_timeout_s)
        raw_target = target_id if target_id is not None else interface.get("target_id")
        if raw_target in (None, ""):
            raw_target = asset_id()
        self.target_id = _target_text(raw_target)
        self.base_topic = build_topic_base(self.interface_id, self.topic_ns)
        self.request_topic = build_request_topic(self.interface_id, self.topic_ns)
        self.response_topic = build_response_topic(self.interface_id, self.topic_ns)
        self.input_topic = f"{self.base_topic}/INPUT"

    def state_topics(self, keys: Iterable[str] | None = None) -> dict[str, str]:
        selected = list(self.DEFAULT_STATE_KEYS if keys is None else keys)
        return build_state_topics(self.base_topic, selected)

    def state(self, key: str, expected_type: type | tuple[type, ...] | None = None) -> Any:
        return get_record_state(self.runtime.state, key, expected_type)

    def flight_control(self) -> VehicleControl | None:
        return self.state(FLIGHT_CONTROL, VehicleControl)

    def location(self) -> GlobalPositionInt | None:
        return self.state(LOCATION, GlobalPositionInt)

    def attitude(self) -> Attitude | None:
        return self.state(ATTITUDE, Attitude)

    def angular_velocity(self) -> AngularVelocity | None:
        return self.state(ANGULAR_VELOCITY, AngularVelocity)

    def send(self, command: MavlinkRecord) -> str:
        if type(command) not in self.IMMEDIATE_COMMAND_TYPES:
            allowed = ", ".join(command_type.__name__ for command_type in self.IMMEDIATE_COMMAND_TYPES)
            raise TypeError(
                f"UavClient accepts MAVLink command records only "
                f"allowed={allowed} actual={type(command).__name__}"
            )
        return send_command(self.runtime.bus, self.request_topic, command)

    def execute(self, command: MavlinkRecord, timeout_s: float | None = None) -> dict[str, Any]:
        """Explicitly wait for an eventual endpoint result when ordering or outcome matters."""
        request_id = self.send(command)
        response = self.runtime._wait_response(
            request_id,
            self.response_timeout_s if timeout_s is None else float(timeout_s),
        )
        if not response.get("ok"):
            raise UavCommandError(f"{type(command).__name__} failed response={response}")
        return response

    def send_input(self, sample: MavlinkRecord) -> None:
        if not isinstance(sample, self.CONTROL_INPUT_TYPES):
            allowed = ", ".join(item.__name__ for item in self.CONTROL_INPUT_TYPES)
            raise TypeError(
                f"UavClient input accepts {allowed} actual={type(sample).__name__}"
            )
        send_input(self.runtime.bus, self.input_topic, sample)

    def arm_command(self, armed: bool = True) -> CommandLong:
        return CommandLong(
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            params=(1.0 if armed else 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def takeoff_altitude_command(self, relative_altitude_m: float) -> ParamSet:
        return ParamSet(param_id=PARAM_TAKEOFF_ALTITUDE, value=float(relative_altitude_m))

    def takeoff_command(self, relative_altitude_m: float | None = None) -> CommandLong:
        altitude = 0.0 if relative_altitude_m is None else float(relative_altitude_m)
        return CommandLong(
            command=MAV_CMD_NAV_TAKEOFF,
            params=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, altitude),
        )

    def land_command(self) -> CommandLong:
        return CommandLong(command=MAV_CMD_NAV_LAND)

    def return_to_launch_command(self) -> CommandLong:
        return CommandLong(command=MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def go_to_command(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        *,
        altitude_reference: str = "relative",
        yaw_rad: float | None = None,
        ground_speed_m_s: float = 0.0,
    ) -> RepositionCommand:
        return RepositionCommand(
            latitude_deg=float(latitude_deg),
            longitude_deg=float(longitude_deg),
            altitude_m=float(altitude_m),
            altitude_reference=str(altitude_reference),
            yaw_rad=None if yaw_rad is None else float(yaw_rad),
            ground_speed_m_s=float(ground_speed_m_s),
        )

    def arm(self) -> str:
        return self.send(self.arm_command(True))

    def disarm(self) -> str:
        return self.send(self.arm_command(False))

    def set_takeoff_altitude(self, relative_altitude_m: float) -> str:
        return self.send(self.takeoff_altitude_command(relative_altitude_m))

    def takeoff(self, relative_altitude_m: float | None = None) -> str:
        return self.send(self.takeoff_command(relative_altitude_m))

    def land(self) -> str:
        return self.send(self.land_command())

    def return_to_launch(self) -> str:
        return self.send(self.return_to_launch_command())

    def go_to(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        *,
        altitude_reference: str = "relative",
        yaw_rad: float | None = None,
    ) -> str:
        return self.send(
            self.go_to_command(
                latitude_deg,
                longitude_deg,
                altitude_m,
                altitude_reference=altitude_reference,
                yaw_rad=yaw_rad,
            )
        )

    def set_mode(
        self,
        *,
        standard_mode: str | None = None,
        native_mode_name: str | None = None,
        native_mode_code: int | None = None,
        enabled: bool = True,
    ) -> str:
        selectors = sum(
            selector is not None
            for selector in (standard_mode, native_mode_name, native_mode_code)
        )
        if selectors != 1:
            raise ValueError("set_mode requires exactly one standard/native selector")
        if not enabled:
            # DISABLE of a mode selector has no MAVLink SET_MODE form; the
            # vehicle-native adapter decides how a mode is left.
            if standard_mode is not None:
                name = standard_mode_name(standard_mode)
            else:
                name = str(native_mode_name or native_mode_code)
            return self.send(SetMode(base_mode=0, custom_mode=0, mode_name=name))
        if standard_mode is not None:
            return self.send(
                SetMode(
                    base_mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode=0,
                    mode_name=standard_mode_name(standard_mode),
                )
            )
        if native_mode_name is not None:
            return self.send(
                SetMode(
                    base_mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode=0,
                    mode_name=str(native_mode_name),
                )
            )
        return self.send(
            SetMode(
                base_mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode=int(native_mode_code),
            )
        )

    def begin_direct_control(self, mode: str, timeout_s: float | None = None) -> dict[str, Any]:
        """Acquire an adapter-local direct-control process before publishing Input samples."""
        normalized = str(getattr(mode, "name", mode)).upper()
        if normalized not in (DIRECT_CONTROL_ATTITUDE, DIRECT_CONTROL_MANUAL):
            raise ValueError(f"unsupported direct-control mode {mode!r}")
        return self.execute(
            DirectControl(enabled=True, mode=normalized),
            timeout_s=timeout_s,
        )

    def end_direct_control(self, timeout_s: float | None = None) -> dict[str, Any]:
        """Release the adapter-local direct-control process after Input publication stops."""
        return self.execute(DirectControl(enabled=False, mode=""), timeout_s=timeout_s)

    def set_attitude(
        self,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        thrust_normalized: float,
    ) -> None:
        self.send_input(
            AttitudeTarget(
                type_mask=0,
                roll_rad=float(roll_rad),
                pitch_rad=float(pitch_rad),
                yaw_rad=float(yaw_rad),
                thrust=float(thrust_normalized),
            )
        )

    def set_control_override(self, override: ControlOverride) -> None:
        if not isinstance(override, ControlOverride):
            raise TypeError(f"expected ControlOverride, got {type(override).__name__}")
        self.send_input(override)

    @staticmethod
    def altitude_error_m(current: GlobalPositionInt, target_altitude_m: float) -> float:
        """Vertical error against an explicit target altitude."""
        return abs(float(current.relative_altitude_m) - float(target_altitude_m))


def _target_text(value: Any) -> str:
    if isinstance(value, str):
        text = value.strip()
        if not text:
            raise ValueError("target entity id cannot be empty")
        return text
    return str(value)


def haversine_m(a: GlobalPositionInt, b: tuple[float, float]) -> float:
    """Great-circle distance between a position record and a (lat, lon) point."""
    radius = 6371008.8
    lat1 = math.radians(float(a.latitude_deg))
    lat2 = math.radians(float(b[0]))
    dlat = lat2 - lat1
    dlon = math.radians(float(b[1]) - float(a.longitude_deg))
    hav = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    return 2.0 * radius * math.asin(min(1.0, math.sqrt(hav)))
