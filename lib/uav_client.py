"""Convenience API for OCCID-native UAV programs.

This is an SDK facade, not a second semantic protocol. Convenience methods map
program operations onto OCCID's generic Command families. High-rate direct
control samples remain latest-value OCCID Input records.
"""

from __future__ import annotations

from typing import Any, Iterable

from lib.common import build_request_topic, build_response_topic, build_state_topics, build_topic_base
from lib.occid_bus import get_occid_state, occid, send_occid_command, send_occid_input
from lib.occid_topics import ANGULAR_VELOCITY, ATTITUDE, FLIGHT_CONTROL, LOCATION
from lib.provisioning import asset_uid
from lib.uav_semantics import (
    DIRECT_CONTROL_ATTITUDE,
    DIRECT_CONTROL_MANUAL,
    PARAM_TAKEOFF_ALTITUDE_M,
    PROCESS_DIRECT_CONTROL,
    PROCESS_DIRECT_CONTROL_ATTITUDE,
    PROCESS_DIRECT_CONTROL_MANUAL,
    PROCESS_LAND,
    PROCESS_RETURN_TO_LAUNCH,
    PROCESS_TAKEOFF,
    PROPERTY_ARMED,
    PROPERTY_NATIVE_FLIGHT_MODE_CODE,
    PROPERTY_NATIVE_FLIGHT_MODE_NAME,
    PROPERTY_STANDARD_FLIGHT_MODE,
)


class UavCommandError(RuntimeError):
    pass


class UavClient:
    """Program-facing UAV API backed by OCCID Commands, Inputs, and State."""

    DEFAULT_STATE_KEYS = (FLIGHT_CONTROL, LOCATION, ATTITUDE, ANGULAR_VELOCITY)
    IMMEDIATE_COMMAND_TYPES = (
        occid.StateChangeCommand,
        occid.ProcessControlCommand,
        occid.ConfigurationCommand,
        occid.MotionCommand,
        occid.ResourceCommand,
        occid.ExecutionCommand,
    )

    def __init__(
        self,
        runtime: Any,
        interface: dict[str, Any],
        response_timeout_s: float,
        *,
        target_uid: Any | None = None,
    ) -> None:
        self.runtime = runtime
        self.interface_id = str(interface["id"])
        self.topic_ns = str(interface["topic_ns"])
        self.response_timeout_s = float(response_timeout_s)
        raw_target = target_uid if target_uid is not None else interface.get("target_uid")
        if raw_target is None:
            raw_target = asset_uid()
        self.target_uid = (
            raw_target if isinstance(raw_target, occid.UID) else occid.UID.model_validate(raw_target)
        )
        self.base_topic = build_topic_base(self.interface_id, self.topic_ns)
        self.request_topic = build_request_topic(self.interface_id, self.topic_ns)
        self.response_topic = build_response_topic(self.interface_id, self.topic_ns)
        self.input_topic = f"{self.base_topic}/INPUT"

    def state_topics(self, keys: Iterable[str] | None = None) -> dict[str, str]:
        selected = list(self.DEFAULT_STATE_KEYS if keys is None else keys)
        return build_state_topics(self.base_topic, selected)

    def state(self, key: str, expected_type: type | tuple[type, ...] | None = None) -> Any:
        return get_occid_state(self.runtime.state, key, expected_type)

    def flight_control(self) -> Any | None:
        return self.state(FLIGHT_CONTROL, occid.FlightControlState)

    def location(self) -> Any | None:
        return self.state(LOCATION, occid.LocationState)

    def attitude(self) -> Any | None:
        return self.state(ATTITUDE, occid.EulerAngles)

    def angular_velocity(self) -> Any | None:
        return self.state(ANGULAR_VELOCITY, occid.AngularVelocityVector)

    def send(self, command: Any) -> str:
        if type(command) not in self.IMMEDIATE_COMMAND_TYPES:
            allowed = ", ".join(command_type.__name__ for command_type in self.IMMEDIATE_COMMAND_TYPES)
            raise TypeError(
                f"UavClient accepts concrete OCCID Command families only "
                f"allowed={allowed} actual={type(command).__name__}"
            )
        if command.target_uid != self.target_uid:
            raise ValueError(
                f"command target_uid does not match client target "
                f"expected={self.target_uid} actual={command.target_uid}"
            )
        return send_occid_command(self.runtime.bus, self.request_topic, command)

    def execute(self, command: Any, timeout_s: float | None = None) -> dict[str, Any]:
        """Explicitly wait for an eventual endpoint result when ordering or outcome matters."""
        request_id = self.send(command)
        response = self.runtime._wait_response(
            request_id,
            self.response_timeout_s if timeout_s is None else float(timeout_s),
        )
        if not response.get("ok"):
            raise UavCommandError(f"{type(command).__name__} failed response={response}")
        return response

    def send_input(self, input_model: Any) -> None:
        send_occid_input(self.runtime.bus, self.input_topic, input_model)

    def _state_change(
        self,
        operation: Any,
        property_name: str,
        value: Any | None = None,
    ) -> Any:
        return occid.StateChangeCommand(
            target_uid=self.target_uid,
            constraints=[],
            operation=operation,
            property_name=property_name,
            value=value,
        )

    def _process(self, operation: Any, process_name: str) -> Any:
        return occid.ProcessControlCommand(
            target_uid=self.target_uid,
            constraints=[],
            operation=operation,
            process_name=process_name,
        )

    def arm_command(self, armed: bool = True) -> Any:
        return self._state_change(
            occid.StateChangeOperation.SET,
            PROPERTY_ARMED,
            occid.MetadataValue(bool=bool(armed)),
        )

    def takeoff_altitude_command(self, relative_altitude_m: float) -> Any:
        return occid.ConfigurationCommand(
            target_uid=self.target_uid,
            constraints=[],
            operation=occid.ConfigurationOperation.SET_PARAMETER,
            parameter_name=PARAM_TAKEOFF_ALTITUDE_M,
            value=occid.MetadataValue(float=float(relative_altitude_m)),
        )

    def takeoff_command(self) -> Any:
        return self._process(occid.ProcessControlOperation.START, PROCESS_TAKEOFF)

    def land_command(self) -> Any:
        return self._process(occid.ProcessControlOperation.START, PROCESS_LAND)

    def return_to_launch_command(self) -> Any:
        return self._process(occid.ProcessControlOperation.START, PROCESS_RETURN_TO_LAUNCH)

    def go_to_command(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        *,
        altitude_datum: Any = None,
        yaw_rad: float | None = None,
    ) -> Any:
        datum = occid.AltitudeDatum.RELATIVE if altitude_datum is None else altitude_datum
        return occid.MotionCommand(
            target_uid=self.target_uid,
            constraints=[],
            operation=occid.MotionOperation.MOVE_TO,
            destination=occid.GlobalPosition(
                lat=float(latitude_deg),
                lon=float(longitude_deg),
                alt=float(altitude_m),
                alt_frame=datum,
            ),
            yaw_rad=None if yaw_rad is None else float(yaw_rad),
        )

    def arm(self) -> str:
        return self.send(self.arm_command(True))

    def disarm(self) -> str:
        return self.send(self.arm_command(False))

    def set_takeoff_altitude(self, relative_altitude_m: float) -> str:
        return self.send(self.takeoff_altitude_command(relative_altitude_m))

    def takeoff(self) -> str:
        return self.send(self.takeoff_command())

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
        altitude_datum: Any = None,
        yaw_rad: float | None = None,
    ) -> str:
        return self.send(
            self.go_to_command(
                latitude_deg,
                longitude_deg,
                altitude_m,
                altitude_datum=altitude_datum,
                yaw_rad=yaw_rad,
            )
        )

    def set_mode(
        self,
        *,
        standard_mode: Any | None = None,
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
        operation = (
            occid.StateChangeOperation.ENABLE
            if enabled
            else occid.StateChangeOperation.DISABLE
        )
        if standard_mode is not None:
            property_name = PROPERTY_STANDARD_FLIGHT_MODE
            value = occid.MetadataValue(str=str(getattr(standard_mode, "name", standard_mode)))
        elif native_mode_name is not None:
            property_name = PROPERTY_NATIVE_FLIGHT_MODE_NAME
            value = occid.MetadataValue(str=str(native_mode_name))
        else:
            property_name = PROPERTY_NATIVE_FLIGHT_MODE_CODE
            value = occid.MetadataValue(int=int(native_mode_code))
        return self.send(self._state_change(operation, property_name, value))

    def begin_direct_control(self, mode: str, timeout_s: float | None = None) -> dict[str, Any]:
        """Acquire an adapter-local direct-control process before publishing Input samples."""
        normalized = str(getattr(mode, "name", mode)).upper()
        if normalized == DIRECT_CONTROL_ATTITUDE:
            process_name = PROCESS_DIRECT_CONTROL_ATTITUDE
        elif normalized == DIRECT_CONTROL_MANUAL:
            process_name = PROCESS_DIRECT_CONTROL_MANUAL
        else:
            raise ValueError(f"unsupported direct-control mode {mode!r}")
        return self.execute(
            self._process(occid.ProcessControlOperation.START, process_name),
            timeout_s=timeout_s,
        )

    def end_direct_control(self, timeout_s: float | None = None) -> dict[str, Any]:
        """Release the adapter-local direct-control process after Input publication stops."""
        return self.execute(
            self._process(occid.ProcessControlOperation.STOP, PROCESS_DIRECT_CONTROL),
            timeout_s=timeout_s,
        )

    def set_attitude(
        self,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        thrust_normalized: float,
        *,
        body_frame: Any = None,
        reference_frame: Any = None,
    ) -> None:
        body = occid.BodyReferenceFrame.FRD if body_frame is None else body_frame
        reference = occid.InertialReferenceFrame.NED if reference_frame is None else reference_frame
        self.send_input(
            occid.ControlAttitudeSetpoint(
                roll_rad=float(roll_rad),
                pitch_rad=float(pitch_rad),
                yaw_rad=float(yaw_rad),
                thrust_normalized=float(thrust_normalized),
                body_frame=body,
                reference_frame=reference,
            )
        )

    def set_control_override(self, override: Any) -> None:
        if not isinstance(override, occid.ControlOverride):
            raise TypeError(f"expected ControlOverride, got {type(override).__name__}")
        self.send_input(override)
