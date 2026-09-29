#!/usr/bin/env python3
"""MAVSDK endpoint adapter: MAVLink-shaped records <-> MAVSDK/PX4/ArduPilot."""

from __future__ import annotations

import asyncio
import logging
import queue
import threading
import time
import traceback
from dataclasses import replace
from typing import Any, Dict

from grpc import StatusCode
from grpc.aio import AioRpcError
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.info import InfoError
from mavsdk.offboard import Attitude, OffboardError
from pymavlink import mavutil

from lib.interop_mavsdk import (
    MavsdkPositionFields,
    angular_velocity_from_body_rates,
    attitude_from_euler_degrees,
    attitude_setpoint_to_fields,
    gnss_fix_type_from_native_value,
    goto_command_to_fields,
    position_to_location_record,
    standard_mode_from_native_name,
)
from lib.common import apply_cfg, build_envelope, build_request_topic, build_response_topic, build_state_scheduler_topics, build_topic_base
from lib.lattice_bus import decode_command, decode_input, pack_record
from lib.bus_topics import (
    ANGULAR_VELOCITY,
    ATTITUDE,
    CONTROL_OUTPUT,
    CONTROL_OVERRIDE,
    FIRMWARE,
    FLIGHT_CONTROL,
    GNSS,
    IMU,
    LOCATION,
    POWER,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from lib.mavlink_models import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_LOITER_UNLIM,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_TAKEOFF,
    MAV_MODE_FLAG_SAFETY_ARMED,
    Attitude as AttitudeRecord,
    AttitudeTarget,
    BatteryStatus,
    CommandLong,
    ControlAxes,
    ControlOverride,
    DirectControl,
    FirmwareVersion,
    GlobalPositionInt,
    GpsRawInt,
    Heartbeat,
    HighresImu,
    NavigationValidity,
    ParamSet,
    Readiness,
    RepositionCommand,
    SetMode,
    VehicleControl,
)
from lib.uav_semantics import (
    DIRECT_CONTROL_ATTITUDE,
    DIRECT_CONTROL_MANUAL,
    PARAM_TAKEOFF_ALTITUDE,
    standard_mode_name,
)

REQUEST_QUEUE_TIMEOUT_S = 0.05
POLL_INTERVAL_S = 0.1


class UnsupportedCommand(RuntimeError):
    pass


class MavsdkInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.system_address = self.conn_str
        self.is_ardupilot = str(self.mav_dialect).upper() == "ARDUPILOT"
        if self.mavsdk_log_debug:
            logging.basicConfig(level=logging.DEBUG, force=True)
            logging.getLogger("mavsdk").setLevel(logging.DEBUG)
            logging.getLogger("mavsdk.system").setLevel(logging.DEBUG)
            logging.getLogger("mavsdk.async_plugin_manager").setLevel(logging.DEBUG)

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.input_topic = f"{base}/INPUT"
        self.client.subscribe(self.request_topic)
        self.client.subscribe(self.input_topic)
        self.init_bus(POLL_INTERVAL_S)
        self.state_scheduler = StateScheduler(
            self.client,
            self.client_id,
            build_state_scheduler_topics(base, self.state_intervals),
        )

        self.drone = System()
        self.request_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.input_queue: "queue.Queue[Any]" = queue.Queue()
        self.stop_event = threading.Event()
        self.loop_error: BaseException | None = None
        self.loop_error_trace: str | None = None
        self.loop_thread: threading.Thread | None = None
        self.shutdown_requested = False

        mav_type = getattr(mavutil.mavlink, str(self.mav_type), mavutil.mavlink.MAV_TYPE_GENERIC)
        autopilot = (
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            if self.is_ardupilot
            else mavutil.mavlink.MAV_AUTOPILOT_PX4
        )
        self.heartbeat = Heartbeat(
            type=int(mav_type),
            autopilot=int(autopilot),
            base_mode=0,
            custom_mode=0,
            system_status=mavutil.mavlink.MAV_STATE_STANDBY,
        )
        self.nav_validity = NavigationValidity()
        self.readiness = Readiness()
        self.gnss_state = GpsRawInt(
            fix_type=mavutil.mavlink.GPS_FIX_TYPE_NO_GPS,
            satellites_visible=0,
            latitude_deg=0.0,
            longitude_deg=0.0,
            altitude_m=0.0,
        )
        self.last_abs_alt_m: float | None = None
        self.last_rel_alt_m: float | None = None
        self.last_attitude: AttitudeRecord | None = None
        self.location_state: GlobalPositionInt | None = None

        initial = dict(self.control_output_initial)
        self.control_output = ControlAxes(
            roll=float(initial.get("roll", 0.0)),
            pitch=float(initial.get("pitch", 0.0)),
            yaw=float(initial.get("yaw", 0.0)),
            throttle=float(initial.get("throttle", -1.0)),
            aux=tuple(float(value) for value in initial.get("aux", [])),
        )
        self.control_override: ControlOverride | None = None
        self.control_override_lock = threading.Lock()
        self.control_override_updated_at = 0.0
        self.direct_control_mode: str | None = None
        self.manual_control_started = False
        self.offboard_attitude_started = False

    def _stream_enabled(self, key: str) -> bool:
        return key in self.state_scheduler.topics

    def _publish_model(self, key: str, model: Any) -> None:
        if not self._stream_enabled(key):
            return
        self.state_scheduler.update(key, pack_record(model))

    def _vehicle_control(self) -> VehicleControl:
        return VehicleControl(
            heartbeat=self.heartbeat,
            readiness=self.readiness,
            navigation_validity=self.nav_validity,
        )

    def _publish_flight_control(self) -> None:
        self._publish_model(FLIGHT_CONTROL, self._vehicle_control())

    def _publish_input_rejected(self, model: Any, error: str) -> None:
        topic = f"DIAG/{self.client_id}/INPUT_REJECTED"
        self.client.publish(
            topic,
            build_envelope(
                self.client_id,
                topic,
                {"event": "INPUT_REJECTED", "model": type(model).__name__, "error": error},
            ),
        )

    def _override_is_fresh(self) -> bool:
        if self.direct_control_mode != DIRECT_CONTROL_MANUAL or self.control_override is None:
            return False
        return time.monotonic() - self.control_override_updated_at <= float(self.control_override_timeout_s)

    @staticmethod
    def _mavsdk_manual_throttle(value: float) -> float:
        """Map signed control position [-1, 1] to MAVSDK throttle [0, 1]."""
        signed = float(value)
        if signed < -1.0 or signed > 1.0:
            raise ValueError(f"throttle {signed} outside [-1, 1]")
        return (signed + 1.0) / 2.0

    def _merge_override(self, override: ControlOverride) -> ControlAxes:
        values = {
            "roll": self.control_output.roll,
            "pitch": self.control_output.pitch,
            "yaw": self.control_output.yaw,
            "throttle": self.control_output.throttle,
        }
        for name in ("roll", "pitch", "yaw", "throttle"):
            value = getattr(override, name)
            if value is not None:
                values[name] = float(value)
        aux = list(self.control_output.aux)
        for channel in override.aux:
            index = int(channel.channel_index)
            if index < 0:
                raise ValueError(f"negative control channel index {index}")
            while len(aux) <= index:
                aux.append(0.0)
            if channel.value is not None:
                aux[index] = float(channel.value)
        return ControlAxes(
            roll=values["roll"],
            pitch=values["pitch"],
            yaw=values["yaw"],
            throttle=values["throttle"],
            aux=tuple(aux),
        )

    def _respond(self, request_id: str, command: Any, ok: bool, data: Dict[str, Any] | None = None, error: str | None = None) -> None:
        payload = {} if data is None else dict(data)
        if error is not None:
            payload["error"] = error
        self.enqueue_response(request_id, type(command).__name__, ok, payload)

    async def _process_requests(self) -> None:
        while not self.stop_event.is_set():
            try:
                request = await asyncio.to_thread(self.request_queue.get, timeout=REQUEST_QUEUE_TIMEOUT_S)
            except queue.Empty:
                await asyncio.sleep(POLL_INTERVAL_S)
                continue
            await self._handle_command(request)

    async def _process_inputs(self) -> None:
        while not self.stop_event.is_set():
            try:
                payload = await asyncio.to_thread(self.input_queue.get, timeout=REQUEST_QUEUE_TIMEOUT_S)
            except queue.Empty:
                await asyncio.sleep(POLL_INTERVAL_S)
                continue
            try:
                model = decode_input(payload)
                await self._handle_input(model)
            except (UnsupportedCommand, TypeError, ValueError, OffboardError) as exc:
                rejected = locals().get("model", payload)
                self._publish_input_rejected(rejected, str(exc))

    async def _set_standard_mode(self, mode_name: str, enabled: bool) -> None:
        if not enabled:
            raise UnsupportedCommand("MAVSDK adapter cannot generically deactivate an arbitrary mode")
        if mode_name == "POSITION_HOLD":
            await self.drone.action.hold()
            return
        raise UnsupportedCommand(
            f"MAVSDK adapter does not map standard mode {mode_name}; "
            "use dedicated semantic processes for takeoff/land/RTL"
        )

    async def _begin_direct_control(self, mode: str) -> None:
        if mode == DIRECT_CONTROL_MANUAL and not self.consume_control_override:
            raise UnsupportedCommand("MANUAL_AXIS direct control is disabled by adapter configuration")
        if mode not in {DIRECT_CONTROL_ATTITUDE, DIRECT_CONTROL_MANUAL}:
            raise UnsupportedCommand(f"unsupported direct-control process mode={mode}")
        if self.direct_control_mode is not None and self.direct_control_mode != mode:
            raise UnsupportedCommand(
                f"direct-control process already active mode={self.direct_control_mode}; stop it before switching"
            )
        self.direct_control_mode = mode

    async def _end_direct_control(self) -> None:
        if self.offboard_attitude_started:
            await self.drone.offboard.stop()
        self.offboard_attitude_started = False
        self.manual_control_started = False
        self.direct_control_mode = None
        with self.control_override_lock:
            self.control_override = None
            self.control_override_updated_at = 0.0
        self._publish_flight_control()

    async def _handle_command_long(self, command: CommandLong) -> None:
        if command.command == MAV_CMD_COMPONENT_ARM_DISARM:
            if float(command.params[0]) == 1.0:
                await self.drone.action.arm()
            else:
                await self.drone.action.disarm()
            return
        if command.command == MAV_CMD_NAV_TAKEOFF:
            await self.drone.action.takeoff()
            return
        if command.command == MAV_CMD_NAV_LAND:
            await self.drone.action.land()
            return
        if command.command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
            await self.drone.action.return_to_launch()
            return
        if command.command == MAV_CMD_NAV_LOITER_UNLIM:
            await self.drone.action.hold()
            return
        raise UnsupportedCommand(f"unsupported MAV_CMD {command.command} for MAVSDK endpoint")

    async def _handle_set_mode(self, command: SetMode) -> None:
        if not command.mode_name:
            raise UnsupportedCommand("MAVSDK adapter does not expose arbitrary native mode selection")
        await self._set_standard_mode(standard_mode_name(command.mode_name), True)

    async def _handle_reposition(self, command: RepositionCommand) -> None:
        fields = goto_command_to_fields(
            command,
            current_absolute_altitude_m=self.last_abs_alt_m,
            current_relative_altitude_m=self.last_rel_alt_m,
            current_yaw_rad=(
                None if self.last_attitude is None else float(self.last_attitude.yaw_rad)
            ),
        )
        await self.drone.action.goto_location(
            fields.latitude_deg,
            fields.longitude_deg,
            fields.absolute_altitude_m,
            fields.yaw_deg,
        )

    async def _handle_input(self, model: Any) -> None:
        if isinstance(model, AttitudeTarget):
            if self.direct_control_mode != DIRECT_CONTROL_ATTITUDE:
                raise UnsupportedCommand("AttitudeTarget requires active ATTITUDE_THRUST direct-control process")
            fields = attitude_setpoint_to_fields(model)
            await self.drone.offboard.set_attitude(
                Attitude(fields.roll_deg, fields.pitch_deg, fields.yaw_deg, fields.thrust_value)
            )
            if not self.offboard_attitude_started:
                await self.drone.offboard.start()
                self.offboard_attitude_started = True
            self._publish_flight_control()
            return
        if isinstance(model, ControlOverride):
            if self.direct_control_mode != DIRECT_CONTROL_MANUAL:
                raise UnsupportedCommand("ControlOverride requires active MANUAL_AXIS direct-control process")
            with self.control_override_lock:
                self.control_override = model
                self.control_override_updated_at = time.monotonic()
                self.control_output = self._merge_override(model)
                control_output = self.control_output
            self._publish_model(CONTROL_OVERRIDE, model)
            self._publish_model(CONTROL_OUTPUT, control_output)
            self._publish_flight_control()
            return
        raise UnsupportedCommand(f"unsupported MAVSDK direct input {type(model).__name__}")

    async def _handle_command(self, request: Dict[str, Any]) -> None:
        request_id, command = decode_command(request)
        try:
            if isinstance(command, CommandLong):
                await self._handle_command_long(command)
            elif isinstance(command, SetMode):
                await self._handle_set_mode(command)
            elif isinstance(command, ParamSet):
                if command.param_id != PARAM_TAKEOFF_ALTITUDE:
                    raise UnsupportedCommand(f"unsupported MAVSDK parameter {command.param_id!r}")
                await self.drone.action.set_takeoff_altitude(float(command.value))
            elif isinstance(command, RepositionCommand):
                await self._handle_reposition(command)
            elif isinstance(command, DirectControl):
                if command.enabled:
                    await self._begin_direct_control(command.mode)
                else:
                    await self._end_direct_control()
            else:
                raise UnsupportedCommand(f"unsupported command record {type(command).__name__} for MAVSDK endpoint")
            self._respond(request_id, command, True)
        except (ActionError, OffboardError, UnsupportedCommand, ValueError, TypeError) as exc:
            self._respond(request_id, command, False, error=str(exc))

    async def _watch_in_air(self) -> None:
        async for in_air in self.drone.telemetry.in_air():
            self._publish_flight_control()
            if self.stop_event.is_set():
                return

    async def _watch_armed(self) -> None:
        async for armed in self.drone.telemetry.armed():
            base_mode = self.heartbeat.base_mode
            if armed:
                base_mode |= MAV_MODE_FLAG_SAFETY_ARMED
            else:
                base_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED
            self.heartbeat = replace(
                self.heartbeat,
                base_mode=base_mode,
                system_status=(
                    mavutil.mavlink.MAV_STATE_ACTIVE
                    if armed
                    else mavutil.mavlink.MAV_STATE_STANDBY
                ),
            )
            self._publish_flight_control()
            if self.stop_event.is_set():
                return

    async def _watch_health(self) -> None:
        async for health in self.drone.telemetry.health():
            self.nav_validity = replace(
                self.nav_validity,
                local_position_ok=bool(health.is_local_position_ok),
                global_position_ok=bool(health.is_global_position_ok),
                home_position_ok=bool(health.is_home_position_ok),
            )
            problems = [
                name
                for name, ok in (
                    ("gyro", health.is_gyrometer_calibration_ok),
                    ("accel", health.is_accelerometer_calibration_ok),
                    ("mag", health.is_magnetometer_calibration_ok),
                )
                if not ok
            ]
            self.readiness = replace(
                self.readiness,
                armable=bool(health.is_armable),
                problems=tuple(problems),
            )
            self._publish_flight_control()
            if self.stop_event.is_set():
                return

    async def _watch_status_text(self) -> None:
        async for status_text in self.drone.telemetry.status_text():
            text = status_text.text
            if self.is_ardupilot and " is using GPS" in text:
                if not self.readiness.ekf_using_gps:
                    print(f"[PLUGIN] {self.client_id} ardupilot_status_text text={text}", flush=True)
                self.readiness = replace(self.readiness, ekf_using_gps=True)
                self._publish_flight_control()
            if self.stop_event.is_set():
                return

    async def _watch_position(self) -> None:
        async for position in self.drone.telemetry.position():
            self.last_abs_alt_m = float(position.absolute_altitude_m)
            self.last_rel_alt_m = float(position.relative_altitude_m)
            self.location_state = position_to_location_record(
                MavsdkPositionFields(
                    latitude_deg=float(position.latitude_deg),
                    longitude_deg=float(position.longitude_deg),
                    absolute_altitude_m=self.last_abs_alt_m,
                    relative_altitude_m=self.last_rel_alt_m,
                )
            )
            self._publish_model(LOCATION, self.location_state)
            if self.stop_event.is_set():
                return

    async def _watch_attitude(self) -> None:
        async for attitude in self.drone.telemetry.attitude_euler():
            self.last_attitude = attitude_from_euler_degrees(
                float(attitude.roll_deg),
                float(attitude.pitch_deg),
                float(attitude.yaw_deg),
            )
            self._publish_model(ATTITUDE, self.last_attitude)
            if self.stop_event.is_set():
                return

    async def _watch_angular_velocity(self) -> None:
        async for velocity in self.drone.telemetry.attitude_angular_velocity_body():
            state = angular_velocity_from_body_rates(
                float(velocity.roll_rad_s),
                float(velocity.pitch_rad_s),
                float(velocity.yaw_rad_s),
            )
            self._publish_model(ANGULAR_VELOCITY, state)
            if self.stop_event.is_set():
                return

    async def _watch_gps_info(self) -> None:
        async for gps_info in self.drone.telemetry.gps_info():
            native_fix = gps_info.fix_type.value if hasattr(gps_info.fix_type, "value") else gps_info.fix_type
            self.gnss_state = replace(
                self.gnss_state,
                fix_type=gnss_fix_type_from_native_value(int(native_fix)),
                satellites_visible=int(gps_info.num_satellites),
            )
            self._publish_model(GNSS, self.gnss_state)
            if self.stop_event.is_set():
                return

    async def _watch_raw_gps(self) -> None:
        async for raw_gps in self.drone.telemetry.raw_gps():
            self.gnss_state = replace(
                self.gnss_state,
                latitude_deg=float(raw_gps.latitude_deg),
                longitude_deg=float(raw_gps.longitude_deg),
                altitude_m=float(raw_gps.absolute_altitude_m),
                eph=float(raw_gps.hdop),
                epv=float(raw_gps.vdop),
                velocity_m_s=float(raw_gps.velocity_m_s),
                cog_deg=float(raw_gps.cog_deg),
            )
            self._publish_model(GNSS, self.gnss_state)
            if self.stop_event.is_set():
                return

    async def _watch_battery(self) -> None:
        async for battery in self.drone.telemetry.battery():
            remaining = battery.remaining_percent
            state = BatteryStatus(
                voltage_v=(
                    None if battery.voltage_v is None else float(battery.voltage_v)
                ),
                current_a=(
                    None
                    if battery.current_battery_a is None
                    else float(battery.current_battery_a)
                ),
                remaining_pct=(
                    None if remaining is None else float(remaining) * 100.0
                ),
                consumed_mah=(
                    None
                    if battery.capacity_consumed_ah is None
                    else float(battery.capacity_consumed_ah) * 1000.0
                ),
                temperature_degc=(
                    None
                    if battery.temperature_degc is None
                    else float(battery.temperature_degc)
                ),
            )
            self._publish_model(POWER, state)
            if self.stop_event.is_set():
                return

    async def _watch_flight_mode(self) -> None:
        async for flight_mode in self.drone.telemetry.flight_mode():
            mode_name = flight_mode.name if hasattr(flight_mode, "name") else str(flight_mode)
            self.heartbeat = replace(
                self.heartbeat,
                mode_name=mode_name,
                standard_mode=standard_mode_from_native_name(mode_name),
            )
            self._publish_flight_control()
            if self.stop_event.is_set():
                return

    async def _watch_imu(self) -> None:
        async for imu in self.drone.telemetry.imu():
            state = HighresImu(
                xgyro=float(imu.angular_velocity_frd.forward_rad_s),
                ygyro=float(imu.angular_velocity_frd.right_rad_s),
                zgyro=float(imu.angular_velocity_frd.down_rad_s),
                temperature_degc=float(imu.temperature_degc),
            )
            self._publish_model(IMU, state)
            if self.stop_event.is_set():
                return

    async def _manual_control_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self._override_is_fresh():
                await asyncio.sleep(float(self.control_override_send_interval_s))
                continue
            with self.control_override_lock:
                output = self.control_output
                override = self.control_override
            await self.drone.manual_control.set_manual_control_input(
                float(output.pitch),
                float(output.roll),
                self._mavsdk_manual_throttle(float(output.throttle)),
                float(output.yaw),
            )
            if not self.manual_control_started:
                await self.drone.manual_control.start_altitude_control()
                self.manual_control_started = True
            if override is not None:
                self._publish_model(CONTROL_OVERRIDE, override)
            self._publish_model(CONTROL_OUTPUT, output)
            await asyncio.sleep(float(self.control_override_send_interval_s))

    async def _publish_fc_info(self) -> None:
        if not self._stream_enabled(FIRMWARE):
            return
        for _ in range(25):
            if self.stop_event.is_set():
                return
            try:
                product = await self.drone.info.get_product()
                version = await self.drone.info.get_version()
                state = FirmwareVersion(
                    firmware=str(product.product_name or product.vendor_name or self.mav_dialect),
                    version=(
                        f"{int(version.flight_sw_major)}.{int(version.flight_sw_minor)}."
                        f"{int(version.flight_sw_patch)}"
                    ),
                    board=str(version.flight_sw_git_hash),
                )
                self._publish_model(FIRMWARE, state)
                return
            except InfoError:
                await asyncio.sleep(0.2)

    async def _async_main(self) -> None:
        print(f"[PLUGIN] {self.client_id} connecting address={self.system_address}", flush=True)
        await self.drone.connect(system_address=self.system_address)
        print(f"[PLUGIN] {self.client_id} connected address={self.system_address}", flush=True)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break
            if self.stop_event.is_set():
                return

        await self._publish_fc_info()
        self.send_online()
        self._publish_flight_control()
        if self.consume_control_override:
            self._publish_model(CONTROL_OUTPUT, self.control_output)

        tasks = [
            asyncio.create_task(self._process_requests()),
            asyncio.create_task(self._process_inputs()),
        ]
        if self._stream_enabled(FLIGHT_CONTROL):
            tasks.extend(
                [
                    asyncio.create_task(self._watch_in_air()),
                    asyncio.create_task(self._watch_armed()),
                    asyncio.create_task(self._watch_health()),
                    asyncio.create_task(self._watch_status_text()),
                    asyncio.create_task(self._watch_flight_mode()),
                ]
            )
        if self._stream_enabled(LOCATION):
            tasks.append(asyncio.create_task(self._watch_position()))
        if self._stream_enabled(ATTITUDE):
            tasks.append(asyncio.create_task(self._watch_attitude()))
        if self._stream_enabled(ANGULAR_VELOCITY):
            tasks.append(asyncio.create_task(self._watch_angular_velocity()))
        if self._stream_enabled(GNSS):
            tasks.extend([asyncio.create_task(self._watch_gps_info()), asyncio.create_task(self._watch_raw_gps())])
        if self._stream_enabled(POWER):
            tasks.append(asyncio.create_task(self._watch_battery()))
        if self._stream_enabled(IMU):
            tasks.append(asyncio.create_task(self._watch_imu()))
        if self.consume_control_override:
            tasks.append(asyncio.create_task(self._manual_control_loop()))

        try:
            while not self.stop_event.is_set():
                for task in tasks:
                    if task.done():
                        exc = task.exception()
                        if exc:
                            if self.stop_event.is_set():
                                return
                            raise exc
                await asyncio.sleep(POLL_INTERVAL_S)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    def _loop_runner(self) -> None:
        try:
            asyncio.run(self._async_main())
        except BaseException as exc:
            self.loop_error = exc
            self.loop_error_trace = traceback.format_exc().strip()

    def run(self) -> None:
        if self.loop_thread is None:
            self.stop_event.clear()
            self.loop_thread = threading.Thread(target=self._loop_runner, name="mavsdk-loop", daemon=True)
            self.loop_thread.start()

        try:
            while True:
                self.state_scheduler.flush()
                self.flush_queue(self.response_queue, self.response_topic)
                if self.loop_error:
                    raise self.loop_error
                try:
                    topic, payload = self._pump_once()
                except SystemExit:
                    self.shutdown_requested = True
                    self.stop_event.set()
                    break
                if topic == self.request_topic:
                    self.request_queue.put(payload["data"])
                elif topic == self.input_topic:
                    self.input_queue.put(payload["data"])
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            if self.loop_thread:
                self.loop_thread.join(timeout=5.0)
                self.loop_thread = None
            self.flush_queue(self.response_queue, self.response_topic)
            if self.loop_error:
                if self.shutdown_requested and isinstance(self.loop_error, AioRpcError):
                    if self.loop_error.code() == StatusCode.UNAVAILABLE:
                        self.stop()
                        self.drone._stop_mavsdk_server()
                        return
                trace = self.loop_error_trace or traceback.format_exception_only(
                    type(self.loop_error), self.loop_error
                )[-1].strip()
                self.publish_error(trace)
                raise self.loop_error
            self.stop()
            self.drone._stop_mavsdk_server()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MavsdkInterface(cfg, bus_config).run()
