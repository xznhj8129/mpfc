#!/usr/bin/env python3
"""
MavSDK interface plugin.
Usage:
    from plugins.mavsdk_interface.mavsdk_interface import run_plugin
    run_plugin(cfg, bus_config)
"""

import asyncio
import logging
import math
import queue
import threading
import time
import traceback
from typing import Any, Dict

from grpc import StatusCode
from grpc.aio import AioRpcError
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.info import InfoError
from mavsdk.offboard import Attitude, OffboardError

from lib.common import (
    apply_cfg,
    build_request_topic,
    build_response_topic,
    build_state_scheduler_topics,
    build_topic_base,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from lib.uav import merge_control_fields
from protocols.namespace_loader import load_protocol_namespace


REQUEST_QUEUE_TIMEOUT_S = 0.05
POLL_INTERVAL_S = 0.1
UAV = load_protocol_namespace("uav")


class MavsdkInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Configure MAVSDK interface plugin.
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.control_override: Dict[str, Any] = {}
        self.control_override_lock = threading.Lock()
        self.control_override_updated_at = 0.0
        self.control_output: Dict[str, Any] | None = None
        if self.consume_control_override:
            self.control_output = dict(self.control_output_initial)
        self.manual_control_started = False
        self.offboard_attitude_started = False
        if self.conn_type != "udp":
            raise RuntimeError(f"unsupported conn_type {self.conn_type}")
        self.system_address = self.conn_str
        if self.mavsdk_log_debug:
            logging.basicConfig(level=logging.DEBUG, force=True)
            logging.getLogger("mavsdk").setLevel(logging.DEBUG)
            logging.getLogger("mavsdk.system").setLevel(logging.DEBUG)
            logging.getLogger("mavsdk.async_plugin_manager").setLevel(logging.DEBUG)

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(POLL_INTERVAL_S)
        state_topics = build_state_scheduler_topics(base, self.state_intervals)
        self.state_scheduler = StateScheduler(self.client, self.client_id, state_topics)

        self.drone = System()
        self.request_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.loop_error: BaseException | None = None
        self.loop_error_trace: str | None = None
        self.loop_thread: threading.Thread | None = None
        self.shutdown_requested = False
        self.last_abs_alt_m: float | None = None
        self.last_rel_alt_m: float | None = None
        self.sensor_config: Dict[str, Any] = {}

    def _override_is_fresh(self) -> bool:  # Check whether the last control override is fresh enough to apply.
        if not self.control_override:
            return False
        return time.monotonic() - self.control_override_updated_at <= float(self.control_override_timeout_s)

    def _update_state(self, key: str, value: Any) -> None:  # Update state only when the field is configured.
        if key not in self.state_scheduler.topics:
            return
        self.state_scheduler.update(key, value)

    async def _process_requests(self) -> None:  # Drain request queue and dispatch actions.
        while not self.stop_event.is_set():
            try:
                request = await asyncio.to_thread(self.request_queue.get, timeout=REQUEST_QUEUE_TIMEOUT_S)
            except queue.Empty:
                await asyncio.sleep(POLL_INTERVAL_S)
                continue
            await self._handle_action(request)

    async def _handle_action(self, request: Dict[str, Any]) -> None:  # Handle one action request.
        request_id = str(request["request_id"])
        action = request["action"]
        params = request.get("params") or {}
        try:
            query_state_key = UAV.QueryToState.get(action)
            if query_state_key is not None:
                snapshot = self.state_scheduler.snapshot()
                self.enqueue_response(request_id, action, True, {query_state_key: snapshot.get(query_state_key)})
                return

            if action == UAV.Action.Flight.Rtl:
                await self.drone.action.return_to_launch()
                self.enqueue_response(request_id, action, True, {})
                return

            if action == UAV.Action.Navigation.GoTo:
                latitude = float(params["Latitude"])
                longitude = float(params["Longitude"])
                altitude_m = float(params["AltitudeM"])
                yaw_deg = float(params["YawDeg"])
                # Core commands use home-relative altitude, while MAVSDK goto_location expects AMSL altitude.
                target_abs_alt_m = float(self.last_abs_alt_m) + (altitude_m - float(self.last_rel_alt_m))
                await self.drone.action.goto_location(latitude, longitude, target_abs_alt_m, yaw_deg)
                self.enqueue_response(
                    request_id,
                    action,
                    True,
                    {
                        "Latitude": latitude,
                        "Longitude": longitude,
                        "AltitudeM": altitude_m,
                        "AbsAltitudeM": target_abs_alt_m,
                        "YawDeg": yaw_deg,
                    },
                )
                return

            if action == UAV.Action.Flight.SetTakeoffAltitude:
                altitude_m = float(params[UAV.State.Navigation.AltitudeM])
                await self.drone.action.set_takeoff_altitude(altitude_m)
                self.enqueue_response(request_id, action, True, {UAV.State.Navigation.AltitudeM: altitude_m})
                return

            if action == UAV.Action.Control.SetControlOverride:
                if not self.consume_control_override:
                    self.enqueue_response(request_id, action, False, {"error": "control override disabled"})
                    return
                with self.control_override_lock:
                    self.control_override = dict(params)
                    self.control_override_updated_at = time.monotonic()
                    self.control_output = merge_control_fields(self.control_output, self.control_override)
                    control_override = dict(self.control_override)
                    control_output = dict(self.control_output)
                self._update_state(UAV.State.Control.ControlOverride, control_override)
                self._update_state(UAV.State.Control.ControlOutput, control_output)
                self.enqueue_response(request_id, action, True, control_override)
                return

            if action == UAV.Action.Control.SetControlAttitude:
                control_attitude = {
                    "RollDeg": float(params["RollDeg"]),
                    "PitchDeg": float(params["PitchDeg"]),
                    "YawDeg": float(params["YawDeg"]),
                    "ThrustValue": float(params["ThrustValue"]),
                }
                await self.drone.offboard.set_attitude(
                    Attitude(
                        control_attitude["RollDeg"],
                        control_attitude["PitchDeg"],
                        control_attitude["YawDeg"],
                        control_attitude["ThrustValue"],
                    )
                )
                self._update_state(UAV.State.Control.ControlAttitude, control_attitude)
                self.enqueue_response(request_id, action, True, control_attitude)
                print(
                    f"[PLUGIN] {self.client_id} offboard_set_attitude "
                    f"roll_deg={control_attitude['RollDeg']} pitch_deg={control_attitude['PitchDeg']} "
                    f"yaw_deg={control_attitude['YawDeg']} thrust={control_attitude['ThrustValue']}",
                    flush=True,
                )
                return

            if action == UAV.Action.Control.StartOffboard:
                await self.drone.offboard.start()
                self.offboard_attitude_started = True
                self.enqueue_response(request_id, action, True, {})
                print(f"[PLUGIN] {self.client_id} offboard_start=True", flush=True)
                return

            if action == UAV.Action.Control.StopOffboard:
                await self.drone.offboard.stop()
                self.offboard_attitude_started = False
                self.enqueue_response(request_id, action, True, {})
                print(f"[PLUGIN] {self.client_id} offboard_stop=True", flush=True)
                return

            action_table = {
                UAV.Action.Flight.Arm: ("arm", None),
                UAV.Action.Flight.Disarm: ("disarm", None),
                UAV.Action.Flight.Takeoff: ("takeoff", None),
                UAV.Action.Flight.Land: ("land", None),
            }
            if action in action_table:
                method_name, param_key = action_table[action]
                method = getattr(self.drone.action, method_name)
                if param_key:
                    value = float(params[param_key])
                    await method(value)
                    self.enqueue_response(request_id, action, True, {param_key: value})
                    return
                await method()
                self.enqueue_response(request_id, action, True, {})
                return

            self.enqueue_response(request_id, action, False, {"error": f"unknown action {action}"})
        except (ActionError, OffboardError) as exc:
            self.enqueue_response(request_id, action, False, {"error": str(exc)})

    async def _watch_in_air(self) -> None:  # Watch in-air telemetry updates.
        async for in_air in self.drone.telemetry.in_air():
            self._update_state(UAV.State.Flight.IsInAir, bool(in_air))
            if self.stop_event.is_set():
                return

    async def _watch_armed(self) -> None:  # Watch armed telemetry updates.
        async for armed in self.drone.telemetry.armed():
            self._update_state(UAV.State.Flight.IsArmed, bool(armed))
            if self.stop_event.is_set():
                return

    async def _watch_health(self) -> None:  # Watch health telemetry updates.
        async for health in self.drone.telemetry.health():
            # [is_gyrometer_calibration_ok: True, is_accelerometer_calibration_ok: True, is_magnetometer_calibration_ok: True, is_local_position_ok: True, is_global_position_ok: True, is_home_position_ok: True, is_armable: True]
            self._update_state(UAV.State.Navigation.IsHomePositionOk, bool(health.is_home_position_ok))
            self._update_state(UAV.State.Navigation.IsGlobalPositionOk, bool(health.is_global_position_ok))
            sensor_config = dict(self.sensor_config)
            sensor_config.update(
                {
                    "GyroOk": bool(health.is_gyrometer_calibration_ok),
                    "AccelOk": bool(health.is_accelerometer_calibration_ok),
                    "MagOk": bool(health.is_magnetometer_calibration_ok),
                    "LocalPositionOk": bool(health.is_local_position_ok),
                    "GlobalPositionOk": bool(health.is_global_position_ok),
                    "HomePositionOk": bool(health.is_home_position_ok),
                    "Armable": bool(health.is_armable),
                }
            )
            self.sensor_config = sensor_config
            self._update_state(
                UAV.State.Sensor.SensorConfig,
                sensor_config,
            )
            if self.stop_event.is_set():
                return

    async def _watch_status_text(self) -> None:  # Watch FC status text for ArduPilot EKF readiness.
        async for status_text in self.drone.telemetry.status_text():
            text = status_text.text
            if " is using GPS" not in text:
                if self.stop_event.is_set():
                    return
                continue
            sensor_config = dict(self.sensor_config)
            if not sensor_config.get("EkfUsingGps"):
                print(f"[PLUGIN] {self.client_id} ardupilot_status_text text={text}", flush=True)
            sensor_config["EkfUsingGps"] = True
            self.sensor_config = sensor_config
            self._update_state(UAV.State.Sensor.SensorConfig, sensor_config)
            if self.stop_event.is_set():
                return

    async def _watch_position(self) -> None:  # Watch position telemetry updates.
        async for position in self.drone.telemetry.position():
            # [latitude_deg: 47.3979705, longitude_deg: 8.5461639, absolute_altitude_m: 0.20900000631809235, relative_altitude_m: -0.055000003427267075]
            self.last_abs_alt_m = float(position.absolute_altitude_m)
            self.last_rel_alt_m = float(position.relative_altitude_m)
            self._update_state(UAV.State.Navigation.AltitudeM, float(position.relative_altitude_m))
            self._update_state(
                UAV.State.Navigation.Position,
                {
                    "LatDeg": position.latitude_deg,
                    "LonDeg": position.longitude_deg,
                    "AbsAltM": position.absolute_altitude_m,
                    "RelAltM": position.relative_altitude_m,
                },
            )
            if self.stop_event.is_set():
                return

    async def _watch_attitude(self) -> None:  # Watch Euler attitude updates.
        async for attitude in self.drone.telemetry.attitude_euler():
            self._update_state(
                UAV.State.Attitude.AttitudeRad,
                {
                    "Roll": math.radians(attitude.roll_deg),
                    "Pitch": math.radians(attitude.pitch_deg),
                    "Yaw": math.radians(attitude.yaw_deg),
                },
            )
            if self.stop_event.is_set():
                return

    async def _watch_angular_velocity(self) -> None:  # Watch body angular velocity updates.
        async for velocity in self.drone.telemetry.attitude_angular_velocity_body():
            self._update_state(
                UAV.State.Attitude.AngVelRadS,
                {
                    "X": velocity.roll_rad_s,
                    "Y": velocity.pitch_rad_s,
                    "Z": velocity.yaw_rad_s,
                },
            )
            if self.stop_event.is_set():
                return

    async def _watch_gps_info(self) -> None:  # Watch GPS fix and satellite count updates.
        async for gps_info in self.drone.telemetry.gps_info():
            fix_type = gps_info.fix_type.value if hasattr(gps_info.fix_type, "value") else gps_info.fix_type
            self._update_state(UAV.State.Navigation.FixType, fix_type)
            self._update_state(UAV.State.Navigation.NumSat, gps_info.num_satellites)
            self._update_state(
                UAV.State.Navigation.GpsInfo,
                {
                    "FixType": fix_type,
                    "NumSat": gps_info.num_satellites,
                },
            )
            if self.stop_event.is_set():
                return

    async def _watch_raw_gps(self) -> None:  # Watch raw GPS telemetry updates.
        async for raw_gps in self.drone.telemetry.raw_gps():
            self._update_state(
                UAV.State.Navigation.RawGps,
                {
                    "LatDeg": raw_gps.latitude_deg,
                    "LonDeg": raw_gps.longitude_deg,
                    "AbsAltM": raw_gps.absolute_altitude_m,
                    "Hdop": raw_gps.hdop,
                    "Vdop": raw_gps.vdop,
                    "GroundSpeedMS": raw_gps.velocity_m_s,
                    "GroundCourseDeg": raw_gps.cog_deg,
                    "YawDeg": raw_gps.yaw_deg,
                },
            )
            if self.stop_event.is_set():
                return

    async def _watch_battery(self) -> None:  # Watch battery telemetry updates.
        async for battery in self.drone.telemetry.battery():
            self._update_state(
                UAV.State.Power.Battery,
                {
                    "VoltageV": battery.voltage_v,
                    "CurrentA": battery.current_battery_a,
                    "RemainingPct": battery.remaining_percent,
                    "ConsumedAh": battery.capacity_consumed_ah,
                    "TemperatureDegC": battery.temperature_degc,
                    "BatteryId": battery.id,
                },
            )
            if self.stop_event.is_set():
                return

    async def _watch_flight_mode(self) -> None:  # Watch active flight mode updates.
        async for flight_mode in self.drone.telemetry.flight_mode():
            mode_name = flight_mode.name if hasattr(flight_mode, "name") else str(flight_mode)
            self._update_state(UAV.State.System.FlightMode, {"FlightMode": mode_name})
            self._update_state(UAV.State.Flight.ActiveModeNames, [mode_name])
            if self.stop_event.is_set():
                return

    async def _watch_imu(self) -> None:  # Watch IMU telemetry updates.
        async for imu in self.drone.telemetry.imu():
            self._update_state(
                UAV.State.Sensor.Imu,
                {
                    "AccelX": imu.acceleration_frd.forward_m_s2,
                    "AccelY": imu.acceleration_frd.right_m_s2,
                    "AccelZ": imu.acceleration_frd.down_m_s2,
                    "GyroRadSX": imu.angular_velocity_frd.forward_rad_s,
                    "GyroRadSY": imu.angular_velocity_frd.right_rad_s,
                    "GyroRadSZ": imu.angular_velocity_frd.down_rad_s,
                    "MagX": imu.magnetic_field_frd.forward_gauss,
                    "MagY": imu.magnetic_field_frd.right_gauss,
                    "MagZ": imu.magnetic_field_frd.down_gauss,
                    "TemperatureDegC": imu.temperature_degc,
                    "TimestampUs": imu.timestamp_us,
                    "Frame": "FRD",
                },
            )
            if self.stop_event.is_set():
                return

    async def _manual_control_loop(self) -> None:  # Send normalized control override through MAVSDK manual control when enabled.
        if not self.consume_control_override:
            return
        while not self.stop_event.is_set():
            if not self._override_is_fresh():
                self.manual_control_started = False
                await asyncio.sleep(float(self.control_override_send_interval_s))
                continue
            with self.control_override_lock:
                control_output = dict(self.control_output)
                control_override = dict(self.control_override)
            # MAVLink MANUAL_CONTROL uses x=pitch/forward, y=roll/right, z=thrust, r=yaw.
            await self.drone.manual_control.set_manual_control_input(
                float(control_output["Pitch"]),
                float(control_output["Roll"]),
                float(control_output["Throttle"]),
                float(control_output["Yaw"]),
            )
            if not self.manual_control_started:
                await self.drone.manual_control.start_altitude_control()
                self.manual_control_started = True
            self._update_state(UAV.State.Control.ControlOverride, control_override)
            self._update_state(UAV.State.Control.ControlOutput, control_output)
            await asyncio.sleep(float(self.control_override_send_interval_s))

    async def _publish_fc_info(self) -> None:  # Publish FC identification and version snapshot.
        for _ in range(25):
            if self.stop_event.is_set():
                return
            try:
                identification = await self.drone.info.get_identification()
                product = await self.drone.info.get_product()
                version = await self.drone.info.get_version()
                self._update_state(
                    UAV.State.System.FcInfo,
                    {
                        "HardwareUid": identification.hardware_uid,
                        "LegacyUid": identification.legacy_uid,
                        "VendorId": product.vendor_id,
                        "VendorName": product.vendor_name,
                        "ProductId": product.product_id,
                        "ProductName": product.product_name,
                        "FlightSwMajor": version.flight_sw_major,
                        "FlightSwMinor": version.flight_sw_minor,
                        "FlightSwPatch": version.flight_sw_patch,
                        "FlightSwGitHash": version.flight_sw_git_hash,
                        "OsSwMajor": version.os_sw_major,
                        "OsSwMinor": version.os_sw_minor,
                        "OsSwPatch": version.os_sw_patch,
                        "OsSwGitHash": version.os_sw_git_hash,
                    },
                )
                return
            except InfoError:
                await asyncio.sleep(0.2)

    async def _async_main(self) -> None:  # Run async MAVSDK tasks.

        print(f"[PLUGIN] {self.client_id} connecting type={self.conn_type} address={self.system_address}",flush=True,)

        await self.drone.connect(system_address=self.system_address)

        print(f"[PLUGIN] {self.client_id} connected type={self.conn_type} address={self.system_address}",flush=True,)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break
            if self.stop_event.is_set():
                return

        await self._publish_fc_info()
        self.send_online()

        if self.consume_control_override:
            self._update_state(UAV.State.Control.ControlOverride, {})
            self._update_state(UAV.State.Control.ControlOutput, dict(self.control_output))

        tasks = [
            asyncio.create_task(self._process_requests()),
            asyncio.create_task(self._watch_in_air()),
            asyncio.create_task(self._watch_armed()),
            asyncio.create_task(self._watch_health()),
            asyncio.create_task(self._watch_status_text()),
            asyncio.create_task(self._watch_position()),
            asyncio.create_task(self._watch_attitude()),
            asyncio.create_task(self._watch_angular_velocity()),
            asyncio.create_task(self._watch_gps_info()),
            asyncio.create_task(self._watch_raw_gps()),
            asyncio.create_task(self._watch_battery()),
            asyncio.create_task(self._watch_flight_mode()),
            asyncio.create_task(self._watch_imu()),
        ]
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

    def _loop_runner(self) -> None:  # Run async main inside thread.

        try:
            asyncio.run(self._async_main())
        except BaseException as exc:
            self.loop_error = exc
            self.loop_error_trace = traceback.format_exc().strip()

    def run(self) -> None:  # Run the plugin main loop.

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
                if topic is None:
                    continue
                if topic == self.request_topic:
                    self.request_queue.put(payload["data"])

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


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Entry point for plugin runner.
    MavsdkInterface(cfg, bus_config).run()
