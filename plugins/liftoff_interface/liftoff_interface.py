#!/usr/bin/env python3
"""Liftoff simulator endpoint adapter publishing MAVLink-shaped UAV state."""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
import traceback
from dataclasses import replace
from typing import Any, Dict

from pymavlink import mavutil

from lib.common import apply_cfg, build_request_topic, build_response_topic, build_state_scheduler_topics, build_topic_base
from lib.lattice_bus import decode_command, pack_record
from lib.bus_topics import ATTITUDE, FLIGHT_CONTROL, LOCATION, POWER, RC_TELEMETRY
from lib.mavlink_models import (
    Altitude,
    Attitude as AttitudeRecord,
    BatteryStatus,
    ControlAxes,
    Heartbeat,
    NavigationValidity,
    Readiness,
    VehicleControl,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler

POLL_INTERVAL_S = 0.05


class LiftoffInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(POLL_INTERVAL_S)
        self.state_scheduler = StateScheduler(
            self.client,
            self.client_id,
            build_state_scheduler_topics(base, self.state_intervals),
        )

        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_socket.bind((self.telemetry_host, int(self.telemetry_port)))
        self.telemetry_socket.settimeout(float(self.socket_timeout_s))

        self.stop_event = threading.Event()
        self.loop_error: BaseException | None = None
        self.loop_error_trace: str | None = None
        self.loop_thread: threading.Thread | None = None
        self.shutdown_requested = False
        self.last_packet_monotonic = 0.0
        self.fc_connected = False
        self._publish_flight_state(in_air=False)

    def _capture_loop_error(self, exc: BaseException) -> None:
        if self.loop_error is not None:
            self.stop_event.set()
            return
        self.loop_error = exc
        self.loop_error_trace = traceback.format_exc().strip()
        print(
            f"[PLUGIN_ERROR] id={self.client_id} telemetry_host={self.telemetry_host} telemetry_port={self.telemetry_port}",
            flush=True,
        )
        print(self.loop_error_trace, flush=True)
        self.stop_event.set()

    def _publish_model(self, key: str, model: Any) -> None:
        if key in self.state_scheduler.topics:
            self.state_scheduler.update(key, pack_record(model))

    def _publish_flight_state(self, in_air: bool) -> None:
        del in_air  # in-air derives from the published relative altitude
        nav = NavigationValidity(
            local_position_ok=self.fc_connected,
            global_position_ok=False,
            home_position_ok=False,
        )
        readiness = Readiness(
            armable=False,
            arm_ready=False,
            takeoff_ready=False,
            problems=("LIFTOFF_SIM",),
        )
        self._publish_model(
            FLIGHT_CONTROL,
            VehicleControl(
                heartbeat=Heartbeat(
                    type=mavutil.mavlink.MAV_TYPE_GENERIC,
                    autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    base_mode=0,
                    custom_mode=0,
                    system_status=mavutil.mavlink.MAV_STATE_STANDBY,
                    mode_name="LIFTOFF_SIM",
                    standard_mode="NON_STANDARD",
                ),
                readiness=readiness,
                navigation_validity=nav,
            ),
        )

    def _handle_command(self, request: Dict[str, Any]) -> None:
        request_id, command = decode_command(request)
        self.enqueue_response(
            request_id,
            type(command).__name__,
            False,
            {"error": "Liftoff interface is telemetry-only"},
        )

    def _telemetry_loop(self) -> None:
        print(
            f"[PLUGIN] {self.client_id} listening telemetry_host={self.telemetry_host} telemetry_port={self.telemetry_port}",
            flush=True,
        )
        try:
            while not self.stop_event.is_set():
                try:
                    data, _ = self.telemetry_socket.recvfrom(int(self.packet_buffer_size))
                except socket.timeout:
                    if self.fc_connected and time.monotonic() - self.last_packet_monotonic > float(self.link_timeout_s):
                        self.fc_connected = False
                        self._publish_flight_state(in_air=False)
                    continue

                unpacked = struct.unpack_from("f" * 17, data)
                altitude_up_m = float(unpacked[3])
                quat_x = float(unpacked[4])
                quat_y = float(unpacked[5])
                quat_z = float(unpacked[6])
                quat_w = float(unpacked[7])
                throttle_input = float(unpacked[11])
                yaw_input = float(unpacked[12])
                pitch_input = float(unpacked[13])
                roll_input = float(unpacked[14])
                battery_remaining = float(unpacked[15])
                battery_voltage = float(unpacked[16])

                quat_norm = math.sqrt(quat_x * quat_x + quat_y * quat_y + quat_z * quat_z + quat_w * quat_w)
                if quat_norm <= 1e-9:
                    raise RuntimeError("invalid zero-norm Liftoff attitude quaternion")
                quat_x /= quat_norm
                quat_y /= quat_norm
                quat_z /= quat_norm
                quat_w /= quat_norm

                siny_cosp = 2.0 * (quat_w * quat_y + quat_z * quat_x)
                cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_x * quat_x)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                sinp = 2.0 * (quat_w * quat_x - quat_z * quat_y)
                pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
                sinr_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y)
                cosr_cosp = 1.0 - 2.0 * (quat_x * quat_x + quat_z * quat_z)
                roll = math.atan2(sinr_cosp, cosr_cosp)

                self.last_packet_monotonic = time.monotonic()
                self.fc_connected = True
                in_air = altitude_up_m >= float(self.in_air_alt_threshold)
                self._publish_flight_state(in_air)
                self._publish_model(
                    LOCATION,
                    Altitude(altitude_relative=altitude_up_m),
                )
                self._publish_model(
                    ATTITUDE,
                    AttitudeRecord(
                        roll_rad=roll,
                        pitch_rad=pitch,
                        yaw_rad=yaw,
                        rollspeed_rad_s=0.0,
                        pitchspeed_rad_s=0.0,
                        yawspeed_rad_s=0.0,
                    ),
                )
                self._publish_model(
                    RC_TELEMETRY,
                    ControlAxes(
                        roll=roll_input,
                        pitch=pitch_input,
                        yaw=yaw_input,
                        throttle=throttle_input,
                        aux=(),
                    ),
                )
                if battery_remaining is not None and battery_remaining <= 1.0:
                    battery_remaining *= 100.0
                self._publish_model(
                    POWER,
                    BatteryStatus(
                        voltage_v=battery_voltage,
                        remaining_pct=battery_remaining,
                    ),
                )
        except BaseException as exc:
            self._capture_loop_error(exc)

    def run(self) -> None:
        if self.loop_thread is None:
            self.stop_event.clear()
            self.loop_thread = threading.Thread(target=self._telemetry_loop, name="liftoff-telemetry", daemon=True)
            self.loop_thread.start()

        self.send_online()
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
                    self._handle_command(payload["data"])
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            if self.loop_thread is not None:
                self.loop_thread.join(timeout=5.0)
                self.loop_thread = None
            self.flush_queue(self.response_queue, self.response_topic)
            self.telemetry_socket.close()
            if self.loop_error:
                trace = self.loop_error_trace or traceback.format_exception_only(type(self.loop_error), self.loop_error)[-1].strip()
                if not self.shutdown_requested:
                    self.publish_error(trace)
                raise self.loop_error
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    LiftoffInterface(cfg, bus_config).run()
