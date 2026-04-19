#!/usr/bin/env python3
"""
Liftoff SITL telemetry interface plugin.
Usage:
    from plugins.liftoff_interface.liftoff_interface import run_plugin
    run_plugin(cfg, bus_config)

Liftoff conventions used

This section is the practical contract used by the `hiveos` integration layer.

### Raw Liftoff telemetry fields (as consumed by Skynet tools)

From the existing Liftoff tooling, raw telemetry packet fields are interpreted as:

```text
position: [x_right, y_forward, z_up]
attitude quaternion: [qx, qy, qz, qw]
gyro: [gx, gy, gz]   (scripts treat these as body rates)
input values: [throttle, yaw, pitch, roll]
```

### Liftoff -> bus mapping rules

For integration code:

```text
Control remap:
  liftoff [roll, pitch, yaw, throttle] -> bus AETR [aileron, elevator, throttle, rudder]

Body-vector remap:
  if source behaves as FRU, convert to FRD with z sign flip only

World/vector remap:
  apply RFU/ENU-like to NED transform at the interface boundary

Altitude:
  keep +Up everywhere for any AltitudeM field, regardless of vector-frame Z sign
```

### Validation note

Liftoff telemetry conventions are partly empirical. Any new parser change should be validated with a short truth-table test:

1. level hover
2. positive roll input
3. positive pitch input
4. positive yaw input
5. climb vs descend

Record expected sign changes in attitude, rates, Z vectors, and `AltitudeM`.

"""

import math
import socket
import struct
import threading
import time
import traceback
from typing import Any, Dict

from lib.common import apply_cfg, build_request_topic, build_response_topic, build_state_scheduler_topics, build_topic_base
from lib.plugin_base import PluginBase
from lib.uav import build_control_fields
from lib.state_scheduler import StateScheduler
from protocols.namespace_loader import load_protocol_namespace

POLL_INTERVAL_S = 0.05
UAV = load_protocol_namespace("uav")


class LiftoffInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(POLL_INTERVAL_S)
        state_topics = build_state_scheduler_topics(base, self.state_intervals)
        self.state_scheduler = StateScheduler(self.client, self.client_id, state_topics)

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

        self._update_state(UAV.State.System.FcConnected, False)
        self._update_state(
            UAV.State.Sensor.SensorConfig,
            {
                "GyroOk": True,
                "AccelOk": False,
                "MagOk": False,
                "LocalPositionOk": True,
                "GlobalPositionOk": False,
                "HomePositionOk": False,
                "Armable": True,
            },
        )
        self._update_state(UAV.State.System.FlightMode, {"FlightMode": "SIM"})

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

    def _update_state(self, key: str, value: Any) -> None:
        if key not in self.state_scheduler.topics:
            return
        self.state_scheduler.update(key, value)

    def _handle_action(self, request: Dict[str, Any]) -> None:
        request_id = str(request["request_id"])
        action = request["action"]
        query_state_key = UAV.QueryToState.get(action)
        if query_state_key is not None:
            snapshot = self.state_scheduler.snapshot()
            self.enqueue_response(request_id, action, True, {query_state_key: snapshot.get(query_state_key)})
            return
        self.enqueue_response(request_id, action, False, {"error": f"unsupported action {action}"})

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
                        self._update_state(UAV.State.System.FcConnected, False)
                    continue

                unpacked = struct.unpack_from("f" * 17, data)
                altitude_up_m = unpacked[3]
                quat_x = unpacked[4]
                quat_y = unpacked[5]
                quat_z = unpacked[6]
                quat_w = unpacked[7]
                throttle_input = unpacked[11]
                yaw_input = unpacked[12]
                pitch_input = unpacked[13]
                roll_input = unpacked[14]
                battery_remaining = unpacked[15]
                battery_voltage = unpacked[16]

                quat_norm = math.sqrt(
                    quat_x * quat_x + quat_y * quat_y + quat_z * quat_z + quat_w * quat_w
                )
                quat_x /= quat_norm
                quat_y /= quat_norm
                quat_z /= quat_norm
                quat_w /= quat_norm

                siny_cosp = 2.0 * (quat_w * quat_y + quat_z * quat_x)
                cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_x * quat_x)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                sinp = 2.0 * (quat_w * quat_x - quat_z * quat_y)
                if abs(sinp) >= 1.0:
                    pitch = math.copysign(math.pi / 2.0, sinp)
                else:
                    pitch = math.asin(sinp)

                sinr_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y)
                cosr_cosp = 1.0 - 2.0 * (quat_x * quat_x + quat_z * quat_z)
                roll = math.atan2(sinr_cosp, cosr_cosp)

                self.last_packet_monotonic = time.monotonic()
                if not self.fc_connected:
                    self.fc_connected = True
                    self._update_state(UAV.State.System.FcConnected, True)

                self._update_state(UAV.State.Navigation.AltitudeM, altitude_up_m)
                self._update_state(UAV.State.Flight.IsInAir, altitude_up_m >= float(self.in_air_alt_threshold))
                self._update_state(
                    UAV.State.Attitude.AttitudeRad,
                    {
                        "Roll": roll,
                        "Pitch": pitch,
                        "Yaw": yaw,
                    },
                )
                self._update_state(
                    UAV.State.Control.RcTelemetry,
                    build_control_fields(roll_input, pitch_input, yaw_input, throttle_input),
                )
                self._update_state(
                    UAV.State.Power.Battery,
                    {
                        "VoltageV": battery_voltage,
                        "RemainingPct": battery_remaining,
                    },
                )
                self._update_state(
                    UAV.State.Power.Analog,
                    {
                        "vbat": battery_voltage,
                        "percentageRemaining": battery_remaining,
                    },
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
                if topic is None:
                    continue
                if topic == self.request_topic:
                    self._handle_action(payload["data"])
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
                trace = self.loop_error_trace or traceback.format_exception_only(type(self.loop_error), self.loop_error)[
                    -1
                ].strip()
                if not self.shutdown_requested:
                    self.publish_error(trace)
                raise self.loop_error
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    LiftoffInterface(cfg, bus_config).run()
