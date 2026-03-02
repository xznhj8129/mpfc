#!/usr/bin/env python3
"""
MSP interface plugin driven by mspapi2.
Usage:
    from plugins.msp_interface.msp_interface import run_plugin
    run_plugin(cfg, bus_config)
"""

import math
import queue
import threading
import time
import traceback
from typing import Any, Dict

from mspapi2.lib import InavEnums, boxes
from mspapi2.msp_api import MSPApi

from lib.common import (
    apply_cfg,
    build_request_topic,
    build_response_topic,
    build_set_topic,
    build_state_scheduler_topics,
    build_topic_base,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from protocols.namespace_loader import load_protocol_namespace

REQUEST_QUEUE_TIMEOUT_S = 0.05
POLL_INTERVAL_S = 0.1
UAV = load_protocol_namespace("uav")


class MspInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Configure MSP interface plugin.
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.override_output = dict(self.override_initial_us)
        self.override_lock = threading.Lock()

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.set_topic = build_set_topic(self.client_id)
        self.client.subscribe(self.request_topic)
        self.init_bus(POLL_INTERVAL_S)

        self.add_set_attr(self.set_topic, "rc_override", self, "override_output", dict, self.override_lock)
        self.add_set_attr(self.set_topic, "override_enabled", self, "override_enabled", bool)
        
        intervals = cfg["state_intervals"]
        state_topics = build_state_scheduler_topics(base, intervals)
        self.state_scheduler = StateScheduler(self.client, self.client_id, state_topics)

        if self.conn_type == "serial":
            self.api = MSPApi(port=self.conn_str, baudrate=self.conn_bitrate)
        elif self.conn_type == "tcp":
            self.api = MSPApi(port=None, baudrate=self.conn_bitrate, tcp_endpoint=self.conn_str)
        else:
            raise RuntimeError(f"unsupported conn_type {self.conn_type}")

        self.api_lock = threading.Lock()
        with self.api_lock:
            self.api.open()
            api_version = self.api.get_api_version()
            fc_variant = self.api.get_fc_variant()
            board_info = self.api.get_board_info()
            sensor_config = self.api.get_sensor_config()
            rx_cfg = self.api.get_rx_config()
            rx_map = self.api.get_rx_map()
            mode_ranges = self.api.get_mode_ranges()
            battery_config = self.api.get_battery_config()
        print(
            f"[PLUGIN_CONN] id={self.client_id} type={self.conn_type} conn={self.conn_str} baud={self.conn_bitrate}",
            flush=True,
        )
        self.fc_info = {"ApiVersion": api_version, "FcVariant": fc_variant, "BoardInfo": board_info}
        self.sensor_config = sensor_config
        self.rx_map = rx_map
        self.battery_config = battery_config
        self.rx_config = rx_cfg
        self.mode_channels: Dict[str, Dict[str, Any]] = {}
        for entry in mode_ranges:
            aux_index = entry["auxChannelIndex"]
            channel_index = aux_index + 4
            if channel_index < len(self.api.chmap):
                channel_name = self.api.chmap[channel_index]
            else:
                channel_name = f"ch{channel_index + 1}"
            pwm_start, pwm_end = entry["pwmRange"]
            active_pwm = int((pwm_start + pwm_end) / 2)
            self.mode_channels[entry["mode"]] = {"channel": channel_name, "pwm": active_pwm}
        self.mode_ranges = mode_ranges

        self.arm_mode_name = "ARM"
        self.override_mode_name = "MSP RC OVERRIDE"
        self.takeoff_altitude_m: float | None = None

        self.request_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.loop_error: BaseException | None = None
        self.loop_error_trace: str | None = None
        self.request_thread: threading.Thread | None = None
        self.state_thread: threading.Thread | None = None
        self.override_thread: threading.Thread | None = None
        self.latest_state: Dict[str, Any] = {}

        self._activate_override()
        self._refresh_state()

    def _update_state(self, key: str, value: Any) -> None:  # Update state only when the field is configured.
        if key not in self.state_scheduler.topics:
            return
        self.state_scheduler.update(key, value)

    def _activate_override(self) -> None:  # Enable override mode if available.
        if self.override_mode_name in self.mode_channels:
            channel = self.mode_channels[self.override_mode_name]["channel"]
            pwm = self.mode_channels[self.override_mode_name]["pwm"]
            with self.api_lock:
                self.api.set_rc_channels({channel: pwm})

    def _apply_mode(self, mode_name: str) -> None:  # Activate a flight mode via RC channel.
        if mode_name not in self.mode_channels:
            raise RuntimeError(f"mode {mode_name} not configured")
        channel = self.mode_channels[mode_name]["channel"]
        pwm = self.mode_channels[mode_name]["pwm"]
        with self.api_lock:
            self.api.set_rc_channels({channel: pwm})

    def _clear_mode(self, mode_name: str) -> None:  # Deactivate a flight mode via RC channel.
        if mode_name not in self.mode_channels:
            raise RuntimeError(f"mode {mode_name} not configured")
        channel = self.mode_channels[mode_name]["channel"]
        pwm = self.rx_config["rxMinUsec"]
        with self.api_lock:
            self.api.set_rc_channels({channel: pwm})

    def _set_throttle(self, value: int) -> None:  # Set throttle channel output.
        with self.api_lock:
            self.api.set_rc_channels({"throttle": value})

    def _refresh_state(self) -> None:  # Query MSP and update state scheduler snapshot.
        with self.api_lock:
            status = self.api.get_inav_status()
            analog = self.api.get_inav_analog()
            alt = self.api.get_altitude()
            gps = self.api.get_raw_gps()
            gps_statistics = self.api.get_gps_statistics()
            waypoint_info = self.api.get_waypoint_info()
            nav_status = self.api.get_nav_status()
            attitude = self.api.get_attitude()
            imu = self.api.get_imu()
            rc_channels = self.api.get_rc_channels()
        is_armed = InavEnums.armingFlag_e.ARMED in status["armingFlags"]
        altitude_m = alt["estimatedAltitude"]
        is_in_air = is_armed and altitude_m is not None and altitude_m >= self.in_air_alt_threshold
        global_ok = gps["fixType"] == InavEnums.gpsFixType_e.GPS_FIX_3D and gps["numSat"] >= self.home_min_satellites
        position = {
            "LatDeg": gps["latitude"],
            "LonDeg": gps["longitude"],
            "AbsAltM": gps["altitude"],
            "RelAltM": altitude_m,
        }
        active_modes = status["activeModes"]
        active_mode_names = [mode.name for mode in active_modes]
        active_mode_ids = [int(mode.value) for mode in active_modes]
        override_active = boxes.BoxEnum.BOXMSPRCOVERRIDE in active_modes
        failsafe = boxes.BoxEnum.BOXFAILSAFE in active_modes
        roll_rad = math.radians(attitude["roll"])
        pitch_rad = math.radians(attitude["pitch"])
        yaw_rad = math.radians(attitude["yaw"])
        gyro = imu["gyro"]
        ang_vel_rad_s = {
            "X": math.radians(gyro["X"]),
            "Y": math.radians(gyro["Y"]),
            "Z": math.radians(gyro["Z"]),
        }
        attitude_rad = {"Roll": roll_rad, "Pitch": pitch_rad, "Yaw": yaw_rad}
        imu_summary = {
            "AccelX": imu["acc"]["X"],
            "AccelY": imu["acc"]["Y"],
            "AccelZ": imu["acc"]["Z"],
            "GyroRadSX": ang_vel_rad_s["X"],
            "GyroRadSY": ang_vel_rad_s["Y"],
            "GyroRadSZ": ang_vel_rad_s["Z"],
            "MagX": imu["mag"]["X"],
            "MagY": imu["mag"]["Y"],
            "MagZ": imu["mag"]["Z"],
            "Frame": "BODY",
        }
        battery = {
            "VoltageV": analog.get("vbat"),
            "CurrentA": analog.get("amperage"),
            "PowerW": analog.get("powerDraw"),
            "ConsumedMah": analog.get("mAhDrawn"),
            "ConsumedMWh": analog.get("mWhDrawn"),
            "RemainingPct": analog.get("percentageRemaining"),
            "RemainingCapacity": analog.get("remainingCapacity"),
            "Rssi": analog.get("rssi"),
        }
        analog_state = {
            "vbat": analog.get("vbat"),
            "amperage": analog.get("amperage"),
            "powerDraw": analog.get("powerDraw"),
            "mAhDrawn": analog.get("mAhDrawn"),
            "mWhDrawn": analog.get("mWhDrawn"),
            "percentageRemaining": analog.get("percentageRemaining"),
            "remainingCapacity": analog.get("remainingCapacity"),
            "rssi": analog.get("rssi"),
        }
        gps_info = {
            "FixType": gps["fixType"],
            "NumSat": gps["numSat"],
        }
        raw_gps = {
            "LatDeg": gps.get("latitude"),
            "LonDeg": gps.get("longitude"),
            "AbsAltM": gps.get("altitude"),
            "GroundSpeedMS": gps.get("speed"),
            "GroundCourseDeg": gps.get("groundCourse"),
            "Hdop": gps.get("hdop"),
        }
        flight_mode = {"FlightMode": active_mode_names[0] if active_mode_names else "NONE"}
        new_state = {
            UAV.State.Flight.IsInAir: is_in_air,
            UAV.State.Flight.IsArmed: is_armed,
            UAV.State.Navigation.IsHomePositionOk: global_ok,
            UAV.State.Navigation.IsGlobalPositionOk: global_ok,
            UAV.State.Navigation.AltitudeM: altitude_m,
            UAV.State.Navigation.Position: position,
            UAV.State.Navigation.FixType: gps["fixType"],
            UAV.State.Navigation.NumSat: gps["numSat"],
            UAV.State.Navigation.NavState: nav_status.get("navState"),
            UAV.State.Attitude.AttitudeRad: attitude_rad,
            UAV.State.Attitude.AngVelRadS: ang_vel_rad_s,
            UAV.State.Sensor.Imu: imu_summary,
            UAV.State.Control.RcChannels: rc_channels,
            UAV.State.Control.RxConfig: self.rx_config,
            UAV.State.Control.ChannelMap: self.rx_map,
            UAV.State.Control.ModeRanges: self.mode_ranges,
            UAV.State.Flight.ActiveModes: active_mode_ids,
            UAV.State.Flight.ActiveModeNames: active_mode_names,
            UAV.State.Flight.OverrideActive: override_active,
            UAV.State.Flight.Failsafe: failsafe,
            UAV.State.Navigation.GpsInfo: gps_info,
            UAV.State.Navigation.RawGps: raw_gps,
            UAV.State.Power.Battery: battery,
            UAV.State.Power.Analog: analog_state,
            UAV.State.Sensor.SensorConfig: self.sensor_config,
            UAV.State.System.FcConnected: True,
            UAV.State.System.CpuLoad: status["cpuLoad"],
            UAV.State.System.CycleTime: status["cycleTime"],
            UAV.State.System.FcInfo: self.fc_info,
            UAV.State.System.WaypointInfo: waypoint_info,
            UAV.State.System.GpsStatistics: gps_statistics,
            UAV.State.System.FlightMode: flight_mode,
        }
        for key, value in new_state.items():
            self._update_state(key, value)
        self.latest_state = new_state

    def _state_loop(self) -> None:  # Background loop to refresh state.
        try:
            while not self.stop_event.is_set():
                self._refresh_state()
                time.sleep(self.state_poll_interval_s)
        except BaseException as exc:
            self.loop_error = exc
            self.loop_error_trace = traceback.format_exc().strip()
            self.stop_event.set()

    def _override_loop(self) -> None:  # Background loop to send RC overrides.
        try:
            last_send = 0.0
            while not self.stop_event.is_set():
                now = time.monotonic()
                elapsed = now - last_send
                if elapsed < self.override_send_interval:
                    time.sleep(self.override_send_interval - elapsed)
                    continue
                last_send = time.monotonic()
                if not self.override_enabled:
                    continue
                snapshot = self.latest_state
                if not snapshot.get(UAV.State.Flight.OverrideActive):
                    continue
                with self.override_lock:
                    override = dict(self.override_output)
                channels = {
                    self.override_channels["roll"]: int(override["roll"]),
                    self.override_channels["pitch"]: int(override["pitch"]),
                    self.override_channels["yaw"]: int(override["yaw"]),
                    self.override_channels["throt"]: int(override["throt"]),
                }
                with self.api_lock:
                    self.api.set_rc_channels(channels)
        except BaseException as exc:
            self.loop_error = exc
            self.loop_error_trace = traceback.format_exc().strip()
            self.stop_event.set()

    def _handle_action(self, request: Dict[str, Any]) -> None:  # Dispatch action requests to MSP.
        request_id = str(request["request_id"])
        action = request["action"]
        params = request.get("params") or {}
        query_state_key = UAV.QueryToState.get(action)
        if query_state_key is not None:
            snapshot = self._state_snapshot()
            self.enqueue_response(request_id, action, True, {query_state_key: snapshot.get(query_state_key)})
            return
        if action == UAV.Action.Flight.SetTakeoffAltitude:
            altitude_m = float(params["AltitudeM"])
            self.takeoff_altitude_m = altitude_m
            self.enqueue_response(request_id, action, True, {"AltitudeM": altitude_m})
            return
        if action == UAV.Action.Flight.Rtl:
            rtl_mode_name = None
            for candidate in ("RTH", "NAV RTH", "NAV_RTH"):
                if candidate in self.mode_channels:
                    rtl_mode_name = candidate
                    break
            if rtl_mode_name is None:
                raise RuntimeError("rtl mode not configured")
            self._apply_mode(rtl_mode_name)
            self.enqueue_response(request_id, action, True, {"Mode": rtl_mode_name, "Enabled": True})
            return
        if action == UAV.Action.Flight.SetMode:
            mode_name = str(params["Mode"])
            enabled = bool(params["Enabled"])
            if enabled:
                self._apply_mode(mode_name)
            else:
                self._clear_mode(mode_name)
            self.enqueue_response(request_id, action, True, {"Mode": mode_name, "Enabled": enabled})
            return
        if action == UAV.Action.Flight.Arm:
            self._activate_override()
            self._apply_mode(self.arm_mode_name)
            self._set_throttle(self.rx_config["rxMinUsec"])
            self.enqueue_response(request_id, action, True, {})
            return
        if action == UAV.Action.Flight.Takeoff:
            self._takeoff()
            self.enqueue_response(request_id, action, True, {})
            return
        if action == UAV.Action.Flight.Land:
            self._land()
            self.enqueue_response(request_id, action, True, {})
            return
        if action == UAV.Action.Navigation.GoTo:
            waypoint_index = int(self.go_to_waypoint["WaypointIndex"])
            action_value = int(self.go_to_waypoint["Action"])
            action_enum = InavEnums.navWaypointActions_e(action_value)
            latitude = float(params["Latitude"])
            longitude = float(params["Longitude"])
            altitude = float(params["AltitudeM"])
            param1 = int(self.go_to_waypoint["Param1"])
            param2 = int(self.go_to_waypoint["Param2"])
            param3 = int(self.go_to_waypoint["Param3"])
            flag = int(self.go_to_waypoint["Flag"])
            with self.api_lock:
                self.api.set_waypoint(
                    waypointIndex=waypoint_index,
                    action=action_enum,
                    latitude=latitude,
                    longitude=longitude,
                    altitude=altitude,
                    param1=param1,
                    param2=param2,
                    param3=param3,
                    flag=flag,
                )
            self.enqueue_response(
                request_id,
                action,
                True,
                {
                    "WaypointIndex": waypoint_index,
                    "Action": int(action_enum),
                    "Latitude": latitude,
                    "Longitude": longitude,
                    "AltitudeM": altitude,
                },
            )
            return
        if action == UAV.Action.Navigation.SetWaypoint:
            waypoint_index = int(params["WaypointIndex"])
            action_value = params["Action"]
            action_enum = InavEnums.navWaypointActions_e(action_value)
            latitude = float(params["Latitude"])
            longitude = float(params["Longitude"])
            altitude = float(params["AltitudeM"])
            param1 = int(params["Param1"])
            param2 = int(params["Param2"])
            param3 = int(params["Param3"])
            flag = int(params["Flag"])
            with self.api_lock:
                self.api.set_waypoint(
                    waypointIndex=waypoint_index,
                    action=action_enum,
                    latitude=latitude,
                    longitude=longitude,
                    altitude=altitude,
                    param1=param1,
                    param2=param2,
                    param3=param3,
                    flag=flag,
                )
            self.enqueue_response(
                request_id,
                action,
                True,
                {
                    "WaypointIndex": waypoint_index,
                    "Action": int(action_enum),
                    "Latitude": latitude,
                    "Longitude": longitude,
                    "AltitudeM": altitude,
                },
            )
            return
        self.enqueue_response(request_id, action, False, {"error": f"unknown action {action}"})

    def _request_loop(self) -> None:  # Background loop to process requests.
        try:
            while not self.stop_event.is_set():
                try:
                    request = self.request_queue.get(timeout=REQUEST_QUEUE_TIMEOUT_S)
                except queue.Empty:
                    continue
                self._handle_action(request)
        except BaseException as exc:
            self.loop_error = exc
            self.loop_error_trace = traceback.format_exc().strip()
            self.stop_event.set()

    def _takeoff(self) -> None:  # Execute takeoff sequence.
        if self.takeoff_altitude_m is None:
            raise RuntimeError("takeoff altitude not set")
        self._activate_override()
        self._apply_mode(self.arm_mode_name)
        start = time.monotonic()
        while not self.stop_event.is_set():
            self._set_throttle(self.takeoff_throttle)
            snapshot = self._state_snapshot()
            altitude_m = snapshot[UAV.State.Navigation.AltitudeM]
            if altitude_m is not None and altitude_m >= self.takeoff_altitude_m:
                self._set_throttle(self.hover_throttle)
                return
            if time.monotonic() - start > self.takeoff_timeout_s:
                raise RuntimeError("takeoff timeout")
            time.sleep(self.state_poll_interval_s)

    def _land(self) -> None:  # Execute landing sequence.
        start = time.monotonic()
        self._activate_override()
        self._apply_mode(self.arm_mode_name)
        while not self.stop_event.is_set():
            self._set_throttle(self.landing_throttle)
            snapshot = self._state_snapshot()
            altitude_m = snapshot[UAV.State.Navigation.AltitudeM]
            if altitude_m is not None and altitude_m <= self.in_air_alt_threshold:
                self._set_throttle(self.rx_config["rxMinUsec"])
                if self.arm_mode_name in self.mode_channels:
                    channel = self.mode_channels[self.arm_mode_name]["channel"]
                    with self.api_lock:
                        self.api.set_rc_channels({channel: self.rx_config["rxMinUsec"]})
                return
            if time.monotonic() - start > self.landing_timeout_s:
                raise RuntimeError("landing timeout")
            time.sleep(self.state_poll_interval_s)

    def _state_snapshot(self) -> Dict[str, Any]:  # Fetch latest state snapshot.
        return self.state_scheduler.snapshot()

    def run(self) -> None:  # Run the plugin main loop.
        if self.request_thread is None:
            self.stop_event.clear()
            self.request_thread = threading.Thread(target=self._request_loop, name="msp-request", daemon=True)
            self.state_thread = threading.Thread(target=self._state_loop, name="msp-state", daemon=True)
            self.override_thread = threading.Thread(target=self._override_loop, name="msp-override", daemon=True)
            self.request_thread.start()
            self.state_thread.start()
            self.override_thread.start()
        self.send_online()
        try:
            while True:
                self.state_scheduler.flush()
                self.flush_queue(self.response_queue, self.response_topic)
                if self.loop_error:
                    raise self.loop_error
                topic, payload = self._pump_once()
                if topic is None:
                    continue
                if topic == self.request_topic:
                    self.request_queue.put(payload["data"])
                    continue
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            if self.request_thread:
                self.request_thread.join(timeout=5.0)
                self.request_thread = None
            if self.state_thread:
                self.state_thread.join(timeout=5.0)
                self.state_thread = None
            if self.override_thread:
                self.override_thread.join(timeout=5.0)
                self.override_thread = None
            self.flush_queue(self.response_queue, self.response_topic)
            with self.api_lock:
                self.api.close()
            if self.loop_error:
                trace = self.loop_error_trace or traceback.format_exception_only(
                    type(self.loop_error), self.loop_error
                )[-1].strip()
                self.publish_error(trace)
                raise self.loop_error
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Entry point for plugin runner.
    MspInterface(cfg, bus_config).run()
