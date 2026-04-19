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
from mspapi2.msp_serial import MSPSerial

from lib.common import (
    apply_cfg,
    build_request_topic,
    build_response_topic,
    build_state_scheduler_topics,
    build_topic_base,
)
from lib.plugin_base import PluginBase
from lib.reference_frames import (
    FRAME_FRD,
    fru_to_frd_vector,
)
from lib.uav import build_control_fields, merge_control_fields, scale_float_pwm, scale_pwm_float
from lib.state_scheduler import StateScheduler
from protocols.namespace_loader import load_protocol_namespace

REQUEST_QUEUE_TIMEOUT_S = 0.05
POLL_INTERVAL_S = 0.1
UAV = load_protocol_namespace("uav")


class MspInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Configure MSP interface plugin.
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.control_override: Dict[str, Any] = {}
        self.control_override_lock = threading.Lock()
        self.control_override_updated_at = 0.0

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(POLL_INTERVAL_S)
        
        intervals = cfg["state_intervals"]
        state_topics = build_state_scheduler_topics(base, intervals)
        self.state_scheduler = StateScheduler(self.client, self.client_id, state_topics)

        if self.conn_type not in {"serial", "tcp"}:
            raise RuntimeError(f"unsupported conn_type {self.conn_type}")
        serial_transport = MSPSerial(
            self.conn_str,
            self.conn_bitrate,
            read_timeout=float(self.conn_read_timeout_s),
            write_timeout=float(self.conn_write_timeout_s),
            tcp=self.conn_type == "tcp",
            max_retries=int(self.conn_max_retries),
            reconnect_delay=float(self.conn_reconnect_delay_s),
        )
        self.api = MSPApi(port=self.conn_str, baudrate=self.conn_bitrate, serial_transport=serial_transport)

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
        self.loop_error_source: str | None = None
        self.loop_error_published = False
        self.worker_threads: Dict[str, threading.Thread] = {}
        self.latest_state: Dict[str, Any] = {}
        self.shutdown_requested = False
        self.serial_transport = serial_transport

        self._activate_override()
        self._refresh_state()

    def _capture_loop_error(self, source: str, exc: BaseException) -> None:  # Record first loop error with immediate context.
        if self.loop_error is not None:
            self.stop_event.set()
            return
        self.loop_error = exc
        self.loop_error_source = source
        self.loop_error_trace = traceback.format_exc().strip()
        print(
            f"[PLUGIN_ERROR] id={self.client_id} source={source} conn_type={self.conn_type} "
            f"conn={self.conn_str} reconnects={self.serial_transport.reconnects} "
            f"serial_diag={self.serial_transport.last_diag}",
            flush=True,
        )
        print(self.loop_error_trace, flush=True)
        try:
            self.publish_error(self.loop_error_trace)
            self.loop_error_published = True
        except Exception as publish_error:
            print(
                f"[PLUGIN_ERROR] id={self.client_id} source={source} publish_error={publish_error} "
                f"publish_error_trace={traceback.format_exc().strip()}",
                flush=True,
            )
        self.stop_event.set()

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

    def _build_rc_telemetry(self, rc_channels: Any) -> Dict[str, Any]:  # Convert observed FC RC channels into normalized semantic fields.
        if type(rc_channels) is dict:
            control = build_control_fields(
                scale_pwm_float(rc_channels[self.override_channels["roll"]], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]),
                scale_pwm_float(rc_channels[self.override_channels["pitch"]], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]),
                scale_pwm_float(rc_channels[self.override_channels["yaw"]], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]),
                scale_pwm_float(rc_channels[self.override_channels["throt"]], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]),
            )
            aux = []
            primary_names = set(self.override_channels.values())
            for channel_name in self.api.chmap[4:]:
                if channel_name in primary_names or channel_name not in rc_channels:
                    continue
                aux.append(
                    scale_pwm_float(rc_channels[channel_name], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"])
                )
            if aux:
                control["Aux"] = aux
            return control
        if type(rc_channels) is list and len(rc_channels) >= 4:
            return build_control_fields(
                float(rc_channels[0]),
                float(rc_channels[1]),
                float(rc_channels[3]),
                float(rc_channels[2]),
            )
        raise RuntimeError(f"unsupported rc_channels type {type(rc_channels).__name__}")

    def _override_is_fresh(self) -> bool:  # Check whether the last control override is still fresh enough to apply.
        if not self.control_override:
            return False
        return time.monotonic() - self.control_override_updated_at <= float(self.control_override_timeout_s)

    def _build_override_channels(self) -> Dict[str, int]:  # Convert sparse normalized control override into protocol channel writes.
        override_channels: Dict[str, int] = {}
        with self.control_override_lock:
            control_override = dict(self.control_override)
        if "Roll" in control_override:
            override_channels[self.override_channels["roll"]] = scale_float_pwm(
                float(control_override["Roll"]), self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]
            )
        if "Pitch" in control_override:
            override_channels[self.override_channels["pitch"]] = scale_float_pwm(
                float(control_override["Pitch"]), self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]
            )
        if "Yaw" in control_override:
            override_channels[self.override_channels["yaw"]] = scale_float_pwm(
                float(control_override["Yaw"]), self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]
            )
        if "Throttle" in control_override:
            override_channels[self.override_channels["throt"]] = scale_float_pwm(
                float(control_override["Throttle"]), self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]
            )
        return override_channels

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
        gyro_frd_x, gyro_frd_y, gyro_frd_z = fru_to_frd_vector(
            math.radians(gyro["X"]),
            math.radians(gyro["Y"]),
            math.radians(gyro["Z"]),
        )
        ang_vel_rad_s = {"X": gyro_frd_x, "Y": gyro_frd_y, "Z": gyro_frd_z}
        attitude_rad = {"Roll": roll_rad, "Pitch": pitch_rad, "Yaw": yaw_rad}
        accel_frd_x, accel_frd_y, accel_frd_z = fru_to_frd_vector(
            imu["acc"]["X"],
            imu["acc"]["Y"],
            imu["acc"]["Z"],
        )
        mag_frd_x, mag_frd_y, mag_frd_z = fru_to_frd_vector(
            imu["mag"]["X"],
            imu["mag"]["Y"],
            imu["mag"]["Z"],
        )
        imu_summary = {
            "AccelX": accel_frd_x,
            "AccelY": accel_frd_y,
            "AccelZ": accel_frd_z,
            "GyroRadSX": ang_vel_rad_s["X"],
            "GyroRadSY": ang_vel_rad_s["Y"],
            "GyroRadSZ": ang_vel_rad_s["Z"],
            "MagX": mag_frd_x,
            "MagY": mag_frd_y,
            "MagZ": mag_frd_z,
            "Frame": FRAME_FRD,
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
        rc_telemetry = self._build_rc_telemetry(rc_channels)
        with self.control_override_lock:
            control_override = dict(self.control_override)
        control_output = merge_control_fields(rc_telemetry, control_override) if self._override_is_fresh() else rc_telemetry
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
            UAV.State.Control.RcTelemetry: rc_telemetry,
            UAV.State.Control.ControlOverride: control_override,
            UAV.State.Control.ControlOutput: control_output,
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
            self._capture_loop_error("state_loop", exc)

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
                if not self._override_is_fresh():
                    continue
                snapshot = self.latest_state
                if not snapshot.get(UAV.State.Flight.OverrideActive):
                    continue
                channels = self._build_override_channels()
                if not channels:
                    continue
                with self.api_lock:
                    self.api.set_rc_channels(channels)
        except BaseException as exc:
            self._capture_loop_error("override_loop", exc)

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
        if action == UAV.Action.Control.SetControlOverride:
            with self.control_override_lock:
                self.control_override = dict(params)
                self.control_override_updated_at = time.monotonic()
            self.enqueue_response(request_id, action, True, dict(params))
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
            altitude_m = float(params["AltitudeM"])
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
                    altitude=altitude_m,
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
                    "AltitudeM": altitude_m,
                },
            )
            return
        if action == UAV.Action.Navigation.SetWaypoint:
            waypoint_index = int(params["WaypointIndex"])
            action_value = params["Action"]
            action_enum = InavEnums.navWaypointActions_e(action_value)
            latitude = float(params["Latitude"])
            longitude = float(params["Longitude"])
            altitude_m = float(params["AltitudeM"])
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
                    altitude=altitude_m,
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
                    "AltitudeM": altitude_m,
                },
            )
            return
        self.enqueue_response(request_id, action, False, {"error": f"unknown action {action}"})

    def _process_requests(self) -> None:  # Background loop to process requests.
        try:
            while not self.stop_event.is_set():
                try:
                    request = self.request_queue.get(timeout=REQUEST_QUEUE_TIMEOUT_S)
                except queue.Empty:
                    continue
                self._handle_action(request)
        except BaseException as exc:
            self._capture_loop_error("process_requests", exc)

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
        if not self.worker_threads:
            self.stop_event.clear()
            worker_specs = {
                "msp-request": self._process_requests,
                "msp-state": self._state_loop,
                "msp-override": self._override_loop,
            }
            for name, target in worker_specs.items():
                thread = threading.Thread(target=target, name=name, daemon=True)
                thread.start()
                self.worker_threads[name] = thread
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
                    self.request_queue.put(payload["data"])
                    continue
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            for thread in self.worker_threads.values():
                thread.join(timeout=5.0)
            self.worker_threads = {}
            self.flush_queue(self.response_queue, self.response_topic)
            with self.api_lock:
                self.api.close()
            if self.loop_error:
                trace = self.loop_error_trace or traceback.format_exception_only(
                    type(self.loop_error), self.loop_error
                )[-1].strip()
                if not self.shutdown_requested and not self.loop_error_published:
                    self.publish_error(trace)
                raise self.loop_error
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Entry point for plugin runner.
    MspInterface(cfg, bus_config).run()
