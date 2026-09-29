#!/usr/bin/env python3
"""MSP endpoint adapter: MAVLink-shaped records <-> INAV/Betaflight MSP."""

from __future__ import annotations

import queue
import threading
import time
import traceback
from typing import Any, Dict

from mspapi2.lib import InavEnums, boxes
from mspapi2.msp_api import MSPApi
from mspapi2.msp_serial import MSPSerial
from pymavlink import mavutil

from lib.interop_common import normalized_to_pwm
from lib.interop_msp import (
    InavGpsFields,
    angular_velocity_from_fru_degrees_s,
    attitude_from_degrees,
    gps_records,
    rc_pwm_mapping_to_control_axes,
    rc_sequence_to_control_axes,
    standard_mode_from_native_names,
)
from lib.common import apply_cfg, build_envelope, build_request_topic, build_response_topic, build_state_scheduler_topics, build_topic_base
from lib.lattice_bus import decode_command, decode_input, pack_record
from lib.bus_topics import (
    ANGULAR_VELOCITY,
    ATTITUDE,
    AUTOPILOT_MISSION,
    CONTROL_OUTPUT,
    CONTROL_OVERRIDE,
    FLIGHT_CONTROL,
    GNSS,
    IMU,
    LOCATION,
    POWER,
    RC_TELEMETRY,
    REMOTE_CONTROL,
    RUNTIME_LOAD,
    SENSOR_CONFIG,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from lib.mavlink_models import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_LOITER_UNLIM,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_TAKEOFF,
    AngularVelocity,
    Attitude,
    BatteryStatus,
    CommandLong,
    ControlAxes,
    ControlOverride,
    DirectControl,
    GlobalPositionInt,
    Heartbeat,
    HighresImu,
    MissionState,
    NavigationValidity,
    ParamSet,
    RcChannelMapEntry,
    RcModeRange,
    RcChannels,
    Readiness,
    ReceiverConfig,
    RemoteControlState,
    RepositionCommand,
    RuntimeLoad,
    SensorConfig,
    SetMode,
    VehicleControl,
)
from lib.uav_semantics import (
    DIRECT_CONTROL_MANUAL,
    PARAM_TAKEOFF_ALTITUDE,
    standard_mode_name,
)

REQUEST_QUEUE_TIMEOUT_S = 0.05
POLL_INTERVAL_S = 0.1


class UnsupportedCommand(RuntimeError):
    pass


class MspInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        if self.conn_type not in {"serial", "tcp"}:
            raise RuntimeError(f"unsupported conn_type {self.conn_type}")

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
            build_state_scheduler_topics(base, cfg["state_intervals"]),
        )

        self.serial_transport = MSPSerial(
            self.conn_str,
            self.conn_bitrate,
            read_timeout=float(self.conn_read_timeout_s),
            write_timeout=float(self.conn_write_timeout_s),
            tcp=self.conn_type == "tcp",
            max_retries=int(self.conn_max_retries),
            reconnect_delay=float(self.conn_reconnect_delay_s),
        )
        self.api = MSPApi(port=self.conn_str, baudrate=self.conn_bitrate, serial_transport=self.serial_transport)
        self.api_lock = threading.Lock()
        with self.api_lock:
            self.api.open()
            self.api_version = self.api.get_api_version()
            self.fc_variant = self.api.get_fc_variant()
            self.board_info = self.api.get_board_info()
            self.sensor_config = self.api.get_sensor_config()
            self.rx_config = self.api.get_rx_config()
            self.rx_map = self.api.get_rx_map()
            self.mode_ranges = self.api.get_mode_ranges()
        print(
            f"[PLUGIN_CONN] id={self.client_id} type={self.conn_type} conn={self.conn_str} baud={self.conn_bitrate}",
            flush=True,
        )

        self.mode_channels: Dict[str, Dict[str, Any]] = {}
        for entry in self.mode_ranges:
            aux_index = int(entry["auxChannelIndex"])
            channel_index = aux_index + 4
            channel_name = self.api.chmap[channel_index] if channel_index < len(self.api.chmap) else f"ch{channel_index + 1}"
            pwm_start, pwm_end = entry["pwmRange"]
            self.mode_channels[str(entry["mode"])] = {
                "channel": channel_name,
                "pwm": int((pwm_start + pwm_end) / 2),
            }

        self.receiver_config_model = ReceiverConfig(
            rx_min_usec=int(self.rx_config["rxMinUsec"]),
            rx_max_usec=int(self.rx_config["rxMaxUsec"]),
            rx_center_usec=int(self.rx_config["midRc"]),
        )
        self.channel_map_models = self._build_channel_map_models()
        self.mode_range_models = self._build_mode_range_models()
        self.sensor_config_model = self._build_sensor_config_model()

        self.arm_mode_name = "ARM"
        self.override_mode_name = "MSP RC OVERRIDE"
        self.takeoff_altitude_m: float | None = None
        self.control_override: ControlOverride | None = None
        self.control_override_lock = threading.Lock()
        self.control_override_updated_at = 0.0
        self.direct_control_mode: str | None = None
        self.latest_flight_control: VehicleControl | None = None
        self.latest_location: GlobalPositionInt | None = None
        self.latest_rc: ControlAxes | None = None

        self.request_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.loop_error: BaseException | None = None
        self.loop_error_trace: str | None = None
        self.loop_error_source: str | None = None
        self.loop_error_published = False
        self.worker_threads: Dict[str, threading.Thread] = {}
        self.shutdown_requested = False

        # Starting an adapter is observation, not acquisition of control authority.
        self._refresh_state()

    def _build_channel_map_models(self) -> list[RcChannelMapEntry]:
        axis_by_name = {
            "roll": "ROLL",
            "pitch": "PITCH",
            "yaw": "YAW",
            "throttle": "THROTTLE",
        }
        models: list[RcChannelMapEntry] = []
        for source_index, entry in sorted(self.rx_map.items(), key=lambda item: int(item[0])):
            name = str(entry.get("name", f"ch{int(source_index) + 1}"))
            models.append(
                RcChannelMapEntry(
                    axis=axis_by_name.get(name.lower(), "AUX"),
                    source_channel=int(source_index),
                    output_channel=None if entry.get("mappedTo") is None else int(entry["mappedTo"]),
                    label=name,
                )
            )
        return models

    def _build_mode_range_models(self) -> list[RcModeRange]:
        models: list[RcModeRange] = []
        for entry in self.mode_ranges:
            pwm_start, pwm_end = entry["pwmRange"]
            models.append(
                RcModeRange(
                    mode_id=None if entry.get("permanentId") is None else int(entry["permanentId"]),
                    mode_name=str(entry["mode"]),
                    channel=int(entry["auxChannelIndex"]) + 4,
                    pwm_start=float(pwm_start),
                    pwm_end=float(pwm_end),
                )
            )
        return models

    @staticmethod
    def _native_name(value: Any) -> str | None:
        if value is None:
            return None
        return str(getattr(value, "name", value))

    def _build_sensor_config_model(self) -> SensorConfig:
        return SensorConfig(
            accelerometer=self._native_name(self.sensor_config.get("accHardware")) or "",
            barometer=self._native_name(self.sensor_config.get("baroHardware")) or "",
            magnetometer=self._native_name(self.sensor_config.get("magHardware")) or "",
            airspeed=self._native_name(self.sensor_config.get("pitotHardware")) or "",
            rangefinder=self._native_name(self.sensor_config.get("rangefinderHardware")) or "",
            optical_flow=self._native_name(self.sensor_config.get("opflowHardware")) or "",
        )

    @staticmethod
    def _box_display_name(mode: Any) -> str:
        permanent_id = int(mode.value)
        definition = boxes.MODEBOXES.get(permanent_id)
        return mode.name if definition is None else str(definition["boxName"])

    def _capture_loop_error(self, source: str, exc: BaseException) -> None:
        if self.loop_error is not None:
            self.stop_event.set()
            return
        self.loop_error = exc
        self.loop_error_source = source
        self.loop_error_trace = traceback.format_exc().strip()
        print(
            f"[PLUGIN_ERROR] id={self.client_id} source={source} conn_type={self.conn_type} "
            f"conn={self.conn_str} reconnects={self.serial_transport.reconnects} serial_diag={self.serial_transport.last_diag}",
            flush=True,
        )
        print(self.loop_error_trace, flush=True)
        try:
            self.publish_error(self.loop_error_trace)
            self.loop_error_published = True
        except Exception as publish_error:
            print(f"[PLUGIN_ERROR] id={self.client_id} publish_error={publish_error}", flush=True)
        self.stop_event.set()

    def _stream_enabled(self, key: str) -> bool:
        return key in self.state_scheduler.topics

    def _publish_model(self, key: str, model: Any) -> None:
        if self._stream_enabled(key):
            self.state_scheduler.update(key, pack_record(model))

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

    def _respond(self, request_id: str, command: Any, ok: bool, data: Dict[str, Any] | None = None, error: str | None = None) -> None:
        payload = {} if data is None else dict(data)
        if error is not None:
            payload["error"] = error
        self.enqueue_response(request_id, type(command).__name__, ok, payload)

    def _activate_override(self) -> None:
        if self.override_mode_name in self.mode_channels:
            self._apply_mode(self.override_mode_name)

    def _deactivate_override(self) -> None:
        if self.override_mode_name in self.mode_channels:
            self._clear_mode(self.override_mode_name)

    def _apply_mode(self, mode_name: str) -> None:
        if mode_name not in self.mode_channels:
            raise UnsupportedCommand(f"mode {mode_name} not configured")
        channel = self.mode_channels[mode_name]["channel"]
        pwm = self.mode_channels[mode_name]["pwm"]
        with self.api_lock:
            self.api.set_rc_channels({channel: pwm})

    def _clear_mode(self, mode_name: str) -> None:
        if mode_name not in self.mode_channels:
            raise UnsupportedCommand(f"mode {mode_name} not configured")
        channel = self.mode_channels[mode_name]["channel"]
        with self.api_lock:
            self.api.set_rc_channels({channel: self.rx_config["rxMinUsec"]})

    def _find_mode(self, candidates: tuple[str, ...]) -> str:
        for candidate in candidates:
            if candidate in self.mode_channels:
                return candidate
        raise UnsupportedCommand(f"none of native modes configured candidates={candidates}")

    def _standard_mode_native_name(self, mode_name: str) -> str:
        if mode_name == "POSITION_HOLD":
            return self._find_mode(("NAV POSHOLD", "POSHOLD"))
        if mode_name == "MISSION":
            return self._find_mode(("NAV WP", "NAV_WP"))
        if mode_name == "ALTITUDE_HOLD":
            return self._find_mode(("NAV ALTHOLD", "ALTHOLD", "ALT HOLD"))
        if mode_name == "CRUISE":
            return self._find_mode(("NAV CRUISE", "CRUISE"))
        raise UnsupportedCommand(
            f"MSP mode command does not map standard mode {mode_name}; use dedicated semantic processes for RTL/land/takeoff"
        )

    def _set_standard_mode(self, mode_name: str, enabled: bool) -> None:
        native_name = self._standard_mode_native_name(mode_name)
        if enabled:
            self._apply_mode(native_name)
        else:
            self._clear_mode(native_name)

    def _set_throttle(self, value: int) -> None:
        with self.api_lock:
            self.api.set_rc_channels({"throttle": value})

    def _rc_channels(self, rc_channels: Any) -> RcChannels:
        if type(rc_channels) is dict:
            values = [float(rc_channels[name]) for name in self.api.chmap if name in rc_channels]
            count = len(values)
        elif type(rc_channels) is list:
            values = [float(value) for value in rc_channels]
            count = len(values)
        else:
            raise RuntimeError(f"unsupported rc_channels type {type(rc_channels).__name__}")
        return RcChannels(channel_count=count, channels=tuple(values))

    def _control_axes(self, rc_channels: Any) -> ControlAxes:
        if type(rc_channels) is dict:
            primary_names = set(self.override_channels.values())
            aux_channels = [
                channel_name
                for channel_name in self.api.chmap[4:]
                if channel_name not in primary_names and channel_name in rc_channels
            ]
            return rc_pwm_mapping_to_control_axes(
                rc_channels,
                roll_channel=self.override_channels["roll"],
                pitch_channel=self.override_channels["pitch"],
                yaw_channel=self.override_channels["yaw"],
                throttle_channel=self.override_channels["throt"],
                aux_channels=aux_channels,
                pwm_min_us=self.rx_config["rxMinUsec"],
                pwm_max_us=self.rx_config["rxMaxUsec"],
            )
        if type(rc_channels) is list:
            return rc_sequence_to_control_axes(
                rc_channels,
                pwm_min_us=self.rx_config["rxMinUsec"],
                pwm_max_us=self.rx_config["rxMaxUsec"],
            )
        raise RuntimeError(f"unsupported rc_channels type {type(rc_channels).__name__}")

    def _override_is_fresh(self) -> bool:
        return (
            self.direct_control_mode == DIRECT_CONTROL_MANUAL
            and self.control_override is not None
            and time.monotonic() - self.control_override_updated_at <= float(self.control_override_timeout_s)
        )

    def _merge_override(self, base: ControlAxes, override: ControlOverride) -> ControlAxes:
        values = {
            "roll": base.roll,
            "pitch": base.pitch,
            "yaw": base.yaw,
            "throttle": base.throttle,
        }
        for name in ("roll", "pitch", "yaw", "throttle"):
            value = getattr(override, name)
            if value is not None:
                values[name] = float(value)
        aux = list(base.aux)
        for channel in override.aux:
            index = int(channel.channel_index)
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

    def _build_override_channels(self) -> Dict[str, int]:
        with self.control_override_lock:
            override = self.control_override
        if override is None:
            return {}
        channels: Dict[str, int] = {}
        primary = {
            "roll": self.override_channels["roll"],
            "pitch": self.override_channels["pitch"],
            "yaw": self.override_channels["yaw"],
            "throttle": self.override_channels["throt"],
        }
        for field, channel_name in primary.items():
            value = getattr(override, field)
            if value is not None:
                channels[channel_name] = normalized_to_pwm(
                    float(value), self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]
                )
        for aux in override.aux:
            index = 4 + int(aux.channel_index)
            if aux.value is None or index >= len(self.api.chmap):
                continue
            channels[self.api.chmap[index]] = normalized_to_pwm(
                float(aux.value), self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]
            )
        return channels

    def _refresh_state(self) -> None:
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
        relative_alt_m = alt["estimatedAltitude"]
        is_in_air = is_armed and relative_alt_m is not None and relative_alt_m >= float(self.in_air_alt_threshold)
        global_ok = gps["fixType"] == InavEnums.gpsFixType_e.GPS_FIX_3D and gps["numSat"] >= int(self.home_min_satellites)
        active_modes = status["activeModes"]
        active_mode_names = [self._box_display_name(mode) for mode in active_modes]
        override_active = boxes.BoxEnum.BOXMSPRCOVERRIDE in active_modes
        failsafe = boxes.BoxEnum.BOXFAILSAFE in active_modes

        nav_validity = NavigationValidity(
            local_position_ok=relative_alt_m is not None,
            global_position_ok=bool(global_ok),
            home_position_ok=bool(global_ok),
            problems=(),
        )
        readiness = Readiness(
            armable=not failsafe,
            ekf_using_gps=False,
            arm_ready=False,
            takeoff_ready=False,
            problems=(),
        )
        runtime_load = RuntimeLoad(
            cpu_load_pct=None if status.get("cpuLoad") is None else float(status["cpuLoad"]),
            cycle_time_us=None if status.get("cycleTime") is None else float(status["cycleTime"]),
        )
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if is_armed else 0
        system_status = (
            mavutil.mavlink.MAV_STATE_ACTIVE
            if is_armed
            else mavutil.mavlink.MAV_STATE_STANDBY
        )
        heartbeat = Heartbeat(
            type=mavutil.mavlink.MAV_TYPE_GENERIC,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            base_mode=base_mode,
            custom_mode=0,
            system_status=system_status,
            mode_name=active_mode_names[0] if active_mode_names else "",
        )
        flight_control = VehicleControl(
            heartbeat=heartbeat,
            readiness=readiness,
            navigation_validity=nav_validity,
        )
        self.latest_flight_control = flight_control
        self._publish_model(FLIGHT_CONTROL, flight_control)
        self._publish_model(RUNTIME_LOAD, runtime_load)

        absolute_alt_m = gps.get("altitude")
        native_fix = gps["fixType"]
        fix_code = getattr(native_fix, "value", native_fix)
        location, gnss = gps_records(
            InavGpsFields(
                latitude_deg=float(gps["latitude"]),
                longitude_deg=float(gps["longitude"]),
                absolute_altitude_m=None if absolute_alt_m is None else float(absolute_alt_m),
                relative_altitude_m=None if relative_alt_m is None else float(relative_alt_m),
                fix_name=getattr(native_fix, "name", str(native_fix)),
                fix_code=int(fix_code),
                satellites_used=int(gps["numSat"]),
                ground_speed_m_s=None if gps.get("speed") is None else float(gps["speed"]),
                ground_course_deg=None if gps.get("groundCourse") is None else float(gps["groundCourse"]),
                hdop=None if gps_statistics.get("hdop") is None else float(gps_statistics["hdop"]),
            )
        )
        self.latest_location = location
        self._publish_model(LOCATION, location)
        self._publish_model(GNSS, gnss)

        active_waypoint = nav_status.get("activeWaypoint") or {}
        self._publish_model(
            AUTOPILOT_MISSION,
            MissionState(
                valid=bool(waypoint_info.get("missionValid")),
                current_waypoint=int(active_waypoint.get("number") or 0),
                waypoint_count=int(waypoint_info.get("waypointCount") or 0),
            ),
        )
        self._publish_model(SENSOR_CONFIG, self.sensor_config_model)

        attitude_state = attitude_from_degrees(
            float(attitude["roll"]),
            float(attitude["pitch"]),
            float(attitude["yaw"]),
        )
        self._publish_model(ATTITUDE, attitude_state)

        gyro = imu["gyro"]
        angular_velocity = angular_velocity_from_fru_degrees_s(
            float(gyro["X"]),
            float(gyro["Y"]),
            float(gyro["Z"]),
        )
        self._publish_model(ANGULAR_VELOCITY, angular_velocity)
        accel = imu.get("accel")
        self._publish_model(
            IMU,
            HighresImu(
                xacc=None if accel is None else float(accel["X"]),
                yacc=None if accel is None else float(accel["Y"]),
                zacc=None if accel is None else float(accel["Z"]),
                xgyro=angular_velocity.x_rad_s,
                ygyro=angular_velocity.y_rad_s,
                zgyro=angular_velocity.z_rad_s,
            ),
        )

        power = BatteryStatus(
            voltage_v=None if analog.get("vbat") is None else float(analog["vbat"]),
            current_a=None if analog.get("amperage") is None else float(analog["amperage"]),
            remaining_pct=(
                None
                if analog.get("percentageRemaining") is None
                else float(analog["percentageRemaining"])
            ),
            consumed_mah=(
                None if analog.get("mAhDrawn") is None else float(analog["mAhDrawn"])
            ),
            consumed_wh=(
                None if analog.get("mWhDrawn") is None else float(analog["mWhDrawn"])
            ),
        )
        self._publish_model(POWER, power)

        rc = self._control_axes(rc_channels)
        self.latest_rc = rc
        self._publish_model(RC_TELEMETRY, self._rc_channels(rc_channels))
        with self.control_override_lock:
            override = self.control_override
        if override is not None:
            self._publish_model(CONTROL_OVERRIDE, override)
        output = self._merge_override(rc, override) if override is not None and self._override_is_fresh() else rc
        self._publish_model(CONTROL_OUTPUT, output)
        self._publish_model(
            REMOTE_CONTROL,
            RemoteControlState(
                rc_telemetry=self._rc_channels(rc_channels),
                control_output=output,
                control_override=override,
                receiver_config=self.receiver_config_model,
                channel_map=tuple(self.channel_map_models),
                mode_ranges=tuple(self.mode_range_models),
            ),
        )

    def _state_loop(self) -> None:
        try:
            while not self.stop_event.is_set():
                self._refresh_state()
                time.sleep(float(self.state_poll_interval_s))
        except BaseException as exc:
            self._capture_loop_error("state_loop", exc)

    def _override_loop(self) -> None:
        try:
            last_send = 0.0
            while not self.stop_event.is_set():
                elapsed = time.monotonic() - last_send
                if elapsed < float(self.override_send_interval):
                    time.sleep(float(self.override_send_interval) - elapsed)
                    continue
                last_send = time.monotonic()
                if not self._override_is_fresh():
                    continue
                if self.latest_flight_control is None or not bool(
                    self.latest_flight_control.heartbeat.armed
                ):
                    continue
                channels = self._build_override_channels()
                if channels:
                    with self.api_lock:
                        self.api.set_rc_channels(channels)
        except BaseException as exc:
            self._capture_loop_error("override_loop", exc)

    def _set_goto(self, command: RepositionCommand) -> None:
        if command.altitude_reference != "relative":
            raise UnsupportedCommand(
                f"INAV MOVE_TO currently requires relative altitude actual={command.altitude_reference}"
            )
        waypoint_index = int(self.go_to_waypoint["WaypointIndex"])
        action_enum = InavEnums.navWaypointActions_e(int(self.go_to_waypoint["Action"]))
        with self.api_lock:
            self.api.set_waypoint(
                waypointIndex=waypoint_index,
                action=action_enum,
                latitude=float(command.latitude_deg),
                longitude=float(command.longitude_deg),
                altitude=float(command.altitude_m),
                param1=int(self.go_to_waypoint["Param1"]),
                param2=int(self.go_to_waypoint["Param2"]),
                param3=int(self.go_to_waypoint["Param3"]),
                flag=int(self.go_to_waypoint["Flag"]),
            )

    def _begin_direct_control(self) -> None:
        if self.direct_control_mode is not None and self.direct_control_mode != DIRECT_CONTROL_MANUAL:
            raise UnsupportedCommand(
                f"direct-control process already active mode={self.direct_control_mode}; stop it before switching"
            )
        self.direct_control_mode = DIRECT_CONTROL_MANUAL
        self._activate_override()

    def _end_direct_control(self) -> None:
        self.direct_control_mode = None
        with self.control_override_lock:
            self.control_override = None
            self.control_override_updated_at = 0.0
        self._deactivate_override()

    def _handle_input(self, payload: Any) -> None:
        model = decode_input(payload)
        if isinstance(model, ControlOverride):
            if self.direct_control_mode != DIRECT_CONTROL_MANUAL:
                self._publish_input_rejected(model, "ControlOverride requires active MANUAL_AXIS direct-control process")
                return
            with self.control_override_lock:
                self.control_override = model
                self.control_override_updated_at = time.monotonic()
            return
        self._publish_input_rejected(model, f"unsupported MSP direct input {type(model).__name__}")

    def _handle_arm(self, command: CommandLong) -> None:
        armed = float(command.params[0]) == 1.0
        if armed:
            self._activate_override()
            self._apply_mode(self.arm_mode_name)
            self._set_throttle(self.rx_config["rxMinUsec"])
        else:
            self._clear_mode(self.arm_mode_name)

    def _handle_set_mode(self, command: SetMode) -> None:
        if command.mode_name:
            mode_name = command.mode_name.upper().replace(" ", "_")
            if mode_name in {
                "POSITION_HOLD",
                "MISSION",
                "ALTITUDE_HOLD",
                "CRUISE",
            }:
                self._set_standard_mode(standard_mode_name(mode_name), True)
                return
            self._apply_mode(command.mode_name)
            return
        raise UnsupportedCommand("MSP adapter does not select native modes by numeric code")

    def _handle_command_long(self, command: CommandLong) -> None:
        if command.command == MAV_CMD_COMPONENT_ARM_DISARM:
            self._handle_arm(command)
            return
        if command.command == MAV_CMD_NAV_TAKEOFF:
            self._takeoff()
            return
        if command.command == MAV_CMD_NAV_LAND:
            self._land()
            return
        if command.command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self._apply_mode(self._find_mode(("NAV RTH", "RTH", "NAV_RTH")))
            return
        if command.command == MAV_CMD_NAV_LOITER_UNLIM:
            self._set_standard_mode("POSITION_HOLD", True)
            return
        raise UnsupportedCommand(
            f"unsupported MAV_CMD {command.command} for MSP endpoint"
        )

    def _handle_command(self, request: Dict[str, Any]) -> None:
        request_id, command = decode_command(request)
        try:
            if isinstance(command, CommandLong):
                self._handle_command_long(command)
            elif isinstance(command, SetMode):
                self._handle_set_mode(command)
            elif isinstance(command, ParamSet):
                if command.param_id != PARAM_TAKEOFF_ALTITUDE:
                    raise UnsupportedCommand(
                        f"unsupported MSP parameter {command.param_id!r}"
                    )
                self.takeoff_altitude_m = float(command.value)
            elif isinstance(command, RepositionCommand):
                self._set_goto(command)
            elif isinstance(command, DirectControl):
                if command.enabled:
                    if command.mode != DIRECT_CONTROL_MANUAL:
                        raise UnsupportedCommand(
                            f"MSP adapter supports MANUAL_AXIS direct control only actual={command.mode}"
                        )
                    self._begin_direct_control()
                else:
                    self._end_direct_control()
            else:
                raise UnsupportedCommand(
                    f"unsupported command record {type(command).__name__} for MSP endpoint"
                )
            self._respond(request_id, command, True)
        except (UnsupportedCommand, ValueError, TypeError) as exc:
            self._respond(request_id, command, False, error=str(exc))

    def _process_requests(self) -> None:
        try:
            while not self.stop_event.is_set():
                try:
                    request = self.request_queue.get(timeout=REQUEST_QUEUE_TIMEOUT_S)
                except queue.Empty:
                    continue
                self._handle_command(request)
        except BaseException as exc:
            self._capture_loop_error("process_requests", exc)

    def _takeoff(self) -> None:
        if self.takeoff_altitude_m is None:
            raise UnsupportedCommand("takeoff altitude not set")
        self._activate_override()
        self._apply_mode(self.arm_mode_name)
        start = time.monotonic()
        while not self.stop_event.is_set():
            self._set_throttle(int(self.takeoff_throttle))
            altitude_m = None
            if self.latest_location is not None:
                altitude_m = self.latest_location.relative_altitude_m
            if altitude_m is not None and altitude_m >= self.takeoff_altitude_m:
                self._set_throttle(int(self.hover_throttle))
                return
            if time.monotonic() - start > float(self.takeoff_timeout_s):
                raise UnsupportedCommand("takeoff timeout")
            time.sleep(float(self.state_poll_interval_s))

    def _land(self) -> None:
        start = time.monotonic()
        self._activate_override()
        self._apply_mode(self.arm_mode_name)
        while not self.stop_event.is_set():
            self._set_throttle(int(self.landing_throttle))
            altitude_m = None
            if self.latest_location is not None:
                altitude_m = self.latest_location.relative_altitude_m
            if altitude_m is not None and altitude_m <= float(self.in_air_alt_threshold):
                self._set_throttle(self.rx_config["rxMinUsec"])
                self._clear_mode(self.arm_mode_name)
                return
            if time.monotonic() - start > float(self.landing_timeout_s):
                raise UnsupportedCommand("landing timeout")
            time.sleep(float(self.state_poll_interval_s))

    def run(self) -> None:
        if not self.worker_threads:
            self.stop_event.clear()
            for name, target in {
                "msp-request": self._process_requests,
                "msp-state": self._state_loop,
                "msp-override": self._override_loop,
            }.items():
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
                if topic == self.request_topic:
                    self.request_queue.put(payload["data"])
                elif topic == self.input_topic:
                    try:
                        self._handle_input(payload["data"])
                    except (TypeError, ValueError, KeyError) as exc:
                        self._publish_input_rejected(payload["data"], str(exc))
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
                trace = self.loop_error_trace or traceback.format_exception_only(type(self.loop_error), self.loop_error)[-1].strip()
                if not self.shutdown_requested and not self.loop_error_published:
                    self.publish_error(trace)
                raise self.loop_error
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MspInterface(cfg, bus_config).run()
