#!/usr/bin/env python3
"""
pymavlink backend bridge for PX4, ArduPilot and INAV.

Usage:
    from plugins.mavlink_interface.mavlink_interface import run_plugin
    run_plugin(cfg, bus_config)

Receive-only: the plugin never writes to the vehicle link (no heartbeat, no
stream requests, no commands). Telemetry rates are whatever the FC or router
already sends. Messages are accepted only from the bound source sysid/compid,
so several runtimes can share one router or radio.
"""

import threading
import time
import traceback
from typing import Any, Dict

from pymavlink import mavutil

from lib.common import (
    apply_cfg,
    build_request_topic,
    build_response_topic,
    build_state_scheduler_topics,
    build_topic_base,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from plugins.mavlink_interface.source import MavSnapshot, MavSourceBinding, SourceSpec
from protocols.namespace_loader import load_protocol_namespace

POLL_INTERVAL_S = 0.1
RECV_TIMEOUT_S = 0.2
UAV = load_protocol_namespace("uav")


class MavlinkInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Configure pymavlink interface plugin.
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.source = SourceSpec.parse(self.conn_str)
        self.snapshot = MavSnapshot(binding=MavSourceBinding(sysid=self.target_sysid, compid=self.target_compid))
        self.fc_connected: bool | None = None

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(POLL_INTERVAL_S)
        state_topics = build_state_scheduler_topics(base, self.state_intervals)
        self.state_scheduler = StateScheduler(self.client, self.client_id, state_topics)

        self.stop_event = threading.Event()
        self.reader_error: BaseException | None = None
        self.reader_error_trace: str | None = None
        self.reader_thread: threading.Thread | None = None

    def _update_state(self, key: str, value: Any) -> None:  # Update state only when the field is configured.
        if key not in self.state_scheduler.topics:
            return
        self.state_scheduler.update(key, value)

    def _publish_message_state(self, kind: str) -> None:  # Map one consumed MAVLink message to UAV state.
        snap = self.snapshot
        if kind == "HEARTBEAT":
            self._update_state(UAV.State.Flight.IsArmed, bool(snap.armed))
            if snap.mode_name is not None:
                self._update_state(UAV.State.System.FlightMode, {"FlightMode": snap.mode_name})
                self._update_state(UAV.State.Flight.ActiveModeNames, [snap.mode_name])
        elif kind == "GLOBAL_POSITION_INT":
            self._update_state(UAV.State.Navigation.AltitudeM, snap.relative_alt_m)
            self._update_state(
                UAV.State.Navigation.Position,
                {"LatDeg": snap.lat, "LonDeg": snap.lon, "AbsAltM": snap.absolute_alt_m, "RelAltM": snap.relative_alt_m},
            )
        elif kind == "ATTITUDE":
            self._update_state(UAV.State.Attitude.AttitudeRad, {"Roll": snap.roll_rad, "Pitch": snap.pitch_rad, "Yaw": snap.yaw_rad})
            self._update_state(
                UAV.State.Attitude.AngVelRadS,
                {"X": snap.roll_rate_rad_s, "Y": snap.pitch_rate_rad_s, "Z": snap.yaw_rate_rad_s},
            )
        elif kind == "GPS_RAW_INT":
            self._update_state(UAV.State.Navigation.FixType, snap.gnss_fix)
            self._update_state(UAV.State.Navigation.NumSat, snap.gnss_sats)
            self._update_state(UAV.State.Navigation.GpsInfo, {"FixType": snap.gnss_fix, "NumSat": snap.gnss_sats, "hdop": snap.gnss_hdop})
            if snap.gnss_lat is not None:
                self._update_state(
                    UAV.State.Navigation.RawGps,
                    {
                        "LatDeg": snap.gnss_lat,
                        "LonDeg": snap.gnss_lon,
                        "AbsAltM": snap.gnss_alt_m,
                        "GroundSpeedMS": snap.gnss_groundspeed_m_s,
                        "GroundCourseDeg": snap.gnss_course_deg,
                        "Hdop": snap.gnss_hdop,
                        "Vdop": snap.gnss_vdop,
                        "YawDeg": snap.gnss_yaw_deg,
                    },
                )
        elif kind in ("SYS_STATUS", "BATTERY_STATUS"):
            battery = snap.battery
            self._update_state(
                UAV.State.Power.Battery,
                {
                    "VoltageV": battery.voltage_v,
                    "CurrentA": battery.current_a,
                    "ConsumedMah": battery.consumed_mah,
                    "RemainingPct": battery.remaining_pct,
                    "TemperatureDegC": battery.temperature_c,
                    "BatteryId": battery.battery_id,
                },
            )
        elif kind == "EXTENDED_SYS_STATE" and snap.in_air is not None:
            self._update_state(UAV.State.Flight.IsInAir, snap.in_air)
        elif kind == "HOME_POSITION":
            self._update_state(UAV.State.Navigation.IsHomePositionOk, True)

    def _sync_link_state(self) -> None:  # Publish FcConnected from bound-source message freshness.
        last = self.snapshot.last_received_monotonic
        connected = last is not None and time.monotonic() - last <= float(self.link_stale_s)
        if connected != self.fc_connected:
            print(
                f"[PLUGIN] {self.client_id} fc_connected={connected} sysid={self.snapshot.sysid} "
                f"compid={self.snapshot.compid} autopilot={self.snapshot.autopilot_name} "
                f"type={self.snapshot.mav_type_name} source={self.source.raw}",
                flush=True,
            )
            self.fc_connected = connected
        self._update_state(UAV.State.System.FcConnected, connected)

    def _reader_main(self) -> None:  # Read the vehicle link; never writes to it.
        kwargs: Dict[str, Any] = {"autoreconnect": False, "robust_parsing": True}
        if self.source.baud is not None:
            kwargs["baud"] = self.source.baud
        try:
            connection = mavutil.mavlink_connection(self.source.connection, **kwargs)
            try:
                while not self.stop_event.is_set():
                    message = connection.recv_match(blocking=True, timeout=RECV_TIMEOUT_S)
                    if message is not None and self.snapshot.consume(message):
                        self._publish_message_state(message.get_type())
            finally:
                connection.close()
        except BaseException as exc:
            self.reader_error = exc
            self.reader_error_trace = traceback.format_exc().strip()

    def _handle_request(self, request: Dict[str, Any]) -> None:  # Refuse actions; no transmit path exists.
        self.enqueue_response(
            str(request["request_id"]),
            request["action"],
            False,
            {"error": f"{self.client_id} is receive-only; action {request['action']} not sent"},
        )

    def run(self) -> None:  # Run the plugin main loop.
        print(f"[PLUGIN] {self.client_id} receive-only source={self.source.raw} target={self.target_sysid}/{self.target_compid}", flush=True)
        self.reader_thread = threading.Thread(target=self._reader_main, name="mavlink-reader", daemon=True)
        self.reader_thread.start()
        self.send_online()
        try:
            while True:
                if self.reader_error is not None:
                    raise RuntimeError(f"mavlink reader failed source={self.source.raw}") from self.reader_error
                self._sync_link_state()
                self.state_scheduler.flush()
                self.flush_queue(self.response_queue, self.response_topic)
                try:
                    topic, payload = self._pump_once()
                except SystemExit:
                    break
                if topic == self.request_topic:
                    self._handle_request(payload["data"])
        except RuntimeError:
            self.publish_error(self.reader_error_trace or traceback.format_exc().strip())
            raise
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            self.reader_thread.join(timeout=2.0)
            self.flush_queue(self.response_queue, self.response_topic)
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Entry point for plugin runner.
    MavlinkInterface(cfg, bus_config).run()
