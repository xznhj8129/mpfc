#!/usr/bin/env python3
"""
Usage:
    from flight_cores.mavlink_core import run_core
    run_core(cfg, bus_config)
"""

import base64
import os
import time
import traceback
from typing import Any, Dict

from pymavlink import mavutil

from lib.common import CONTROL_SHUTDOWN_TOPIC, build_envelope
from lib.core_base import CoreBase

MAVLINK_TOPIC = "MAVLINK"
POLL_INTERVAL = 0.05
GUIDED_CUSTOM_MODE = 4
GCS_SYSID = 245
GCS_COMPID = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
TARGET_COMPID = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
ALT_IN_AIR = 0.5
ALT_ON_GROUND = 0.2
HOME_REQ_INTERVAL = 1.0


class MavlinkCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.target_sysid = int(cfg["sysid"])
        self.takeoff_altitude = float(cfg["takeoff_altitude_m"])
        self.hold_duration = float(cfg["post_takeoff_hold_s"])
        self.bus_topic = MAVLINK_TOPIC
        self.client.subscribe(self.bus_topic)
        self.encoder = mavutil.mavlink.MAVLink(None, srcSystem=GCS_SYSID, srcComponent=GCS_COMPID)
        self.encoder.use_mavlink2 = True
        self.parser = mavutil.mavlink.MAVLink(None)
        self.parser.use_mavlink2 = True
        self.parser.robust_parsing = True
        self.connected = False
        self.guided_mode = False
        self.global_position_ok = False
        self.home_position_ok = False
        self.armed = False
        self.in_air = False
        self.altitude_m = None
        self.last_mode = None
        self.takeoff_started_at = None
        self.stage = 0
        self.sequence_done = False
        self.last_frame_time = None
        self.last_frame_meta = None
        self.set_mode_sent = False
        self.arm_sent = False
        self.takeoff_sent = False
        self.land_sent = False
        self.arm_ack = False
        self.takeoff_ack = False
        self.home_request_at = None

    def _request_message(self, msg_id: int) -> Any:
        return self._cmd_long(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, (msg_id, 0, 0, 0, 0, 0, 0))

    def _cmd_long(self, command: int, params: tuple[float, float, float, float, float, float, float]) -> Any:
        p1, p2, p3, p4, p5, p6, p7 = params
        return self.encoder.command_long_encode(
            self.target_sysid,
            TARGET_COMPID,
            command,
            0,
            p1,
            p2,
            p3,
            p4,
            p5,
            p6,
            p7,
        )

    def run(self) -> None:
        try:
            while True:
                topic, payload, message = self.recv_message(POLL_INTERVAL)
                if topic == self.bus_topic:
                    src_client = payload["client"] or message.get("src")
                    if src_client != self.client_id:
                        frame = base64.b64decode(payload["data"]["frame"])
                        self._handle_frame(frame)

                if self.stage == 0:
                    if self.connected and self.global_position_ok:
                        now = time.monotonic()
                        if not self.home_position_ok:
                            if self.home_request_at is None or now - self.home_request_at >= HOME_REQ_INTERVAL:
                                req_home = self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION)
                                req_origin = self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)
                                get_home = self._cmd_long(mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, (0, 0, 0, 0, 0, 0, 0))
                                self._publish_frame(req_home)
                                self._publish_frame(req_origin)
                                self._publish_frame(get_home)
                                self.home_request_at = now
                                print(f"[CORE_CMD] id={self.client_id} cmd=REQ_HOME", flush=True)
                        if self.home_position_ok:
                            print(
                                f"[CORE_HEALTH] id={self.client_id} global_ok={self.global_position_ok} home_ok={self.home_position_ok}",
                                flush=True,
                            )
                            self.stage = 1
                elif self.stage == 1:
                    if not self.set_mode_sent:
                        base_mode = (
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                            | mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
                        )
                        msg = self.encoder.set_mode_encode(self.target_sysid, base_mode, GUIDED_CUSTOM_MODE)
                        self._publish_frame(msg)
                        self.set_mode_sent = True
                        print(
                            f"[CORE_CMD] id={self.client_id} cmd=SET_MODE base_mode={base_mode} custom_mode={GUIDED_CUSTOM_MODE}",
                            flush=True,
                        )
                    if self.guided_mode:
                        self.stage = 2
                elif self.stage == 2:
                    if self.armed or self.arm_ack:
                        self.stage = 3
                    else:
                        if not self.arm_sent:
                            msg = self._cmd_long(
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                (1, 0, 0, 0, 0, 0, 0),
                            )
                            self._publish_frame(msg)
                            self.arm_sent = True
                            print(f"[CORE_CMD] id={self.client_id} cmd=ARM target_sysid={self.target_sysid}", flush=True)
                elif self.stage == 3:
                    if self.armed and not self.takeoff_sent:
                        msg = self._cmd_long(
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            (0, 0, 0, 0, 0, 0, self.takeoff_altitude),
                        )
                        self._publish_frame(msg)
                        self.takeoff_sent = True
                        print(
                            f"[CORE_CMD] id={self.client_id} cmd=TAKEOFF alt_m={self.takeoff_altitude} hold_s={self.hold_duration}",
                            flush=True,
                        )
                    if self.takeoff_sent and (self.takeoff_ack or self.in_air):
                        self.stage = 4
                elif self.stage == 4:
                    if self.in_air and self.takeoff_started_at is None:
                        self.takeoff_started_at = time.monotonic()
                    if self.in_air and self.takeoff_started_at is not None:
                        if time.monotonic() - self.takeoff_started_at >= self.hold_duration:
                            if not self.land_sent:
                                msg = self._cmd_long(
                                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                                    (0, 0, 0, 0, 0, 0, 0),
                                )
                                self._publish_frame(msg)
                                self.land_sent = True
                                print(f"[CORE_CMD] id={self.client_id} cmd=LAND target_sysid={self.target_sysid}", flush=True)
                            self.stage = 5
                elif self.stage == 5:
                    if not self.in_air and not self.armed:
                self.sequence_done = True

        if self.sequence_done:
            self.client.publish(CONTROL_SHUTDOWN_TOPIC, build_envelope(self.client_id, CONTROL_SHUTDOWN_TOPIC, {}))
            break
        except RuntimeError:
            crash_tb = traceback.format_exc().strip()
            print(f"[CORE_CRASH] id={self.client_id} error={crash_tb}", flush=True)
            error_topic = f"DIAG.{self.client_id}.ERROR"
            error_payload = build_envelope(
                self.client_id, error_topic, {"event": "ERROR", "traceback": crash_tb}
            )
            self.client.publish(error_topic, error_payload)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _handle_frame(self, frame: bytes) -> None:
        msg = None
        for byte in frame:
            candidate = self.parser.parse_char(bytes([byte]))
            if candidate is not None:
                msg = candidate
        if msg is None:
            return
        self.last_frame_time = time.monotonic()
        self.last_frame_meta = (msg.get_type(), msg.get_srcSystem(), msg.get_srcComponent())
        if msg.get_srcSystem() != self.target_sysid:
            return
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            self.connected = True
            current_mode = mavutil.mode_string_v10(msg)
            self.guided_mode = current_mode == "GUIDED"
            if current_mode != self.last_mode:
                self.last_mode = current_mode
                print(f"[CORE_MODE] id={self.client_id} mode={current_mode}", flush=True)
            armed_now = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if armed_now != self.armed:
                self.armed = armed_now
                print(f"[CORE_ARM_STATE] id={self.client_id} armed={self.armed}", flush=True)
            return
        if msg_type == "GLOBAL_POSITION_INT":
            altitude = round(msg.relative_alt / 1000.0, 3)
            if altitude != self.altitude_m:
                self.altitude_m = altitude
                print(f"[CORE_ALT] id={self.client_id} altitude_m={altitude}", flush=True)
            if altitude > ALT_IN_AIR and not self.in_air:
                self.in_air = True
                print(f"[CORE_AIR] id={self.client_id} in_air=True", flush=True)
                if self.takeoff_started_at is None:
                    self.takeoff_started_at = time.monotonic()
            if altitude <= ALT_ON_GROUND and self.in_air:
                self.in_air = False
                print(f"[CORE_AIR] id={self.client_id} in_air=False", flush=True)
            if msg.lat != 0 or msg.lon != 0:
                self.global_position_ok = True
            return
        if msg_type == "EXTENDED_SYS_STATE":
            landed_state = msg.landed_state
            in_air_now = landed_state in (
                mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF,
                mavutil.mavlink.MAV_LANDED_STATE_FLYING,
            )
            if in_air_now != self.in_air:
                self.in_air = in_air_now
                print(f"[CORE_AIR] id={self.client_id} in_air={self.in_air}", flush=True)
                if self.in_air and self.takeoff_started_at is None:
                    self.takeoff_started_at = time.monotonic()
            return
        if msg_type == "HOME_POSITION" or msg_type == "GPS_GLOBAL_ORIGIN":
            self.home_position_ok = True
            print(f"[CORE_HOME] id={self.client_id} source={msg_type}", flush=True)
            return
        if msg_type == "COMMAND_ACK":
            command = msg.command
            result = msg.result
            result_name = mavutil.mavlink.enums.get("MAV_RESULT", {}).get(result)
            result_text = result_name.name if result_name else str(result)
            if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                print(f"[CORE_ACK] id={self.client_id} cmd=ARM result={result_text}", flush=True)
                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.arm_ack = True
                else:
                    raise RuntimeError(f"ARM failed result={result_text}")
            elif command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                print(f"[CORE_ACK] id={self.client_id} cmd=TAKEOFF result={result_text}", flush=True)
                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.takeoff_ack = True
                else:
                    raise RuntimeError(f"TAKEOFF failed result={result_text}")
            return

    def _publish_frame(self, msg: Any) -> None:
        buf = msg.pack(self.encoder)
        envelope = build_envelope(
            self.client_id,
            self.bus_topic,
            {
                "frame": base64.b64encode(buf).decode("ascii"),
                "length": len(buf),
                "msgid": msg.get_msgId(),
                "type": msg.get_type(),
                "sysid": self.encoder.srcSystem,
                "compid": self.encoder.srcComponent,
            },
        )
        self.client.publish(self.bus_topic, envelope)


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MavlinkCore(cfg, bus_config).run()
