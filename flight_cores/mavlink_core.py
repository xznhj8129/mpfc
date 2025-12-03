#!/usr/bin/env python3
"""
MAVLink mode plugin for main_uav: maintains MAVLink connection, handles AP commands, emits HL telemetry.
"""

import time
from typing import Any, Dict, Optional

from pymavlink import mavutil

CLIENT_ID = "FlightController"
MAVLINK_TOPIC = "MAVLINK"
DATALINK_IN_TOPIC = "Datalink.IN"
DATALINK_OUT_TOPIC = "Datalink.OUT"


class MavlinkAP:
    def __init__(self, conn_str: str) -> None:
        self.conn_str = conn_str
        self.master: Optional[mavutil.mavlink_connection] = None
        self.last_hb: float = 0.0
        self.mode_str: str = "UNKNOWN"
        self.airspeed: int = 0
        self.groundspeed: int = 0
        self.heading: int = 0
        self.msl_alt: int = 0
        self.lat: Optional[float] = None
        self.lon: Optional[float] = None

    def connect(self) -> None:
        self.master = mavutil.mavlink_connection(self.conn_str, autoreconnect=True)
        try:
            self.master.wait_heartbeat(timeout=5)
            self.last_hb = time.time()
            self._update_mode()
            print(
                f"[MAV] Connected sysid={self.master.target_system} compid={self.master.target_component} mode={self.mode_str}"
            )
        except Exception as exc:
            self.master = None
            raise RuntimeError(f"MAVLink heartbeat not received: {exc}")

    def poll(self) -> None:
        if self.master is None:
            self.connect()
        if self.master is None:
            return
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "BAD_DATA":
                continue
            if mtype == "HEARTBEAT":
                self.last_hb = time.time()
                self._update_mode()
            elif mtype == "VFR_HUD":
                self.groundspeed = int(msg.groundspeed or 0)
                self.airspeed = int(msg.airspeed or 0)
                self.heading = int(msg.heading or 0)
                self.msl_alt = int(msg.alt or 0)
            elif mtype == "GLOBAL_POSITION_INT":
                if msg.lat is not None and msg.lon is not None:
                    self.lat = msg.lat / 1e7
                    self.lon = msg.lon / 1e7

    def arm(self, arm: bool) -> None:
        self._ensure_connected()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def set_mode(self, mode_str: str) -> bool:
        self._ensure_connected()
        mode = mode_str.strip().upper()
        mapping = self.master.mode_mapping()
        if not mapping or mode not in mapping:
            print(f"[MAV] Mode '{mode}' unsupported; mapping={mapping}")
            return False
        mode_id = mapping[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        return True

    def takeoff(self, alt_m: int, yaw_deg: float = float("nan")) -> None:
        self._ensure_connected()
        self.arm(True)
        self.set_mode("GUIDED")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            (0.0 if not (yaw_deg == yaw_deg) else yaw_deg),
            0,
            0,
            float(alt_m),
        )

    def land(self) -> None:
        self._ensure_connected()
        self.set_mode("LAND")

    def select_mission(self, seq: int) -> None:
        self._ensure_connected()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            0,
            float(seq),
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.set_mode("AUTO")

    def stop(self) -> None:
        if self.master is None:
            return
        try:
            self.master.close()
        except Exception:
            pass
        self.master = None

    def _ensure_connected(self) -> None:
        if self.master is None:
            raise RuntimeError("MAVLink not connected")

    def _update_mode(self) -> None:
        try:
            self.mode_str = str(getattr(self.master, "flightmode", None) or "UNKNOWN")
        except Exception:
            self.mode_str = "UNKNOWN"


class MavlinkPlugin:
    def __init__(self, client_id: str, mav_cfg: Dict[str, Any], datalink_cfg: Dict[str, Any], telem_topic: str) -> None:
        self.client_id = client_id
        self.telem_topic = telem_topic
        self.ap = MavlinkAP(mav_cfg["conn_str"])
        self.datalink_cfg = datalink_cfg
        self.period = 1.0 / float(mav_cfg["hl_rate_hz"])
        self.next_telem: Optional[float] = None

    def start(self) -> None:
        self.ap.connect()
        self.next_telem = time.monotonic() + self.period

    def tick(self, client) -> None:
        self.ap.poll()
        now = time.monotonic()
        if self.next_telem is None or now < self.next_telem:
            return
        if self.ap.lat is None or self.ap.lon is None:
            self.next_telem = now + self.period
            return
        telem_payload = {
            "mode_str": self.ap.mode_str,
            "airspeed": int(self.ap.airspeed),
            "groundspeed": int(self.ap.groundspeed),
            "heading": int(self.ap.heading),
            "msl_alt": int(self.ap.msl_alt),
            "lat": int(self.ap.lat * 1e7),
            "lon": int(self.ap.lon * 1e7),
        }
        envelope = {
            "client": self.client_id,
            "topic": self.telem_topic,
            "time": int(time.time() * 1000),
            "data": {
                "destination": self.datalink_cfg["destination"],
                "message_path": "Status.AP.HL_TELEM",
                "payload": telem_payload,
                "transport": self.datalink_cfg["transport"],
            },
        }
        client.publish(self.telem_topic, envelope)
        print(
            f"[HL_TELEM] lat={telem_payload['lat']} lon={telem_payload['lon']} alt={telem_payload['msl_alt']} gs={telem_payload['groundspeed']}",
            flush=True,
        )
        self.next_telem = now + self.period

    def handle_bus_message(self, topic: str, payload: Dict[str, Any], message: Dict[str, Any]) -> bool:
        if topic != DATALINK_IN_TOPIC: # wrong, we do not reject all messages
            return False
        data_field = payload.get("payload")
        if not isinstance(data_field, dict):
            raise TypeError("datalink payload missing payload object")

        msg_name = payload.get("message")

        if msg_name == "Command.AP.ARM":
            arm_flag = int(data_field.get("arm", 1)) != 0
            self.ap.arm(arm_flag)
            print(f"[CMD] ARM={arm_flag}", flush=True)
            return True
        if msg_name == "Command.AP.DISARM":
            self.ap.arm(False)
            print("[CMD] DISARM", flush=True)
            return True
        if msg_name == "Command.AP.SET_MODE":
            mode_str = data_field.get("mode_str", "")
            if isinstance(mode_str, (bytes, bytearray)):
                mode_str = mode_str.decode("utf-8", errors="ignore")
            ok = self.ap.set_mode(str(mode_str))
            print(f"[CMD] SET_MODE {mode_str} -> {ok}", flush=True)
            return True
        if msg_name == "Command.AP.TAKEOFF":
            alt_m = int(data_field.get("alt_m", 10))
            self.ap.takeoff(alt_m)
            print(f"[CMD] TAKEOFF alt={alt_m}", flush=True)
            return True
        if msg_name == "Command.AP.LAND":
            self.ap.land()
            print("[CMD] LAND", flush=True)
            return True
        if msg_name == "Command.AP.SELECT_MISSION":
            seq = int(data_field.get("seq", 0))
            self.ap.select_mission(seq)
            print(f"[CMD] SELECT_MISSION seq={seq}", flush=True)
            return True
        return False

    def stop(self) -> None:
        self.ap.stop()
