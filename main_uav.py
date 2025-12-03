#!/usr/bin/env python3
"""
Usage:
    python main_uav.py
Reads process_config.json beside this file for all settings; no CLI arguments are used.
"""

import json
import queue
import time
from pathlib import Path
from typing import Any, Dict, Optional

from message_bus_client import BusClientSync
from pymavlink import mavutil

DATALINK_IN_TOPIC = "Datalink.IN"
DATALINK_OUT_TOPIC = "Datalink.OUT"
MSP_REQUEST_TOPIC = "MSP.REQUEST"
MSP_REPLY_TOPIC = "MSP.REPLY"
RECEIVE_TIMEOUT_S = 0.1
CONFIG_PATH = Path(__file__).resolve().with_name("process_config.json")
CLIENT_ID = "main_uav"


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


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def build_bus_envelope(client_id: str, topic: str, data: Any) -> Dict[str, Any]:
    if not client_id:
        raise ValueError("client_id is required for envelope")
    if not topic:
        raise ValueError("topic is required for envelope")
    timestamp_ms = int(time.time() * 1000)
    return {
        "client": client_id,
        "topic": topic,
        "time": timestamp_ms,
        "data": data,
    }


def main() -> None:
    if not CONFIG_PATH.is_file():
        raise FileNotFoundError(f"process config not found: {CONFIG_PATH}")
    process_config = load_json(CONFIG_PATH)
    if "bus_config_path" not in process_config or "schema_path" not in process_config or "main_uav" not in process_config:
        raise KeyError("process_config missing required keys: bus_config_path, schema_path, main_uav")

    base_dir = CONFIG_PATH.parent

    bus_config_path = Path(process_config["bus_config_path"])
    if not bus_config_path.is_absolute():
        bus_config_path = base_dir / bus_config_path
    if not bus_config_path.is_file():
        raise FileNotFoundError(f"bus config not found: {bus_config_path}")
    bus_config = load_json(bus_config_path)

    schema_path = Path(process_config["schema_path"])
    if not schema_path.is_absolute():
        schema_path = base_dir / schema_path
    if not schema_path.is_file():
        raise FileNotFoundError(f"schema file not found: {schema_path}")
    schema = load_json(schema_path)
    if "topics" not in schema:
        raise KeyError("schema missing topics")

    main_cfg = process_config["main_uav"]
    if "mode" not in main_cfg:
        raise KeyError("main_uav config missing mode")
    mode = main_cfg["mode"]
    if mode not in ("msp", "mavlink"):
        raise ValueError("main_uav mode must be 'msp' or 'mavlink'")
    msp_requests = main_cfg.get("msp", {}).get("requests", [])
    if mode == "msp":
        if not isinstance(msp_requests, list):
            raise TypeError("main_uav.msp.requests must be a list")
    mavlink_cfg = main_cfg.get("mavlink", {})
    datalink_cfg = main_cfg.get("datalink", {})
    if mode == "mavlink":
        if "conn_str" not in mavlink_cfg:
            raise KeyError("main_uav.mavlink.conn_str is required")
        if "hl_rate_hz" not in mavlink_cfg:
            raise KeyError("main_uav.mavlink.hl_rate_hz is required")
        hl_rate_hz = float(mavlink_cfg["hl_rate_hz"])
        if hl_rate_hz <= 0:
            raise ValueError("main_uav.mavlink.hl_rate_hz must be > 0")
        if "destination" not in datalink_cfg or "transport" not in datalink_cfg:
            raise KeyError("main_uav.datalink must contain destination and transport")

    endpoint = bus_config["endpoint"]
    endpoint_type = endpoint.get("type")
    if endpoint_type == "tcp":
        host = endpoint.get("host")
        port = endpoint.get("port")
        if host is None or port is None:
            raise KeyError("tcp endpoint requires host and port")
        client = BusClientSync.connect_tcp(str(host), int(port), CLIENT_ID)
    elif endpoint_type == "unix":
        path = endpoint.get("path")
        if not path:
            raise KeyError("unix endpoint requires path")
        client = BusClientSync.connect_unix(str(path), CLIENT_ID)
    else:
        raise ValueError("endpoint.type must be tcp or unix")

    topics = schema["topics"]
    if DATALINK_IN_TOPIC not in topics:
        raise KeyError(f"schema missing required topic {DATALINK_IN_TOPIC}")
    if mode == "mavlink" and DATALINK_OUT_TOPIC not in topics:
        raise KeyError(f"schema missing required topic {DATALINK_OUT_TOPIC} for mavlink mode")

    client.subscribe(DATALINK_IN_TOPIC)
    client.subscribe(MSP_REPLY_TOPIC)

    mavlink_cfg = main_cfg.get("mavlink", {})
    datalink_cfg = main_cfg.get("datalink", {})
    telem_period = (1.0 / float(mavlink_cfg["hl_rate_hz"])) if mode == "mavlink" else None
    mav = MavlinkAP(mavlink_cfg["conn_str"]) if mode == "mavlink" else None
    next_telem: Optional[float] = time.monotonic() + telem_period if mode == "mavlink" else None

    if mode == "msp":
        for entry in msp_requests:
            if not isinstance(entry, dict):
                raise TypeError("msp_requests entries must be objects")
            if "op" not in entry:
                raise KeyError("msp_request entry missing op")
            if "data" not in entry:
                raise KeyError("msp_request entry missing data")
            op = entry["op"]
            payload_data = entry["data"]
            envelope = build_bus_envelope(
                CLIENT_ID,
                MSP_REQUEST_TOPIC,
                {"op": op, "data": payload_data},
            )
            client.publish(MSP_REQUEST_TOPIC, envelope)
            print(f"[MSP_REQUEST] op={op} topic={MSP_REQUEST_TOPIC} envelope_time_ms={envelope['time']}", flush=True)
    else:
        mav.connect()

    try:
        while True:
            if mode == "mavlink":
                mav.poll()
                now = time.monotonic()
                if next_telem is not None and now >= next_telem:
                    if mav.lat is not None and mav.lon is not None:
                        telem_payload = {
                            "mode_str": mav.mode_str,
                            "airspeed": int(mav.airspeed),
                            "groundspeed": int(mav.groundspeed),
                            "heading": int(mav.heading),
                            "msl_alt": int(mav.msl_alt),
                            "lat": int(mav.lat * 1e7),
                            "lon": int(mav.lon * 1e7),
                        }
                        envelope = build_bus_envelope(
                            CLIENT_ID,
                            DATALINK_OUT_TOPIC,
                            {
                                "destination": datalink_cfg["destination"],
                                "message_path": "Status.AP.HL_TELEM",
                                "payload": telem_payload,
                                "transport": datalink_cfg["transport"],
                            },
                        )
                        client.publish(DATALINK_OUT_TOPIC, envelope)
                        print(
                            f"[HL_TELEM] lat={telem_payload['lat']} lon={telem_payload['lon']} alt={telem_payload['msl_alt']} gs={telem_payload['groundspeed']}",
                            flush=True,
                        )
                    next_telem = now + telem_period
            try:
                message, raw = client.receive(timeout=RECEIVE_TIMEOUT_S)
            except queue.Empty:
                continue

            payload = message.get("payload")
            topic = message.get("topic")
            if payload is None:
                raise KeyError(f"bus message missing payload raw={raw}")
            if topic == MSP_REPLY_TOPIC:
                data_field = payload.get("data")
                op_name = data_field.get("op") if isinstance(data_field, dict) else None
                print(
                    f"[MSP_REPLY] topic={topic} op={op_name} src={message.get('src')} issued={payload.get('time')} data={data_field}",
                    flush=True,
                )
                continue
            if topic == DATALINK_IN_TOPIC:
                data_field = payload.get("payload")
                if not isinstance(data_field, dict):
                    raise TypeError("datalink payload missing payload object")
                if mode == "mavlink":
                    msg_name = payload.get("message")
                    if msg_name == "Command.AP.ARM":
                        arm_flag = int(data_field.get("arm", 1)) != 0
                        mav.arm(arm_flag)
                        print(f"[CMD] ARM={arm_flag}", flush=True)
                        continue
                    if msg_name == "Command.AP.DISARM":
                        mav.arm(False)
                        print("[CMD] DISARM", flush=True)
                        continue
                    if msg_name == "Command.AP.SET_MODE":
                        mode_str = data_field.get("mode_str", "")
                        if isinstance(mode_str, (bytes, bytearray)):
                            mode_str = mode_str.decode("utf-8", errors="ignore")
                        ok = mav.set_mode(str(mode_str))
                        print(f"[CMD] SET_MODE {mode_str} -> {ok}", flush=True)
                        continue
                    if msg_name == "Command.AP.TAKEOFF":
                        alt_m = int(data_field.get("alt_m", 10))
                        mav.takeoff(alt_m)
                        print(f"[CMD] TAKEOFF alt={alt_m}", flush=True)
                        continue
                    if msg_name == "Command.AP.LAND":
                        mav.land()
                        print("[CMD] LAND", flush=True)
                        continue
                    if msg_name == "Command.AP.SELECT_MISSION":
                        seq = int(data_field.get("seq", 0))
                        mav.select_mission(seq)
                        print(f"[CMD] SELECT_MISSION seq={seq}", flush=True)
                        continue
                print(
                    f"[DATALINK_IN] src={message.get('src')} via={payload.get('interface')} from={payload.get('source')} msg={payload.get('message')} payload={data_field}",
                    flush=True,
                )
                continue
            print(f"[UNHANDLED] topic={topic} raw={raw}", flush=True)
    finally:
        if mode == "mavlink" and mav is not None:
            mav.stop()
        client.close()


if __name__ == "__main__":
    main()
