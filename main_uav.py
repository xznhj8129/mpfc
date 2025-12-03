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
from typing import Any, Dict

from message_bus_client import BusClientSync

DATALINK_IN_TOPIC = "Datalink.IN"
MSP_REQUEST_TOPIC = "MSP.REQUEST"
MSP_REPLY_TOPIC = "MSP.REPLY"
RECEIVE_TIMEOUT_S = 0.5
CONFIG_PATH = Path(__file__).resolve().with_name("process_config.json")
CLIENT_ID = "main_uav"


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
    if "msp_requests" not in main_cfg:
        raise KeyError("main_uav config must contain msp_requests (list)")
    msp_requests = main_cfg["msp_requests"]
    if not isinstance(msp_requests, list):
        raise TypeError("main_uav.msp_requests must be a list")

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

    client.subscribe(DATALINK_IN_TOPIC)
    client.subscribe(MSP_REPLY_TOPIC)

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

    try:
        while True:
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
                data_field = payload.get("data")
                print(
                    f"[DATALINK_IN] src={message.get('src')} via={payload.get('interface')} from={payload.get('source')} msg={payload.get('message')} payload={data_field}",
                    flush=True,
                )
                continue
            print(f"[UNHANDLED] topic={topic} raw={raw}", flush=True)
    finally:
        client.close()


if __name__ == "__main__":
    main()
