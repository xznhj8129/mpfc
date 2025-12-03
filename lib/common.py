import json
import time
from pathlib import Path
from typing import Any, Dict

from lib.message_bus_client import BusClientSync

ENCODING = "utf-8"


def load_json(path: Path) -> Dict[str, Any]:
    if not path:
        raise ValueError("json path is required")
    with path.open("r", encoding=ENCODING) as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} root must be a JSON object")
    return data


def connect_bus_client(bus_config: Dict[str, Any], client_id: str) -> BusClientSync:
    if not client_id:
        raise ValueError("client id is required for bus connection")
    if "endpoint" not in bus_config or not isinstance(bus_config["endpoint"], dict):
        raise ValueError("bus_config.endpoint is required")
    endpoint = bus_config["endpoint"]
    etype = endpoint.get("type")
    if etype == "tcp":
        host = endpoint.get("host")
        port = endpoint.get("port")
        if not isinstance(host, str) or not host:
            raise ValueError("bus_config.endpoint.host is required for tcp")
        if not isinstance(port, int) or port < 1 or port > 65535:
            raise ValueError("bus_config.endpoint.port must be int 1-65535 for tcp")
        return BusClientSync.connect_tcp(host, port, client_id)
    if etype == "unix":
        path = endpoint.get("path")
        if not isinstance(path, str) or not path:
            raise ValueError("bus_config.endpoint.path is required for unix")
        return BusClientSync.connect_unix(path, client_id)
    raise ValueError("bus_config.endpoint.type must be 'tcp' or 'unix'")


def build_envelope(client_id: str, topic: str, data: Any) -> Dict[str, Any]:
    if not client_id:
        raise ValueError("envelope client is required")
    if not topic:
        raise ValueError("envelope topic is required")
    return {
        "client": client_id,
        "topic": topic,
        "time": int(time.time() * 1000),
        "data": data,
    }
