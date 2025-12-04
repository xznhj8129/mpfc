import json
import time
from pathlib import Path
from typing import Any, Dict

from lib.message_bus_client import BusClientSync

ENCODING = "utf-8"


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding=ENCODING) as handle:
        data = json.load(handle)
    return data


def connect_bus_client(bus_config: Dict[str, Any], client_id: str) -> BusClientSync:
    endpoint = bus_config["endpoint"]
    etype = endpoint["type"]
    if etype == "tcp":
        host = endpoint["host"]
        port = endpoint["port"]
        return BusClientSync.connect_tcp(host, port, client_id)
    if etype == "unix":
        path = endpoint["path"]
        return BusClientSync.connect_unix(path, client_id)


def build_envelope(client_id: str, topic: str, data: Any) -> Dict[str, Any]:
    return {
        "client": client_id,
        "topic": topic,
        "time": int(time.time() * 1000),
        "data": data,
    }
