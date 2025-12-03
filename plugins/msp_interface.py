
import asyncio
import json
from pathlib import Path
from typing import Any, Dict

from message_bus_client import BusClientAsync

IS_INTERFACE = True
CONFIG_PATH = Path(__file__).resolve().with_name("process_config.json")
CLIENT_ID = "FlightController"
MSP_REQUEST_TOPIC = "MSP.REQUEST"
MSP_REPLY_TOPIC = "MSP.REPLY"


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


class UAVInterface:
    def __init__(self, client: BusClientAsync, client_id: str) -> None:
        self.client = client
        self.client_id = client_id

    async def _handle_request(self, message: Dict[str, Any], raw: str) -> None:
        topic = message.get("topic")
        if topic != MSP_REQUEST_TOPIC: #must be MSP.REQUEST.<anything>, not just first two
            return
        payload = message.get("payload")
        requester = payload["client"]
        req_data = payload.get("data")
        op_name = req_data.get("op") if isinstance(req_data, dict) else None
        raise NotImplementedError(
            f"MSP handler not implemented for requester={requester} topic={topic} op={op_name} raw={raw}"
        )

    async def run(self) -> None:
        await self.client.subscribe(MSP_REQUEST_TOPIC)
        await self.client.receive_loop(self._handle_request)


async def main_async() -> None:
    if not CONFIG_PATH.is_file():
        raise FileNotFoundError(f"process config not found: {CONFIG_PATH}")
    process_config = load_json(CONFIG_PATH)
    if "bus_config_path" not in process_config or "uav_interface" not in process_config:
        raise KeyError("process_config missing required keys: bus_config_path, uav_interface")

    base_dir = CONFIG_PATH.parent
    bus_config_path = Path(process_config["bus_config_path"])
    if not bus_config_path.is_absolute():
        bus_config_path = base_dir / bus_config_path
    if not bus_config_path.is_file():
        raise FileNotFoundError(f"bus config not found: {bus_config_path}")
    bus_config = load_json(bus_config_path)

    endpoint = bus_config["endpoint"]
    endpoint_type = endpoint.get("type")
    if endpoint_type == "tcp":
        client = await BusClientAsync.connect_tcp(str(endpoint["host"]), int(endpoint["port"]), CLIENT_ID)
    elif endpoint_type == "unix":
        client = await BusClientAsync.connect_unix(str(endpoint["path"]), CLIENT_ID)
    else:
        raise ValueError("endpoint.type must be tcp or unix")

    uav = UAVInterface(client, CLIENT_ID)
    await uav.run()


def main() -> None:
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
