#!/usr/bin/env python3
"""
* Connect to Bus
- Any publisher → `Hivelink.OUT` (unencoded payload) → bus → `hivelink_interface.py` → encodes → `DatalinkInterface.send`.
- HiveLink radios → `DatalinkInterface.receive` → `hivelink_interface.py` → `Hivelink.IN` (decoded payload) → bus → subscribers.
"""

import asyncio
import contextlib
import json
import time
from pathlib import Path
from typing import Any, Dict

from hivelink.datalinks import DatalinkInterface
from hivelink.msglib import decode_message, encode_message, messageid, message_str_from_id
from hivelink.protocol import Messages
from message_bus_client import BusClientAsync

IS_DATALINK = True
DATALINK_IN_TOPIC = "Datalink.IN"
DATALINK_OUT_TOPIC = "Datalink.OUT"
POLL_INTERVAL_S = 0.1
CONFIG_PATH = "? fed from main" #Path(__file__).resolve().with_name("process_config.json")
CLIENT_ID = "Hivelink"




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


class HivelinkBusBridge:
    def __init__(self, client: BusClientAsync, datalink: DatalinkInterface, client_id: str) -> None:
        self.client = client
        self.datalink = datalink
        self.client_id = client_id

    @staticmethod
    def _resolve_message(path: str):
        category_name, subcategory_name, message_name = path.split(".")
        category = getattr(Messages, category_name)
        subcategory = getattr(category, subcategory_name)
        return getattr(subcategory, message_name)

    async def _publish_incoming(self, inbound: Dict[str, Any]) -> None:
        enum_member, decoded_payload = decode_message(inbound["data"])
        envelope = build_bus_envelope(
            self.client_id,
            DATALINK_IN_TOPIC,
            {
                "interface": inbound["intf"],
                "source": inbound["from"],
                "message": message_str_from_id(messageid(enum_member)),
                "payload": decoded_payload,
            },
        )
        await self.client.publish(DATALINK_IN_TOPIC, envelope)

    async def _handle_bus_message(self, message: Dict[str, Any], raw: str) -> None:
        topic = message.get("topic")
        if topic != DATALINK_OUT_TOPIC:
            return
        payload = message.get("payload")
        if payload is None:
            raise KeyError(f"datalink outbound bus message missing payload raw={raw}")
        envelope_topic = payload.get("topic")
        if envelope_topic != DATALINK_OUT_TOPIC:
            raise ValueError(f"datalink outbound envelope topic mismatch envelope_topic={envelope_topic} raw={raw}")
        data = payload.get("data")
        destination = data["destination"]
        message_path = data["message_path"]
        payload_fields = data["payload"]
        transport = data["transport"]

        enum_member = self._resolve_message(message_path)
        payload_list = enum_member.payload(**payload_fields)
        encoded = encode_message(enum_member, payload_list)
        sent = self.datalink.send(
            encoded,
            dest=destination if destination else None,
            udp=transport == "udp",
            meshtastic=transport == "meshtastic",
            multicast=transport == "multicast",
        )
        if not sent:
            raise RuntimeError(
                f"datalink send failed destination={destination} message={message_path} transport={transport}"
            )

    async def run(self) -> None:
        await self.client.subscribe(DATALINK_OUT_TOPIC)
        self.datalink.start()
        bus_task = asyncio.create_task(self.client.receive_loop(self._handle_bus_message))
        try:
            while True:
                incoming = self.datalink.receive()
                for packet in incoming:
                    await self._publish_incoming(packet)
                await asyncio.sleep(POLL_INTERVAL_S)
        finally:
            bus_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await bus_task
            self.datalink.stop()
            await self.client.close()


async def main_async() -> None:
    # TODO: Reimplement config

    datalink = DatalinkInterface(
        use_meshtastic=False,
        use_udp=True,
        use_multicast=bool(multicast_group),
        wlan_device=None,
        radio_port=None,
        meshtastic_dataport=260,
        meshtastic_channel=0,
        socket_host=hcfg["udp_host"],
        socket_port=int(hcfg["udp_port"]),
        my_name=my_name,
        my_id=int(my_info["meshid"]),
        nodemap=nodemap,
        multicast_group=multicast_group or "",
        multicast_port=resolved_multicast_port,
        mqtt_enable=False,
        mqtt_broker="",
        mqtt_port=1883,
        mqtt_client_id="",
        mqtt_username=None,
        mqtt_password=None,
        mqtt_base="/hivelink/v1",
        incumbent_window=600,
    )

    bridge = HivelinkBusBridge(client, datalink, CLIENT_ID)
    await bridge.run()


def main() -> None:
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
