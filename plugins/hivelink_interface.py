#!/usr/bin/env python3
"""
HiveLink datalink bridge.
Subscribes to `Hivelink.OUT` on the bus, encodes and forwards via HiveLink radios; publishes decoded inbound frames as `Hivelink.IN`.
"""

import asyncio
import contextlib
from pathlib import Path
from typing import Any, Dict, Iterable

from hivelink.datalinks import DatalinkInterface
from hivelink.msglib import decode_message, encode_message, messageid, message_str_from_id
from hivelink.protocol import Messages

from lib.common import build_envelope, load_json
from message_bus_client import BusClientAsync

IS_DATALINK = True
HIVELINK_IN_TOPIC = "Hivelink.IN"
HIVELINK_OUT_TOPIC = "Hivelink.OUT"
POLL_INTERVAL_S = 0.1


def _parse_bool(field: Any, name: str) -> bool:
    if isinstance(field, bool):
        return field
    raise TypeError(f"{name} must be boolean")


def _require_str(field: Any, name: str) -> str:
    if isinstance(field, str) and field:
        return field
    raise TypeError(f"{name} must be a non-empty string")


def _require_int(field: Any, name: str) -> int:
    if isinstance(field, int):
        return field
    if isinstance(field, str) and field.isdecimal():
        return int(field)
    raise TypeError(f"{name} must be an int")


async def run_plugin(bus_config: Dict[str, Any], cfg: Dict[str, Any]) -> None:
    client_id = cfg.get("id")
    if not client_id:
        raise ValueError("hivelink plugin cfg missing id")
    cfg_path_text = cfg.get("cfg_path")
    if not cfg_path_text:
        raise ValueError("hivelink plugin cfg missing cfg_path")
    cfg_path = Path(cfg_path_text)
    if not cfg_path.is_absolute():
        cfg_path = Path.cwd() / cfg_path
    if not cfg_path.is_file():
        raise FileNotFoundError(f"hivelink cfg file not found: {cfg_path}")
    hcfg = load_json(cfg_path)

    if "my_name" not in hcfg or "my_id" not in hcfg:
        raise KeyError("hivelink cfg missing my_name or my_id")
    my_name = _require_str(hcfg["my_name"], "my_name")
    my_id = _require_int(hcfg["my_id"], "my_id")

    if "udp" not in hcfg or "meshtastic" not in hcfg or "mqtt" not in hcfg:
        raise KeyError("hivelink cfg missing udp/meshtastic/mqtt sections")
    udp_cfg = hcfg["udp"]
    meshtastic_cfg = hcfg["meshtastic"]
    mqtt_cfg = hcfg["mqtt"]
    if not isinstance(udp_cfg, dict) or not isinstance(meshtastic_cfg, dict) or not isinstance(mqtt_cfg, dict):
        raise TypeError("udp/meshtastic/mqtt must be objects")

    udp_use = _parse_bool(udp_cfg.get("use"), "udp.use")
    udp_host = _require_str(udp_cfg.get("host"), "udp.host") if udp_use else ""
    udp_port = _require_int(udp_cfg.get("port"), "udp.port") if udp_use else 0
    udp_multicast = _parse_bool(udp_cfg.get("use_multicast"), "udp.use_multicast") if udp_use else False
    udp_mc_group = _require_str(udp_cfg.get("multicast_group"), "udp.multicast_group") if udp_use and udp_multicast else ""
    udp_mc_port = _require_int(udp_cfg.get("multicast_port"), "udp.multicast_port") if udp_use and udp_multicast else 0

    meshtastic_use = _parse_bool(meshtastic_cfg.get("use"), "meshtastic.use")
    meshtastic_port = meshtastic_cfg.get("radio_serial")
    meshtastic_dataport = _require_int(meshtastic_cfg.get("app_portnum"), "meshtastic.app_portnum") if meshtastic_use else 0
    meshtastic_channel = _require_int(meshtastic_cfg.get("channel"), "meshtastic.channel") if meshtastic_use else 0
    if meshtastic_use and not meshtastic_port:
        raise ValueError("meshtastic.use true requires radio_serial")

    mqtt_use = _parse_bool(mqtt_cfg.get("use"), "mqtt.use")
    mqtt_base = _require_str(mqtt_cfg.get("base"), "mqtt.base") if mqtt_use else ""
    mqtt_broker = _require_str(mqtt_cfg.get("broker"), "mqtt.broker") if mqtt_use else ""
    mqtt_port = _require_int(mqtt_cfg.get("port"), "mqtt.port") if mqtt_use else 0
    mqtt_client_id = _require_str(mqtt_cfg.get("client_id"), "mqtt.client_id") if mqtt_use else ""
    mqtt_username = mqtt_cfg.get("username") if mqtt_use else None
    mqtt_password = mqtt_cfg.get("password") if mqtt_use else None

    nodemap = hcfg.get("nodemap")
    if nodemap is None:
        raise KeyError("hivelink cfg missing nodemap")
    if not isinstance(nodemap, dict):
        raise TypeError("nodemap must be an object")

    datalink = DatalinkInterface(
        use_meshtastic=meshtastic_use,
        use_udp=udp_use,
        use_multicast=udp_multicast,
        wlan_device=None,
        radio_port=meshtastic_port if meshtastic_use else None,
        meshtastic_dataport=meshtastic_dataport,
        meshtastic_channel=meshtastic_channel,
        socket_host=udp_host if udp_use else "",
        socket_port=udp_port if udp_use else 0,
        my_name=my_name,
        my_id=my_id,
        nodemap=nodemap,
        multicast_group=udp_mc_group if udp_multicast else "",
        multicast_port=udp_mc_port if udp_multicast else 0,
        mqtt_enable=mqtt_use,
        mqtt_broker=mqtt_broker if mqtt_use else "",
        mqtt_port=mqtt_port if mqtt_use else 0,
        mqtt_client_id=mqtt_client_id if mqtt_use else "",
        mqtt_username=mqtt_username if mqtt_use else None,
        mqtt_password=mqtt_password if mqtt_use else None,
        mqtt_base=mqtt_base if mqtt_use else "",
        incumbent_window=600,
    )

    endpoint_cfg = bus_config.get("endpoint")
    if not isinstance(endpoint_cfg, dict):
        raise TypeError("bus_config.endpoint must be an object")
    etype = endpoint_cfg.get("type")
    if etype == "tcp":
        host = endpoint_cfg.get("host")
        port = endpoint_cfg.get("port")
        if not isinstance(host, str) or not host:
            raise ValueError("bus_config.endpoint.host is required for tcp")
        if not isinstance(port, int) or port < 1 or port > 65535:
            raise ValueError("bus_config.endpoint.port must be int 1-65535 for tcp")
        client = await BusClientAsync.connect_tcp(host, port, client_id)
    elif etype == "unix":
        path = endpoint_cfg.get("path")
        if not isinstance(path, str) or not path:
            raise ValueError("bus_config.endpoint.path is required for unix")
        client = await BusClientAsync.connect_unix(path, client_id)
    else:
        raise ValueError("bus_config.endpoint.type must be 'tcp' or 'unix'")

    diag_ping_topic = f"Diag.{client_id}.PING"
    diag_pong_topic = f"Diag.{client_id}.PONG"
    diag_online_topic = f"Diag.{client_id}.ONLINE"
    diag_stopped_topic = f"Diag.{client_id}.STOPPED"
    await client.subscribe(HIVELINK_OUT_TOPIC)
    await client.subscribe(diag_ping_topic)
    await client.publish(diag_online_topic, build_envelope(client_id, diag_online_topic, {"event": "ONLINE"}))

    async def handle_bus_message(message: Dict[str, Any], raw: str) -> None:
        topic = message.get("topic")
        if topic == diag_ping_topic:
            payload = message.get("payload")
            pong_payload = build_envelope(
                client_id, diag_pong_topic, {"ping_time": payload.get("time") if isinstance(payload, dict) else None}
            )
            await client.publish(diag_pong_topic, pong_payload)
            return
        if topic != HIVELINK_OUT_TOPIC:
            return
        payload = message.get("payload")
        if not isinstance(payload, dict):
            raise TypeError(f"datalink outbound bus message missing payload object raw={raw}")
        envelope_topic = payload.get("topic")
        if envelope_topic != HIVELINK_OUT_TOPIC:
            raise ValueError(f"datalink outbound envelope topic mismatch envelope_topic={envelope_topic} raw={raw}")
        data = payload.get("data")
        if not isinstance(data, dict):
            raise TypeError("datalink outbound payload missing data object")
        destination = data.get("destination")
        message_path = data.get("message_path")
        payload_fields = data.get("payload")
        transport = data.get("transport")
        if not isinstance(message_path, str) or not message_path:
            raise ValueError("message_path missing for datalink outbound")
        message_parts = message_path.split(".")
        if len(message_parts) != 3:
            raise ValueError(f"message_path must be Category.Subcategory.Message, got {message_path}")
        enum_member = getattr(getattr(getattr(Messages, message_parts[0]), message_parts[1]), message_parts[2])
        payload_list = enum_member.payload(**payload_fields)
        sent = datalink.send(
            encode_message(enum_member, payload_list),
            dest=destination if destination else None,
            udp=transport == "udp",
            meshtastic=transport == "meshtastic",
            multicast=transport == "multicast",
        )
        if not sent:
            raise RuntimeError(
                f"datalink send failed destination={destination} message={message_path} transport={transport}"
            )

    bus_task = asyncio.create_task(client.receive_loop(handle_bus_message))
    try:
        datalink.start()
        while True:
            incoming = datalink.receive()
            if not isinstance(incoming, Iterable):
                raise TypeError("datalink.receive must return iterable")
            for packet in incoming:
                if not isinstance(packet, dict):
                    raise TypeError("datalink packet must be a dict")
                enum_member, decoded_payload = decode_message(packet["data"])
                envelope = build_envelope(
                    client_id,
                    HIVELINK_IN_TOPIC,
                    {
                        "interface": packet["intf"],
                        "source": packet["from"],
                        "message": message_str_from_id(messageid(enum_member)),
                        "payload": decoded_payload,
                    },
                )
                await client.publish(HIVELINK_IN_TOPIC, envelope)
            await asyncio.sleep(POLL_INTERVAL_S)
    except asyncio.CancelledError:
        raise
    finally:
        bus_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await bus_task
        datalink.stop()
        await client.publish(diag_stopped_topic, build_envelope(client_id, diag_stopped_topic, {"event": "STOPPED"}))
        await client.close()


def main() -> None:
    raise RuntimeError("Run via host process that provides bus_config and cfg to run_plugin")


if __name__ == "__main__":
    main()
