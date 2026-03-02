#!/usr/bin/env python3
"""
HiveLink datalink bridge.
Usage example (main config fragment):
    {
        "plugins": [
            {
                "plugin": "hivelink_interface",
                "cfg": {
                    "id": "hivelink",
                    "config_path": "hivelink_config.yaml",
                    "rx_poll_interval": 0.1,
                    "bus_poll_interval": 0.1
                }
            }
        ]
    }
Config path is resolved relative to this plugin folder when not absolute.
Subscribes to `DATALINK/OUT` on the bus, encodes and forwards via HiveLink transports; publishes decoded inbound frames as `DATALINK/IN`.
"""

import asyncio
import base64
import queue
import threading
import time
import traceback
from pathlib import Path
from typing import Any, Dict

import hivelink.protocol as hl_proto
from hivelink.datalinks import DatalinkInterface, _to_jsonable

from lib.common import build_envelope, load_config
from lib.plugin_base import PluginBase

DATALINK_OUT_TOPIC = "DATALINK/OUT"
DATALINK_IN_TOPIC = "DATALINK/IN"
START_TIMEOUT = 5.0
Proto = hl_proto.Proto
Messages = hl_proto.Messages
Messages = hl_proto.Messages


class HiveLinkPlugin(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.cfg = cfg
        self.bus_config = bus_config

        self.bus_poll_interval = float(cfg["bus_poll_interval"])
        self.rx_poll_interval = float(cfg["rx_poll_interval"])

        self.loop = None
        self.loop_thread: threading.Thread | None = None
        self.loop_stop_event = threading.Event()
        self.datalink_ready = threading.Event()
        self.datalink: DatalinkInterface | None = None
        self.loop_error: BaseException | None = None
        self.inbound_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()

        config_path = cfg.get("config_path")
        inline_cfg = cfg.get("link_config")
        link_cfg = inline_cfg
        if config_path:
            cfg_path = Path(config_path)
            if not cfg_path.is_absolute():
                cfg_path = Path(__file__).resolve().parent / cfg_path
            link_cfg = load_config(cfg_path)

        my_name = link_cfg["my_name"]
        my_mesh_id = int(link_cfg["my_id"])
        nodemap = link_cfg["nodemap"]

        udp_cfg = link_cfg["udp"]
        udp_use = bool(udp_cfg["use"])
        udp_host = udp_cfg["host"]
        udp_port = udp_cfg["port"]
        udp_mc = bool(udp_cfg["use_multicast"])
        mc_group = udp_cfg["multicast_group"]
        mc_port = udp_cfg["multicast_port"]

        mesh_cfg = link_cfg["meshtastic"]
        mesh_use = bool(mesh_cfg["use"])
        mesh_port = mesh_cfg["radio_serial"]
        mesh_app_port = mesh_cfg["app_portnum"]
        mesh_channel = mesh_cfg["channel"]

        mqtt_cfg = link_cfg["mqtt"]
        mqtt_use = bool(mqtt_cfg["use"])
        mqtt_base = mqtt_cfg["base"]
        mqtt_broker = mqtt_cfg["broker"]
        mqtt_port = mqtt_cfg["port"]
        mqtt_client_id = mqtt_cfg.get("client_id") or ""
        mqtt_username = mqtt_cfg.get("username")
        mqtt_password = mqtt_cfg.get("password")

        mqtt_client_id_final = mqtt_client_id if mqtt_client_id else my_name
        self.interface_params = {
            "use_meshtastic": mesh_use,
            "use_udp": udp_use,
            "use_multicast": udp_mc,
            "socket_host": udp_host,
            "socket_port": udp_port,
            "multicast_group": mc_group,
            "multicast_port": mc_port,
            "my_name": my_name,
            "my_id": my_mesh_id,
            "nodemap": nodemap,
            "radio_port": mesh_port,
            "meshtastic_dataport": mesh_app_port,
            "meshtastic_channel": mesh_channel,
            "mqtt_enable": mqtt_use,
            "mqtt_broker": mqtt_broker,
            "mqtt_port": mqtt_port,
            "mqtt_client_id": mqtt_client_id_final,
            "mqtt_username": mqtt_username,
            "mqtt_password": mqtt_password,
            "mqtt_base": mqtt_base
        }

        self.client.subscribe(DATALINK_OUT_TOPIC)

    def _loop_main(self) -> None:
        asyncio.set_event_loop(self.loop)
        try:
            self.datalink = DatalinkInterface(**self.interface_params)
        except BaseException as exc:
            self.loop_error = exc
            self.datalink_ready.set()
            return

        async def _poll_inbound():
            while not self.loop_stop_event.is_set():
                messages = self.datalink.receive()
                for msg in messages:
                    self.inbound_queue.put(msg)
                await asyncio.sleep(self.rx_poll_interval)

        def _start_link():
            try:
                self.datalink.start()
            except BaseException as exc:
                self.loop_error = exc
                self.loop_stop_event.set()
                self.loop.call_soon_threadsafe(self.loop.stop)

        def _task_done(task: asyncio.Task) -> None:
            exc = task.exception()
            if exc:
                self.loop_error = exc
                self.loop_stop_event.set()
                self.loop.call_soon_threadsafe(self.loop.stop)

        self.loop.call_soon(_start_link)
        inbound_task = self.loop.create_task(_poll_inbound())
        inbound_task.add_done_callback(_task_done)
        self.datalink_ready.set()
        try:
            self.loop.run_forever()
        finally:
            try:
                if self.datalink:
                    self.datalink.stop()
            finally:
                pending = asyncio.all_tasks()
                for task in pending:
                    task.cancel()
                if pending:
                    self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                self.loop.close()

    def _flush_inbound(self) -> None:
        while True:
            try:
                msg = self.inbound_queue.get_nowait()
            except queue.Empty:
                return
            raw_data = msg["data"]
            src = msg["from"]
            intf = msg["intf"]
            tstamp = msg.get("time") or time.time()
            enum_member, decoded_payload = Proto.decode_message(raw_data)
            payload = {
                "from": src,
                "intf": intf,
                "msgid": Proto.message_str_from_id(Proto.messageid(enum_member)),
                "payload": _to_jsonable(decoded_payload),
                "time": int(tstamp),
            }
            envelope = build_envelope(self.client_id, DATALINK_IN_TOPIC, payload)
            self.client.publish(DATALINK_IN_TOPIC, envelope)
            print(
                f"[PLUGIN_RECV] id={self.client_id} src={src} intf={intf} msgid={payload['msgid']} time={payload['time']}",
                flush=True,
            )

    def _send_outbound(self, payload: Dict[str, Any], raw_message: Dict[str, Any]) -> None:
        data = payload["data"]
        dest = data.get("dest")
        msgid_str = data.get("msgid")
        body = data.get("payload")
        encoded_b64 = data.get("encoded_b64")
        send_udp = bool(data.get("udp"))
        send_mesh = bool(data.get("meshtastic"))
        send_multicast = bool(data.get("multicast"))
        datalink = self.datalink

        if encoded_b64:
            encoded_bytes = base64.b64decode(encoded_b64, validate=True)
        else:
            parts = msgid_str.split(".")
            enum_member = getattr(getattr(getattr(Messages, parts[0]), parts[1]), parts[2])
            payload_obj = enum_member.payload(**body)
            encoded_bytes = Proto.encode_message(enum_member, payload_obj)

        datalink.send(
            encoded_bytes,
            dest=dest,
            udp=send_udp,
            meshtastic=send_mesh,
            multicast=send_multicast,
        )

    def run(self) -> None:
        if self.loop_thread is None:
            self.loop_stop_event.clear()
            self.loop = asyncio.new_event_loop()
            self.loop_thread = threading.Thread(target=self._loop_main, name="hivelink-loop", daemon=True)
            self.loop_thread.start()
            self.datalink_ready.wait(timeout=START_TIMEOUT)
            if self.loop_error:
                raise self.loop_error
            if self.datalink is None:
                raise RuntimeError("datalink failed to start")
        self.send_online()
        try:
            while True:
                self._flush_inbound()
                topic, payload = self.recv_message(self.bus_poll_interval)
                if topic is None:
                    continue
                if topic == DATALINK_OUT_TOPIC:
                    self._send_outbound(payload, payload)
                    continue
        except KeyboardInterrupt:
            pass
        except Exception:
            self.publish_error(traceback.format_exc().strip())
            raise
        finally:
            self.stop()

    def stop(self) -> None:
        try:
            self.loop_stop_event.set()
            if self.loop and self.loop.is_running():
                self.loop.call_soon_threadsafe(self.loop.stop)
            if self.loop_thread:
                self.loop_thread.join(timeout=START_TIMEOUT)
                self.loop_thread = None
        finally:
            super().stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    HiveLinkPlugin(cfg, bus_config).run()
