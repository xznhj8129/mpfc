#!/usr/bin/env python3
"""Bridge canonical OCCID models between MPFC's private bus and HiveLink.

This plugin is deliberately semantically dumb. It does not know about
``execution_ingress``, UAV control, or any other MPFC plugin. Remote OCCID
models arrive on ``OCCID/IN``; local components send OCCID models to remote
nodes through ``OCCID/OUT``. MPFC's MQTT topic structure remains private to the
node and never becomes the network API.
"""

from __future__ import annotations

import asyncio
import base64
import queue
import threading
import time
import traceback
from pathlib import Path
from typing import Any, Dict

from hivelink.datalinks import DatalinkInterface

from lib.common import build_envelope, load_config
from lib.occid_bus import pack_occid, unpack_occid
from lib.plugin_base import PluginBase

OCCID_OUT_TOPIC = "OCCID/OUT"
OCCID_IN_TOPIC = "OCCID/IN"
START_TIMEOUT = 5.0


class HiveLinkPlugin(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.cfg = cfg
        self.bus_config = bus_config
        self.bus_poll_interval = float(cfg.get("bus_poll_interval", 0.1))
        self.rx_poll_interval = float(cfg.get("rx_poll_interval", 0.05))
        self.out_topic = str(cfg.get("out_topic", OCCID_OUT_TOPIC))
        self.in_topic = str(cfg.get("in_topic", OCCID_IN_TOPIC))

        self.loop: asyncio.AbstractEventLoop | None = None
        self.loop_thread: threading.Thread | None = None
        self.loop_stop_event = threading.Event()
        self.datalink_ready = threading.Event()
        self.datalink: DatalinkInterface | None = None
        self.loop_error: BaseException | None = None
        self.inbound_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()

        link_cfg = cfg.get("link_config")
        config_path = cfg.get("config_path")
        if config_path:
            cfg_path = Path(str(config_path))
            if not cfg_path.is_absolute():
                cfg_path = Path(__file__).resolve().parent / cfg_path
            link_cfg = load_config(cfg_path)
        if type(link_cfg) is not dict:
            raise ValueError("hivelink_interface requires link_config or config_path")

        udp_cfg = dict(link_cfg.get("udp") or {})
        mesh_cfg = dict(link_cfg.get("meshtastic") or {})

        self.default_udp = bool(udp_cfg.get("use", True))
        self.default_mesh = bool(mesh_cfg.get("use", False))
        self.default_multicast = bool(udp_cfg.get("use_multicast", False))

        my_name = str(link_cfg["my_name"])
        self.interface_params = {
            "use_meshtastic": self.default_mesh,
            "use_udp": self.default_udp,
            "use_multicast": self.default_multicast,
            "socket_host": str(udp_cfg.get("host", "0.0.0.0")),
            "socket_port": int(udp_cfg.get("port", 5555)),
            "multicast_group": str(udp_cfg.get("multicast_group", "")),
            "multicast_port": int(
                udp_cfg.get("multicast_port", udp_cfg.get("port", 5555))
            ),
            "my_name": my_name,
            "my_id": int(link_cfg.get("my_id", 0)),
            "nodemap": dict(link_cfg.get("nodemap") or {}),
            "radio_port": mesh_cfg.get("radio_serial"),
            "meshtastic_dataport": int(mesh_cfg.get("app_portnum", 260)),
            "meshtastic_channel": int(mesh_cfg.get("channel", 0)),
        }

        self.client.subscribe(self.out_topic)

    def _loop_main(self) -> None:
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.datalink = DatalinkInterface(**self.interface_params)

            async def _main() -> None:
                assert self.datalink is not None
                self.datalink.start()
                self.datalink_ready.set()
                while not self.loop_stop_event.is_set():
                    for msg in self.datalink.receive_models():
                        self.inbound_queue.put(msg)
                    await asyncio.sleep(self.rx_poll_interval)

            self.loop.run_until_complete(_main())
        except BaseException as exc:
            self.loop_error = exc
            self.datalink_ready.set()
        finally:
            try:
                if self.datalink is not None:
                    self.datalink.stop()
            finally:
                if self.loop is not None:
                    pending = asyncio.all_tasks(self.loop)
                    for task in pending:
                        task.cancel()
                    if pending:
                        self.loop.run_until_complete(
                            asyncio.gather(*pending, return_exceptions=True)
                        )
                    self.loop.close()

    def _flush_inbound(self) -> None:
        while True:
            try:
                msg = self.inbound_queue.get_nowait()
            except queue.Empty:
                return
            model = msg["model"]
            source = str(msg["from"])
            interface = str(msg["intf"])
            received_at = float(msg.get("time", time.time()))
            data = {
                "source": source,
                "interface": interface,
                "received_at": received_at,
                "model": pack_occid(model),
            }
            self.client.publish(
                self.in_topic,
                build_envelope(self.client_id, self.in_topic, data),
            )
            print(
                f"[HIVELINK_RX] source={source} interface={interface} "
                f"model={type(model).__name__}",
                flush=True,
            )

    def _send_outbound(self, envelope: Dict[str, Any]) -> None:
        data = envelope["data"]
        if type(data) is not dict:
            raise ValueError("OCCID/OUT data must be an object")
        dest = str(data["dest"])
        if not dest:
            raise ValueError("OCCID/OUT requires dest")
        datalink = self.datalink
        if datalink is None:
            raise RuntimeError("HiveLink datalink is not started")

        if "sdk_payload" in data:
            # Sigma SDK messages travel as opaque bytes; HiveLink is delivery.
            payload = base64.b64decode(str(data["sdk_payload"]))
            sent = datalink.send(
                payload,
                dest,
                udp=bool(data.get("udp", self.default_udp)),
                meshtastic=bool(data.get("meshtastic", self.default_mesh)),
                multicast=bool(data.get("multicast", self.default_multicast)),
            )
            if not sent:
                raise RuntimeError(f"HiveLink could not send SDK payload dest={dest}")
            print(f"[HIVELINK_TX] dest={dest} sdk_payload={len(payload)}B", flush=True)
            return

        model = unpack_occid(data["model"])
        sent = datalink.send_model(
            model,
            dest,
            udp=bool(data.get("udp", self.default_udp)),
            meshtastic=bool(data.get("meshtastic", self.default_mesh)),
            multicast=bool(data.get("multicast", self.default_multicast)),
        )
        if not sent:
            raise RuntimeError(
                f"HiveLink could not send model={type(model).__name__} dest={dest}"
            )
        print(f"[HIVELINK_TX] dest={dest} model={type(model).__name__}", flush=True)

    def run(self) -> None:
        self.loop_stop_event.clear()
        self.datalink_ready.clear()
        self.loop_error = None
        self.loop_thread = threading.Thread(
            target=self._loop_main,
            name="hivelink-loop",
            daemon=True,
        )
        self.loop_thread.start()
        if not self.datalink_ready.wait(timeout=START_TIMEOUT):
            raise RuntimeError("HiveLink datalink start timed out")
        if self.loop_error is not None:
            raise self.loop_error
        if self.datalink is None:
            raise RuntimeError("HiveLink datalink failed to start")

        self.send_online()
        try:
            while True:
                if self.loop_error is not None:
                    raise self.loop_error
                self._flush_inbound()
                topic, payload = self.recv_message(self.bus_poll_interval)
                if topic == self.out_topic and payload is not None:
                    self._send_outbound(payload)
        except KeyboardInterrupt:
            pass
        except Exception:
            self.publish_error(traceback.format_exc().strip())
            raise
        finally:
            self.stop()

    def stop(self) -> None:
        self.loop_stop_event.set()
        if self.loop is not None and self.loop.is_running():
            self.loop.call_soon_threadsafe(lambda: None)
        if self.loop_thread is not None:
            self.loop_thread.join(timeout=START_TIMEOUT)
            self.loop_thread = None
        super().stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    HiveLinkPlugin(cfg, bus_config).run()
