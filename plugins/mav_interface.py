#!/usr/bin/env python3
"""
Usage:
    from plugins.mav_interface import run_plugin
    # cfg and bus_config must be loaded from a HiveOS config file, e.g. config/config_mavlink.json
    run_plugin(cfg, bus_config)
"""

import base64
import time
import traceback
from typing import Any, Dict

from pymavlink import mavutil

from lib.common import build_envelope
from lib.plugin_base import PluginBase

MAVLINK_TOPIC = "MAVLINK.RAW"
POLL_INTERVAL = 0.01


class MavlinkInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.conn_type = cfg["conn_type"]
        self.conn_str = cfg["conn_str"]
        self.conn_bitrate = int(cfg["conn_bitrate"])
        self.bus_topic = MAVLINK_TOPIC
        self.client.subscribe(self.bus_topic)
        self.link = mavutil.mavlink_connection(self.conn_str, baud=self.conn_bitrate)

    def run(self) -> None:
        try:
            while True:
                topic, payload, message = self.recv_message(POLL_INTERVAL)
                if topic == self.bus_topic:
                    src_client = payload["client"] or message.get("src")
                    if src_client != self.client_id:
                        raw_bytes = base64.b64decode(payload["data"]["frame"])
                        self.link.write(raw_bytes)
                        print(
                            f"[PLUGIN_TX] id={self.client_id} src={src_client} bytes={len(raw_bytes)} link={self.conn_str} type={self.conn_type}",
                            flush=True,
                        )
                mav_msg = self.link.recv_match(blocking=False, timeout=0)
                while mav_msg is not None:
                    buf = bytes(mav_msg.get_msgbuf())
                    envelope = build_envelope(
                        self.client_id,
                        self.bus_topic,
                        {
                            "frame": base64.b64encode(buf).decode("ascii"),
                            "msgid": mav_msg.get_msgId(),
                            "type": mav_msg.get_type(),
                            "length": len(buf),
                            "link": self.conn_str,
                        },
                    )
                    self.client.publish(self.bus_topic, envelope)
                    print(
                        f"[PLUGIN_RX] id={self.client_id} msgid={mav_msg.get_msgId()} type={mav_msg.get_type()} bytes={len(buf)} link={self.conn_str}",
                        flush=True,
                    )
                    mav_msg = self.link.recv_match(blocking=False, timeout=0)
                time.sleep(POLL_INTERVAL)
        except RuntimeError:
            error_topic = f"DIAG.{self.client_id}.ERROR"
            error_payload = build_envelope(
                self.client_id, error_topic, {"event": "ERROR", "traceback": traceback.format_exc().strip()}
            )
            self.client.publish(error_topic, error_payload)
            raise
        except KeyboardInterrupt:
            pass
        finally:
            self.link.close()
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MavlinkInterface(cfg, bus_config).run()
