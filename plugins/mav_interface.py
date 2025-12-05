#!/usr/bin/env python3
"""
Usage:
    from plugins.mav_interface import run_plugin
    # cfg and bus_config must be loaded from a HiveOS config file, e.g. config/config_mavlink.json
    run_plugin(cfg, bus_config)
"""

import base64
import socket
import time
import traceback
from typing import Any, Dict

from pymavlink import mavutil

from lib.common import build_envelope
from lib.plugin_base import PluginBase

MAVLINK_TOPIC = "MAVLINK"
POLL_INTERVAL = 0.01
START_TIMEOUT = 5.0
STALL_TIMEOUT = 15.0


class MavlinkInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.conn_type = cfg["conn_type"]
        self.conn_str = cfg["conn_str"]
        self.conn_bitrate = int(cfg["conn_bitrate"])
        self.is_interface = bool(cfg.get("is_interface"))
        self.is_datalink = bool(cfg.get("is_datalink"))
        self.bus_topic = MAVLINK_TOPIC
        self.client.subscribe(self.bus_topic)
        self.link = mavutil.mavlink_connection(self.conn_str, baud=self.conn_bitrate)
        try:
            self.link.robust_parsing = True
        except AttributeError:
            pass
        try:
            self.link.mav.robust_parsing = True
            self.link.mav.use_mavlink2 = True
        except AttributeError:
            pass
        self.last_frame_time = None
        self.last_frame_meta = None
        if self.link is None:
            error_topic = f"DIAG.{self.client_id}.ERROR"
            error_payload = build_envelope(
                self.client_id,
                error_topic,
                {
                    "event": "ERROR",
                    "traceback": f"failed to open MAVLink {self.conn_type}::{self.conn_str} baud={self.conn_bitrate}",
                },
            )
            self.client.publish(error_topic, error_payload)
            self.stop()
            raise RuntimeError(f"failed to open MAVLink {self.conn_type}::{self.conn_str}")
        if self.is_interface:
            print(
                f"[PLUGIN_LINK] id={self.client_id} waiting_heartbeat type={self.conn_type} conn={self.conn_str}",
                flush=True,
            )
            first_hb = self.link.recv_match(type="HEARTBEAT", blocking=True, timeout=START_TIMEOUT)
            if first_hb is None:
                error_topic = f"DIAG.{self.client_id}.ERROR"
                error_payload = build_envelope(
                    self.client_id,
                    error_topic,
                    {
                        "event": "ERROR",
                        "traceback": f"MAVLink {self.conn_type}::{self.conn_str} heartbeat timeout {START_TIMEOUT}s",
                    },
                )
                self.client.publish(error_topic, error_payload)
                self.stop()
                raise RuntimeError(f"MAVLink {self.conn_type}::{self.conn_str} heartbeat timeout")
            print(
                f"[PLUGIN_LINK] id={self.client_id} heartbeat sysid={first_hb.get_srcSystem()} compid={first_hb.get_srcComponent()}",
                flush=True,
            )
            buf = bytes(first_hb.get_msgbuf())
            envelope = build_envelope(
                self.client_id,
                self.bus_topic,
                {
                    "frame": base64.b64encode(buf).decode("ascii"),
                    "sysid": first_hb.get_srcSystem(),
                    "compid": first_hb.get_srcComponent(),
                    "msgid": first_hb.get_msgId(),
                    "type": first_hb.get_type(),
                    "length": len(buf),
                    "data": first_hb.to_dict(),
                    "link": self.conn_str,
                },
            )
            self.client.publish(self.bus_topic, envelope)
            print(
                f"[PLUGIN_RX] id={self.client_id} msgid={first_hb.get_msgId()} type={first_hb.get_type()} bytes={len(buf)} link={self.conn_str}",
                flush=True,
            )
            self.last_frame_time = time.monotonic()
            self.last_frame_meta = (first_hb.get_type(), first_hb.get_srcSystem(), first_hb.get_srcComponent())

    def run(self) -> None:
        try:
            while True:
                topic, payload, message = self.recv_message(POLL_INTERVAL)
                if topic == self.bus_topic:
                    src_client = payload["client"] or message.get("src")
                    if src_client != self.client_id:
                        raw_bytes = base64.b64decode(payload["data"]["frame"])
                        self.link.write(raw_bytes)
                        #print(f"[PLUGIN_TX] id={self.client_id} src={src_client} bytes={len(raw_bytes)} link={self.conn_str} type={self.conn_type}",flush=True,)
                while True:
                    try:
                        raw = self.link.port.recv(65535, socket.MSG_DONTWAIT)
                    except BlockingIOError:
                        break
                    except OSError:
                        break
                    if not raw:
                        break
                    for byte in raw:
                        candidate = self.link.mav.parse_char(bytes([byte]))
                        if candidate is None:
                            continue
                        buf = bytes(candidate.get_msgbuf())
                        envelope = build_envelope(
                            self.client_id,
                            self.bus_topic,
                            {
                                "frame": base64.b64encode(buf).decode("ascii"),
                                "sysid": candidate.get_srcSystem(),
                                "compid": candidate.get_srcComponent(),
                                "msgid": candidate.get_msgId(),
                                "type": candidate.get_type(),
                                "length": len(buf),
                                "data": candidate.to_dict(),
                                "direction": "IN",
                                "link": self.conn_str,
                            },
                        )
                        self.client.publish(self.bus_topic, envelope)
                        self.last_frame_time = time.monotonic()
                        self.last_frame_meta = (candidate.get_type(), candidate.get_srcSystem(), candidate.get_srcComponent())
                        #print(f"[PLUGIN_RX] id={self.client_id} msgid={candidate.get_msgId()} type={candidate.get_type()} bytes={len(buf)} link={self.conn_str}",flush=True,)
                if self.is_interface and self.last_frame_time is not None:
                    age = time.monotonic() - self.last_frame_time
                    if age > STALL_TIMEOUT:
                        last_type, last_sysid, last_compid = self.last_frame_meta or ("?", "?", "?")
                        peek_len = 0
                        raw_hex = ""
                        try:
                            raw_peek = self.link.port.recv(512, socket.MSG_PEEK | socket.MSG_DONTWAIT)
                            peek_len = len(raw_peek)
                            raw_hex = raw_peek[:32].hex()
                        except Exception:
                            pass
                        print(
                            f"[PLUGIN_STALL] id={self.client_id} last_frame_age_s={round(age,2)} last_msg={last_type} last_sysid={last_sysid} last_compid={last_compid} peek_len={peek_len} peek_hex={raw_hex}",
                            flush=True,
                        )
                        raise RuntimeError("mavlink link stalled")
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
