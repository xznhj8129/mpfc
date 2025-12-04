#!/usr/bin/env python3
"""
Shared synchronous core base class.
Provides bus connection, auto diag subscription/ping-pong, ONLINE/STOPPED events, and a helper to fetch the next non-diag message.
"""

import queue
import time
from typing import Any, Dict

from lib.common import build_envelope, connect_bus_client


class CoreBase:
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        self.client_id = cfg.get("id")
        self.client = connect_bus_client(bus_config, self.client_id)
        self.cfg = cfg
        self.bus_config = bus_config
        self.diag_ping_topic = f"Diag.{self.client_id}.PING"
        self.diag_pong_topic = f"Diag.{self.client_id}.PONG"
        self.diag_online_topic = f"Diag.{self.client_id}.ONLINE"
        self.diag_stopped_topic = f"Diag.{self.client_id}.STOPPED"
        self.client.subscribe(self.diag_ping_topic)
        self.send_online()

    def send_online(self) -> None:
        self.client.publish(
            self.diag_online_topic, build_envelope(self.client_id, self.diag_online_topic, {"event": "ONLINE"})
        )
        print(f"[CORE_ONLINE] id={self.client_id}", flush=True)

    def stop(self) -> None:
        self.client.publish(
            self.diag_stopped_topic, build_envelope(self.client_id, self.diag_stopped_topic, {"event": "STOPPED"})
        )
        self.client.close()
        print(f"[COR_STOP] id={self.client_id}", flush=True)

    def recv_message(self, timeout: float) -> tuple[Any, Any, Any]:
        if timeout < 0:
            timeout = 0.0
        try:
            message, _ = self.client.receive(timeout=timeout)
        except queue.Empty:
            return None, None, None
        if not isinstance(message, dict):
            raise TypeError("bus message must be a dictionary")
        topic = message.get("topic")
        payload = message.get("payload")
        if not isinstance(topic, str) or not topic:
            raise KeyError("bus message missing topic")
        if not isinstance(payload, dict):
            raise TypeError("bus message payload must be a dictionary")
        if topic == self.diag_ping_topic:
            pong_payload = build_envelope(self.client_id, self.diag_pong_topic, {"ping_time": payload.get("time")})
            self.client.publish(self.diag_pong_topic, pong_payload)
            return None, None, None
        return topic, payload, message

    def recv_until(self, deadline: float) -> tuple[Any, Any, Any]:
        timeout = max(0.0, deadline - time.monotonic())
        return self.recv_message(timeout)