#!/usr/bin/env python3
"""
Shared synchronous core base class.
Provides bus connection, auto diag subscription/ping-pong, ONLINE/STOPPED events, and a helper to fetch the next non-diag message.
"""

import queue
import time
from typing import Any, Dict

from lib.common import CONTROL_SHUTDOWN_TOPIC, build_envelope, connect_bus_client


class CoreBase:
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        self.client_id = cfg.get("id")
        self.client = connect_bus_client(bus_config, self.client_id)
        self.cfg = cfg
        self.bus_config = bus_config
        self._stopped = False
        self.diag_ping_topic = f"Diag.{self.client_id}.PING"
        self.diag_pong_topic = f"Diag.{self.client_id}.PONG"
        self.diag_online_topic = f"Diag.{self.client_id}.ONLINE"
        self.diag_stopped_topic = f"Diag.{self.client_id}.STOPPED"
        self.client.subscribe(self.diag_ping_topic)
        self.client.subscribe(CONTROL_SHUTDOWN_TOPIC)
        self.send_online()

    def send_online(self) -> None:
        self.client.publish(
            self.diag_online_topic, build_envelope(self.client_id, self.diag_online_topic, {"event": "ONLINE"})
        )
        print(f"[CORE_ONLINE] id={self.client_id}", flush=True)

    def publish_shutdown(self) -> None:
        try:
            self.client.publish(CONTROL_SHUTDOWN_TOPIC, build_envelope(self.client_id, CONTROL_SHUTDOWN_TOPIC, {}))
        except Exception:
            pass

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        try:
            self.client.publish(
                self.diag_stopped_topic,
                build_envelope(self.client_id, self.diag_stopped_topic, {"event": "STOPPED"}),
            )
        except Exception:
            pass
        try:
            self.client.close()
        except Exception:
            pass
        print(f"[CORE_STOP] id={self.client_id}", flush=True)

    def finish(self, exit_code: int = 0) -> None:
        self.publish_shutdown()
        self.stop()
        raise SystemExit(exit_code)

    def recv_message(self, timeout: float) -> tuple[Any, Any, Any]:
        if timeout < 0:
            timeout = 0.0
        try:
            message, _ = self.client.receive(timeout=timeout)
        except queue.Empty:
            return None, None, None
        topic = message.get("topic")
        payload = message.get("payload")
        if topic == self.diag_ping_topic:
            pong_payload = build_envelope(self.client_id, self.diag_pong_topic, {"ping_time": payload.get("time")})
            self.client.publish(self.diag_pong_topic, pong_payload)
            return None, None, None
        if topic == CONTROL_SHUTDOWN_TOPIC:
            print(f"[CORE_CTRL] id={self.client_id} shutdown=True", flush=True)
            self.finish(0)
        return topic, payload, message

    def recv_until(self, deadline: float) -> tuple[Any, Any, Any]:
        timeout = max(0.0, deadline - time.monotonic())
        return self.recv_message(timeout)
