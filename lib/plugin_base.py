#!/usr/bin/env python3
"""
Shared synchronous plugin base class.
Provides bus connection, auto diag subscription/ping-pong, ONLINE/STOPPED events, and a helper to fetch the next non-diag message.
"""

import queue
from typing import Any, Dict

from lib.common import RuntimeBase, build_envelope


class PluginBase(RuntimeBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Initialize plugin base and bus.
        self.state_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.response_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        super().__init__(cfg, bus_config)

    def _log_starting(self) -> None:  # Log STARTING event.
        print(f"[PLUGIN_STARTING] id={self.client_id}", flush=True)

    def _log_online(self) -> None:  # Log ONLINE event.
        print(f"[PLUGIN_ONLINE] id={self.client_id}", flush=True)

    def _log_stopped(self) -> None:  # Log STOPPED event.
        print(f"[PLUGIN] {self.client_id} STOP", flush=True)

    def _on_control_shutdown(self) -> None:  # Handle CONTROL/SHUTDOWN.
        print(f"[PLUGIN] {self.client_id} control shutdown=True", flush=True)
        self.stop()
        raise SystemExit

    def enqueue_response(self, request_id: str, action: str, ok: bool, data: Dict[str, Any]) -> None:  # Queue response payload.
        response = {"request_id": request_id, "action": action, "ok": ok, "data": data}
        self.response_queue.put(response)

    def flush_queue(self, q: "queue.Queue[Dict[str, Any]]", topic: str) -> None:  # Publish queued payloads.
        while True:
            try:
                payload = q.get_nowait()
            except queue.Empty:
                return
            envelope = build_envelope(self.client_id, topic, payload)
            self.client.publish(topic, envelope)

    def _publish_event(self, key: str, value: Any) -> None:  # Publish EVENT payload without scheduler throttling.
        if key not in self.event_topics:
            raise RuntimeError(f"unknown event key {key}")
        topic = self.event_topics[key]
        payload = value
        if type(payload) is not dict:
            field_name = key.rsplit(".", 1)[-1]
            payload = {field_name: value}
        self.client.publish(topic, build_envelope(self.client_id, topic, payload))

    def publish_error(self, trace: str) -> None:  # Publish ERROR diag event.
        topic = f"DIAG/{self.client_id}/ERROR"
        self.client.publish(topic, build_envelope(self.client_id, topic, {"event": "ERROR", "traceback": trace}))
