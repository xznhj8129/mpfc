#!/usr/bin/env python3
"""
State scheduler that rate-limits per-signal publishes on the bus.
"""

import threading
import time
from typing import Any, Dict

from lib.common import build_envelope


class StateScheduler:
    def __init__(self, client, client_id: str, topics: Dict[str, Dict[str, Any]]) -> None:
        self.client = client
        self.client_id = client_id
        self.topics: Dict[str, str] = {}
        self.intervals: Dict[str, float] = {}
        self.last_sent: Dict[str, float] = {}
        for key, entry in topics.items():
            topic = entry["topic"]
            interval_s = float(entry["interval_s"])
            if not topic:
                raise RuntimeError(f"missing topic for state key {key}")
            if interval_s <= 0:
                raise RuntimeError(f"invalid interval for state key {key}: {interval_s}")
            self.topics[key] = topic
            self.intervals[key] = interval_s
            self.last_sent[key] = 0.0

        self.state: Dict[str, Any] = {}
        self.observed_at_ms: dict[str, int] = {}
        self.pending: set[str] = set()
        self.lock = threading.Lock()

    def update(self, key: str, value: Any) -> None:
        if key not in self.topics:
            raise RuntimeError(f"unknown state key {key}")
        with self.lock:
            self.state[key] = value
            self.observed_at_ms[key] = int(time.time() * 1000)
            self.pending.add(key)

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return dict(self.state)

    def flush(self) -> None:
        now = time.monotonic()
        to_publish: list[tuple[str, Any, int]] = []
        with self.lock:
            for key in list(self.pending):
                last = self.last_sent[key]
                if last > 0.0 and now - last < self.intervals[key]:
                    continue
                value = self.state[key]
                self.last_sent[key] = now
                self.pending.discard(key)
                to_publish.append((key, value, self.observed_at_ms[key]))

        for key, value, observed_at_ms in to_publish:
            topic = self.topics[key]
            payload = value
            if type(payload) is not dict:
                field_name = key.rsplit(".", 1)[-1]
                payload = {field_name: value}
            envelope = build_envelope(self.client_id, topic, payload, observed_at_ms)
            self.client.publish(topic, envelope)
