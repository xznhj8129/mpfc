#!/usr/bin/env python3
"""
Minimal MQTT client wrapper matching the sync bus client interface.
Publishes JSON envelopes to hiveos-prefixed topics with QoS0/no retain.
"""

import json
import queue
import threading
import time
from typing import Dict, Tuple

import paho.mqtt.client as mqtt

ENCODING = "utf-8"
MQTT_CONNECT_TIMEOUT_S = 5.0


class MqttPublishError(RuntimeError):
    def __init__(self, topic: str, rc: int) -> None:
        super().__init__(f"mqtt publish failed topic={topic} rc={rc}")
        self.topic = topic
        self.rc = rc


class MqttBusClient:
    def __init__(self, host: str, port: int, client_id: str, topic_prefix: str) -> None:
        self.host = host
        self.port = port
        self.client_id = client_id
        self.topic_prefix = topic_prefix
        self.inbox: "queue.Queue[Tuple[Dict, str]]" = queue.Queue()
        self.connected = threading.Event()
        self.connect_rc: int | None = None
        self.client = mqtt.Client(client_id=client_id, clean_session=True, protocol=mqtt.MQTTv311)
        self.client.enable_logger()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        connect_result = self.client.connect_async(self.host, self.port, keepalive=60)
        if connect_result is not None and connect_result != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(
                f"mqtt connect setup failed host={self.host} port={self.port} client={self.client_id} rc={connect_result}"
            )
        self.client.loop_start()
        if not self.connected.wait(timeout=MQTT_CONNECT_TIMEOUT_S):
            self.close()
            raise RuntimeError(f"mqtt connect timeout host={self.host} port={self.port} client={self.client_id}")
        if self.connect_rc != 0:
            self.close()
            raise RuntimeError(
                f"mqtt connect failed host={self.host} port={self.port} client={self.client_id} rc={self.connect_rc}"
            )

    def _on_connect(self, client: mqtt.Client, userdata, flags, rc, properties=None) -> None:
        self.connect_rc = int(rc)
        self.connected.set()

    def _on_message(self, client: mqtt.Client, userdata, msg: mqtt.MQTTMessage) -> None:
        try:
            raw = msg.payload.decode(ENCODING)
            parsed = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
            return
        self.inbox.put((parsed, raw))

    def _topic(self, topic: str) -> str:
        return f"{self.topic_prefix}/{topic}"

    def subscribe(self, topic: str) -> None:
        full = self._topic(topic)
        result, _ = self.client.subscribe(full, qos=0)
        if result != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"mqtt subscribe failed topic={full} rc={result}")

    def publish(self, topic: str, payload: Dict) -> None:
        full = self._topic(topic)
        encoded = json.dumps(payload, separators=(",", ":")).encode(ENCODING)
        result = self.client.publish(full, encoded, qos=0, retain=False)
        status = result.rc
        if status != mqtt.MQTT_ERR_SUCCESS:
            raise MqttPublishError(full, status)

    def receive(self, timeout: float | None = None) -> Tuple[Dict, str]:
        if timeout is None:
            item = self.inbox.get()
            return item
        item = self.inbox.get(timeout=timeout)
        return item

    def close(self) -> None:
        try:
            self.client.disconnect()
        finally:
            self.client.loop_stop()
