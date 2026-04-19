#!/usr/bin/env python3
"""
Minimal MQTT client wrapper matching the sync bus client interface.
Publishes JSON envelopes to hiveos-prefixed topics with QoS0/no retain.
"""

import json
import queue
import threading
from typing import Any, Dict

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
        self.topic_prefix_slash = f"{topic_prefix}/"
        self.inbox: "queue.Queue[tuple[str, Dict[str, Any]]]" = queue.Queue()
        self.connected = threading.Event()
        self.connect_result = threading.Event()
        self.connect_rc: int | None = None
        self.subscriptions: set[str] = set()
        self.subscription_lock = threading.Lock()
        self.background_error: RuntimeError | None = None
        self.client = mqtt.Client(client_id=client_id, clean_session=True, protocol=mqtt.MQTTv311)
        self.client.enable_logger()
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        connect_result = self.client.connect_async(self.host, self.port, keepalive=60)
        if connect_result is not None and connect_result != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(
                f"mqtt connect setup failed host={self.host} port={self.port} client={self.client_id} rc={connect_result}"
            )
        self.client.loop_start()
        if not self.connect_result.wait(timeout=MQTT_CONNECT_TIMEOUT_S):
            self.close()
            raise RuntimeError(f"mqtt connect timeout host={self.host} port={self.port} client={self.client_id}")
        if self.connect_rc != 0:
            self.close()
            raise RuntimeError(
                f"mqtt connect failed host={self.host} port={self.port} client={self.client_id} rc={self.connect_rc}"
            )

    def _on_connect(self, client: mqtt.Client, userdata, flags, rc, properties=None) -> None:
        self.connect_rc = int(rc)
        if self.connect_rc != 0:
            self.connected.clear()
            self.connect_result.set()
            return
        self.connected.set()
        self.connect_result.set()
        with self.subscription_lock:
            topics = list(self.subscriptions)
        for topic in topics:
            try:
                self._subscribe_topic(topic)
            except RuntimeError as exc:
                self.background_error = exc
                return

    def _on_disconnect(self, client: mqtt.Client, userdata, rc, properties=None) -> None:
        self.connected.clear()

    def _on_message(self, client: mqtt.Client, userdata, msg: mqtt.MQTTMessage) -> None:
        try:
            raw = msg.payload.decode(ENCODING)
            parsed = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
            return
        full_topic = str(msg.topic)
        if not full_topic.startswith(self.topic_prefix_slash):
            return
        topic = full_topic[len(self.topic_prefix_slash) :]
        self.inbox.put((topic, parsed))

    def _topic(self, topic: str) -> str:
        return f"{self.topic_prefix}/{topic}"

    def _subscribe_topic(self, topic: str) -> None:
        full = self._topic(topic)
        result, _ = self.client.subscribe(full, qos=0)
        if result != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"mqtt subscribe failed topic={full} rc={result}")

    def _raise_background_error(self) -> None:
        if self.background_error is not None:
            raise self.background_error

    def subscribe(self, topic: str) -> None:
        self._raise_background_error()
        with self.subscription_lock:
            self.subscriptions.add(topic)
        if not self.connected.is_set():
            return
        self._subscribe_topic(topic)

    def publish(self, topic: str, payload: Dict) -> None:
        self._raise_background_error()
        full = self._topic(topic)
        encoded = json.dumps(payload, separators=(",", ":")).encode(ENCODING)
        result = self.client.publish(full, encoded, qos=0, retain=False)
        status = result.rc
        if status != mqtt.MQTT_ERR_SUCCESS:
            raise MqttPublishError(full, status)

    def receive(self, timeout: float | None = None) -> tuple[str, Dict[str, Any]]:
        self._raise_background_error()
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
