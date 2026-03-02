import queue
import time
from pathlib import Path
from typing import Any, Callable, Dict

import yaml

from lib.mqtt_bus_client import MqttBusClient

ENCODING = "utf-8"
CONTROL_SHUTDOWN_TOPIC = "CONTROL/SHUTDOWN"


def load_config(path: Path) -> Dict[str, Any]:  # Load YAML config file into dict.
    suffix = path.suffix.lower()
    if suffix not in {".yaml", ".yml"}:
        raise RuntimeError(f"unsupported config extension {suffix} path={path}")
    with path.open("r", encoding=ENCODING) as handle:
        data = yaml.safe_load(handle)
    if type(data) is not dict:
        raise RuntimeError(f"invalid config root type {type(data).__name__} path={path}")
    return data


def apply_cfg(target: Any, cfg: Dict[str, Any], skip: set[str] | None = None) -> None:  # Copy config keys to object attributes.
    ignored = set() if skip is None else set(skip)
    for key, value in cfg.items():
        if key in ignored:
            continue
        setattr(target, key, value)


def connect_bus_client(bus_config: Dict[str, Any], client_id: str) -> MqttBusClient:  # Build MQTT bus client.
    endpoint = bus_config["endpoint"]
    host = endpoint["host"]
    port = endpoint["port"]
    topic_prefix = bus_config["topic_prefix"]
    return MqttBusClient(host, port, client_id, topic_prefix=topic_prefix)


def build_envelope(client_id: str, topic: str, data: Any) -> Dict[str, Any]:  # Build message envelope.
    return {
        "client": client_id,
        "topic": topic,
        "time": int(time.time() * 1000),
        "data": data,
    }


def build_topic_base(component_id: str, topic_ns: str) -> str:  # Build runtime topic base.
    return f"{component_id}/{topic_ns}"


def build_request_topic(component_id: str, topic_ns: str) -> str:  # Build REQUEST topic path.
    return f"{build_topic_base(component_id, topic_ns)}/REQUEST"


def build_response_topic(component_id: str, topic_ns: str) -> str:  # Build RESPONSE topic path.
    return f"{build_topic_base(component_id, topic_ns)}/RESPONSE"


def build_set_topic(component_id: str) -> str:  # Build SET topic path.
    return f"SET/{component_id}"


def build_state_scheduler_topics(base: str, intervals: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:  # Build scheduler config from state intervals.
    state_base = f"{base}/STATE"
    return {key: {"topic": f"{state_base}/{key}", "interval_s": float(interval_s)} for key, interval_s in intervals.items()}


def build_state_topics(base: str, keys: list[str]) -> Dict[str, str]:  # Build STATE topic map from keys.
    return {key: f"{base}/STATE/{key}" for key in keys}


def build_event_topics(base: str, keys: list[str]) -> Dict[str, str]:  # Build EVENT topic map from keys.
    return {key: f"{base}/EVENT/{key}" for key in keys}


class BusRouter:
    def __init__(  # Initialize bus router state.
        self,
        client_id: str,
        recv_message: Callable[[float], tuple[Any, Any]],
        publish: Callable[[str, Dict[str, Any]], None],
        subscribe: Callable[[str], None],
        poll_interval_s: float,
    ) -> None:
        self.client_id = client_id
        self.recv_message = recv_message
        self.publish = publish
        self.subscribe = subscribe
        self.poll_interval_s = float(poll_interval_s)
        self.state: Dict[str, Any] = {}
        self.responses: Dict[str, Dict[str, Any]] = {}
        self.state_topics: Dict[str, str] = {}
        self.response_topics: set[str] = set()
        self.set_attr_handlers: Dict[str, Dict[str, tuple[Any, str, Callable[[Any], Any] | None, Any | None]]] = {}
        self.request_counter = 0
        self.state_log_keys: list[str] = []
        self.state_log_interval_s: float | None = None
        self.state_log_last = 0.0
        self.state_log_prefix: str | None = None

    def add_state_topic(self, topic: str, key: str) -> None:  # Register single state topic.
        self.state_topics[topic] = key
        self.subscribe(topic)

    def add_state_topics(self, topics: Dict[str, str]) -> None:  # Register multiple state topics.
        for key, topic in topics.items():
            self.add_state_topic(topic, key)

    def add_response_topic(self, topic: str) -> None:  # Register response topic.
        self.response_topics.add(topic)
        self.subscribe(topic)

    def add_set_attr(self, topic: str, name: str, target: Any, attr: str, cast: Callable[[Any], Any] | None = None, lock: Any | None = None) -> None:  # Register SET attr handler.
        if topic not in self.set_attr_handlers:
            self.set_attr_handlers[topic] = {}
            self.subscribe(topic)
        self.set_attr_handlers[topic][name] = (target, attr, cast, lock)

    def enable_state_logging(self, keys: list[str], interval_s: float, prefix: str) -> None:  # Enable periodic state logging.
        self.state_log_keys = list(keys)
        self.state_log_interval_s = float(interval_s)
        self.state_log_prefix = prefix
        self.state_log_last = 0.0

    def pump_once(self, deadline: float | None = None) -> tuple[Any, Any]:  # Poll once and handle state/response/SET.
        poll_timeout = self.poll_interval_s
        if deadline is not None:
            remaining = deadline - time.monotonic()
            poll_timeout = 0 if remaining <= 0 else min(self.poll_interval_s, remaining)

        topic, payload = self.recv_message(poll_timeout)
        if topic is None:
            return None, None

        if topic in self.state_topics:
            state_key = self.state_topics[topic]
            state_payload = payload["data"]
            if type(state_payload) is dict and len(state_payload) == 1:
                field_name = state_key.rsplit(".", 1)[-1]
                if field_name in state_payload:
                    self.state[state_key] = state_payload[field_name]
                elif "value" in state_payload:
                    self.state[state_key] = state_payload["value"]
                else:
                    self.state[state_key] = state_payload
            else:
                self.state[state_key] = state_payload
            if self.state_log_interval_s is not None and state_key in self.state_log_keys:
                now = time.monotonic()
                if now - self.state_log_last >= self.state_log_interval_s:
                    parts = [f"{key}={self.state.get(key)}" for key in self.state_log_keys]
                    print(f"[{self.state_log_prefix}] id={self.client_id} " + " ".join(parts), flush=True)
                    self.state_log_last = now
            return topic, payload

        if topic in self.response_topics:
            response = payload["data"]
            self.responses[response["request_id"]] = response
            return topic, payload

        if topic in self.set_attr_handlers:
            command = payload["data"]
            name = command["name"]
            value = command["value"]
            handlers = self.set_attr_handlers[topic]
            if name not in handlers:
                raise RuntimeError(f"unknown set command {name}")
            target, attr, cast, lock = handlers[name]
            new_value = value if cast is None else cast(value)
            if lock is None:
                setattr(target, attr, new_value)
                return topic, payload
            with lock:
                setattr(target, attr, new_value)
            return topic, payload

        return topic, payload

    def send_action(self, request_topic: str, action: str, params: Dict[str, Any]) -> str:  # Publish REQUEST action.
        self.request_counter += 1
        request_id = f"req-{self.request_counter}"
        payload = {"request_id": request_id, "action": action, "params": params}
        envelope = build_envelope(self.client_id, request_topic, payload)
        self.publish(request_topic, envelope)
        return request_id

    def wait_response(self, request_id: str, timeout_s: float) -> Dict[str, Any]:  # Wait for response id.
        deadline = time.monotonic() + timeout_s
        while True:
            if request_id in self.responses:
                return self.responses.pop(request_id)
            if time.monotonic() > deadline:
                raise RuntimeError(f"timeout waiting for response id={request_id}")
            self.pump_once(deadline)

    def publish_set(self, topic: str, name: str, value: Any) -> None:  # Publish SET command.
        payload = build_envelope(self.client_id, topic, {"name": name, "value": value})
        self.publish(topic, payload)


class RuntimeBase:
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Initialize shared runtime endpoint state.
        self.client_id = cfg.get("id")
        self.client = connect_bus_client(bus_config, self.client_id)
        self.cfg = cfg
        self.bus_config = bus_config
        self._stopped = False
        self.diag_ping_topic = f"DIAG/{self.client_id}/PING"
        self.diag_pong_topic = f"DIAG/{self.client_id}/PONG"
        self.diag_starting_topic = f"DIAG/{self.client_id}/STARTING"
        self.diag_online_topic = f"DIAG/{self.client_id}/ONLINE"
        self.diag_stopped_topic = f"DIAG/{self.client_id}/STOPPED"
        self.client.subscribe(self.diag_ping_topic)
        self.client.subscribe(CONTROL_SHUTDOWN_TOPIC)
        self.client.publish(
            self.diag_starting_topic,
            build_envelope(self.client_id, self.diag_starting_topic, {"event": "STARTING"}),
        )
        self._log_starting()

    def _log_starting(self) -> None:  # Log STARTING event.
        raise NotImplementedError

    def _log_online(self) -> None:  # Log ONLINE event.
        raise NotImplementedError

    def _log_stopped(self) -> None:  # Log STOPPED event.
        raise NotImplementedError

    def _on_control_shutdown(self) -> None:  # Handle CONTROL/SHUTDOWN.
        raise NotImplementedError

    def send_online(self) -> None:  # Publish ONLINE event.
        self.client.publish(
            self.diag_online_topic, build_envelope(self.client_id, self.diag_online_topic, {"event": "ONLINE"})
        )
        self._log_online()

    def stop(self) -> None:  # Publish STOPPED and close bus.
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
        self._log_stopped()

    def recv_message(self, timeout: float) -> tuple[Any, Any]:  # Receive one message with diag handling.
        if timeout < 0:
            timeout = 0.0
        try:
            message, _ = self.client.receive(timeout=timeout)
        except queue.Empty:
            return None, None
        topic = message.get("topic")
        payload = message.get("payload") or message
        if topic == self.diag_ping_topic:
            pong_payload = build_envelope(self.client_id, self.diag_pong_topic, {"ping_time": payload.get("time")})
            self.client.publish(self.diag_pong_topic, pong_payload)
            return None, None
        if topic == CONTROL_SHUTDOWN_TOPIC:
            self._on_control_shutdown()
        return topic, payload

    def recv_until(self, deadline: float) -> tuple[Any, Any]:  # Receive with absolute deadline.
        timeout = max(0.0, deadline - time.monotonic())
        return self.recv_message(timeout)

    def init_bus(  # Initialize BusRouter wiring.
        self,
        poll_interval_s: float,
        state_topics: Dict[str, str] | None = None,
        response_topic: str | None = None,
    ) -> None:
        self.bus = BusRouter(
            self.client_id,
            self.recv_message,
            self.client.publish,
            self.client.subscribe,
            poll_interval_s,
        )
        if state_topics:
            self.bus.add_state_topics(state_topics)
        if response_topic:
            self.bus.add_response_topic(response_topic)
        self.state = self.bus.state

    def add_set_attr(self, topic: str, name: str, target: Any, attr: str, cast: Any | None = None, lock: Any | None = None) -> None:  # Register SET handler.
        self.bus.add_set_attr(topic, name, target, attr, cast, lock)

    def enable_state_logging(self, keys: list[str], interval_s: float, prefix: str) -> None:  # Enable state logging.
        self.bus.enable_state_logging(keys, interval_s, prefix)

    def _send_action(self, action: str, params: Dict[str, Any]) -> str:  # Send REQUEST action.
        return self.bus.send_action(self.request_topic, action, params)

    def _wait_response(self, request_id: str, timeout_s: float) -> Dict[str, Any]:  # Wait for RESPONSE.
        return self.bus.wait_response(request_id, timeout_s)

    def _publish_set(self, topic: str, name: str, value: Any) -> None:  # Publish SET command.
        self.bus.publish_set(topic, name, value)

    def _pump_once(self, deadline: float | None = None) -> tuple[Any, Any]:  # Pump BusRouter once.
        return self.bus.pump_once(deadline)

    def _raise_timeout(self, timeout_error: Any) -> None:  # Raise timeout error from message/exception/callable.
        if callable(timeout_error):
            raise timeout_error()
        if isinstance(timeout_error, BaseException):
            raise timeout_error
        raise RuntimeError(str(timeout_error))

    def wait_until(  # Wait for condition while pumping bus.
        self,
        condition: Callable[[], bool],
        timeout_s: float | None,
        timeout_error: Any = "wait timeout",
        on_tick: Callable[[], None] | None = None,
    ) -> None:
        deadline = None if timeout_s is None else time.monotonic() + float(timeout_s)
        while True:
            if condition():
                return
            if deadline is not None and time.monotonic() > deadline:
                self._raise_timeout(timeout_error)
            if on_tick is not None:
                on_tick()
            self._pump_once(deadline)

    def wait_for_state(  # Wait for state key to match predicate and return value.
        self,
        key: str,
        timeout_s: float | None,
        timeout_error: Any = "state wait timeout",
        predicate: Callable[[Any], bool] | None = None,
    ) -> Any:
        check = predicate if predicate is not None else (lambda value: value is not None)
        self.wait_until(lambda: check(self.state.get(key)), timeout_s, timeout_error)
        return self.state.get(key)

    def pump_for(self, duration_s: float) -> None:  # Pump bus for a fixed duration.
        deadline = time.monotonic() + float(duration_s)
        while time.monotonic() < deadline:
            self._pump_once(deadline)
