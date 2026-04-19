#!/usr/bin/env python3
"""
Usage:
    from plugins.uav_controller.uav_controller import run_plugin
    run_plugin(cfg, bus_config)
"""

import time
import traceback
from typing import Any, Dict

from lib.common import apply_cfg, build_envelope, build_event_topics, build_request_topic, build_response_topic, build_state_topics, build_topic_base
from lib.plugin_base import PluginBase
from protocols.namespace_loader import load_protocol_namespace

UAV = load_protocol_namespace("uav")


class ControllerActionError(RuntimeError):
    pass


class UavController(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.poll_interval_s = float(cfg["poll_interval_s"])
        self.response_timeout_s = float(cfg["response_timeout_s"])
        self.vehicle = cfg["vehicle"]
        self.backend = cfg["backend"]
        self.backend_state_keys = list(cfg["backend_state_keys"])
        self.backend_event_keys = list(cfg["backend_event_keys"])
        self.arm_ready_since: float | None = None
        self.takeoff_ready_since: float | None = None

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.state_publish_topics = build_state_topics(base, self.backend_state_keys)
        self.event_topics = build_event_topics(base, self.backend_event_keys)
        self.client.subscribe(self.request_topic)

        backend_base = build_topic_base(self.backend["id"], self.backend["topic_ns"])
        self.backend_request_topic = build_request_topic(self.backend["id"], self.backend["topic_ns"])
        self.backend_response_topic = build_response_topic(self.backend["id"], self.backend["topic_ns"])
        self.backend_state_topics = build_state_topics(backend_base, self.backend_state_keys)
        self.backend_state_topic_to_key = {topic: key for key, topic in self.backend_state_topics.items()}
        self.backend_event_topics = build_event_topics(backend_base, self.backend_event_keys)
        self.backend_event_topic_to_key = {topic: key for key, topic in self.backend_event_topics.items()}
        self.init_bus(self.poll_interval_s, self.backend_state_topics, self.backend_response_topic)
        self.responses = self.bus.responses
        for topic in self.backend_event_topics.values():
            self.client.subscribe(topic)

    def _publish_state(self, key: str, payload: Any) -> None:
        topic = self.state_publish_topics[key]
        self.client.publish(topic, build_envelope(self.client_id, topic, payload))

    def _build_sensor_config_state(self) -> Dict[str, Any] | None:
        raw_sensor_config = self.state.get(UAV.State.Sensor.SensorConfig)
        if raw_sensor_config is None:
            self.arm_ready_since = None
            self.takeoff_ready_since = None
            return None
        sensor_config = dict(raw_sensor_config)
        arm_ready = bool(sensor_config.get("Armable"))
        arm_hold_s = float(self.arm_ready_hold_s)
        takeoff_ready = arm_ready and self.state.get(UAV.State.Navigation.Position) is not None
        if "LocalPositionOk" in sensor_config:
            takeoff_ready = takeoff_ready and bool(sensor_config.get("LocalPositionOk"))
        if "GlobalPositionOk" in sensor_config:
            takeoff_ready = takeoff_ready and bool(sensor_config.get("GlobalPositionOk"))
        if "HomePositionOk" in sensor_config:
            takeoff_ready = takeoff_ready and bool(sensor_config.get("HomePositionOk"))
        hold_s = float(self.takeoff_ready_hold_s)
        if self.vehicle["autopilot"] == UAV.Enums.FCAutopilotType.Ardupilot:
            arm_hold_s = float(self.ardupilot_arm_ready_hold_s)
            takeoff_ready = takeoff_ready and bool(sensor_config.get("EkfUsingGps"))
            hold_s = float(self.ardupilot_takeoff_ready_hold_s)
        now = time.monotonic()
        if arm_ready:
            if self.arm_ready_since is None:
                self.arm_ready_since = now
        else:
            self.arm_ready_since = None
        if takeoff_ready:
            if self.takeoff_ready_since is None:
                self.takeoff_ready_since = now
        else:
            self.takeoff_ready_since = None
        sensor_config["ArmReady"] = arm_ready and self.arm_ready_since is not None and now - self.arm_ready_since >= arm_hold_s
        sensor_config["TakeoffReady"] = (
            takeoff_ready
            and self.takeoff_ready_since is not None
            and now - self.takeoff_ready_since >= hold_s
        )
        return sensor_config

    def _pump_controller_once(self, deadline: float | None = None) -> tuple[Any, Any]:
        topic, payload = self._pump_once(deadline)
        if topic in self.backend_state_topic_to_key:
            state_key = self.backend_state_topic_to_key[topic]
            self._publish_state(state_key, payload["data"])
            if state_key in {UAV.State.Sensor.SensorConfig, UAV.State.Navigation.Position}:
                sensor_config = self._build_sensor_config_state()
                if sensor_config is not None:
                    self._publish_state(UAV.State.Sensor.SensorConfig, sensor_config)
        elif topic in self.backend_event_topic_to_key:
            self._publish_event(self.backend_event_topic_to_key[topic], payload["data"])
        return topic, payload

    def _wait_until(self, condition: Any, timeout_s: float, timeout_error: str) -> None:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            if condition():
                return
            if time.monotonic() > deadline:
                raise ControllerActionError(timeout_error)
            self._pump_controller_once(deadline)

    def _wait_backend_response(self, request_id: str, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            if request_id in self.responses:
                return self.responses.pop(request_id)
            if time.monotonic() > deadline:
                raise ControllerActionError(f"backend response wait timed out request_id={request_id}")
            self._pump_controller_once(deadline)

    def _pump_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + float(duration_s)
        while time.monotonic() < deadline:
            self._pump_controller_once(deadline)

    def _send_backend_action(self, action: str, params: Dict[str, Any]) -> Dict[str, Any]:
        backend_request_id = self.bus.send_action(self.backend_request_topic, action, params)
        return self._wait_backend_response(backend_request_id, float(self.response_timeout_s))

    def _handle_query(self, request_id: str, action: str) -> None:
        query_state_key = UAV.QueryToState.get(action)
        self.enqueue_response(request_id, action, True, {query_state_key: self.state.get(query_state_key)})

    def _handle_request(self, request: Dict[str, Any]) -> None:
        request_id = str(request["request_id"])
        action = request["action"]
        params = request.get("params") or {}
        query_state_key = UAV.QueryToState.get(action)
        if query_state_key is not None:
            self._handle_query(request_id, action)
            return
        try:
            backend_response = self._send_backend_action(action, params)
            self.enqueue_response(request_id, action, bool(backend_response["ok"]), backend_response["data"])
        except ControllerActionError as exc:
            self.enqueue_response(request_id, action, False, {"error": str(exc)})

    def run(self) -> None:
        self.send_online()
        try:
            while True:
                self.flush_queue(self.response_queue, self.response_topic)
                deadline = time.monotonic() + self.poll_interval_s
                topic, payload = self._pump_controller_once(deadline)
                if topic is None:
                    continue
                if topic == self.request_topic:
                    self._handle_request(payload["data"])
                    self.flush_queue(self.response_queue, self.response_topic)
        except RuntimeError:
            self.publish_error(traceback.format_exc().strip())
            raise
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    UavController(cfg, bus_config).run()
