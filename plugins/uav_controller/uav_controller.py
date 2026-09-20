#!/usr/bin/env python3
"""OCCID-native UAV service plugin.

Programs talk to this plugin as the stable UAV API. It applies reusable
vehicle-level policy, type-gates concrete generic OCCID Command families,
forwards commands without blocking on endpoint mechanics, and relays high-rate
OCCID Input samples on a latest-value path.
"""

from __future__ import annotations

import time
import traceback
from typing import Any, Dict

from lib.common import (
    apply_cfg,
    build_envelope,
    build_event_topics,
    build_request_topic,
    build_response_topic,
    build_state_topics,
    build_topic_base,
)
from lib.occid_bus import (
    decode_occid_command,
    decode_occid_input,
    occid,
    pack_occid,
    send_occid_command,
    send_occid_input,
    unpack_occid,
)
from lib.occid_topics import FLIGHT_CONTROL, LOCATION
from lib.plugin_base import PluginBase
from lib.provisioning import asset_uid


class UavController(PluginBase):
    """Backend-independent UAV service built on OCCID Commands, Inputs, and State."""

    IMMEDIATE_COMMAND_TYPES = (
        occid.StateChangeCommand,
        occid.ProcessControlCommand,
        occid.ConfigurationCommand,
        occid.MotionCommand,
        occid.ResourceCommand,
        occid.ExecutionCommand,
    )
    DIRECT_INPUT_TYPES = (
        occid.ControlAttitudeSetpoint,
        occid.ControlOverride,
    )

    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.poll_interval_s = float(cfg["poll_interval_s"])
        self.response_timeout_s = float(cfg["response_timeout_s"])
        self.vehicle = dict(cfg["vehicle"])
        self.backend = dict(cfg["backend"])
        self.backend_state_keys = list(cfg["backend_state_keys"])
        self.backend_event_keys = list(cfg.get("backend_event_keys", []))
        raw_target = cfg.get("target_uid")
        if raw_target is None:
            raw_target = asset_uid()
        self.target_uid = (
            raw_target if isinstance(raw_target, occid.UID) else occid.UID.model_validate(raw_target)
        )
        self.arm_ready_since: float | None = None
        self.takeoff_ready_since: float | None = None
        self.backend_flight_control: Any | None = None
        self.pending_backend_requests: dict[str, tuple[str, str, float]] = {}

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.input_topic = f"{base}/INPUT"
        self.state_publish_topics = build_state_topics(base, self.backend_state_keys)
        self.event_topics = build_event_topics(base, self.backend_event_keys)
        self.client.subscribe(self.request_topic)
        self.client.subscribe(self.input_topic)

        backend_base = build_topic_base(self.backend["id"], self.backend["topic_ns"])
        self.backend_request_topic = build_request_topic(self.backend["id"], self.backend["topic_ns"])
        self.backend_response_topic = build_response_topic(self.backend["id"], self.backend["topic_ns"])
        self.backend_input_topic = f"{backend_base}/INPUT"
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

    def _is_ardupilot(self) -> bool:
        return str(self.vehicle.get("autopilot", "")).upper() == occid.AutopilotType.ARDUPILOT.name

    def _location_available(self) -> bool:
        payload = self.state.get(LOCATION)
        if payload is None:
            return False
        try:
            return isinstance(unpack_occid(payload), occid.LocationState)
        except (TypeError, ValueError, KeyError):
            return False

    def _apply_readiness_policy(self, flight_control: Any) -> Any:
        readiness = flight_control.readiness
        if readiness is None:
            readiness = occid.NavReadinessState(mode_problems=[], health_problems=[])
        nav = flight_control.navigation_validity
        if nav is None:
            nav = occid.NavigationValidity()

        arm_candidate = bool(readiness.armable)
        takeoff_candidate = (
            arm_candidate
            and self._location_available()
            and bool(nav.local_position_ok)
            and bool(nav.global_position_ok)
            and bool(nav.home_position_ok)
        )

        arm_hold_s = float(self.arm_ready_hold_s)
        takeoff_hold_s = float(self.takeoff_ready_hold_s)
        if self._is_ardupilot():
            arm_hold_s = float(self.ardupilot_arm_ready_hold_s)
            takeoff_hold_s = float(self.ardupilot_takeoff_ready_hold_s)
            takeoff_candidate = takeoff_candidate and bool(readiness.ekf_using_gps)

        now = time.monotonic()
        if arm_candidate:
            if self.arm_ready_since is None:
                self.arm_ready_since = now
        else:
            self.arm_ready_since = None

        if takeoff_candidate:
            if self.takeoff_ready_since is None:
                self.takeoff_ready_since = now
        else:
            self.takeoff_ready_since = None

        readiness = readiness.model_copy(
            update={
                "arm_ready": (
                    arm_candidate
                    and self.arm_ready_since is not None
                    and now - self.arm_ready_since >= arm_hold_s
                ),
                "takeoff_ready": (
                    takeoff_candidate
                    and self.takeoff_ready_since is not None
                    and now - self.takeoff_ready_since >= takeoff_hold_s
                ),
            }
        )
        return flight_control.model_copy(update={"readiness": readiness, "navigation_validity": nav})

    def _publish_controller_flight_control(self) -> None:
        if self.backend_flight_control is None or FLIGHT_CONTROL not in self.state_publish_topics:
            return
        model = self._apply_readiness_policy(self.backend_flight_control)
        self._publish_state(FLIGHT_CONTROL, pack_occid(model))

    def _forward_backend_response(self, payload: Dict[str, Any]) -> None:
        backend_request_id = str(payload["request_id"])
        pending = self.pending_backend_requests.pop(backend_request_id, None)
        self.responses.pop(backend_request_id, None)
        if pending is None:
            return
        request_id, command_name, _ = pending
        self.enqueue_response(
            request_id,
            command_name,
            bool(payload.get("ok")),
            dict(payload.get("data") or {}),
        )

    def _expire_pending_requests(self) -> None:
        now = time.monotonic()
        expired = [
            backend_request_id
            for backend_request_id, (_, _, created_at) in self.pending_backend_requests.items()
            if now - created_at > self.response_timeout_s
        ]
        for backend_request_id in expired:
            request_id, command_name, _ = self.pending_backend_requests.pop(backend_request_id)
            self.responses.pop(backend_request_id, None)
            self.enqueue_response(
                request_id,
                command_name,
                False,
                {"error": f"backend result timed out request_id={backend_request_id}"},
            )

    def _pump_controller_once(self, deadline: float | None = None) -> tuple[Any, Any]:
        topic, payload = self._pump_once(deadline)
        if topic in self.backend_state_topic_to_key:
            state_key = self.backend_state_topic_to_key[topic]
            state_payload = payload["data"]
            if state_key == FLIGHT_CONTROL:
                model = unpack_occid(state_payload)
                if not isinstance(model, occid.FlightControlState):
                    raise RuntimeError(
                        f"backend flight_control payload must be FlightControlState actual={type(model).__name__}"
                    )
                self.backend_flight_control = model
                self._publish_controller_flight_control()
            else:
                self._publish_state(state_key, state_payload)
                if state_key == LOCATION:
                    self._publish_controller_flight_control()
        elif topic == self.backend_response_topic:
            self._forward_backend_response(payload["data"])
        elif topic in self.backend_event_topic_to_key:
            self._publish_event(self.backend_event_topic_to_key[topic], payload["data"])
        return topic, payload

    def _handle_request(self, request: Dict[str, Any]) -> None:
        request_id = str(request.get("request_id", "unknown"))
        command_name = "Command"
        try:
            request_id, command = decode_occid_command(request)
            command_name = type(command).__name__
            if type(command) not in self.IMMEDIATE_COMMAND_TYPES:
                allowed = ", ".join(command_type.__name__ for command_type in self.IMMEDIATE_COMMAND_TYPES)
                raise TypeError(
                    f"uav_controller accepts concrete OCCID Command families only "
                    f"allowed={allowed} actual={command_name}"
                )
            if command.target_uid != self.target_uid:
                raise ValueError(
                    f"command target_uid does not address this UAV "
                    f"expected={self.target_uid} actual={command.target_uid}"
                )
            backend_request_id = send_occid_command(self.bus, self.backend_request_topic, command)
            self.pending_backend_requests[backend_request_id] = (
                request_id,
                command_name,
                time.monotonic(),
            )
        except (TypeError, ValueError, KeyError) as exc:
            self.enqueue_response(request_id, command_name, False, {"error": str(exc)})

    def _handle_input(self, payload: Any) -> None:
        model = decode_occid_input(payload)
        if not isinstance(model, self.DIRECT_INPUT_TYPES):
            allowed = ", ".join(input_type.__name__ for input_type in self.DIRECT_INPUT_TYPES)
            raise TypeError(
                f"uav_controller accepts direct UAV input types only allowed={allowed} actual={type(model).__name__}"
            )
        send_occid_input(self.bus, self.backend_input_topic, model)

    def run(self) -> None:
        self.send_online()
        try:
            while True:
                self.flush_queue(self.response_queue, self.response_topic)
                self._expire_pending_requests()
                deadline = time.monotonic() + self.poll_interval_s
                topic, payload = self._pump_controller_once(deadline)
                if topic is None:
                    continue
                if topic == self.request_topic:
                    self._handle_request(payload["data"])
                elif topic == self.input_topic:
                    try:
                        self._handle_input(payload["data"])
                    except (TypeError, ValueError, KeyError) as exc:
                        error_topic = f"DIAG/{self.client_id}/INPUT_REJECTED"
                        self.client.publish(
                            error_topic,
                            build_envelope(
                                self.client_id,
                                error_topic,
                                {"event": "INPUT_REJECTED", "error": str(exc)},
                            ),
                        )
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
