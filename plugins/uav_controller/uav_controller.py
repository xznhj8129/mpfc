#!/usr/bin/env python3
"""Stable UAV service plugin for MPFC flight cores.

Programs talk to this plugin as the stable UAV API.  It applies reusable
vehicle-level policy, gates MAVLink command records, forwards commands without
blocking on endpoint mechanics, and relays high-rate control samples on a
latest-value path.  Vehicle-facing state is MAVLink-shaped (DEC-1).
"""

from __future__ import annotations

import time
import traceback
from dataclasses import replace
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
from lib.bus_topics import FLIGHT_CONTROL, LOCATION
from lib.lattice_bus import (
    decode_command,
    decode_input,
    send_command,
    send_input,
)
from lib.mavlink_models import (
    AttitudeTarget,
    CommandLong,
    ControlOverride,
    DirectControl,
    GlobalPositionInt,
    ParamSet,
    RepositionCommand,
    SetMode,
    VehicleControl,
    unpack_record,
)
from lib.plugin_base import PluginBase


class UavController(PluginBase):
    """Backend-independent UAV service built on MAVLink command/state records."""

    IMMEDIATE_COMMAND_TYPES = (
        CommandLong,
        RepositionCommand,
        SetMode,
        ParamSet,
        DirectControl,
    )
    DIRECT_INPUT_TYPES = (
        AttitudeTarget,
        ControlOverride,
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
        self.target_system = int(cfg.get("target_system", 0))
        self.arm_ready_since: float | None = None
        self.takeoff_ready_since: float | None = None
        self.backend_flight_control: VehicleControl | None = None
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
        return str(self.vehicle.get("autopilot", "")).upper() == "ARDUPILOT"

    def _location_available(self) -> bool:
        payload = self.state.get(LOCATION)
        if payload is None:
            return False
        try:
            return isinstance(unpack_record(payload, GlobalPositionInt), GlobalPositionInt)
        except (TypeError, ValueError, KeyError):
            return False

    def _apply_readiness_policy(self, control: VehicleControl) -> VehicleControl:
        readiness = control.readiness
        nav = control.navigation_validity

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

        readiness = replace(
            readiness,
            arm_ready=(
                arm_candidate
                and self.arm_ready_since is not None
                and now - self.arm_ready_since >= arm_hold_s
            ),
            takeoff_ready=(
                takeoff_candidate
                and self.takeoff_ready_since is not None
                and now - self.takeoff_ready_since >= takeoff_hold_s
            ),
        )
        return replace(control, readiness=readiness, navigation_validity=nav)

    def _publish_controller_flight_control(self) -> None:
        if self.backend_flight_control is None or FLIGHT_CONTROL not in self.state_publish_topics:
            return
        control = self._apply_readiness_policy(self.backend_flight_control)
        self._publish_state(FLIGHT_CONTROL, control.to_bus())

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
                model = unpack_record(state_payload, VehicleControl)
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
            request_id, command = decode_command(request)
            command_name = type(command).__name__
            if type(command) not in self.IMMEDIATE_COMMAND_TYPES:
                allowed = ", ".join(command_type.__name__ for command_type in self.IMMEDIATE_COMMAND_TYPES)
                raise TypeError(
                    f"uav_controller accepts MAVLink command records only "
                    f"allowed={allowed} actual={command_name}"
                )
            if (
                self.target_system
                and isinstance(command, CommandLong)
                and command.target_system
                and command.target_system != self.target_system
            ):
                raise ValueError(
                    f"command target_system does not address this UAV "
                    f"expected={self.target_system} actual={command.target_system}"
                )
            backend_request_id = send_command(self.bus, self.backend_request_topic, command)
            self.pending_backend_requests[backend_request_id] = (
                request_id,
                command_name,
                time.monotonic(),
            )
        except (TypeError, ValueError, KeyError) as exc:
            self.enqueue_response(request_id, command_name, False, {"error": str(exc)})

    def _handle_input(self, payload: Any) -> None:
        model = decode_input(payload)
        if not isinstance(model, self.DIRECT_INPUT_TYPES):
            allowed = ", ".join(input_type.__name__ for input_type in self.DIRECT_INPUT_TYPES)
            raise TypeError(
                f"uav_controller accepts direct UAV input types only allowed={allowed} actual={type(model).__name__}"
            )
        send_input(self.bus, self.backend_input_topic, model)

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
