#!/usr/bin/env python3
"""
Usage:
    from flight_cores.test_takeoff_land.test_takeoff_land import run_core
    run_core(cfg, bus_config)
"""

import time
import traceback
from typing import Any, Dict, Callable

from lib.common import CONTROL_SHUTDOWN_TOPIC, build_envelope
from lib.core_base import CoreBase
from plugins.mavsdk_interface.actions import (
    ACTION_ARM,
    ACTION_LAND,
    ACTION_SET_TAKEOFF_ALTITUDE,
    ACTION_TAKEOFF,
)

POLL_INTERVAL_S = 0.5
STATE_TIMEOUT_S = 60.0
RESPONSE_TIMEOUT_S = 15.0
ALTITUDE_OK_FRACTION = 0.8


class TakeoffLandCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.takeoff_altitude = float(cfg["takeoff_altitude_m"])
        self.hold_duration = float(cfg["post_takeoff_hold_s"])
        self.mavsdk_id = cfg["mavsdk_id"]

        base_topic = f"{self.mavsdk_id}.MAVSDK"
        self.request_topic = f"{base_topic}.REQUEST"
        self.response_topic = f"{base_topic}.RESPONSE"
        self.state_topic = f"{base_topic}.STATE"
        self.client.subscribe(self.state_topic)
        self.client.subscribe(self.response_topic)

        self.state: Dict[str, Any] = {}
        self.responses: Dict[str, Dict[str, Any]] = {}
        self.request_counter = 0

    def _next_request_id(self) -> str:
        self.request_counter += 1
        return f"req-{self.request_counter}"

    def _send_action(self, action: str, params: Dict[str, Any] | None = None) -> str:
        request_id = self._next_request_id()
        payload = {"request_id": request_id, "action": action, "params": params or {}}
        envelope = build_envelope(self.client_id, self.request_topic, payload)
        self.client.publish(self.request_topic, envelope)
        return request_id

    def _pump_once(self, deadline: float | None = None) -> None:
        timeout = POLL_INTERVAL_S
        if deadline is not None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                timeout = 0
            else:
                timeout = min(timeout, remaining)
        topic, payload, message = self.recv_message(timeout)
        if topic is None:
            return
        if topic == self.state_topic:
            self.state = payload["data"]
            if self.state.get("altitude_m") is not None:
                print(
                    f"[CORE_STATE] id={self.client_id} in_air={self.state.get('is_in_air')} "
                    f"armed={self.state.get('is_armed')} alt_m={round(self.state.get('altitude_m'),3)}",
                    flush=True,
                )
            return
        if topic == self.response_topic:
            data = payload["data"]
            self.responses[data["request_id"]] = data
            return

    def _wait_response(self, request_id: str, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        while True:
            if request_id in self.responses:
                return self.responses.pop(request_id)
            if time.monotonic() > deadline:
                raise RuntimeError(f"timeout waiting for response id={request_id}")
            self._pump_once(deadline)

    def _wait_state(self, predicate: Callable[[Dict[str, Any]], bool], timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        while True:
            if predicate(self.state):
                return self.state
            if time.monotonic() > deadline:
                raise RuntimeError("state wait timed out")
            self._pump_once(deadline)

    def run(self) -> None:
        try:
            self._wait_state(
                lambda s: bool(s.get("is_home_position_ok")) and bool(s.get("is_global_position_ok")),
                STATE_TIMEOUT_S,
            )
            print(
                f"[CORE_HEALTH] id={self.client_id} home_ok={self.state.get('is_home_position_ok')} "
                f"global_ok={self.state.get('is_global_position_ok')}",
                flush=True,
            )

            takeoff_alt_req = self._send_action(ACTION_SET_TAKEOFF_ALTITUDE, {"altitude_m": self.takeoff_altitude})
            print(
                f"[CORE_CMD] id={self.client_id} cmd=SET_TAKEOFF_ALT alt_m={self.takeoff_altitude} req={takeoff_alt_req}",
                flush=True,
            )
            resp = self._wait_response(takeoff_alt_req, RESPONSE_TIMEOUT_S)
            if not resp.get("ok"):
                raise RuntimeError(f"set_takeoff_altitude failed: {resp}")

            arm_req = self._send_action(ACTION_ARM, {})
            print(f"[CORE_CMD] id={self.client_id} cmd=ARM req={arm_req}", flush=True)
            resp = self._wait_response(arm_req, RESPONSE_TIMEOUT_S)
            if not resp.get("ok"):
                raise RuntimeError(f"arm failed: {resp}")
            self._wait_state(lambda s: bool(s.get("is_armed")), STATE_TIMEOUT_S)
            print(f"[CORE_STATE] id={self.client_id} armed=True", flush=True)

            takeoff_req = self._send_action(ACTION_TAKEOFF, {})
            print(f"[CORE_CMD] id={self.client_id} cmd=TAKEOFF req={takeoff_req}", flush=True)
            resp = self._wait_response(takeoff_req, RESPONSE_TIMEOUT_S)
            if not resp.get("ok"):
                raise RuntimeError(f"takeoff failed: {resp}")
            self._wait_state(
                lambda s: bool(s.get("is_in_air"))
                and s.get("altitude_m") is not None
                and s.get("altitude_m") >= self.takeoff_altitude * ALTITUDE_OK_FRACTION,
                STATE_TIMEOUT_S,
            )
            print(
                f"[CORE_STATE] id={self.client_id} in_air=True alt_m={self.state.get('altitude_m')}",
                flush=True,
            )

            hold_deadline = time.monotonic() + self.hold_duration
            print(
                f"[CORE_HOLD] id={self.client_id} hold_s={self.hold_duration} alt_target_m={self.takeoff_altitude}",
                flush=True,
            )
            while time.monotonic() < hold_deadline:
                self._pump_once(hold_deadline)

            land_req = self._send_action(ACTION_LAND, {})
            print(f"[CORE_CMD] id={self.client_id} cmd=LAND req={land_req}", flush=True)
            resp = self._wait_response(land_req, RESPONSE_TIMEOUT_S)
            if not resp.get("ok"):
                raise RuntimeError(f"land failed: {resp}")
            self._wait_state(lambda s: not s.get("is_in_air"), STATE_TIMEOUT_S)
            print(f"[CORE_STATE] id={self.client_id} in_air=False", flush=True)

            self.client.publish(CONTROL_SHUTDOWN_TOPIC, build_envelope(self.client_id, CONTROL_SHUTDOWN_TOPIC, {}))
        except RuntimeError:
            error_topic = f"DIAG.{self.client_id}.ERROR"
            error_payload = build_envelope(
                self.client_id,
                error_topic,
                {"event": "ERROR", "traceback": traceback.format_exc().strip()},
            )
            self.client.publish(error_topic, error_payload)
            raise
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    TakeoffLandCore(cfg, bus_config).run()
