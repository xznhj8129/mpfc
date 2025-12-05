#!/usr/bin/env python3
"""
MavSDK interface plugin.
Usage:
    from plugins.mavsdk_interface.mavsdk_interface import run_plugin
    run_plugin(cfg, bus_config)
"""

import asyncio
import queue
import threading
import traceback
from typing import Any, Dict

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.telemetry import Position

from lib.common import build_envelope
from lib.plugin_base import PluginBase
from plugins.mavsdk_interface.actions import (
    ACTION_ARM,
    ACTION_GET_ALTITUDE,
    ACTION_GET_POSITION,
    ACTION_GLOBAL_OK,
    ACTION_HOME_OK,
    ACTION_IS_ARMED,
    ACTION_IS_IN_AIR,
    ACTION_LAND,
    ACTION_SET_TAKEOFF_ALTITUDE,
    ACTION_TAKEOFF,
)

REQUEST_QUEUE_TIMEOUT_S = 0.05
POLL_INTERVAL_S = 0.1


class MavsdkInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.system_address = cfg["system_address"]

        self.request_topic = f"{self.client_id}.MAVSDK.REQUEST"
        self.response_topic = f"{self.client_id}.MAVSDK.RESPONSE"
        self.state_topic = f"{self.client_id}.MAVSDK.STATE"
        self.client.subscribe(self.request_topic)

        self.drone = System()
        self.state: Dict[str, Any] = {
            "is_in_air": None,
            "is_armed": None,
            "is_home_position_ok": None,
            "is_global_position_ok": None,
            "altitude_m": None,
            "position": None,
        }

        self.request_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.response_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.state_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.loop_error: BaseException | None = None
        self.loop_error_trace: str | None = None
        self.loop_thread: threading.Thread | None = None

    def _queue_response(self, request_id: str, action: str, ok: bool, data: Dict[str, Any]) -> None:
        payload = {"request_id": request_id, "action": action, "ok": ok, "data": data}
        self.response_queue.put(payload)

    def _queue_state(self) -> None:
        snapshot = {
            "is_in_air": self.state["is_in_air"],
            "is_armed": self.state["is_armed"],
            "is_home_position_ok": self.state["is_home_position_ok"],
            "is_global_position_ok": self.state["is_global_position_ok"],
            "altitude_m": self.state["altitude_m"],
            "position": self.state.get("position"),
        }
        self.state_queue.put(snapshot)

    def _update_state(self, key: str, value: Any) -> None:
        current = self.state.get(key)
        if current == value:
            return
        self.state[key] = value
        self._queue_state()

    async def _process_requests(self) -> None:
        while not self.stop_event.is_set():
            try:
                request = await asyncio.to_thread(self.request_queue.get, timeout=REQUEST_QUEUE_TIMEOUT_S)
            except queue.Empty:
                await asyncio.sleep(POLL_INTERVAL_S)
                continue
            await self._handle_action(request)

    async def _handle_action(self, request: Dict[str, Any]) -> None:
        request_id = str(request["request_id"])
        action = request["action"]
        params = request.get("params") or {}
        try:
            if action == ACTION_IS_IN_AIR:
                self._queue_response(request_id, action, True, {"is_in_air": self.state["is_in_air"]})
                return
            if action == ACTION_IS_ARMED:
                self._queue_response(request_id, action, True, {"is_armed": self.state["is_armed"]})
                return
            if action == ACTION_HOME_OK:
                self._queue_response(request_id, action, True, {"is_home_position_ok": self.state["is_home_position_ok"]})
                return
            if action == ACTION_GLOBAL_OK:
                self._queue_response(request_id, action, True, {"is_global_position_ok": self.state["is_global_position_ok"]})
                return
            if action == ACTION_GET_ALTITUDE:
                self._queue_response(request_id, action, True, {"altitude_m": self.state["altitude_m"]})
                return
            if action == ACTION_GET_POSITION:
                self._queue_response(request_id, action, True, {"position": self.state["position"]})
                return
            if action == ACTION_SET_TAKEOFF_ALTITUDE:
                altitude_m = float(params["altitude_m"])
                await self.drone.action.set_takeoff_altitude(altitude_m)
                self._queue_response(request_id, action, True, {"altitude_m": altitude_m})
                return
            if action == ACTION_ARM:
                await self.drone.action.arm()
                self._queue_response(request_id, action, True, {})
                return
            if action == ACTION_TAKEOFF:
                await self.drone.action.takeoff()
                self._queue_response(request_id, action, True, {})
                return
            if action == ACTION_LAND:
                await self.drone.action.land()
                self._queue_response(request_id, action, True, {})
                return
            self._queue_response(request_id, action, False, {"error": f"unknown action {action}"})
        except ActionError as exc:
            self._queue_response(request_id, action, False, {"error": str(exc)})

    async def _watch_in_air(self) -> None:
        async for in_air in self.drone.telemetry.in_air():
            self._update_state("is_in_air", bool(in_air))
            if self.stop_event.is_set():
                return

    async def _watch_armed(self) -> None:
        async for armed in self.drone.telemetry.armed():
            self._update_state("is_armed", bool(armed))
            if self.stop_event.is_set():
                return

    async def _watch_health(self) -> None:
        async for health in self.drone.telemetry.health():
            self._update_state("is_home_position_ok", bool(health.is_home_position_ok))
            self._update_state("is_global_position_ok", bool(health.is_global_position_ok))
            if self.stop_event.is_set():
                return

    async def _watch_position(self) -> None:
        async for position in self.drone.telemetry.position():
            self.state["altitude_m"] = position.relative_altitude_m
            self.state["position"] = {
                "lat_deg": position.latitude_deg,
                "lon_deg": position.longitude_deg,
                "abs_alt_m": position.absolute_altitude_m,
                "rel_alt_m": position.relative_altitude_m,
            }
            self._queue_state()
            if self.stop_event.is_set():
                return

    async def _async_main(self) -> None:
        print(
            f"[PLUGIN_CONN] id={self.client_id} connecting address={self.system_address}",
            flush=True,
        )
        await self.drone.connect(system_address=self.system_address)
        print(
            f"[PLUGIN_CONN] id={self.client_id} connected address={self.system_address}",
            flush=True,
        )
        tasks = [
            asyncio.create_task(self._process_requests()),
            asyncio.create_task(self._watch_in_air()),
            asyncio.create_task(self._watch_armed()),
            asyncio.create_task(self._watch_health()),
            asyncio.create_task(self._watch_position()),
        ]
        try:
            while not self.stop_event.is_set():
                for task in tasks:
                    if task.done():
                        exc = task.exception()
                        if exc:
                            raise exc
                await asyncio.sleep(POLL_INTERVAL_S)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    def _loop_runner(self) -> None:
        try:
            asyncio.run(self._async_main())
        except BaseException as exc:
            self.loop_error = exc
            self.loop_error_trace = traceback.format_exc().strip()

    def _flush_state(self) -> None:
        while True:
            try:
                state = self.state_queue.get_nowait()
            except queue.Empty:
                return
            payload = build_envelope(self.client_id, self.state_topic, state)
            self.client.publish(self.state_topic, payload)

    def _flush_responses(self) -> None:
        while True:
            try:
                response = self.response_queue.get_nowait()
            except queue.Empty:
                return
            payload = build_envelope(self.client_id, self.response_topic, response)
            self.client.publish(self.response_topic, payload)

    def run(self) -> None:
        if self.loop_thread is None:
            self.stop_event.clear()
            self.loop_thread = threading.Thread(target=self._loop_runner, name="mavsdk-loop", daemon=True)
            self.loop_thread.start()
        try:
            while True:
                self._flush_state()
                self._flush_responses()
                topic, payload, message = self.recv_message(POLL_INTERVAL_S)
                if topic is None:
                    continue
                if topic == self.request_topic:
                    self.request_queue.put(payload["data"])
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            if self.loop_thread:
                self.loop_thread.join(timeout=5.0)
                self.loop_thread = None
            self._flush_state()
            self._flush_responses()
            if self.loop_error:
                error_topic = f"DIAG.{self.client_id}.ERROR"
                error_payload = build_envelope(
                    self.client_id,
                    error_topic,
                    {
                        "event": "ERROR",
                        "traceback": self.loop_error_trace
                        or traceback.format_exception_only(type(self.loop_error), self.loop_error)[-1].strip(),
                    },
                )
                self.client.publish(error_topic, error_payload)
                raise self.loop_error
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MavsdkInterface(cfg, bus_config).run()
