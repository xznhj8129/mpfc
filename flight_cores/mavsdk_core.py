#!/usr/bin/env python3
"""
Usage:
    from flight_cores.mavsdk_core import run_core
    run_core(cfg, bus_config)
"""

import asyncio
import time
import traceback
from typing import Any, Dict

from mavsdk import System
from mavsdk.action import ActionError

from lib.common import CONTROL_SHUTDOWN_TOPIC, build_envelope
from lib.core_base import CoreBase

ALTITUDE_ROUND_DIGITS = 3
HOLD_POLL_INTERVAL_S = 0.1
CONNECTION_TIMEOUT_S = 30.0
HEALTH_TIMEOUT_S = 30.0
AIRBORNE_TIMEOUT_S = 45.0
LAND_TIMEOUT_S = 60.0
ACTION_RETRY_DELAY_S = 1.0
ACTION_RETRY_TIMEOUT_S = 30.0
ARMED_TIMEOUT_S = 20.0
TASK_CANCEL_TIMEOUT_S = 2.0


class MavsdkCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.system_address = cfg["system_address"]
        self.takeoff_altitude = float(cfg["takeoff_altitude_m"])
        self.hold_duration = float(cfg["post_takeoff_hold_s"])
        self.drone = System()

    async def _log_altitude(self) -> None:
        last_alt = None
        async for position in self.drone.telemetry.position():
            altitude = round(position.relative_altitude_m, ALTITUDE_ROUND_DIGITS)
            if altitude != last_alt:
                last_alt = altitude
                print(f"[CORE_ALT] id={self.client_id} altitude_m={altitude}", flush=True)

    async def _log_flight_mode(self) -> None:
        last_mode = None
        async for flight_mode in self.drone.telemetry.flight_mode():
            if flight_mode != last_mode:
                last_mode = flight_mode
                print(f"[CORE_MODE] id={self.client_id} mode={flight_mode}", flush=True)

    async def _await_connection(self) -> None:
        print(
            f"[CORE_CONN] id={self.client_id} connecting address={self.system_address} timeout_s={CONNECTION_TIMEOUT_S}",
            flush=True,
        )
        try:
            await asyncio.wait_for(self.drone.connect(system_address=self.system_address), timeout=CONNECTION_TIMEOUT_S)
        except asyncio.TimeoutError as exc:
            raise RuntimeError(f"connect timeout system_address={self.system_address}") from exc
        start = time.monotonic()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"[CORE_CONN] id={self.client_id} connected=True", flush=True)
                return
            if time.monotonic() - start >= CONNECTION_TIMEOUT_S:
                raise RuntimeError(
                    f"connection_state timeout system_address={self.system_address} timeout_s={CONNECTION_TIMEOUT_S}"
                )
        raise RuntimeError("connection_state stream ended before connected")

    async def _await_health(self) -> None:
        start = time.monotonic()
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(
                    f"[CORE_HEALTH] id={self.client_id} global_ok={health.is_global_position_ok} home_ok={health.is_home_position_ok}",
                    flush=True,
                )
                return
            if time.monotonic() - start >= HEALTH_TIMEOUT_S:
                raise RuntimeError(
                    f"health timeout system_address={self.system_address} timeout_s={HEALTH_TIMEOUT_S} "
                    f"global_ok={health.is_global_position_ok} home_ok={health.is_home_position_ok}"
                )
        raise RuntimeError("health stream ended before global/home ok")

    async def _await_armed(self) -> None:
        start = time.monotonic()
        async for is_armed in self.drone.telemetry.armed():
            if is_armed:
                print(f"[CORE_ARM_STATE] id={self.client_id} armed=True", flush=True)
                return
            if time.monotonic() - start >= ARMED_TIMEOUT_S:
                raise RuntimeError(f"arm wait timeout system_address={self.system_address} timeout_s={ARMED_TIMEOUT_S}")
        raise RuntimeError("armed stream ended before armed")

    async def _observe_air(self, airborne_event: asyncio.Event, landed_event: asyncio.Event) -> None:
        seen_in_air = False
        async for in_air in self.drone.telemetry.in_air():
            if in_air and not seen_in_air:
                seen_in_air = True
                airborne_event.set()
                print(f"[CORE_AIR] id={self.client_id} in_air=True", flush=True)
            if seen_in_air and not in_air:
                landed_event.set()
                print(f"[CORE_AIR] id={self.client_id} in_air=False", flush=True)
                return

    async def _perform_action(self, op_name: str, op_coro_factory) -> None:
        start = time.monotonic()
        attempt = 0
        while True:
            attempt += 1
            try:
                await op_coro_factory()
                print(f"[CORE_ACTION] id={self.client_id} op={op_name} attempt={attempt} status=OK", flush=True)
                return
            except ActionError as exc:
                elapsed = time.monotonic() - start
                print(
                    f"[CORE_ACTION] id={self.client_id} op={op_name} attempt={attempt} elapsed_s={round(elapsed,2)} error={exc}",
                    flush=True,
                )
                if elapsed >= ACTION_RETRY_TIMEOUT_S:
                    raise RuntimeError(f"{op_name} failed address={self.system_address} error={exc}") from exc
                await asyncio.sleep(ACTION_RETRY_DELAY_S)

    def _shutdown_mavsdk_server(self) -> None:
        stopper = getattr(self.drone, "_stop_mavsdk_server", None)
        if stopper is not None:
            stopper()

    async def _async_main(self) -> None:
        await self._await_connection()

        altitude_task = asyncio.create_task(self._log_altitude())
        mode_task = asyncio.create_task(self._log_flight_mode())
        airborne_event = asyncio.Event()
        landed_event = asyncio.Event()
        air_task = asyncio.create_task(self._observe_air(airborne_event, landed_event))

        await asyncio.wait_for(self._await_health(), timeout=HEALTH_TIMEOUT_S)

        print(f"[CORE_CMD] id={self.client_id} cmd=ARM", flush=True)
        await self._perform_action("ARM", lambda: self.drone.action.arm())
        await asyncio.wait_for(self._await_armed(), timeout=ARMED_TIMEOUT_S)

        print(
            f"[CORE_CMD] id={self.client_id} cmd=SET_TAKEOFF_ALT alt_m={self.takeoff_altitude} hold_s={self.hold_duration}",
            flush=True,
        )
        await self._perform_action(
            "SET_TAKEOFF_ALTITUDE", lambda: self.drone.action.set_takeoff_altitude(self.takeoff_altitude)
        )

        print(f"[CORE_CMD] id={self.client_id} cmd=TAKEOFF", flush=True)
        await self._perform_action("TAKEOFF", lambda: self.drone.action.takeoff())

        await asyncio.wait_for(airborne_event.wait(), timeout=AIRBORNE_TIMEOUT_S)
        hold_start = time.monotonic()
        while time.monotonic() - hold_start < self.hold_duration:
            await asyncio.sleep(HOLD_POLL_INTERVAL_S)

        print(f"[CORE_CMD] id={self.client_id} cmd=LAND", flush=True)
        await self._perform_action("LAND", lambda: self.drone.action.land())
        await asyncio.wait_for(landed_event.wait(), timeout=LAND_TIMEOUT_S)

        air_task.cancel()
        altitude_task.cancel()
        mode_task.cancel()
        done, pending = await asyncio.wait(
            (air_task, altitude_task, mode_task),
            timeout=TASK_CANCEL_TIMEOUT_S,
            return_when=asyncio.ALL_COMPLETED,
        )
        for task in pending:
            task.cancel()
        self._shutdown_mavsdk_server()
        self.drone._stop_mavsdk_server()
        print(f"[CORE_CMD] Finished")
        self.publish_shutdown()

    def run(self) -> None:
        try:
            asyncio.run(self._async_main())
        except KeyboardInterrupt:
            pass
        except BaseException:
            crash_tb = traceback.format_exc().strip()
            print(f"[CORE_CRASH] id={self.client_id} error={crash_tb}", flush=True)
            error_topic = f"DIAG.{self.client_id}.ERROR"
            error_payload = build_envelope(self.client_id, error_topic, {"event": "ERROR", "traceback": crash_tb})
            self.client.publish(error_topic, error_payload)
            self.publish_shutdown()
            raise
        finally:
            self._shutdown_mavsdk_server()
            self.finish(0)


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MavsdkCore(cfg, bus_config).run()
