#!/usr/bin/env python3
"""
Usage:
    from flight_cores.mavlink_core import run_core
    # cfg and bus_config must be loaded from a HiveOS config file, e.g. config/config_mavlink.json
    run_core(cfg, bus_config)
"""

import asyncio
import traceback
from typing import Any, Dict

from mavsdk import System

from lib.common import build_envelope
from lib.core_base import CoreBase

ALT_TOPIC = "telemetry.altitude"
MODE_TOPIC = "telemetry.flight_mode"
AIR_TOPIC = "telemetry.in_air"
STATUS_TOPIC = "status"


class MavlinkCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.system_address = cfg["system_address"]
        self.takeoff_altitude = float(cfg["takeoff_altitude_m"])
        self.hold_duration = float(cfg["post_takeoff_hold_s"])
        self.sysid = int(cfg["sysid"])
        self.autopilot = cfg["mav_autopilot"]
        self.vehicle_type = cfg["mav_type"]
        self.status_topic = f"{self.client_id}.{STATUS_TOPIC}"
        self.alt_topic = f"{self.client_id}.{ALT_TOPIC}"
        self.mode_topic = f"{self.client_id}.{MODE_TOPIC}"
        self.air_topic = f"{self.client_id}.{AIR_TOPIC}"

    def run(self) -> None:
        try:
            asyncio.run(self._main())
        except RuntimeError:
            error_topic = f"DIAG.{self.client_id}.ERROR"
            error_payload = build_envelope(
                self.client_id, error_topic, {"event": "ERROR", "traceback": traceback.format_exc().strip()}
            )
            self.client.publish(error_topic, error_payload)
            raise
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    async def _main(self) -> None:
        drone = System()
        await drone.connect(system_address=self.system_address)
        print(f"[CORE_CONNECT] id={self.client_id} sysid={self.sysid} addr={self.system_address}", flush=True)
        async for state in drone.core.connection_state():
            if state.is_connected:
                self.client.publish(
                    self.status_topic,
                    build_envelope(
                        self.client_id,
                        self.status_topic,
                        {
                            "event": "CONNECTED",
                            "system_id": self.sysid,
                            "autopilot": self.autopilot,
                            "vehicle": self.vehicle_type,
                            "address": self.system_address,
                        },
                    ),
                )
                print(f"[CORE_READY] id={self.client_id} system_connected={state.is_connected}", flush=True)
                break

        altitude_task = asyncio.create_task(self._watch_altitude(drone))
        mode_task = asyncio.create_task(self._watch_flight_mode(drone))
        running_tasks = [altitude_task, mode_task]
        termination_task = asyncio.create_task(self._observe_in_air(drone, running_tasks))

        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.client.publish(
                    self.status_topic,
                    build_envelope(
                        self.client_id,
                        self.status_topic,
                        {"event": "HEALTH_OK", "global_position": health.is_global_position_ok, "home": health.is_home_position_ok},
                    ),
                )
                print(
                    f"[CORE_HEALTH] id={self.client_id} global_ok={health.is_global_position_ok} home_ok={health.is_home_position_ok}",
                    flush=True,
                )
                break

        print(f"[CORE_ARM] id={self.client_id}", flush=True)
        self.client.publish(
            self.status_topic,
            build_envelope(self.client_id, self.status_topic, {"event": "ARMING", "address": self.system_address}),
        )
        await drone.action.arm()

        print(f"[CORE_TAKEOFF] id={self.client_id} altitude_m={self.takeoff_altitude}", flush=True)
        self.client.publish(
            self.status_topic,
            build_envelope(
                self.client_id,
                self.status_topic,
                {"event": "TAKEOFF", "altitude_m": self.takeoff_altitude, "hold_s": self.hold_duration},
            ),
        )
        await drone.action.set_takeoff_altitude(self.takeoff_altitude)
        await drone.action.takeoff()

        await asyncio.sleep(self.hold_duration)

        print(f"[CORE_LAND] id={self.client_id}", flush=True)
        self.client.publish(
            self.status_topic,
            build_envelope(self.client_id, self.status_topic, {"event": "LANDING", "address": self.system_address}),
        )
        await drone.action.land()
        await termination_task

    async def _watch_altitude(self, drone: System) -> None:
        previous_altitude = None
        try:
            async for position in drone.telemetry.position():
                altitude = round(position.relative_altitude_m, 3)
                if altitude != previous_altitude:
                    previous_altitude = altitude
                    self.client.publish(
                        self.alt_topic,
                        build_envelope(self.client_id, self.alt_topic, {"relative_m": altitude}),
                    )
                    print(f"[CORE_ALT] id={self.client_id} altitude_m={altitude}", flush=True)
        except asyncio.CancelledError:
            return

    async def _watch_flight_mode(self, drone: System) -> None:
        previous_mode = None
        try:
            async for flight_mode in drone.telemetry.flight_mode():
                if flight_mode != previous_mode:
                    previous_mode = flight_mode
                    self.client.publish(
                        self.mode_topic,
                        build_envelope(
                            self.client_id, self.mode_topic, {"flight_mode": str(flight_mode), "sysid": self.sysid}
                        ),
                    )
                    print(f"[CORE_MODE] id={self.client_id} mode={flight_mode}", flush=True)
        except asyncio.CancelledError:
            return

    async def _observe_in_air(self, drone: System, running_tasks) -> None:
        was_in_air = False
        try:
            async for is_in_air in drone.telemetry.in_air():
                if is_in_air:
                    was_in_air = True
                    self.client.publish(
                        self.air_topic, build_envelope(self.client_id, self.air_topic, {"in_air": True})
                    )
                    print(f"[CORE_AIR] id={self.client_id} in_air={is_in_air}", flush=True)
                if was_in_air and not is_in_air:
                    self.client.publish(
                        self.air_topic, build_envelope(self.client_id, self.air_topic, {"in_air": False})
                    )
                    print(f"[CORE_AIR] id={self.client_id} in_air={is_in_air}", flush=True)
                    for task in running_tasks:
                        task.cancel()
                        try:
                            await task
                        except asyncio.CancelledError:
                            pass
                    await asyncio.get_event_loop().shutdown_asyncgens()
                    return
        except asyncio.CancelledError:
            return


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MavlinkCore(cfg, bus_config).run()
