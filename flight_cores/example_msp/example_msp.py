#!/usr/bin/env python3
"""Telemetry snapshot program for an MSP-backed UAV."""

from __future__ import annotations

import time
from typing import Any, Dict

from lib.common import apply_cfg
from lib.core_base import CoreBase
from lib.bus_topics import (
    ANGULAR_VELOCITY,
    ATTITUDE,
    CONTROL_OUTPUT,
    CONTROL_OVERRIDE,
    FLIGHT_CONTROL,
    GNSS,
    IMU,
    LOCATION,
    POWER,
    RC_TELEMETRY,
    RUNTIME_LOAD,
)
from lib.mavlink_models import (
    AngularVelocity,
    Attitude,
    BatteryStatus,
    ControlAxes,
    ControlOverride,
    GpsRawInt,
    HighresImu,
    RuntimeLoad,
    VehicleControl,
)
from lib.uav_client import UavClient


class ExampleMspCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.uav = UavClient(self, cfg["interface"], float(self.response_timeout_s))
        self.state_keys = [
            FLIGHT_CONTROL,
            LOCATION,
            ATTITUDE,
            ANGULAR_VELOCITY,
            GNSS,
            POWER,
            IMU,
            RC_TELEMETRY,
            CONTROL_OVERRIDE,
            CONTROL_OUTPUT,
            RUNTIME_LOAD,
        ]
        self.init_bus(float(self.poll_interval_s), self.uav.state_topics(self.state_keys), self.uav.response_topic)

    def _print_snapshot(self) -> None:
        print("\n=== MSP Snapshot ===", flush=True)
        for key, expected in (
            (FLIGHT_CONTROL, VehicleControl),
            (LOCATION, None),
            (ATTITUDE, Attitude),
            (ANGULAR_VELOCITY, AngularVelocity),
            (GNSS, GpsRawInt),
            (POWER, BatteryStatus),
            (IMU, HighresImu),
            (RC_TELEMETRY, ControlAxes),
            (CONTROL_OVERRIDE, ControlOverride),
            (CONTROL_OUTPUT, ControlAxes),
            (RUNTIME_LOAD, RuntimeLoad),
        ):
            print(f"{key}: {self.uav.state(key, expected)}", flush=True)

    def run(self) -> None:
        self.send_online()
        self.wait_until(
            lambda: self.uav.flight_control() is not None,
            float(self.state_timeout_s),
            RuntimeError("MSP vehicle flight state timeout"),
        )
        print(f"[CORE] {self.client_id} msp_uav_state_online=True", flush=True)
        last_print = 0.0
        try:
            while True:
                self._pump_once()
                now = time.monotonic()
                if now - last_print < float(self.print_interval_s):
                    continue
                last_print = now
                self._print_snapshot()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    ExampleMspCore(cfg, bus_config).run()
