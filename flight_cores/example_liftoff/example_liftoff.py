#!/usr/bin/env python3
"""
Usage:
    from flight_cores.example_liftoff.example_liftoff import run_core
    run_core(cfg, bus_config)
"""

import time
from typing import Any, Dict

from lib.common import apply_cfg, build_state_topics, build_topic_base
from lib.core_base import CoreBase
from protocols.namespace_loader import load_protocol_namespace

UAV = load_protocol_namespace("uav")


class ExampleLiftoffCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        interface_cfg = cfg["interface"]
        base = build_topic_base(interface_cfg["id"], interface_cfg["topic_ns"])
        self.state_keys = [
            UAV.State.System.FcConnected,
            UAV.State.Sensor.SensorConfig,
            UAV.State.System.FlightMode,
            UAV.State.Flight.IsInAir,
            UAV.State.Navigation.AltitudeM,
            UAV.State.Attitude.AttitudeRad,
            UAV.State.Control.RcTelemetry,
            UAV.State.Power.Battery,
            UAV.State.Power.Analog,
        ]
        state_topics = build_state_topics(base, self.state_keys)
        self.init_bus(float(self.poll_interval_s), state_topics)

    def _print_snapshot(self) -> None:  # Print one structured telemetry snapshot.
        snapshot = self.state
        print("\n=== Liftoff Snapshot ===", flush=True)
        print(f"FcConnected: {snapshot.get(UAV.State.System.FcConnected)}", flush=True)
        print(f"SensorConfig: {snapshot.get(UAV.State.Sensor.SensorConfig)}", flush=True)
        print(f"FlightMode: {snapshot.get(UAV.State.System.FlightMode)}", flush=True)
        print(f"IsInAir: {snapshot.get(UAV.State.Flight.IsInAir)}", flush=True)
        print(f"AltitudeM: {snapshot.get(UAV.State.Navigation.AltitudeM)}", flush=True)
        print(f"AttitudeRad: {snapshot.get(UAV.State.Attitude.AttitudeRad)}", flush=True)
        print(f"RcTelemetry: {snapshot.get(UAV.State.Control.RcTelemetry)}", flush=True)
        print(f"Battery: {snapshot.get(UAV.State.Power.Battery)}", flush=True)
        print(f"Analog: {snapshot.get(UAV.State.Power.Analog)}", flush=True)

    def run(self) -> None:
        self.send_online()
        self.wait_for_state(
            UAV.State.System.FcConnected,
            float(self.state_timeout_s),
            RuntimeError("liftoff fc connection timeout"),
            lambda value: bool(value),
        )
        print(f"[CORE] {self.client_id} fc_connected=True", flush=True)
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
    ExampleLiftoffCore(cfg, bus_config).run()
