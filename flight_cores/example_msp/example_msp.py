#!/usr/bin/env python3
"""
Usage:
    from flight_cores.example_msp.example_msp import run_core
    run_core(cfg, bus_config)
"""

import time
from typing import Any, Dict

from lib.common import apply_cfg, build_state_topics, build_topic_base
from lib.core_base import CoreBase
from protocols.namespace_loader import load_protocol_namespace

UAV = load_protocol_namespace("uav")


class ExampleMspCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        interface_cfg = cfg["interface"]
        base = build_topic_base(interface_cfg["id"], interface_cfg["topic_ns"])
        self.state_keys = [
            UAV.State.System.FcConnected,
            UAV.State.System.FcInfo,
            UAV.State.Sensor.SensorConfig,
            UAV.State.Control.ModeRanges,
            UAV.State.Control.RxConfig,
            UAV.State.Control.ChannelMap,
            UAV.State.Power.Analog,
            UAV.State.Power.Battery,
            UAV.State.Navigation.GpsInfo,
            UAV.State.System.GpsStatistics,
            UAV.State.Navigation.RawGps,
            UAV.State.System.WaypointInfo,
            UAV.State.Attitude.AttitudeRad,
            UAV.State.Attitude.AngVelRadS,
            UAV.State.Sensor.Imu,
            UAV.State.Navigation.AltitudeM,
            UAV.State.Navigation.Position,
            UAV.State.Navigation.NavState,
            UAV.State.Control.RcTelemetry,
            UAV.State.Control.ControlOverride,
            UAV.State.Control.ControlOutput,
            UAV.State.Flight.ActiveModeNames,
            UAV.State.System.FlightMode,
            UAV.State.System.CpuLoad,
            UAV.State.System.CycleTime,
        ]
        state_topics = build_state_topics(base, self.state_keys)
        self.init_bus(float(self.poll_interval_s), state_topics)

    def _print_snapshot(self) -> None:  # Print one structured telemetry snapshot.
        snapshot = self.state
        print("\n=== MSP Snapshot ===", flush=True)
        print(f"FcConnected: {snapshot.get(UAV.State.System.FcConnected)}", flush=True)
        print(f"FcInfo: {snapshot.get(UAV.State.System.FcInfo)}", flush=True)
        print(f"SensorConfig: {snapshot.get(UAV.State.Sensor.SensorConfig)}", flush=True)
        print(f"ModeRanges: {snapshot.get(UAV.State.Control.ModeRanges)}", flush=True)
        print(f"RxConfig: {snapshot.get(UAV.State.Control.RxConfig)}", flush=True)
        print(f"ChannelMap: {snapshot.get(UAV.State.Control.ChannelMap)}", flush=True)
        print(f"Analog: {snapshot.get(UAV.State.Power.Analog)}", flush=True)
        print(f"Battery: {snapshot.get(UAV.State.Power.Battery)}", flush=True)
        print(f"GpsInfo: {snapshot.get(UAV.State.Navigation.GpsInfo)}", flush=True)
        print(f"GpsStatistics: {snapshot.get(UAV.State.System.GpsStatistics)}", flush=True)
        print(f"RawGps: {snapshot.get(UAV.State.Navigation.RawGps)}", flush=True)
        print(f"WaypointInfo: {snapshot.get(UAV.State.System.WaypointInfo)}", flush=True)
        print(f"AttitudeRad: {snapshot.get(UAV.State.Attitude.AttitudeRad)}", flush=True)
        print(f"AngVelRadS: {snapshot.get(UAV.State.Attitude.AngVelRadS)}", flush=True)
        print(f"Imu: {snapshot.get(UAV.State.Sensor.Imu)}", flush=True)
        print(f"AltitudeM: {snapshot.get(UAV.State.Navigation.AltitudeM)}", flush=True)
        print(f"Position: {snapshot.get(UAV.State.Navigation.Position)}", flush=True)
        print(f"NavState: {snapshot.get(UAV.State.Navigation.NavState)}", flush=True)
        print(f"RcTelemetry: {snapshot.get(UAV.State.Control.RcTelemetry)}", flush=True)
        print(f"ControlOverride: {snapshot.get(UAV.State.Control.ControlOverride)}", flush=True)
        print(f"ControlOutput: {snapshot.get(UAV.State.Control.ControlOutput)}", flush=True)
        print(f"ActiveModeNames: {snapshot.get(UAV.State.Flight.ActiveModeNames)}", flush=True)
        print(f"FlightMode: {snapshot.get(UAV.State.System.FlightMode)}", flush=True)
        print(f"CpuLoad: {snapshot.get(UAV.State.System.CpuLoad)}", flush=True)
        print(f"CycleTime: {snapshot.get(UAV.State.System.CycleTime)}", flush=True)

    def run(self) -> None:
        self.send_online()
        self.wait_for_state(
            UAV.State.System.FcConnected,
            float(self.state_timeout_s),
            RuntimeError("msp fc connection timeout"),
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
    ExampleMspCore(cfg, bus_config).run()
