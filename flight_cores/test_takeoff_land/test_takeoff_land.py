#!/usr/bin/env python3
"""
Usage:
    from flight_cores.test_takeoff_land.test_takeoff_land import run_core
    run_core(cfg, bus_config)
"""

import time
import traceback
from typing import Any, Dict

from lib.common import (
    apply_cfg,
    build_envelope,
    build_request_topic,
    build_response_topic,
    build_state_topics,
    build_topic_base,
)
from lib.core_base import CoreBase
from lib.geo_utils import GPSposition, gps_distance_m, vector_to_gps
from protocols.namespace_loader import load_protocol_namespace

UAV = load_protocol_namespace("uav")


class MissionAbort(RuntimeError):
    pass


class TakeoffLandCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        mission_cfg = cfg["mission"]
        apply_cfg(self, mission_cfg)
        self.poll_interval_s = float(mission_cfg["poll_interval_s"])
        interface_cfg = cfg["interface"]
        self.interface_id = interface_cfg["id"]
        self.interface_ns = interface_cfg["topic_ns"]

        base = build_topic_base(self.interface_id, self.interface_ns)
        self.request_topic = build_request_topic(self.interface_id, self.interface_ns)
        self.response_topic = build_response_topic(self.interface_id, self.interface_ns)

        self.state_topics = build_state_topics(
            base,
            self.state_keys,
        )

        self.init_bus(self.poll_interval_s, self.state_topics, self.response_topic)
        self.enable_state_logging(self.state_logging["keys"], self.state_logging["interval_s"], self.state_logging["prefix"])

    def run(self) -> None:
        self.send_online()
        try:
            self.wait_until(
                lambda: bool(self.state.get(UAV.State.Navigation.IsHomePositionOk))
                and bool(self.state.get(UAV.State.Navigation.IsGlobalPositionOk)),
                float(self.state_timeout_s),
                MissionAbort("health wait timed out"),
            )

            print(
                f"[CORE] {self.client_id} health home_ok={self.state.get(UAV.State.Navigation.IsHomePositionOk)} "
                f"global_ok={self.state.get(UAV.State.Navigation.IsGlobalPositionOk)}",
                flush=True,
            )

            home_state = self.wait_for_state(
                UAV.State.Navigation.Position, float(self.state_timeout_s), MissionAbort("home position wait timed out")
            )

            home_pos = GPSposition(
                float(home_state["LatDeg"]),
                float(home_state["LonDeg"]),
                float(home_state["RelAltM"]),
            )

            takeoff_alt_req = self._send_action(UAV.Action.Flight.SetTakeoffAltitude, {"AltitudeM": float(self.takeoff_altitude_m)})
            print(
                f"[CORE] {self.client_id} cmd SET_TAKEOFF_ALT alt_m={self.takeoff_altitude_m} req={takeoff_alt_req}",
                flush=True,
            )
            resp = self._wait_response(takeoff_alt_req, float(self.response_timeout_s))
            if not resp.get("ok"):
                raise MissionAbort(f"set_takeoff_altitude failed resp={resp}")

            self.wait_until(
                lambda: self.state.get(UAV.State.Sensor.SensorConfig) is not None
                and bool(self.state.get(UAV.State.Sensor.SensorConfig).get("ArmReady")),
                float(self.state_timeout_s),
                MissionAbort(
                    f"arm readiness wait timed out sensor_config={self.state.get(UAV.State.Sensor.SensorConfig)} "
                    f"modes={self.state.get(UAV.State.Flight.ActiveModeNames)}"
                ),
            )
            print(
                f"[CORE] {self.client_id} arm_ready sensor_config={self.state.get(UAV.State.Sensor.SensorConfig)} "
                f"modes={self.state.get(UAV.State.Flight.ActiveModeNames)}",
                flush=True,
            )

            arm_req = self._send_action(UAV.Action.Flight.Arm, {})
            print(f"[CORE] {self.client_id} cmd ARM req={arm_req}", flush=True)
            resp = self._wait_response(arm_req, float(self.response_timeout_s))
            if not resp.get("ok"):
                raise MissionAbort(f"arm failed resp={resp}")

            self.wait_for_state(
                UAV.State.Flight.IsArmed,
                float(self.state_timeout_s),
                MissionAbort("armed wait timed out"),
                lambda value: bool(value),
            )
            print(f"[CORE] {self.client_id} armed", flush=True)

            self.wait_until(
                lambda: bool(self.state.get(UAV.State.Flight.IsArmed))
                and self.state.get(UAV.State.Sensor.SensorConfig) is not None
                and bool(self.state.get(UAV.State.Sensor.SensorConfig).get("TakeoffReady")),
                float(self.state_timeout_s),
                MissionAbort(
                    f"takeoff readiness wait timed out sensor_config={self.state.get(UAV.State.Sensor.SensorConfig)} "
                    f"modes={self.state.get(UAV.State.Flight.ActiveModeNames)}"
                ),
            )
            print(
                f"[CORE] {self.client_id} takeoff_ready sensor_config={self.state.get(UAV.State.Sensor.SensorConfig)} "
                f"modes={self.state.get(UAV.State.Flight.ActiveModeNames)}",
                flush=True,
            )

            takeoff_req = self._send_action(UAV.Action.Flight.Takeoff, {})
            print(f"[CORE] {self.client_id} cmd TAKEOFF req={takeoff_req}", flush=True)
            resp = self._wait_response(takeoff_req, float(self.response_timeout_s))
            if not resp.get("ok"):
                raise MissionAbort(f"takeoff failed resp={resp}")

            self.wait_until(
                lambda: bool(self.state.get(UAV.State.Flight.IsInAir))
                and self.state.get(UAV.State.Navigation.AltitudeM) is not None
                and self.state.get(UAV.State.Navigation.AltitudeM) >= float(self.takeoff_altitude_m) * float(self.takeoff_altitude_ok_fraction),
                float(self.state_timeout_s),
                MissionAbort("in-air wait timed out"),
            )
            print(f"[CORE] {self.client_id} in_air alt_m={self.state.get(UAV.State.Navigation.AltitudeM)}", flush=True)

            print(
                f"[CORE] {self.client_id} hold wait_s={self.post_takeoff_wait_s} alt_target_m={self.takeoff_altitude_m}",
                flush=True,
            )
            self.pump_for(float(self.post_takeoff_wait_s))

            goto_start_state = self.wait_for_state(
                UAV.State.Navigation.Position, float(self.state_timeout_s), MissionAbort("go-to start position wait timed out")
            )

            goto_start = GPSposition(
                float(goto_start_state["LatDeg"]),
                float(goto_start_state["LonDeg"]),
                float(goto_start_state["RelAltM"]),
            )
            goto_target = vector_to_gps(goto_start, dist=float(self.go_north_distance_m), az=0.0)
            go_to_req = self._send_action(
                UAV.Action.Navigation.GoTo,
                {
                    "Latitude": float(goto_target.lat),
                    "Longitude": float(goto_target.lon),
                    "AltitudeM": float(self.takeoff_altitude_m),
                    "YawDeg": float(self.goto_yaw_deg),
                },
            )
            print(
                f"[CORE] {self.client_id} cmd GO_TO_NORTH req={go_to_req} "
                f"north_m={self.go_north_distance_m} lat={goto_target.lat} lon={goto_target.lon}",
                flush=True,
            )
            resp = self._wait_response(go_to_req, float(self.response_timeout_s))
            if not resp.get("ok"):
                raise MissionAbort(f"go_to failed resp={resp}")

            target_distance_m = None
            goto_deadline = time.monotonic() + float(self.goto_timeout_s)
            while True:
                position_state = self.state.get(UAV.State.Navigation.Position)
                if position_state is not None:
                    current_pos = GPSposition(
                        float(position_state["LatDeg"]),
                        float(position_state["LonDeg"]),
                        float(position_state["RelAltM"]),
                    )
                    target_distance_m = gps_distance_m(current_pos, goto_target)
                    if target_distance_m <= float(self.goto_arrival_radius_m):
                        break
                if time.monotonic() > goto_deadline:
                    raise MissionAbort("go-to arrival wait timed out")
                self._pump_once(goto_deadline)
            print(
                f"[CORE] {self.client_id} arrived target_dist_m={target_distance_m} "
                f"threshold_m={self.goto_arrival_radius_m}",
                flush=True,
            )

            alt_req = self._send_action(
                UAV.Action.Navigation.GoTo,
                {
                    "Latitude": float(goto_target.lat),
                    "Longitude": float(goto_target.lon),
                    "AltitudeM": float(self.target_altitude_m),
                    "YawDeg": float(self.goto_yaw_deg),
                },
            )
            print(
                f"[CORE] {self.client_id} cmd GO_TO_ALT req={alt_req} alt_m={self.target_altitude_m}",
                flush=True,
            )
            resp = self._wait_response(alt_req, float(self.response_timeout_s))
            if not resp.get("ok"):
                raise MissionAbort(f"go_to altitude failed resp={resp}")

            self.wait_until(
                lambda: self.state.get(UAV.State.Navigation.AltitudeM) is not None
                and abs(float(self.state.get(UAV.State.Navigation.AltitudeM)) - float(self.target_altitude_m))
                <= float(self.altitude_tolerance_m),
                float(self.altitude_change_timeout_s),
                MissionAbort("altitude change wait timed out"),
            )
            print(
                f"[CORE] {self.client_id} altitude reached alt_m={self.state.get(UAV.State.Navigation.AltitudeM)} "
                f"target_m={self.target_altitude_m} tol_m={self.altitude_tolerance_m}",
                flush=True,
            )

            rtl_req = self._send_action(UAV.Action.Flight.Rtl, {})
            print(f"[CORE] {self.client_id} cmd RTL req={rtl_req}", flush=True)
            resp = self._wait_response(rtl_req, float(self.response_timeout_s))
            if not resp.get("ok"):
                raise MissionAbort(f"rtl failed resp={resp}")

            home_distance_m = None
            rtl_deadline = time.monotonic() + float(self.rtl_timeout_s)
            while True:
                position_state = self.state.get(UAV.State.Navigation.Position)
                if position_state is not None:
                    current_pos = GPSposition(
                        float(position_state["LatDeg"]),
                        float(position_state["LonDeg"]),
                        float(position_state["RelAltM"]),
                    )
                    home_distance_m = gps_distance_m(current_pos, home_pos)
                    if home_distance_m <= float(self.home_arrival_radius_m):
                        break
                if time.monotonic() > rtl_deadline:
                    raise MissionAbort("rtl home wait timed out")
                self._pump_once(rtl_deadline)
            print(
                f"[CORE] {self.client_id} home reached dist_m={home_distance_m} "
                f"threshold_m={self.home_arrival_radius_m}",
                flush=True,
            )

            land_req = self._send_action(UAV.Action.Flight.Land, {})
            print(f"[CORE] {self.client_id} cmd LAND req={land_req}", flush=True)
            resp = self._wait_response(land_req, float(self.response_timeout_s))
            if not resp.get("ok"):
                raise MissionAbort(f"land failed resp={resp}")

            self.wait_until(
                lambda: not bool(self.state.get(UAV.State.Flight.IsInAir))
                and self.state.get(UAV.State.Navigation.AltitudeM) is not None
                and self.state.get(UAV.State.Navigation.AltitudeM) <= float(self.land_altitude_threshold_m),
                float(self.land_timeout_s),
                MissionAbort("landed wait timed out"),
            )

            print(
                f"[CORE] {self.client_id} landed in_air={self.state.get(UAV.State.Flight.IsInAir)} "
                f"alt_m={self.state.get(UAV.State.Navigation.AltitudeM)}",
                flush=True,
            )

            self.publish_shutdown()

        except MissionAbort as exc:
            if bool(self.state.get(UAV.State.Flight.IsInAir)):  # Airborne abort handling is not implemented yet.
                pass
            abort_topic = f"DIAG/{self.client_id}/ABORT"
            abort_data = {"event": "ABORT", "reason": str(exc)}
            self.client.publish(abort_topic, build_envelope(self.client_id, abort_topic, abort_data))
            print(f"[CORE] {self.client_id} mission_abort reason={exc}", flush=True)
            self.publish_shutdown()

        except RuntimeError:
            error_topic = f"DIAG/{self.client_id}/ERROR"
            error_payload = build_envelope(
                self.client_id,
                error_topic,
                {"event": "ERROR", "traceback": traceback.format_exc().strip()},
            )
            self.client.publish(error_topic, error_payload)
            raise
        except KeyboardInterrupt:
            abort_topic = f"DIAG/{self.client_id}/ABORT"
            abort_data = {"event": "ABORT", "reason": "KeyboardInterrupt"}
            self.client.publish(abort_topic, build_envelope(self.client_id, abort_topic, abort_data))
            print(f"[CORE] {self.client_id} keyboard_interrupt in_air={self.state.get(UAV.State.Flight.IsInAir)}", flush=True)
            raise
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    TakeoffLandCore(cfg, bus_config).run()
