#!/usr/bin/env python3
"""Takeoff/goto/RTL/land acceptance program over MAVLink-shaped records."""

from __future__ import annotations

import math
import time
import traceback
from typing import Any, Dict

from lib.common import apply_cfg, build_envelope
from lib.core_base import CoreBase
from lib.geo_utils import GPSposition, gps_distance_m, vector_to_gps
from lib.bus_topics import ANGULAR_VELOCITY, ATTITUDE, FLIGHT_CONTROL, LOCATION
from lib.uav_client import UavClient, UavCommandError


class MissionAbort(RuntimeError):
    pass


class TakeoffLandCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        mission_cfg = cfg["mission"]
        apply_cfg(self, mission_cfg)
        self.poll_interval_s = float(mission_cfg["poll_interval_s"])

        self.uav = UavClient(self, cfg["interface"], float(self.response_timeout_s))
        self.state_topics = self.uav.state_topics(self.state_keys)
        self.init_bus(self.poll_interval_s, self.state_topics, self.uav.response_topic)

    def _flight_control(self) -> Any | None:
        return self.uav.flight_control()

    def _location(self) -> Any | None:
        return self.uav.location()

    def _attitude(self) -> Any | None:
        return self.uav.attitude()

    def _angular_velocity(self) -> Any | None:
        return self.uav.angular_velocity()

    def _in_air(self) -> bool:
        altitude = self._relative_altitude()
        return altitude is not None and float(altitude) > 0.5

    def _relative_altitude(self, location: Any | None = None) -> float | None:
        state = self._location() if location is None else location
        if state is None:
            return None
        return float(state.relative_altitude_m)

    def _gps_position(self, location: Any) -> GPSposition:
        relative_altitude = self._relative_altitude(location)
        if relative_altitude is None:
            raise MissionAbort("location record has no relative altitude")
        return GPSposition(
            float(location.latitude_deg),
            float(location.longitude_deg),
            float(relative_altitude),
        )

    def _validate_frame_contract(self) -> None:
        attitude = self._attitude()
        angular_velocity = self._angular_velocity()
        if attitude is None or angular_velocity is None:
            raise MissionAbort("attitude/angular-velocity state unavailable")
        # MAVLink body attitude and rates are FRD relative to NED; the records
        # carry those conventions without a frame tag.
        print(
            f"[CORE] {self.client_id} frames body=FRD reference=NED "
            f"attitude_rad=({attitude.roll_rad:.4f},{attitude.pitch_rad:.4f},{attitude.yaw_rad:.4f})",
            flush=True,
        )

    def run(self) -> None:
        self.send_online()
        try:
            self.wait_until(
                lambda: (
                    self._flight_control() is not None
                    and self._flight_control().navigation_validity is not None
                    and bool(self._flight_control().navigation_validity.home_position_ok)
                    and bool(self._flight_control().navigation_validity.global_position_ok)
                    and self._location() is not None
                    and self._attitude() is not None
                    and self._angular_velocity() is not None
                ),
                float(self.state_timeout_s),
                MissionAbort("initial vehicle state wait timed out"),
            )
            self._validate_frame_contract()

            flight = self._flight_control()
            print(
                f"[CORE] {self.client_id} health home_ok={flight.navigation_validity.home_position_ok} "
                f"global_ok={flight.navigation_validity.global_position_ok} "
                f"mode={flight.heartbeat.standard_mode} native_mode={flight.heartbeat.mode_name}",
                flush=True,
            )
            home_pos = self._gps_position(self._location())

            print(f"[CORE] {self.client_id} set_takeoff_altitude alt_m={self.takeoff_altitude_m}", flush=True)
            self.uav.set_takeoff_altitude(float(self.takeoff_altitude_m))

            self.wait_until(
                lambda: (
                    self._flight_control() is not None
                    and self._flight_control().readiness is not None
                    and bool(self._flight_control().readiness.arm_ready)
                ),
                float(self.state_timeout_s),
                MissionAbort(f"arm readiness wait timed out flight_control={self._flight_control()}"),
            )
            print(f"[CORE] {self.client_id} arm_ready readiness={self._flight_control().readiness}", flush=True)

            print(f"[CORE] {self.client_id} arm", flush=True)
            self.uav.arm()
            self.wait_until(
                lambda: self._flight_control() is not None and bool(self._flight_control().heartbeat.armed),
                float(self.state_timeout_s),
                MissionAbort("armed wait timed out"),
            )

            self.wait_until(
                lambda: (
                    self._flight_control() is not None
                    and bool(self._flight_control().heartbeat.armed)
                    and self._flight_control().readiness is not None
                    and bool(self._flight_control().readiness.takeoff_ready)
                ),
                float(self.state_timeout_s),
                MissionAbort(f"takeoff readiness wait timed out flight_control={self._flight_control()}"),
            )
            print(f"[CORE] {self.client_id} takeoff_ready readiness={self._flight_control().readiness}", flush=True)

            print(f"[CORE] {self.client_id} takeoff", flush=True)
            self.uav.takeoff()
            self.wait_until(
                lambda: (
                    self._in_air()
                    and self._relative_altitude() is not None
                    and float(self._relative_altitude())
                    >= float(self.takeoff_altitude_m) * float(self.takeoff_altitude_ok_fraction)
                ),
                float(self.state_timeout_s),
                MissionAbort("in-air wait timed out"),
            )
            print(f"[CORE] {self.client_id} in_air alt_m={self._relative_altitude()}", flush=True)
            self.pump_for(float(self.post_takeoff_wait_s))

            goto_start = self._gps_position(self._location())
            goto_target = vector_to_gps(goto_start, dist=float(self.go_north_distance_m), az=0.0)
            print(
                f"[CORE] {self.client_id} go_to north_m={self.go_north_distance_m} "
                f"lat={goto_target.lat} lon={goto_target.lon} alt_m={self.takeoff_altitude_m}",
                flush=True,
            )
            self.uav.go_to(
                float(goto_target.lat),
                float(goto_target.lon),
                float(self.takeoff_altitude_m),
                altitude_reference="relative",
                yaw_rad=math.radians(float(self.goto_yaw_deg)),
            )

            target_distance_m = None
            deadline = time.monotonic() + float(self.goto_timeout_s)
            while True:
                location = self._location()
                if location is not None:
                    target_distance_m = gps_distance_m(self._gps_position(location), goto_target)
                    if target_distance_m <= float(self.goto_arrival_radius_m):
                        break
                if time.monotonic() > deadline:
                    raise MissionAbort("go-to arrival wait timed out")
                self._pump_once(deadline)
            print(
                f"[CORE] {self.client_id} arrived target_dist_m={target_distance_m} threshold_m={self.goto_arrival_radius_m}",
                flush=True,
            )

            print(f"[CORE] {self.client_id} go_to altitude_m={self.target_altitude_m}", flush=True)
            self.uav.go_to(
                float(goto_target.lat),
                float(goto_target.lon),
                float(self.target_altitude_m),
                altitude_reference="relative",
                yaw_rad=math.radians(float(self.goto_yaw_deg)),
            )
            self.wait_until(
                lambda: (
                    self._relative_altitude() is not None
                    and abs(float(self._relative_altitude()) - float(self.target_altitude_m))
                    <= float(self.altitude_tolerance_m)
                ),
                float(self.altitude_change_timeout_s),
                MissionAbort("altitude change wait timed out"),
            )
            print(
                f"[CORE] {self.client_id} altitude reached alt_m={self._relative_altitude()} "
                f"target_m={self.target_altitude_m} tol_m={self.altitude_tolerance_m}",
                flush=True,
            )

            print(f"[CORE] {self.client_id} return_to_launch", flush=True)
            self.uav.return_to_launch()
            home_distance_m = None
            deadline = time.monotonic() + float(self.rtl_timeout_s)
            while True:
                location = self._location()
                if location is not None:
                    home_distance_m = gps_distance_m(self._gps_position(location), home_pos)
                    if home_distance_m <= float(self.home_arrival_radius_m):
                        break
                if time.monotonic() > deadline:
                    raise MissionAbort("rtl home wait timed out")
                self._pump_once(deadline)
            print(
                f"[CORE] {self.client_id} home reached dist_m={home_distance_m} threshold_m={self.home_arrival_radius_m}",
                flush=True,
            )

            print(f"[CORE] {self.client_id} land", flush=True)
            self.uav.land()
            self.wait_until(
                lambda: (
                    not self._in_air()
                    and self._relative_altitude() is not None
                    and float(self._relative_altitude()) <= float(self.land_altitude_threshold_m)
                ),
                float(self.land_timeout_s),
                MissionAbort("landed wait timed out"),
            )
            print(
                f"[CORE] {self.client_id} landed in_air={self._in_air()} alt_m={self._relative_altitude()}",
                flush=True,
            )
            self.publish_shutdown()

        except (MissionAbort, UavCommandError) as exc:
            flight = self._flight_control()
            if flight is not None and bool(flight.heartbeat.armed) and self._in_air():
                pass  # Airborne abort/recovery policy remains separate work.
            abort_topic = f"DIAG/{self.client_id}/ABORT"
            self.client.publish(
                abort_topic,
                build_envelope(self.client_id, abort_topic, {"event": "ABORT", "reason": str(exc)}),
            )
            print(f"[CORE] {self.client_id} mission_abort reason={exc}", flush=True)
            self.publish_shutdown()
        except RuntimeError:
            error_topic = f"DIAG/{self.client_id}/ERROR"
            self.client.publish(
                error_topic,
                build_envelope(
                    self.client_id,
                    error_topic,
                    {"event": "ERROR", "traceback": traceback.format_exc().strip()},
                ),
            )
            raise
        except KeyboardInterrupt:
            abort_topic = f"DIAG/{self.client_id}/ABORT"
            flight = self._flight_control()
            self.client.publish(
                abort_topic,
                build_envelope(self.client_id, abort_topic, {"event": "ABORT", "reason": "KeyboardInterrupt"}),
            )
            print(
                f"[CORE] {self.client_id} keyboard_interrupt "
                f"in_air={None if flight is None else self._in_air()}",
                flush=True,
            )
            raise
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    TakeoffLandCore(cfg, bus_config).run()
