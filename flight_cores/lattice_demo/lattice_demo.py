#!/usr/bin/env python3
"""
Lattice demo core: execute Lattice agent tasks with MPFC.

Usage:
    from flight_cores.lattice_demo.lattice_demo import run_core
    run_core(cfg, bus_config)

The core is the mission policy layer of the demo: it fetches decoded tasks
from the Lattice interface, drives them through the mission lifecycle
plugin, executes them with the UAV controller, reports versioned status
back through the Lattice interface, and applies the link-loss failsafe when
the Lattice link is lost while armed.

Sequence per task:
    received -> submit/queue -> arm/takeoff -> go-to -> EXECUTING ->
    arrival hold/orbit -> DONE_OK; cancel and complete requests interrupt
    the active execution and are reported.
"""

import math
import time
import traceback
from dataclasses import dataclass
from typing import Any, Dict

from lib.altitude import hae_to_relative, home_hae_from_telemetry, relative_to_hae
from lib.common import (
    apply_cfg,
    build_event_topics,
    build_request_topic,
    build_response_topic,
    build_state_topics,
    build_topic_base,
)
from lib.core_base import CoreBase
from lib.geo_utils import GPSposition, gps_distance_m
from lib.lattice_specs import ORBIT_SPECIFICATION_URL
from protocols.namespace_loader import load_protocol_namespace

UAV = load_protocol_namespace("uav")
LATTICE = load_protocol_namespace("lattice")
MISSION = load_protocol_namespace("mission")

class LatticeDemoError(RuntimeError):
    """A demo execution step failed with visible diagnostics."""


class TaskInterrupted(RuntimeError):
    """Active execution was interrupted by a cancel or complete request."""


@dataclass(frozen=True)
class Destination:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    altitude_reference: str

    @staticmethod
    def from_task(task: Dict[str, Any]) -> "Destination":  # Build a destination from a decoded task payload.
        latitude = task.get("LatitudeDeg")
        longitude = task.get("LongitudeDeg")
        altitude = task.get("AltitudeM")
        reference = task.get("AltitudeReference")
        if latitude is None or longitude is None or altitude is None or reference is None:
            raise LatticeDemoError(f"task has no usable destination task_id={task.get('TaskId')}")
        if task.get("SpecificationType") == ORBIT_SPECIFICATION_URL:
            height = task.get("OrbitHeightM")
            if height is None:
                raise LatticeDemoError(f"Orbit task has no height task_id={task.get('TaskId')}")
            altitude = float(altitude) + float(height)
        return Destination(
            latitude_deg=float(latitude),
            longitude_deg=float(longitude),
            altitude_m=float(altitude),
            altitude_reference=str(reference),
        )


def command_altitude_m(destination: Destination, home_hae_m: float | None) -> float:  # Destination -> home-relative command altitude.
    reference = destination.altitude_reference
    if reference == LATTICE.Enums.LatticeAltitudeReference.Hae:
        if home_hae_m is None:
            raise LatticeDemoError("HAE destination requires the home HAE datum")
        return hae_to_relative(destination.altitude_m, home_hae_m)
    if reference in (
        LATTICE.Enums.LatticeAltitudeReference.Agl,
        LATTICE.Enums.LatticeAltitudeReference.Relative,
    ):
        # MAVSDK GoTo altitude is home-relative; AGL equals home-relative while
        # home sits at ground level.
        return destination.altitude_m
    raise LatticeDemoError(f"unsupported altitude reference {reference!r}")


def observed_altitude_m(
    reference: str, home_hae_m: float | None, absolute_alt_m: float, relative_alt_m: float
) -> float:  # Vehicle altitude expressed in the destination reference.
    if reference == LATTICE.Enums.LatticeAltitudeReference.Hae:
        if home_hae_m is None:
            raise LatticeDemoError("HAE destination requires the home HAE datum")
        return relative_to_hae(relative_alt_m, home_hae_m)
    if reference in (
        LATTICE.Enums.LatticeAltitudeReference.Agl,
        LATTICE.Enums.LatticeAltitudeReference.Relative,
    ):
        return relative_alt_m
    if reference == LATTICE.Enums.LatticeAltitudeReference.Asl:
        return absolute_alt_m
    raise LatticeDemoError(f"unsupported altitude reference {reference!r}")


def arrival_metrics(
    destination: Destination,
    home_hae_m: float | None,
    latitude_deg: float,
    longitude_deg: float,
    absolute_alt_m: float,
    relative_alt_m: float,
) -> tuple[float, float]:  # Horizontal and altitude error to the destination.
    horizontal_m = gps_distance_m(
        GPSposition(latitude_deg, longitude_deg, relative_alt_m),
        GPSposition(destination.latitude_deg, destination.longitude_deg, 0.0),
    )
    observed_m = observed_altitude_m(
        destination.altitude_reference, home_hae_m, absolute_alt_m, relative_alt_m
    )
    return horizontal_m, abs(observed_m - destination.altitude_m)


class LatticeDemo(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Initialize the demo core.
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        policy = cfg["policy"]
        apply_cfg(self, policy)
        self.interface = dict(cfg["interface"])
        self.lattice = dict(cfg["lattice"])
        self.mission = dict(cfg["mission"])
        self.state_keys = list(cfg["state_keys"])
        self.lattice_state_keys = list(cfg["lattice_state_keys"])
        self.poll_interval_s = float(cfg["poll_interval_s"])
        self.state_logging = dict(cfg["state_logging"])

        interface_base = build_topic_base(self.interface["id"], self.interface["topic_ns"])
        self.interface_request_topic = build_request_topic(self.interface["id"], self.interface["topic_ns"])
        self.interface_response_topic = build_response_topic(self.interface["id"], self.interface["topic_ns"])
        lattice_base = build_topic_base(self.lattice["id"], self.lattice["topic_ns"])
        self.lattice_request_topic = build_request_topic(self.lattice["id"], self.lattice["topic_ns"])
        self.lattice_response_topic = build_response_topic(self.lattice["id"], self.lattice["topic_ns"])
        self.mission_request_topic = build_request_topic(self.mission["id"], self.mission["topic_ns"])
        self.mission_response_topic = build_response_topic(self.mission["id"], self.mission["topic_ns"])

        state_topics = build_state_topics(interface_base, self.state_keys)
        state_topics.update(build_state_topics(lattice_base, self.lattice_state_keys))
        self.init_bus(self.poll_interval_s, state_topics, self.interface_response_topic)
        self.bus.add_response_topic(self.lattice_response_topic)
        self.bus.add_response_topic(self.mission_response_topic)
        self.lattice_event_topics = build_event_topics(
            lattice_base,
            [
                LATTICE.Event.Task.CancelRequested,
                LATTICE.Event.Task.CompleteRequested,
            ],
        )
        for topic in self.lattice_event_topics.values():
            self.client.subscribe(topic)
        self.enable_state_logging(self.state_logging["keys"], self.state_logging["interval_s"], self.state_logging["prefix"])

        self.home_hae_m: float | None = None
        self.active_task_id: str | None = None
        self.tasks: Dict[str, Dict[str, Any]] = {}
        self.pending_cancels: set[str] = set()
        self.pending_completes: set[str] = set()
        self.stop_reason: str | None = None
        self.failsafe_triggered = False
        self.last_fetch_at = 0.0

    # -- bus plumbing --------------------------------------------------------

    def _pump_messages(self, wait_s: float) -> None:  # Pump the bus and handle subscribed messages.
        deadline = time.monotonic() + wait_s
        while True:
            topic, payload = self._pump_once(deadline)
            if topic is not None:
                self._handle_message(topic, payload)
            if time.monotonic() >= deadline:
                break

    def _tick(self, wait_s: float = 0.0) -> None:  # Pump the bus and service demo duties.
        self._pump_messages(wait_s)
        self._service_tasks()
        self._check_failsafe()

    def _handle_message(self, topic: str, payload: Dict[str, Any]) -> None:  # Handle subscribed bus messages.
        if topic in self.lattice_event_topics.values():
            data = payload["data"]
            if topic == self.lattice_event_topics[LATTICE.Event.Task.CancelRequested]:
                self._on_cancel_requested(str(data["TaskId"]))
            elif topic == self.lattice_event_topics[LATTICE.Event.Task.CompleteRequested]:
                self._on_complete_requested(str(data["TaskId"]))

    def _request(self, request_topic: str, action: str, params: Dict[str, Any], timeout_s: float | None = None) -> Dict[str, Any]:
        request_id = self.bus.send_action(request_topic, action, params)
        deadline = time.monotonic() + float(self.response_timeout_s if timeout_s is None else timeout_s)
        while True:
            if request_id in self.bus.responses:
                return self.bus.responses.pop(request_id)
            if time.monotonic() > deadline:
                raise LatticeDemoError(f"response timeout action={action} topic={request_topic}")
            self._pump_messages(0.1)

    def _uav_action(self, action: str, params: Dict[str, Any]) -> Dict[str, Any]:
        response = self._request(self.interface_request_topic, action, params)
        print(
            f"[MAVSDK_ACK] action={action} ok={response.get('ok')} data={response.get('data')}",
            flush=True,
        )
        return response

    def _lattice_report(
        self,
        task_id: str,
        status_key: str,
        *,
        progress: float | None = None,
        error_code: str | None = None,
        error_message: str | None = None,
    ) -> Dict[str, Any]:
        params: Dict[str, Any] = {"TaskId": task_id, "Status": status_key}
        if progress is not None:
            params["Progress"] = float(progress)
        if error_code is not None:
            params["ErrorCode"] = error_code
        if error_message is not None:
            params["ErrorMessage"] = error_message
        response = self._request(self.lattice_request_topic, LATTICE.Action.Task.Report, params)
        if response.get("ok"):
            print(
                f"[CORE] status task_id={task_id} status={response['data'].get('Status')} "
                f"version={response['data'].get('Version')}",
                flush=True,
            )
        else:
            print(
                f"[CORE] status_failed task_id={task_id} status={status_key} "
                f"error={response.get('data', {}).get('error')}",
                flush=True,
            )
        return response

    def _mission_action(self, action: str, params: Dict[str, Any]) -> Dict[str, Any]:
        return self._request(self.mission_request_topic, action, params)

    # -- task intake ---------------------------------------------------------

    def _service_tasks(self) -> None:  # Fetch and submit at most one task, then activate the next queued task.
        now = time.monotonic()
        if now - self.last_fetch_at >= float(self.fetch_interval_s):
            self.last_fetch_at = now
            response = self._request(self.lattice_request_topic, LATTICE.Action.Task.Fetch, {})
            if response.get("ok") and response["data"].get("TaskId"):
                self._submit_task(response["data"])
        if self.active_task_id is not None:
            return
        for task_id, task in list(self.tasks.items()):
            if task_id in self.pending_cancels:
                self.pending_cancels.discard(task_id)
                self._drop_queued_task(task_id)
                return
            if task_id in self.pending_completes:
                self.pending_completes.discard(task_id)
                self._complete_queued_task(task_id)
                return
            self._activate_task(task)
            return

    def _submit_task(self, task: Dict[str, Any]) -> None:  # Submit one decoded task to the mission lifecycle.
        task_id = str(task["TaskId"])
        response = self._mission_action(MISSION.Action.Task.Submit, {"TaskId": task_id})
        if not response.get("ok"):
            data = response.get("data") or {}
            if "duplicate" in str(data.get("error", "")):
                print(f"[TASK_DUPLICATE] task_id={task_id} mission rejected duplicate", flush=True)
            else:
                print(f"[TASK_SUBMIT_FAILED] task_id={task_id} error={data.get('error')}", flush=True)
            return
        self.tasks[task_id] = task
        if task_id in self.pending_cancels:
            self.pending_cancels.discard(task_id)
            self._drop_queued_task(task_id)
            return
        if task_id in self.pending_completes:
            self.pending_completes.discard(task_id)
            self._complete_queued_task(task_id)
            return
        if self.active_task_id is None:
            self._activate_task(task)
            return
        self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.Ack)
        print(f"[TASK_QUEUED] task_id={task_id} active={self.active_task_id}", flush=True)

    def _activate_task(self, task: Dict[str, Any]) -> None:  # Activate a submitted task and execute it.
        task_id = str(task["TaskId"])
        response = self._mission_action(MISSION.Action.Task.Activate, {"TaskId": task_id})
        if not response.get("ok"):
            print(
                f"[TASK_ACTIVATE_FAILED] task_id={task_id} error={response.get('data', {}).get('error')}",
                flush=True,
            )
            self.tasks.pop(task_id, None)
            return
        self.active_task_id = task_id
        self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.Ack)
        print(
            f"[TASK_EXECUTE] task_id={task_id} spec={task.get('SpecificationType')} "
            f"display_name={task.get('DisplayName')!r}",
            flush=True,
        )
        self._execute_task(task)

    # -- lifecycle actions ---------------------------------------------------

    def _on_cancel_requested(self, task_id: str) -> None:  # Apply a Lattice cancel request to the lifecycle.
        print(f"[TASK_CANCEL] task_id={task_id}", flush=True)
        if task_id == self.active_task_id:
            response = self._mission_action(MISSION.Action.Task.Cancel, {"TaskId": task_id})
            if response.get("ok") and response["data"].get("CancelRequested"):
                self.stop_reason = "cancel"
                return
            print(f"[TASK_CANCEL_FAILED] task_id={task_id} error={response.get('data', {}).get('error')}", flush=True)
            return
        if task_id in self.tasks:
            self._drop_queued_task(task_id)
            return
        self.pending_cancels.add(task_id)
        print(f"[TASK_CANCEL_PENDING] task_id={task_id}", flush=True)

    def _on_complete_requested(self, task_id: str) -> None:  # Apply a Lattice complete request to the lifecycle.
        print(f"[TASK_COMPLETE] task_id={task_id}", flush=True)
        if task_id == self.active_task_id:
            self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.DoneOk, progress=1.0)
            self.stop_reason = "complete"
            return
        if task_id in self.tasks:
            self._complete_queued_task(task_id)
            return
        self.pending_completes.add(task_id)
        print(f"[TASK_COMPLETE_PENDING] task_id={task_id}", flush=True)

    def _drop_queued_task(self, task_id: str) -> None:  # Drop a queued task; it must never execute.
        response = self._mission_action(MISSION.Action.Task.Cancel, {"TaskId": task_id})
        if not response.get("ok"):
            print(f"[TASK_QUEUED_CANCEL_FAILED] task_id={task_id} error={response.get('data', {}).get('error')}", flush=True)
            return
        self.tasks.pop(task_id, None)
        print(f"[TASK_QUEUED_CANCELLED] task_id={task_id}", flush=True)
        self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.CancelRequested)
        self._lattice_report(
            task_id,
            LATTICE.Enums.LatticeTaskStatus.DoneNotOk,
            error_code=LATTICE.Enums.LatticeTaskErrorCode.Cancelled,
            error_message="task cancelled before execution",
        )

    def _complete_queued_task(self, task_id: str) -> None:  # Manager completed a queued task; it never executes.
        self._mission_action(MISSION.Action.Task.Cancel, {"TaskId": task_id})
        self.tasks.pop(task_id, None)
        print(f"[TASK_QUEUED_COMPLETED] task_id={task_id}", flush=True)
        self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.DoneOk, progress=1.0)

    # -- execution -----------------------------------------------------------

    def _check_stop(self) -> None:  # Raise when a cancel or complete interrupted execution.
        if self.stop_reason is not None:
            raise TaskInterrupted(self.stop_reason)

    def _position(self) -> Dict[str, Any] | None:
        return self.state.get(UAV.State.Navigation.Position)

    def _execute_task(self, task: Dict[str, Any]) -> None:  # Execute one active task end to end.
        task_id = str(task["TaskId"])
        outcome: str | None = None
        try:
            destination = Destination.from_task(task)
            self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.Executing, progress=0.0)
            self._prepare_vehicle(destination)
            if task.get("SpecificationType") == ORBIT_SPECIFICATION_URL:
                self._execute_orbit(task_id, destination)
            else:
                self._execute_transit(task_id, destination)
        except TaskInterrupted as exc:
            outcome = str(exc)
            if outcome == "cancel":
                self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.CancelRequested)
                self._lattice_report(
                    task_id,
                    LATTICE.Enums.LatticeTaskStatus.DoneNotOk,
                    error_code=LATTICE.Enums.LatticeTaskErrorCode.Cancelled,
                    error_message="task cancelled",
                )
        except LatticeDemoError as exc:
            outcome = "failed"
            print(f"[TASK_FAILED] task_id={task_id} error={exc}", flush=True)
            self._lattice_report(
                task_id,
                LATTICE.Enums.LatticeTaskStatus.DoneNotOk,
                error_code=LATTICE.Enums.LatticeTaskErrorCode.Failed,
                error_message=str(exc),
            )
        finally:
            if outcome == "cancel":
                self._mission_action(MISSION.Action.Task.AcknowledgeCancel, {"TaskId": task_id})
            else:
                self._mission_action(MISSION.Action.Task.Complete, {"TaskId": task_id})
            self.stop_reason = None
            self.active_task_id = None
            self.tasks.pop(task_id, None)
            print(f"[TASK_ENDED] task_id={task_id} outcome={outcome or 'done_ok'}", flush=True)

    def _prepare_vehicle(self, destination: Destination) -> None:  # Arm and take off when the task needs flight.
        if bool(self.state.get(UAV.State.Flight.IsArmed)) and bool(self.state.get(UAV.State.Flight.IsInAir)):
            return
        target_alt_m = command_altitude_m(destination, self.home_hae_m)
        takeoff_alt_m = float(self.takeoff_altitude_m)
        if target_alt_m > 0.0:
            takeoff_alt_m = min(takeoff_alt_m, target_alt_m)
        response = self._uav_action(UAV.Action.Flight.SetTakeoffAltitude, {"AltitudeM": takeoff_alt_m})
        if not response.get("ok"):
            raise LatticeDemoError(f"set_takeoff_altitude failed {response.get('data')}")
        self._wait_arm_ready()
        if not bool(self.state.get(UAV.State.Flight.IsArmed)):
            response = self._uav_action(UAV.Action.Flight.Arm, {})
            if not response.get("ok"):
                raise LatticeDemoError(f"arm failed {response.get('data')}")
            self._wait_until(
                lambda: bool(self.state.get(UAV.State.Flight.IsArmed)),
                float(self.state_timeout_s),
                "armed wait timed out",
            )
            print(f"[CORE] armed alt_target_m={takeoff_alt_m}", flush=True)
        self._wait_takeoff_ready()
        response = self._uav_action(UAV.Action.Flight.Takeoff, {})
        if not response.get("ok"):
            raise LatticeDemoError(f"takeoff failed {response.get('data')}")
        self._wait_until(
            lambda: bool(self.state.get(UAV.State.Flight.IsInAir))
            and self.state.get(UAV.State.Navigation.AltitudeM) is not None
            and float(self.state.get(UAV.State.Navigation.AltitudeM))
            >= takeoff_alt_m * float(self.takeoff_altitude_ok_fraction),
            float(self.state_timeout_s),
            "in-air wait timed out",
        )
        print(
            f"[CORE] in_air alt_m={self.state.get(UAV.State.Navigation.AltitudeM)} "
            f"target_m={takeoff_alt_m}",
            flush=True,
        )
        if float(self.post_takeoff_wait_s) > 0.0:
            deadline = time.monotonic() + float(self.post_takeoff_wait_s)
            while time.monotonic() < deadline:
                self._tick(0.1)

    def _wait_arm_ready(self) -> None:  # Wait for the controller ArmReady flag.
        self._wait_until(
            lambda: self.state.get(UAV.State.Sensor.SensorConfig) is not None
            and bool(self.state.get(UAV.State.Sensor.SensorConfig).get("ArmReady")),
            float(self.state_timeout_s),
            f"arm readiness wait timed out sensor_config={self.state.get(UAV.State.Sensor.SensorConfig)}",
        )

    def _wait_takeoff_ready(self) -> None:  # Wait for the controller TakeoffReady flag.
        self._wait_until(
            lambda: bool(self.state.get(UAV.State.Flight.IsArmed))
            and self.state.get(UAV.State.Sensor.SensorConfig) is not None
            and bool(self.state.get(UAV.State.Sensor.SensorConfig).get("TakeoffReady")),
            float(self.state_timeout_s),
            f"takeoff readiness wait timed out sensor_config={self.state.get(UAV.State.Sensor.SensorConfig)}",
        )

    def _wait_until(self, condition: Any, timeout_s: float, error: str) -> None:  # Wait with bus and duty servicing.
        deadline = time.monotonic() + float(timeout_s)
        while True:
            self._check_stop()
            if condition():
                return
            if time.monotonic() > deadline:
                raise LatticeDemoError(error)
            self._tick(0.1)

    def _go_to(self, destination: Destination) -> None:  # Command the vehicle to the destination.
        altitude_m = command_altitude_m(destination, self.home_hae_m)
        response = self._uav_action(
            UAV.Action.Navigation.GoTo,
            {
                "Latitude": destination.latitude_deg,
                "Longitude": destination.longitude_deg,
                "AltitudeM": altitude_m,
                "YawDeg": 0.0,
            },
        )
        if not response.get("ok"):
            raise LatticeDemoError(f"go_to failed {response.get('data')}")

    def _arrival(self, destination: Destination) -> tuple[bool, float, float]:  # Check arrival at the destination.
        position = self._position()
        if position is None:
            return False, math.nan, math.nan
        horizontal_m, altitude_error_m = arrival_metrics(
            destination,
            self.home_hae_m,
            float(position["LatDeg"]),
            float(position["LonDeg"]),
            float(position["AbsAltM"]),
            float(position["RelAltM"]),
        )
        arrived = horizontal_m <= float(self.arrival_radius_m) and altitude_error_m <= float(
            self.arrival_altitude_tolerance_m
        )
        return arrived, horizontal_m, altitude_error_m

    def _progress(self, horizontal_m: float, altitude_error_m: float, denominators: tuple[float, float]) -> float:
        horizontal_denominator, altitude_denominator = denominators
        remaining = max(horizontal_m / horizontal_denominator, altitude_error_m / altitude_denominator)
        return max(0.0, min(1.0, 1.0 - remaining))

    def _execute_transit(self, task_id: str, destination: Destination) -> None:  # Fly the route and report arrival.
        self._go_to(destination)
        same, horizontal_m, altitude_error_m = self._arrival(destination)
        horizontal_denominator = max(horizontal_m, float(self.arrival_radius_m), 0.01) if not same else 1.0
        altitude_denominator = (
            max(altitude_error_m, float(self.arrival_altitude_tolerance_m), 0.01) if not same else 1.0
        )
        deadline = time.monotonic() + float(self.execution_timeout_s)
        last_progress_at = 0.0
        while True:
            self._check_stop()
            arrived, horizontal_m, altitude_error_m = self._arrival(destination)
            if arrived:
                self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.DoneOk, progress=1.0)
                print(
                    f"[TASK_ARRIVED] task_id={task_id} horizontal_m={horizontal_m:.2f} "
                    f"altitude_error_m={altitude_error_m:.2f}",
                    flush=True,
                )
                return
            now = time.monotonic()
            if now - last_progress_at >= float(self.progress_interval_s):
                progress = self._progress(
                    horizontal_m, altitude_error_m, (horizontal_denominator, altitude_denominator)
                )
                self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.Executing, progress=progress)
                last_progress_at = now
            if now > deadline:
                raise LatticeDemoError(
                    f"transit timed out after {float(self.execution_timeout_s):.1f}s without arrival"
                )
            self._tick(0.2)

    def _execute_orbit(self, task_id: str, destination: Destination) -> None:  # Hold the orbit point after arrival.
        print(
            f"[TASK_ORBIT] task_id={task_id} lat={destination.latitude_deg} "
            f"lon={destination.longitude_deg} altitude_m={destination.altitude_m} "
            f"reference={destination.altitude_reference}",
            flush=True,
        )
        self._go_to(destination)
        arrived, horizontal_m, altitude_error_m = self._arrival(destination)
        horizontal_denominator = max(horizontal_m, float(self.arrival_radius_m), 0.01) if not arrived else 1.0
        altitude_denominator = (
            max(altitude_error_m, float(self.arrival_altitude_tolerance_m), 0.01) if not arrived else 1.0
        )
        deadline = time.monotonic() + float(self.execution_timeout_s)
        last_progress_at = 0.0
        while not arrived:
            self._check_stop()
            arrived, horizontal_m, altitude_error_m = self._arrival(destination)
            if arrived:
                break
            now = time.monotonic()
            if now - last_progress_at >= float(self.progress_interval_s):
                progress = self._progress(
                    horizontal_m, altitude_error_m, (horizontal_denominator, altitude_denominator)
                )
                self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.Executing, progress=progress)
                last_progress_at = now
            if now > deadline:
                raise LatticeDemoError(
                    f"orbit ingress timed out after {float(self.execution_timeout_s):.1f}s without arrival"
                )
            self._tick(0.2)
        print(
            f"[TASK_ORBIT_ARRIVED] task_id={task_id} horizontal_m={horizontal_m:.2f} "
            f"altitude_error_m={altitude_error_m:.2f}",
            flush=True,
        )
        self._lattice_report(task_id, LATTICE.Enums.LatticeTaskStatus.Executing, progress=1.0)
        while True:
            self._check_stop()
            self._tick(0.2)

    # -- failsafe ------------------------------------------------------------

    def _check_failsafe(self) -> None:  # Command the safe action on Lattice link loss while armed.
        if float(self.link_loss_timeout_s) <= 0.0:
            return
        if not bool(self.state.get(UAV.State.Flight.IsArmed)):
            self.failsafe_triggered = False
            return
        age_s = self.state.get(LATTICE.State.Link.LastOkAgeS)
        if age_s is None or float(age_s) <= float(self.link_loss_timeout_s):
            self.failsafe_triggered = False
            return
        if self.failsafe_triggered:
            return
        self.failsafe_triggered = True
        action = str(self.failsafe_action)
        print(f"[FAILSAFE] link_loss_s={float(age_s):.1f} action={action}", flush=True)
        try:
            if action == "land":
                self._uav_action(UAV.Action.Flight.Land, {})
            else:
                self._uav_action(UAV.Action.Flight.Rtl, {})
            print(f"[FAILSAFE] action={action} acknowledged", flush=True)
        except LatticeDemoError as exc:
            print(f"[FAILSAFE_FAILED] action={action} error={exc}", flush=True)

    # -- runtime -------------------------------------------------------------

    def _resolve_home_hae(self) -> None:  # Resolve the home HAE datum from config or telemetry.
        if self.home_altitude_hae_m is not None:
            self.home_hae_m = float(self.home_altitude_hae_m)
            print(f"[HOME_ALTITUDE] source=config hae_m={self.home_hae_m:.3f}", flush=True)
            return
        deadline = time.monotonic() + float(self.state_timeout_s)
        while True:
            position = self._position()
            if position is not None:
                self.home_hae_m = home_hae_from_telemetry(
                    float(position["AbsAltM"]), float(position["RelAltM"])
                )
                print(
                    f"[HOME_ALTITUDE] source=telemetry hae_m={self.home_hae_m:.3f} "
                    f"abs_m={float(position['AbsAltM']):.3f} rel_m={float(position['RelAltM']):.3f}",
                    flush=True,
                )
                return
            if time.monotonic() > deadline:
                raise LatticeDemoError(
                    "home HAE datum unavailable: home_altitude_hae_m is not configured and no "
                    f"position telemetry arrived within {float(self.state_timeout_s):.1f}s"
                )
            self._tick(0.2)

    def run(self) -> None:  # Run the Lattice demo loop.
        self.send_online()
        try:
            self._resolve_home_hae()
            while True:
                self._tick(0.2)
        except KeyboardInterrupt:
            pass
        except RuntimeError:
            self.publish_error(traceback.format_exc().strip())
            raise
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    LatticeDemo(cfg, bus_config).run()
