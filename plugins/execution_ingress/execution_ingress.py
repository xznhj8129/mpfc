#!/usr/bin/env python3
"""OCCID-native autonomous execution ingress for MPFC.

Remote Location, Plan, Task, Assignment, and Execution records arrive as
canonical OCCID on the node-local ``OCCID/IN`` bridge topic. This plugin owns
execution semantics; HiveLink owns only delivery and MPFC's MQTT bus remains
private node-local IPC.

Execution acceptance and status travel back to the control node as Sigma SDK
messages (``ExecutionAcceptance`` and ``ExecutionStatusReport``) carried as
opaque HiveLink payloads. OCCID ``ExecutionStatus`` is the durable status fact.
"""
from __future__ import annotations

import base64
import math
import time
import traceback
import uuid
from dataclasses import dataclass
from typing import Any, Dict

from sigma_sdk.codec import MessageCodec
from sigma_sdk.messages import ExecutionAcceptance, ExecutionStatusReport, TaskDelta

from lib.common import apply_cfg, build_envelope
from lib.occid_bus import occid, pack_occid, unpack_occid
from lib.plugin_base import PluginBase
from lib.provisioning import asset_uid, node_uid
from lib.uav_client import UavClient


OCCID_IN_TOPIC = "OCCID/IN"
OCCID_OUT_TOPIC = "OCCID/OUT"
EARTH_RADIUS_M = 6371008.8


def _uid_text(value: Any) -> str:
    return str(uuid.UUID(bytes=bytes(value.root)))


def _uid_key(value: Any) -> str:
    return _uid_text(value)


def _distance_m(a: Any, b: Any) -> float:
    lat1 = math.radians(float(a.lat))
    lat2 = math.radians(float(b.lat))
    dlat = lat2 - lat1
    dlon = math.radians(float(b.lon) - float(a.lon))
    hav = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(hav)))


def _altitude_for_datum(location: Any, datum: Any) -> float:
    altitude = location.altitude
    if altitude is not None:
        if datum == altitude.absolute_datum and altitude.absolute_m is not None:
            return float(altitude.absolute_m)
        if datum == altitude.relative_datum and altitude.relative_m is not None:
            return float(altitude.relative_m)

    position = location.position
    if position is not None and position.alt_frame == datum:
        return float(position.alt)
    raise RuntimeError(
        "LocationState has no altitude observation for destination datum "
        f"{datum.name}"
    )


def _arrival_metrics(location: Any, destination: Any) -> tuple[float, float]:
    if location.position is None:
        raise RuntimeError("cannot evaluate arrival without LocationState.position")
    horizontal_m = _distance_m(location.position, destination)
    observed_altitude_m = _altitude_for_datum(location, destination.alt_frame)
    altitude_error_m = abs(observed_altitude_m - float(destination.alt))
    return horizontal_m, altitude_error_m


def _location_identity(location: Any) -> Any:
    uid = getattr(location, "uid", None)
    if uid is None:
        raise ValueError(
            f"Location record {type(location).__name__} has no stable UID field"
        )
    return uid


def _location_position(location: Any) -> Any:
    position = getattr(location, "position", None)
    if isinstance(position, occid.GlobalPosition):
        return position
    raise ValueError(
        f"Location record {type(location).__name__} does not carry a GlobalPosition"
    )


def _record(origin: str, now: float, provenance: list[str] | None = None) -> Any:
    stamp = occid.Timestamp(utime=now, tz=0)
    return occid.Record(
        uid=occid.UID(root=uuid.uuid4().bytes),
        id=occid.IntID(root=0),
        created_ts=stamp,
        updated_ts=stamp,
        origin_system=origin,
        provenance=list(provenance or ()),
    )


@dataclass(frozen=True)
class ExecutionBundle:
    execution: Any
    assignment: Any
    task: Any
    plan: Any


@dataclass(frozen=True)
class RemoteExecution:
    source: str
    dispatch_id: str
    bundle: ExecutionBundle


def validate_execution_bundle(
    execution: Any,
    assignment: Any,
    task: Any,
    plan: Any,
    *,
    executor_uid: Any,
    asset_uid: Any,
) -> ExecutionBundle:
    if not occid.is_a(execution, occid.Execution):
        raise TypeError(f"expected Execution, got {type(execution).__name__}")
    if not occid.is_a(assignment, occid.Assignment):
        raise TypeError(f"expected Assignment, got {type(assignment).__name__}")
    if not occid.is_a(task, occid.Task):
        raise TypeError(f"expected Task, got {type(task).__name__}")
    if not occid.is_a(plan, occid.Plan):
        raise TypeError(f"expected Plan, got {type(plan).__name__}")

    task_uid = getattr(assignment, "task_uid", None)
    if task_uid is None:
        raise ValueError(
            f"Assignment {type(assignment).__name__} does not reference a Task"
        )
    if execution.assignment_uid != assignment.uid:
        raise ValueError(
            "Execution.assignment_uid does not match supplied Assignment: "
            f"{_uid_text(execution.assignment_uid)} != {_uid_text(assignment.uid)}"
        )
    if task_uid != task.uid:
        raise ValueError(
            "Assignment.task_uid does not match supplied Task: "
            f"{_uid_text(task_uid)} != {_uid_text(task.uid)}"
        )
    if execution.executor_uid != executor_uid:
        raise ValueError(
            "Execution.executor_uid does not address this executor: "
            f"{_uid_text(execution.executor_uid)} != {_uid_text(executor_uid)}"
        )
    if assignment.assignee_uid != asset_uid:
        raise ValueError(
            "Assignment.assignee_uid does not address this asset: "
            f"{_uid_text(assignment.assignee_uid)} != {_uid_text(asset_uid)}"
        )
    if plan.approval_state != occid.PlanApprovalState.APPROVED:
        raise ValueError(f"supplied Plan is not approved state={plan.approval_state.name}")
    plan_task_uids = getattr(plan, "task_uids", None)
    if plan_task_uids is not None and task.uid not in plan_task_uids:
        raise ValueError("supplied Plan does not contain the assigned Task")
    plan_assignment_uids = getattr(plan, "assignment_uids", None)
    if plan_assignment_uids is not None and assignment.uid not in plan_assignment_uids:
        raise ValueError("supplied Plan does not contain the Assignment")
    if assignment.status not in (
        occid.AssignmentStatus.ASSIGNED,
        occid.AssignmentStatus.ACCEPTED,
        occid.AssignmentStatus.ACTIVE,
    ):
        raise ValueError(
            f"supplied Assignment is not executable state={assignment.status.name}"
        )
    if task.status in (
        occid.TaskStatus.COMPLETE,
        occid.TaskStatus.FAILED,
        occid.TaskStatus.CANCELLED,
    ):
        raise ValueError(f"supplied Task is terminal state={task.status.name}")
    if execution.phase not in (
        occid.ExecutionPhase.CREATED,
        occid.ExecutionPhase.QUEUED,
    ):
        raise ValueError(
            f"supplied Execution is not dispatchable phase={execution.phase.name}"
        )

    return ExecutionBundle(
        execution=execution,
        assignment=assignment,
        task=task,
        plan=plan,
    )


class ExecutionIngress(PluginBase):
    """High-level OCCID execution consumer backed by existing MPFC UAV services."""

    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.poll_interval_s = float(cfg.get("poll_interval_s", 0.1))
        self.response_timeout_s = float(cfg.get("response_timeout_s", 20.0))
        self.state_timeout_s = float(cfg.get("state_timeout_s", 60.0))
        self.execution_timeout_s = float(cfg.get("execution_timeout_s", 180.0))
        self.progress_interval_s = float(cfg.get("progress_interval_s", 1.0))
        self.arrival_radius_m = float(cfg.get("arrival_radius_m", 3.0))
        self.arrival_altitude_tolerance_m = float(
            cfg.get("arrival_altitude_tolerance_m", 2.0)
        )
        self.auto_takeoff_for_move = bool(cfg.get("auto_takeoff_for_move", False))
        self.takeoff_altitude_m = float(cfg.get("takeoff_altitude_m", 10.0))
        self.takeoff_altitude_ok_fraction = float(
            cfg.get("takeoff_altitude_ok_fraction", 0.8)
        )
        self.post_takeoff_wait_s = float(cfg.get("post_takeoff_wait_s", 1.0))
        self.control_node = str(cfg.get("control_node", "control"))
        self.state_publish_interval_s = float(cfg.get("state_publish_interval_s", 1.0))

        self.executor_uid = self._uid_from_cfg(cfg.get("executor_uid"), node_uid())
        self.asset_uid = self._uid_from_cfg(cfg.get("asset_uid"), asset_uid())
        self.in_topic = str(cfg.get("occid_in_topic", OCCID_IN_TOPIC))
        self.out_topic = str(cfg.get("occid_out_topic", OCCID_OUT_TOPIC))
        self.client.subscribe(self.in_topic)

        self.uav = UavClient(
            self,
            dict(cfg["interface"]),
            self.response_timeout_s,
            target_uid=self.asset_uid,
        )
        self.init_bus(
            self.poll_interval_s,
            state_topics=self.uav.state_topics(),
            response_topic=self.uav.response_topic,
        )

        self.records: dict[tuple[str, str, str], Any] = {}
        self.pending_executions: dict[tuple[str, str], Any] = {}
        self.active_execution_id: Any | None = None
        self.active_dispatch_id: str | None = None
        self.last_state_publish = 0.0
        self.lifecycle_state = "STARTING"
        self.lifecycle_topic = f"DIAG/{self.client_id}/LIFECYCLE"

    @staticmethod
    def _uid_from_cfg(raw: Any, default: Any) -> Any:
        if raw is None:
            return default
        if isinstance(raw, occid.UID):
            return raw
        return occid.UID(root=uuid.UUID(str(raw)).bytes)

    def _set_lifecycle(
        self,
        state: str,
        remote: RemoteExecution | None = None,
        detail: str | None = None,
    ) -> None:
        self.lifecycle_state = str(state)
        data: dict[str, Any] = {"state": self.lifecycle_state}
        if remote is not None:
            data["dispatch_id"] = remote.dispatch_id
            data["execution_id"] = _uid_text(remote.bundle.execution.uid)
            data["task_instruction"] = remote.bundle.task.instruction
        if detail:
            data["detail"] = str(detail)
        self.client.publish(
            self.lifecycle_topic,
            build_envelope(self.client_id, self.lifecycle_topic, data),
        )
        suffix = ""
        if remote is not None:
            suffix = (
                f" dispatch_id={remote.dispatch_id} "
                f"task={type(remote.bundle.task).__name__}"
            )
        if detail:
            suffix += f" detail={detail}"
        print(
            f"[EXECUTION_LIFECYCLE] state={self.lifecycle_state}{suffix}",
            flush=True,
        )

    @staticmethod
    def _dispatch_id(execution: Any) -> str:
        if not execution.external_job_refs:
            raise ValueError("Execution has no persisted dispatch identity")
        dispatch_id = str(execution.external_job_refs[-1])
        if not dispatch_id:
            raise ValueError("Execution dispatch identity is empty")
        return dispatch_id

    def _send_sdk(self, dest: str, message: Any) -> None:
        payload = base64.b64encode(MessageCodec.encode(message)).decode("ascii")
        data = {"dest": str(dest), "sdk_payload": payload}
        self.client.publish(
            self.out_topic,
            build_envelope(self.client_id, self.out_topic, data),
        )

    def _send_acceptance(
        self,
        dest: str,
        execution: Any,
        dispatch_id: str,
        *,
        accepted: bool,
        retryable: bool = False,
        reason: str | None = None,
    ) -> None:
        report = ExecutionAcceptance(
            source_node_uid=self.executor_uid,
            execution_uid=execution.uid,
            dispatch_ref=str(dispatch_id),
            accepted=bool(accepted),
            retryable=bool(retryable),
            reason=reason,
            reported_at=occid.Timestamp(utime=time.time(), tz=0),
        )
        self._send_sdk(dest, report)

    def _task_delta(
        self,
        bundle: ExecutionBundle,
        phase: Any,
        *,
        progress: float | None = None,
    ) -> TaskDelta:
        return TaskDelta(
            task_uid=bundle.task.uid,
            phase=phase,
            progress=progress,
            updated_at=occid.Timestamp(utime=time.time(), tz=0),
        )

    def _entity_state(
        self,
        location: Any,
        *,
        provenance: list[str] | None = None,
    ) -> Any:
        if location.attitude is None:
            attitude = self.uav.attitude()
            if attitude is not None:
                location = location.model_copy(update={"attitude": attitude})
        stamp = occid.Timestamp(utime=time.time(), tz=0)
        return occid.EntityState(
            record=_record(f"mpfc.{self.client_id}", time.time(), provenance),
            subject_uid=self.asset_uid,
            timestamp=stamp,
            position=location,
            flight_control=self.uav.flight_control(),
            operational_status=occid.EntityOperationalState.ACTIVE,
            lifecycle_status=occid.EntityLifecycleStatus.ACTIVE,
            link_states={},
            received_ts=stamp,
            published_ts=stamp,
        )

    def _publish_entity_state(self) -> None:
        """Publish current entity state to the control node as native OCCID."""
        location = self.uav.location()
        if location is None or location.position is None:
            return
        state = self._entity_state(location)
        data = {"dest": self.control_node, "model": pack_occid(state)}
        self.client.publish(
            self.out_topic,
            build_envelope(self.client_id, self.out_topic, data),
        )

    def _publish_status(
        self,
        dest: str,
        bundle: ExecutionBundle,
        dispatch_id: str,
        phase: Any,
        *,
        task_delta: TaskDelta | None = None,
        entity_state: Any | None = None,
        progress: float | None = None,
        failure: str | None = None,
    ) -> None:
        status = occid.ExecutionStatus(
            record=_record(f"mpfc.{self.client_id}", time.time(), [str(dispatch_id)]),
            subject_uid=bundle.execution.uid,
            timestamp=occid.Timestamp(utime=time.time(), tz=0),
            phase=phase,
            progress=progress,
            failure=failure,
        )
        report = ExecutionStatusReport(
            source_node_uid=self.executor_uid,
            status=status,
            task_delta=task_delta,
            entity_state=entity_state,
        )
        self._send_sdk(dest, report)

    def _store_model(self, source: str, model: Any) -> bool:
        if occid.is_a(model, occid.Location):
            self.records[(source, "location", _uid_key(model.uid))] = model
            return True
        if occid.is_a(model, occid.Plan):
            self.records[(source, "plan", _uid_key(model.uid))] = model
            return True
        if occid.is_a(model, occid.Task):
            self.records[(source, "task", _uid_key(model.uid))] = model
            return True
        if occid.is_a(model, occid.Assignment):
            self.records[(source, "assignment", _uid_key(model.uid))] = model
            return True
        if occid.is_a(model, occid.Execution):
            dispatch_id = self._dispatch_id(model)
            self.pending_executions[(source, dispatch_id)] = model
            return True
        return False

    def _move_dependency_missing(self, source: str, task: Any) -> bool:
        if not occid.is_a(task, occid.TaskManeuver):
            return False
        if task.intent != occid.ManeuverIntent.MOVE:
            return False
        if len(task.location_uids) != 1:
            return False
        return (
            source,
            "location",
            _uid_key(task.location_uids[0]),
        ) not in self.records

    def _resolve_move_destination(self, source: str, task: Any) -> Any:
        if not occid.is_a(task, occid.TaskManeuver):
            raise TypeError(
                f"no local handler for {type(task).__name__}; current handler is TaskManeuver/MOVE"
            )
        if task.intent != occid.ManeuverIntent.MOVE:
            raise TypeError(
                f"no local handler for TaskManeuver/{task.intent.name}; current handler is MOVE"
            )
        if len(task.location_uids) != 1:
            raise ValueError(
                f"TaskManeuver/MOVE requires exactly one location_uid; got {len(task.location_uids)}"
            )
        location_ref = task.location_uids[0]
        location = self.records.get((source, "location", _uid_key(location_ref)))
        if location is None:
            raise ValueError(
                f"TaskManeuver/MOVE location_uid is unresolved: {_uid_text(location_ref)}"
            )
        return _location_position(location)

    def _assemble_ready(self, source: str) -> list[RemoteExecution]:
        ready: list[RemoteExecution] = []
        for (pending_source, dispatch_id), execution in list(
            self.pending_executions.items()
        ):
            if pending_source != source:
                continue
            assignment = self.records.get(
                (source, "assignment", _uid_key(execution.assignment_uid))
            )
            if assignment is None:
                continue
            task_uid = getattr(assignment, "task_uid", None)
            if task_uid is None:
                continue
            task = self.records.get((source, "task", _uid_key(task_uid)))
            if task is None:
                continue
            plan = self._plan_for(source, task, assignment)
            if plan is None:
                continue
            if self._move_dependency_missing(source, task):
                continue

            self.pending_executions.pop((source, dispatch_id), None)
            try:
                bundle = validate_execution_bundle(
                    execution,
                    assignment,
                    task,
                    plan,
                    executor_uid=self.executor_uid,
                    asset_uid=self.asset_uid,
                )
            except Exception as exc:
                self._send_acceptance(
                    source,
                    execution,
                    dispatch_id,
                    accepted=False,
                    retryable=False,
                    reason=str(exc),
                )
                continue
            ready.append(
                RemoteExecution(
                    source=source,
                    dispatch_id=dispatch_id,
                    bundle=bundle,
                )
            )
        return ready

    def _plan_for(self, source: str, task: Any, assignment: Any) -> Any | None:
        for (record_source, kind, _), plan in self.records.items():
            if record_source != source or kind != "plan":
                continue
            plan_task_uids = getattr(plan, "task_uids", None)
            plan_assignment_uids = getattr(plan, "assignment_uids", None)
            if plan_task_uids is not None and task.uid in plan_task_uids:
                return plan
            if plan_assignment_uids is not None and assignment.uid in plan_assignment_uids:
                return plan
        return None

    def _ingest_occid(self, envelope: Dict[str, Any]) -> list[RemoteExecution]:
        data = envelope.get("data")
        if type(data) is not dict:
            raise ValueError("OCCID/IN data must be an object")
        source = str(data.get("source") or "")
        if not source:
            raise ValueError("OCCID/IN is missing source node id")
        model = unpack_occid(data["model"])
        if not self._store_model(source, model):
            return []
        return self._assemble_ready(source)

    def _reject_busy(self, remote: RemoteExecution) -> None:
        self._send_acceptance(
            remote.source,
            remote.bundle.execution,
            remote.dispatch_id,
            accepted=False,
            retryable=True,
            reason="execution ingress is busy",
        )

    def _pump_with_ingress(self, deadline: float | None = None) -> tuple[Any, Any]:
        topic, payload = self._pump_once(deadline)
        if topic == self.in_topic and payload is not None:
            for remote in self._ingest_occid(payload):
                self._reject_busy(remote)
            return None, None
        return topic, payload

    def _wait_response(self, request_id: str, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            if request_id in self.bus.responses:
                return self.bus.responses.pop(request_id)
            if time.monotonic() > deadline:
                raise RuntimeError(f"timeout waiting for UAV result id={request_id}")
            self._pump_with_ingress(deadline)

    def _wait_for_location(self, timeout_s: float) -> Any:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            location = self.uav.location()
            if location is not None and location.position is not None:
                return location
            if time.monotonic() > deadline:
                raise RuntimeError("timed out waiting for UAV LocationState.position")
            self._pump_with_ingress(deadline)

    def _wait_until(self, predicate, timeout_s: float, error: str) -> None:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            if predicate():
                return
            if time.monotonic() > deadline:
                raise RuntimeError(error)
            self._pump_with_ingress(deadline)

    def _relative_altitude(self) -> float | None:
        location = self.uav.location()
        if location is None or location.altitude is None:
            return None
        if location.altitude.relative_datum != occid.AltitudeDatum.RELATIVE:
            return None
        return location.altitude.relative_m

    def _prepare_vehicle_for_move(self, destination: Any) -> None:
        flight = self.uav.flight_control()
        if flight is not None and bool(flight.in_air):
            return
        if not self.auto_takeoff_for_move:
            raise RuntimeError(
                "TaskManeuver/MOVE requires an airborne vehicle when auto_takeoff_for_move is false"
            )

        takeoff_altitude_m = self.takeoff_altitude_m
        if (
            destination.alt_frame == occid.AltitudeDatum.RELATIVE
            and float(destination.alt) > 0.0
        ):
            takeoff_altitude_m = min(takeoff_altitude_m, float(destination.alt))
        if takeoff_altitude_m <= 0.0:
            raise RuntimeError("TaskManeuver/MOVE automatic takeoff altitude must be positive")

        self.uav.execute(
            self.uav.takeoff_altitude_command(takeoff_altitude_m),
            timeout_s=self.response_timeout_s,
        )
        self._wait_until(
            lambda: (
                self.uav.flight_control() is not None
                and self.uav.flight_control().readiness is not None
                and bool(self.uav.flight_control().readiness.arm_ready)
            ),
            self.state_timeout_s,
            "timed out waiting for UAV arm readiness",
        )

        flight = self.uav.flight_control()
        if flight is None or not bool(flight.armed):
            self.uav.execute(
                self.uav.arm_command(True),
                timeout_s=self.response_timeout_s,
            )
            self._wait_until(
                lambda: (
                    self.uav.flight_control() is not None
                    and bool(self.uav.flight_control().armed)
                ),
                self.state_timeout_s,
                "timed out waiting for UAV armed state",
            )

        self._wait_until(
            lambda: (
                self.uav.flight_control() is not None
                and bool(self.uav.flight_control().armed)
                and self.uav.flight_control().readiness is not None
                and bool(self.uav.flight_control().readiness.takeoff_ready)
            ),
            self.state_timeout_s,
            "timed out waiting for UAV takeoff readiness",
        )
        self.uav.execute(
            self.uav.takeoff_command(),
            timeout_s=self.response_timeout_s,
        )
        self._wait_until(
            lambda: (
                self.uav.flight_control() is not None
                and bool(self.uav.flight_control().in_air)
                and self._relative_altitude() is not None
                and float(self._relative_altitude())
                >= takeoff_altitude_m * self.takeoff_altitude_ok_fraction
            ),
            self.state_timeout_s,
            "timed out waiting for UAV takeoff",
        )
        if self.post_takeoff_wait_s > 0.0:
            deadline = time.monotonic() + self.post_takeoff_wait_s
            while time.monotonic() < deadline:
                self._pump_with_ingress(deadline)

    def _execute_move(self, remote: RemoteExecution, destination: Any) -> None:
        bundle = remote.bundle
        self._wait_for_location(self.state_timeout_s)
        self._prepare_vehicle_for_move(destination)
        current = self._wait_for_location(self.state_timeout_s)
        initial_distance_m, initial_altitude_error_m = _arrival_metrics(
            current,
            destination,
        )
        horizontal_denominator = max(initial_distance_m, self.arrival_radius_m, 0.01)
        altitude_denominator = max(
            initial_altitude_error_m,
            self.arrival_altitude_tolerance_m,
            0.01,
        )

        self.uav.execute(
            self.uav.go_to_command(
                float(destination.lat),
                float(destination.lon),
                float(destination.alt),
                altitude_datum=destination.alt_frame,
            ),
            timeout_s=self.response_timeout_s,
        )
        running = self._task_delta(bundle, occid.TaskPhase.RUNNING, progress=0.0)
        self._publish_status(
            remote.source,
            bundle,
            remote.dispatch_id,
            occid.ExecutionPhase.RUNNING,
            task_delta=running,
            entity_state=self._entity_state(current, provenance=[_uid_text(bundle.execution.uid)]),
            progress=0.0,
        )

        deadline = time.monotonic() + self.execution_timeout_s
        last_progress_publish = 0.0
        while True:
            current = self.uav.location()
            if current is not None and current.position is not None:
                horizontal_m, altitude_error_m = _arrival_metrics(
                    current,
                    destination,
                )
                remaining_fraction = max(
                    horizontal_m / horizontal_denominator,
                    altitude_error_m / altitude_denominator,
                )
                progress = max(0.0, min(1.0, 1.0 - remaining_fraction))
                if (
                    horizontal_m <= self.arrival_radius_m
                    and altitude_error_m <= self.arrival_altitude_tolerance_m
                ):
                    complete = self._task_delta(
                        bundle,
                        occid.TaskPhase.DONE_OK,
                        progress=1.0,
                    )
                    self._publish_status(
                        remote.source,
                        bundle,
                        remote.dispatch_id,
                        occid.ExecutionPhase.SUCCEEDED,
                        task_delta=complete,
                        entity_state=self._entity_state(current, provenance=[_uid_text(bundle.execution.uid)]),
                        progress=1.0,
                    )
                    return

                now = time.monotonic()
                if now - last_progress_publish >= self.progress_interval_s:
                    delta = self._task_delta(
                        bundle,
                        occid.TaskPhase.RUNNING,
                        progress=progress,
                    )
                    self._publish_status(
                        remote.source,
                        bundle,
                        remote.dispatch_id,
                        occid.ExecutionPhase.RUNNING,
                        task_delta=delta,
                        entity_state=self._entity_state(current, provenance=[_uid_text(bundle.execution.uid)]),
                        progress=progress,
                    )
                    last_progress_publish = now

            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"TaskManeuver/MOVE timed out after {self.execution_timeout_s:.1f}s without arrival"
                )
            self._pump_with_ingress(
                min(deadline, time.monotonic() + self.poll_interval_s)
            )

    def _execute_remote(self, remote: RemoteExecution) -> None:
        bundle = remote.bundle
        semantic_accepted = False
        try:
            destination = self._resolve_move_destination(remote.source, bundle.task)

            self.active_execution_id = bundle.execution.uid
            self.active_dispatch_id = remote.dispatch_id
            self._set_lifecycle("EXECUTING", remote)
            print(
                f"[TASK] instruction={bundle.task.instruction!r} "
                f"location_uid={_uid_text(bundle.task.location_uids[0])}",
                flush=True,
            )
            self._send_acceptance(
                remote.source,
                bundle.execution,
                remote.dispatch_id,
                accepted=True,
            )
            semantic_accepted = True

            accepted_delta = self._task_delta(
                bundle,
                occid.TaskPhase.DISPATCHED,
                progress=0.0,
            )
            self._publish_status(
                remote.source,
                bundle,
                remote.dispatch_id,
                occid.ExecutionPhase.QUEUED,
                task_delta=accepted_delta,
                progress=0.0,
            )

            self._execute_move(remote, destination)
        except Exception as exc:
            if not semantic_accepted:
                self._send_acceptance(
                    remote.source,
                    bundle,
                    remote.dispatch_id,
                    accepted=False,
                    retryable=False,
                    reason=str(exc),
                )
            else:
                failed = self._task_delta(bundle, occid.TaskPhase.DONE_FAIL)
                location = self.uav.location()
                self._publish_status(
                    remote.source,
                    bundle,
                    remote.dispatch_id,
                    occid.ExecutionPhase.FAILED,
                    task_delta=failed,
                    entity_state=(
                        None
                        if location is None or location.position is None
                        else self._entity_state(location, provenance=[_uid_text(bundle.execution.uid)])
                    ),
                    failure=str(exc),
                )
                print(
                    f"[EXECUTION_FAILED] dispatch_id={remote.dispatch_id} error={exc}\n"
                    f"{traceback.format_exc().strip()}",
                    flush=True,
                )
        finally:
            self.active_execution_id = None
            self.active_dispatch_id = None
            self._set_lifecycle("IDLE")

    def run(self) -> None:
        self.send_online()
        self._set_lifecycle("IDLE")
        try:
            while True:
                topic, payload = self._pump_once(
                    time.monotonic() + self.poll_interval_s
                )
                now = time.monotonic()
                if now - self.last_state_publish >= self.state_publish_interval_s:
                    self._publish_entity_state()
                    self.last_state_publish = now
                if topic != self.in_topic or payload is None:
                    continue
                for remote in self._ingest_occid(payload):
                    self._execute_remote(remote)
        except KeyboardInterrupt:
            pass
        except Exception:
            trace = traceback.format_exc().strip()
            try:
                self._set_lifecycle("FAULTED", detail=trace.splitlines()[-1])
            except Exception:
                pass
            self.publish_error(trace)
            raise
        finally:
            if self.lifecycle_state != "FAULTED":
                try:
                    self._set_lifecycle("STOPPING")
                except Exception:
                    pass
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    ExecutionIngress(cfg, bus_config).run()
