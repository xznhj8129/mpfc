from __future__ import annotations

import unittest
import uuid

from lib.occid_bus import occid
from plugins.execution_ingress.execution_ingress import (
    ExecutionIngress,
    _arrival_metrics,
    _distance_m,
    _location_identity,
    _location_position,
    validate_execution_bundle,
)


def uid(value: str | None = None) -> object:
    return occid.UID(root=uuid.UUID(value).bytes if value else uuid.uuid4().bytes)


def record(value: str) -> object:
    now = occid.Timestamp(utime=1.0, tz=0)
    return occid.Record(
        uid=uid(),
        id=occid.IntID(root=0),
        created_ts=now,
        updated_ts=now,
        origin_system="mpfc.tests",
        provenance=[],
    )


ASSET_UID = uid("11111111-1111-4111-8111-111111111111")
EXECUTOR_UID = uid("22222222-2222-4222-8222-222222222222")
CONTROL_UID = uid("33333333-3333-4333-8333-333333333333")


def build_bundle() -> tuple[object, object, object, object, object]:
    location = occid.Mark(
        record=record("location"),
        uid=uid(),
        id=occid.IntID(root=1),
        name="Target",
        position=occid.GlobalPosition(
            lat=36.530440,
            lon=-83.216383,
            alt=20.0,
            alt_frame=occid.AltitudeDatum.RELATIVE,
        ),
    )
    task = occid.TaskManeuver(
        record=record("task"),
        uid=uid(),
        id=occid.IntID(root=1),
        instruction="Move to the designated target point and hold there.",
        target_uids=[],
        location_uids=[location.uid],
        objective_uid=None,
        constraints=[],
        priority=occid.TaskPriority.ROUTINE,
        status=occid.TaskStatus.ACCEPTED,
        phase=occid.TaskPhase.ASSIGNED,
        intent=occid.ManeuverIntent.MOVE,
    )
    assignment = occid.TaskAssignment(
        record=record("assignment"),
        uid=uid(),
        id=occid.IntID(root=1),
        assignee_uid=ASSET_UID,
        authority_uid=CONTROL_UID,
        assigned_by_uid=CONTROL_UID,
        status=occid.AssignmentStatus.ASSIGNED,
        constraints=[],
        task_uid=task.uid,
    )
    plan = occid.OperationalPlan(
        record=record("plan"),
        uid=uid(),
        id=occid.IntID(root=1),
        name="move",
        approval_state=occid.PlanApprovalState.APPROVED,
        objective_uids=[],
        task_uids=[task.uid],
        actor_uids=[ASSET_UID],
        resource_uids=[],
        assignment_uids=[assignment.uid],
        constraints=[],
        contingencies=[],
    )
    execution = occid.Execution(
        record=record("execution"),
        uid=uid(),
        id=occid.IntID(root=1),
        assignment_uid=assignment.uid,
        executor_uid=EXECUTOR_UID,
        attempt=0,
        phase=occid.ExecutionPhase.CREATED,
        started_at=occid.Timestamp(utime=1.0, tz=0),
        completed_at=occid.Timestamp(utime=1.0, tz=0),
        external_job_refs=["dispatch.move.1"],
    )
    return location, task, plan, assignment, execution


def _validate(execution, assignment, task, plan):
    return validate_execution_bundle(
        execution,
        assignment,
        task,
        plan,
        executor_uid=EXECUTOR_UID,
        asset_uid=ASSET_UID,
    )


class ExecutionIngressTests(unittest.TestCase):
    def test_bundle_validation_preserves_execution_correlation(self) -> None:
        _, task, plan, assignment, execution = build_bundle()
        bundle = _validate(execution, assignment, task, plan)
        self.assertEqual(bundle.task.uid, assignment.task_uid)
        self.assertEqual(bundle.execution.assignment_uid, assignment.uid)

    def test_bundle_validation_rejects_mismatched_assignment(self) -> None:
        _, task, plan, assignment, execution = build_bundle()
        bad_execution = execution.model_copy(update={"assignment_uid": uid()})
        with self.assertRaisesRegex(ValueError, "assignment_uid"):
            _validate(bad_execution, assignment, task, plan)

    def test_unapproved_plan_is_rejected_independently(self) -> None:
        _, task, plan, assignment, execution = build_bundle()
        draft = plan.model_copy(update={"approval_state": occid.PlanApprovalState.DRAFT})
        with self.assertRaisesRegex(ValueError, "not approved"):
            _validate(execution, assignment, task, draft)

    def test_move_task_resolves_global_position_through_location_ref(self) -> None:
        location, task, _, _, _ = build_bundle()
        ingress = ExecutionIngress.__new__(ExecutionIngress)
        ingress.records = {
            ("control", "location", _uid_key(location.uid)): location,
        }
        destination = ingress._resolve_move_destination("control", task)
        self.assertEqual(destination, location.position)
        self.assertEqual(_location_identity(location), location.uid)
        self.assertEqual(_location_position(location), location.position)

    def test_move_task_rejects_unresolved_location(self) -> None:
        _, task, _, _, _ = build_bundle()
        ingress = ExecutionIngress.__new__(ExecutionIngress)
        ingress.records = {}
        with self.assertRaisesRegex(ValueError, "unresolved"):
            ingress._resolve_move_destination("control", task)

    def test_unsupported_task_family_rejects_before_execution(self) -> None:
        _, _, _, _, _ = build_bundle()
        info_task = occid.TaskInformation(
            record=record("info"),
            uid=uid(),
            id=occid.IntID(root=1),
            instruction="search",
            target_uids=[],
            location_uids=[],
            objective_uid=None,
            constraints=[],
            priority=occid.TaskPriority.ROUTINE,
            status=occid.TaskStatus.ACCEPTED,
            phase=occid.TaskPhase.ASSIGNED,
            intent=occid.InformationIntent.SEARCH,
        )
        ingress = ExecutionIngress.__new__(ExecutionIngress)
        ingress.records = {}
        with self.assertRaisesRegex(TypeError, "TaskManeuver/MOVE"):
            ingress._resolve_move_destination("control", info_task)

    def test_horizontal_distance_is_metric_and_symmetric(self) -> None:
        a = occid.GlobalPosition(
            lat=45.0,
            lon=-73.0,
            alt=10.0,
            alt_frame=occid.AltitudeDatum.RELATIVE,
        )
        b = occid.GlobalPosition(
            lat=45.001,
            lon=-73.0,
            alt=10.0,
            alt_frame=occid.AltitudeDatum.RELATIVE,
        )
        ab = _distance_m(a, b)
        ba = _distance_m(b, a)
        self.assertAlmostEqual(ab, ba, places=6)
        self.assertGreater(ab, 110.0)
        self.assertLess(ab, 112.0)

    def test_arrival_metrics_respect_relative_altitude_datum(self) -> None:
        location, _, _, _, _ = build_bundle()
        observed = occid.LocationState(
            position=occid.GlobalPosition(
                lat=location.position.lat,
                lon=location.position.lon,
                alt=500.0,
                alt_frame=occid.AltitudeDatum.SEA_LEVEL,
            ),
            altitude=occid.AltitudeState(
                absolute_m=500.0,
                absolute_datum=occid.AltitudeDatum.SEA_LEVEL,
                relative_m=19.5,
                relative_datum=occid.AltitudeDatum.RELATIVE,
            ),
        )
        horizontal_m, altitude_error_m = _arrival_metrics(observed, location.position)
        self.assertAlmostEqual(horizontal_m, 0.0, places=5)
        self.assertAlmostEqual(altitude_error_m, 0.5, places=5)


def _uid_key(value: object) -> str:
    return str(uuid.UUID(bytes=bytes(value.root)))


if __name__ == "__main__":
    unittest.main()
