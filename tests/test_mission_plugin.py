#!/usr/bin/env python3
"""Unit tests for the mission lifecycle state machine and bus contract."""

import pytest

from plugins.mission.mission import (
    MissionCancelReason,
    MissionStateMachine,
    MissionTaskEvent,
    MissionTaskState,
    MissionTransitionError,
)
from protocols.namespace_loader import load_protocol_namespace


def test_protocol_contract_tokens() -> None:
    mission = load_protocol_namespace("mission")
    assert mission.Enums.MissionTaskState.Paused == "Paused"
    assert mission.Enums.MissionTaskEvent.CancelRequested == "CancelRequested"
    assert mission.Enums.MissionCancelReason.Dropped == "Dropped"
    assert mission.State.Queue.Depth == "Depth"
    assert mission.State.Active.CancelRequested == "CancelRequested"
    assert mission.Action.Task.AcknowledgeCancel == "AcknowledgeCancel"
    assert mission.Event.Task.DuplicateRejected == "DuplicateRejected"
    assert MissionTaskState.ACTIVE.value == "Active"
    assert MissionTaskEvent.QUEUED.value == "Queued"


def test_submit_queues_and_dedups() -> None:
    machine = MissionStateMachine()
    transition = machine.submit("task-1")
    assert transition.event == MissionTaskEvent.QUEUED
    assert transition.state == MissionTaskState.QUEUED
    assert machine.queue_depth == 1
    assert machine.queued_task_ids == ["task-1"]
    with pytest.raises(MissionTransitionError) as excinfo:
        machine.submit("task-1")
    assert excinfo.value.task_id == "task-1"
    assert "duplicate" in excinfo.value.detail
    assert machine.queue_depth == 1


def test_activate_requires_queued_and_single_active_slot() -> None:
    machine = MissionStateMachine()
    machine.submit("a")
    machine.submit("b")
    transition = machine.activate("a")
    assert transition.event == MissionTaskEvent.ACTIVATED
    assert transition.state == MissionTaskState.ACTIVE
    assert machine.active_task_id == "a"
    assert machine.queued_task_ids == ["b"]
    with pytest.raises(MissionTransitionError):
        machine.activate("a")  # no double execution of an active task
    with pytest.raises(MissionTransitionError):
        machine.activate("b")  # active slot is occupied
    assert machine.get("b").state == MissionTaskState.QUEUED


def test_pause_resume_and_complete_transitions() -> None:
    machine = MissionStateMachine()
    machine.submit("a")
    with pytest.raises(MissionTransitionError):
        machine.pause("a")  # only the active task can pause
    with pytest.raises(MissionTransitionError):
        machine.complete("a")  # only the active task can complete
    machine.activate("a")
    paused = machine.pause("a")
    assert paused.event == MissionTaskEvent.PAUSED
    assert paused.state == MissionTaskState.PAUSED
    resumed = machine.resume("a")
    assert resumed.event == MissionTaskEvent.RESUMED
    assert resumed.state == MissionTaskState.ACTIVE
    completed = machine.complete("a")
    assert completed.event == MissionTaskEvent.COMPLETED
    assert completed.state == MissionTaskState.COMPLETED
    assert machine.active_task_id is None
    with pytest.raises(MissionTransitionError):
        machine.activate("a")  # terminal tasks never execute again
    with pytest.raises(MissionTransitionError):
        machine.cancel("a")
    with pytest.raises(MissionTransitionError):
        machine.resume("a")


def test_queued_cancel_drops_and_keeps_dedup() -> None:
    machine = MissionStateMachine()
    machine.submit("a")
    machine.submit("b")
    transition = machine.cancel("a")
    assert transition.event == MissionTaskEvent.CANCELLED
    assert transition.reason == MissionCancelReason.DROPPED
    assert transition.state == MissionTaskState.CANCELLED
    assert machine.queued_task_ids == ["b"]
    assert machine.queue_depth == 1
    with pytest.raises(MissionTransitionError):
        machine.submit("a")  # dropped id stays deduplicated
    assert machine.get("a").state == MissionTaskState.CANCELLED


def test_active_cancel_requests_then_acknowledges() -> None:
    machine = MissionStateMachine()
    machine.submit("a")
    machine.submit("b")
    machine.activate("a")
    requested = machine.cancel("a")
    assert requested.event == MissionTaskEvent.CANCEL_REQUESTED
    assert requested.state == MissionTaskState.ACTIVE
    assert requested.cancel_requested is True
    assert machine.get("a").cancel_requested is True
    assert machine.active_task_id == "a"
    with pytest.raises(MissionTransitionError):
        machine.acknowledge_cancel("b")  # not cancel-requested
    with pytest.raises(MissionTransitionError):
        machine.activate("b")  # active slot still held
    cancelled = machine.acknowledge_cancel("a")
    assert cancelled.event == MissionTaskEvent.CANCELLED
    assert cancelled.reason == MissionCancelReason.ACKNOWLEDGED
    assert cancelled.state == MissionTaskState.CANCELLED
    assert cancelled.cancel_requested is False
    assert machine.active_task_id is None
    machine.activate("b")  # slot freed for the next task


def test_paused_cancel_requests_then_acknowledges() -> None:
    machine = MissionStateMachine()
    machine.submit("a")
    machine.activate("a")
    machine.pause("a")
    requested = machine.cancel("a")
    assert requested.event == MissionTaskEvent.CANCEL_REQUESTED
    assert requested.state == MissionTaskState.PAUSED
    cancelled = machine.acknowledge_cancel("a")
    assert cancelled.reason == MissionCancelReason.ACKNOWLEDGED
    assert machine.active_task_id is None


def test_completion_wins_over_pending_cancel() -> None:
    machine = MissionStateMachine()
    machine.submit("a")
    machine.activate("a")
    machine.cancel("a")
    completed = machine.complete("a")
    assert completed.state == MissionTaskState.COMPLETED
    assert completed.cancel_requested is False
    assert machine.active_task_id is None


def test_unknown_task_is_rejected() -> None:
    machine = MissionStateMachine()
    for action in (
        machine.activate,
        machine.pause,
        machine.resume,
        machine.cancel,
        machine.acknowledge_cancel,
        machine.complete,
    ):
        with pytest.raises(MissionTransitionError):
            action("missing")
