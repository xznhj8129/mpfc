#!/usr/bin/env python3
"""
Generic task and mission state machine plus its bus plugin.

Usage:
    from plugins.mission.mission import run_plugin
    run_plugin(cfg, bus_config)

The machine tracks task ids through one explicit lifecycle:

    Received -> Queued -> Active <-> Paused -> Completed
    Received/Queued -> Cancelled (cancel drops the task from the queue)
    Active/Paused  -> Cancelled (cancel request, then acknowledgement)

Guarantees:
  - dedup by task id: a task id is admitted once and stays tracked;
  - one active slot: no second task activates while a task is Active or Paused;
  - no double execution: a task activates at most once.

The plugin knows nothing about Lattice, MAVSDK, MSP, or any other
protocol or platform; cores and other plugins drive it over the bus
messages declared in protocols/mission.yaml.
"""

import time
import traceback
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict

from lib.common import (
    apply_cfg,
    build_event_topics,
    build_request_topic,
    build_response_topic,
    build_state_scheduler_topics,
    build_topic_base,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from protocols.namespace_loader import load_protocol_namespace

MISSION = load_protocol_namespace("mission")

# Bus event keys declared in protocols/mission.yaml.
MISSION_EVENT_KEYS = (
    MISSION.Event.Task.Queued,
    MISSION.Event.Task.Activated,
    MISSION.Event.Task.Paused,
    MISSION.Event.Task.Resumed,
    MISSION.Event.Task.CancelRequested,
    MISSION.Event.Task.Cancelled,
    MISSION.Event.Task.Completed,
    MISSION.Event.Task.DuplicateRejected,
)


class MissionTaskState(str, Enum):
    RECEIVED = MISSION.Enums.MissionTaskState.Received
    QUEUED = MISSION.Enums.MissionTaskState.Queued
    ACTIVE = MISSION.Enums.MissionTaskState.Active
    PAUSED = MISSION.Enums.MissionTaskState.Paused
    CANCELLED = MISSION.Enums.MissionTaskState.Cancelled
    COMPLETED = MISSION.Enums.MissionTaskState.Completed


class MissionTaskEvent(str, Enum):
    QUEUED = MISSION.Enums.MissionTaskEvent.Queued
    ACTIVATED = MISSION.Enums.MissionTaskEvent.Activated
    PAUSED = MISSION.Enums.MissionTaskEvent.Paused
    RESUMED = MISSION.Enums.MissionTaskEvent.Resumed
    CANCEL_REQUESTED = MISSION.Enums.MissionTaskEvent.CancelRequested
    CANCELLED = MISSION.Enums.MissionTaskEvent.Cancelled
    COMPLETED = MISSION.Enums.MissionTaskEvent.Completed
    DUPLICATE_REJECTED = MISSION.Enums.MissionTaskEvent.DuplicateRejected


class MissionCancelReason(str, Enum):
    DROPPED = MISSION.Enums.MissionCancelReason.Dropped
    ACKNOWLEDGED = MISSION.Enums.MissionCancelReason.Acknowledged


class MissionTransitionError(ValueError):
    def __init__(self, task_id: str, detail: str) -> None:
        self.task_id = task_id
        self.detail = detail
        super().__init__(f"mission transition rejected task_id={task_id} detail={detail}")


@dataclass
class MissionTask:
    task_id: str
    state: MissionTaskState
    cancel_requested: bool = False


@dataclass(frozen=True)
class MissionTransition:
    task_id: str
    event: MissionTaskEvent
    state: MissionTaskState
    cancel_requested: bool
    reason: MissionCancelReason | None = None


class MissionStateMachine:
    def __init__(self) -> None:  # Initialize empty task lifecycle state.
        self._tasks: Dict[str, MissionTask] = {}
        self._queue: list[str] = []
        self._active_task_id: str | None = None

    def submit(self, task_id: str) -> MissionTransition:  # Admit and queue a task id.
        task_id = str(task_id)
        if task_id in self._tasks:
            raise MissionTransitionError(task_id, f"duplicate task id state={self._tasks[task_id].state.value}")
        task = MissionTask(task_id=task_id, state=MissionTaskState.RECEIVED)
        self._tasks[task_id] = task
        self._queue.append(task_id)
        return self._transition(task, MissionTaskState.QUEUED, MissionTaskEvent.QUEUED)

    def activate(self, task_id: str) -> MissionTransition:  # Move a queued task to the active slot.
        task = self._require(task_id)
        if task.state != MissionTaskState.QUEUED:
            raise MissionTransitionError(task.task_id, f"activate requires Queued state={task.state.value}")
        if self._active_task_id is not None:
            raise MissionTransitionError(
                task.task_id, f"active slot occupied by task_id={self._active_task_id}"
            )
        self._queue.remove(task.task_id)
        self._active_task_id = task.task_id
        return self._transition(task, MissionTaskState.ACTIVE, MissionTaskEvent.ACTIVATED)

    def pause(self, task_id: str) -> MissionTransition:  # Pause the active task.
        task = self._require(task_id)
        if task.state != MissionTaskState.ACTIVE:
            raise MissionTransitionError(task.task_id, f"pause requires Active state={task.state.value}")
        return self._transition(task, MissionTaskState.PAUSED, MissionTaskEvent.PAUSED)

    def resume(self, task_id: str) -> MissionTransition:  # Resume the paused task.
        task = self._require(task_id)
        if task.state != MissionTaskState.PAUSED:
            raise MissionTransitionError(task.task_id, f"resume requires Paused state={task.state.value}")
        if self._active_task_id != task.task_id:
            raise MissionTransitionError(task.task_id, "paused task does not hold the active slot")
        return self._transition(task, MissionTaskState.ACTIVE, MissionTaskEvent.RESUMED)

    def cancel(self, task_id: str) -> MissionTransition:  # Drop or request cancel of a task.
        task = self._require(task_id)
        if task.state in (MissionTaskState.CANCELLED, MissionTaskState.COMPLETED):
            raise MissionTransitionError(task.task_id, f"cancel requires a live task state={task.state.value}")
        if task.state in (MissionTaskState.RECEIVED, MissionTaskState.QUEUED):
            self._queue.remove(task.task_id)
            return self._transition(
                task,
                MissionTaskState.CANCELLED,
                MissionTaskEvent.CANCELLED,
                reason=MissionCancelReason.DROPPED,
            )
        task.cancel_requested = True
        return MissionTransition(
            task_id=task.task_id,
            event=MissionTaskEvent.CANCEL_REQUESTED,
            state=task.state,
            cancel_requested=task.cancel_requested,
        )

    def acknowledge_cancel(self, task_id: str) -> MissionTransition:  # Stop a cancel-requested task.
        task = self._require(task_id)
        if task.state not in (MissionTaskState.ACTIVE, MissionTaskState.PAUSED):
            raise MissionTransitionError(
                task.task_id, f"acknowledge_cancel requires Active or Paused state={task.state.value}"
            )
        if not task.cancel_requested:
            raise MissionTransitionError(task.task_id, "acknowledge_cancel requires a pending cancel request")
        self._active_task_id = None
        return self._transition(
            task,
            MissionTaskState.CANCELLED,
            MissionTaskEvent.CANCELLED,
            reason=MissionCancelReason.ACKNOWLEDGED,
        )

    def complete(self, task_id: str) -> MissionTransition:  # Complete the active task.
        task = self._require(task_id)
        if task.state != MissionTaskState.ACTIVE:
            raise MissionTransitionError(task.task_id, f"complete requires Active state={task.state.value}")
        self._active_task_id = None
        return self._transition(task, MissionTaskState.COMPLETED, MissionTaskEvent.COMPLETED)

    @property
    def active_task_id(self) -> str | None:  # Active slot task id (Active or Paused), None when free.
        return self._active_task_id

    @property
    def queue_depth(self) -> int:  # Number of queued tasks.
        return len(self._queue)

    @property
    def queued_task_ids(self) -> list[str]:  # Queued task ids in queue order.
        return list(self._queue)

    def get(self, task_id: str) -> MissionTask | None:  # Look up a tracked task.
        return self._tasks.get(str(task_id))

    def _require(self, task_id: str) -> MissionTask:  # Fetch a tracked task or reject.
        task = self._tasks.get(str(task_id))
        if task is None:
            raise MissionTransitionError(str(task_id), "unknown task id")
        return task

    def _transition(
        self,
        task: MissionTask,
        state: MissionTaskState,
        event: MissionTaskEvent,
        reason: MissionCancelReason | None = None,
    ) -> MissionTransition:
        task.state = state
        if state in (MissionTaskState.CANCELLED, MissionTaskState.COMPLETED):
            task.cancel_requested = False
        return MissionTransition(
            task_id=task.task_id,
            event=event,
            state=task.state,
            cancel_requested=task.cancel_requested,
            reason=reason,
        )


class MissionPlugin(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Initialize mission lifecycle plugin.
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)

        self.machine = MissionStateMachine()
        self.last_event = ""
        self.action_handlers: Dict[str, Callable[[str], MissionTransition]] = {
            MISSION.Action.Task.Submit: self.machine.submit,
            MISSION.Action.Task.Activate: self.machine.activate,
            MISSION.Action.Task.Pause: self.machine.pause,
            MISSION.Action.Task.Resume: self.machine.resume,
            MISSION.Action.Task.Cancel: self.machine.cancel,
            MISSION.Action.Task.AcknowledgeCancel: self.machine.acknowledge_cancel,
            MISSION.Action.Task.Complete: self.machine.complete,
        }

        base = build_topic_base(self.client_id, self.topic_ns)
        self.state_scheduler = StateScheduler(
            self.client,
            self.client_id,
            build_state_scheduler_topics(base, self.state_intervals),
        )
        self.event_topics = build_event_topics(base, list(MISSION_EVENT_KEYS))
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(float(cfg["bus_poll_interval_s"]))
        self.loop_interval_s = float(cfg["loop_interval_s"])
        self._sync_state()

    def _publish_transition(self, transition: MissionTransition) -> None:  # Publish a lifecycle event.
        payload: Dict[str, Any] = {"TaskId": transition.task_id}
        if transition.reason is not None:
            payload["Reason"] = transition.reason.value
        self._publish_event(transition.event.value, payload)
        self.last_event = transition.event.value

    def _task_data(self, transition: MissionTransition) -> Dict[str, Any]:  # Build response payload for a transition.
        task = self.machine.get(transition.task_id)
        return {
            "TaskId": task.task_id,
            "State": task.state.value,
            "CancelRequested": task.cancel_requested,
            "Event": transition.event.value,
            "Depth": self.machine.queue_depth,
        }

    def _sync_state(self) -> None:  # Publish the current lifecycle state snapshot.
        machine = self.machine
        active_task_id = machine.active_task_id
        active_task = machine.get(active_task_id) if active_task_id is not None else None
        self.state_scheduler.update(MISSION.State.Queue.Depth, machine.queue_depth)
        self.state_scheduler.update(MISSION.State.Queue.TaskIds, machine.queued_task_ids)
        self.state_scheduler.update(MISSION.State.Active.TaskId, active_task_id or "")
        self.state_scheduler.update(MISSION.State.Active.State, active_task.state.value if active_task else "")
        self.state_scheduler.update(
            MISSION.State.Active.CancelRequested, bool(active_task and active_task.cancel_requested)
        )
        self.state_scheduler.update(MISSION.State.System.LastEvent, self.last_event)

    def _handle_request(self, request: Dict[str, Any]) -> None:  # Handle a bus REQUEST action.
        request_id = str(request["request_id"])
        action = request["action"]
        params = request.get("params") or {}
        handler = self.action_handlers.get(action)
        if handler is None:
            self.enqueue_response(request_id, action, False, {"error": f"unknown action {action}"})
            return
        task_id = str(params["TaskId"])
        try:
            transition = handler(task_id)
        except MissionTransitionError as exc:
            if action == MISSION.Action.Task.Submit and self.machine.get(task_id) is not None:
                self._publish_event(MISSION.Event.Task.DuplicateRejected, {"TaskId": task_id})
                self.last_event = MissionTaskEvent.DUPLICATE_REJECTED.value
                self._sync_state()
            self.enqueue_response(request_id, action, False, {"error": str(exc)})
            return
        self._publish_transition(transition)
        self._sync_state()
        self.enqueue_response(request_id, action, True, self._task_data(transition))

    def run(self) -> None:  # Run mission lifecycle loop.
        self.send_online()
        self._sync_state()
        try:
            while True:
                self.state_scheduler.flush()
                self.flush_queue(self.response_queue, self.response_topic)
                deadline = time.monotonic() + self.loop_interval_s
                while time.monotonic() < deadline:
                    topic, payload = self._pump_once(deadline)
                    if topic == self.request_topic:
                        self._handle_request(payload["data"])
        except (KeyboardInterrupt, SystemExit):
            pass
        except RuntimeError:
            self.publish_error(traceback.format_exc().strip())
            raise
        finally:
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MissionPlugin(cfg, bus_config).run()
