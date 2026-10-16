"""Native Lattice delivery leases. No task interpretation or flight policy."""
import time
from dataclasses import dataclass

from anduril import Task


@dataclass(frozen=True)
class TaskDelivery:
    task: Task
    received_at_ms: int

    @property
    def task_id(self) -> str:
        return self.task.version.task_id

    def payload(self) -> dict:
        return {"TaskId": self.task_id, "ReceivedAtMs": self.received_at_ms,
                "TaskJson": self.task.model_dump_json(by_alias=True, exclude_unset=True)}


def task_definition(task: Task) -> dict:
    document = task.model_dump(mode="json", by_alias=True, exclude_unset=True)
    for name in ("status", "lastUpdateTime", "lastUpdatedBy"):
        document.pop(name, None)
    document["version"].pop("statusVersion", None)
    return document


class TaskDeliveries:
    TERMINAL = frozenset({"STATUS_DONE_OK", "STATUS_DONE_NOT_OK", "STATUS_REPLACED"})
    RECOVERABLE = frozenset({"STATUS_MACHINE_RECEIPT", "STATUS_ACK"})

    def __init__(self, fetch_lease_s: float):
        if fetch_lease_s <= 0:
            raise ValueError("task_fetch_lease_s must be positive")
        self.fetch_lease_s = fetch_lease_s
        self.seen: dict[str, Task] = {}
        self.pending: dict[str, TaskDelivery] = {}
        self.unreceipted: set[str] = set()
        self.offered_until: dict[str, float] = {}
        self.blocked: set[str] = set()
        self.last_task: TaskDelivery | None = None

    @property
    def pending_count(self) -> int:
        return len(self.pending)

    def admit(self, task: Task) -> bool:
        if not task.version or not task.version.task_id or not task.version.status_version:
            raise ValueError("Native task identity and status version are required")
        task_id = task.version.task_id
        if task_id in self.seen:
            if task_definition(task) != task_definition(self.seen[task_id]):
                raise ValueError(f"Conflicting native task definition task_id={task_id}")
            return False
        self.seen[task_id] = task
        self.pending[task_id] = TaskDelivery(task, int(time.time() * 1000))
        self.unreceipted.add(task_id)
        return True

    def received(self, task_id: str) -> None:
        self.unreceipted.discard(task_id)
        self.last_task = self.pending.get(task_id)

    def acknowledge(self, task_id: str, status: str) -> None:
        if status == "STATUS_MACHINE_RECEIPT":
            self.received(task_id)
        elif status != "STATUS_SENT":
            self.pending.pop(task_id, None)
            self.unreceipted.discard(task_id)
            self.offered_until.pop(task_id, None)
            if status in self.TERMINAL:
                # A terminal status reported by this edge resolves a task blocked at recovery.
                self.blocked.discard(task_id)

    def restore(self, task: Task) -> None:
        status = task.status.status
        if status == "STATUS_SENT" or status in self.TERMINAL:
            return
        if status in self.RECOVERABLE:
            self.admit(task)
            self.received(task.version.task_id)
        else:
            self.blocked.add(task.version.task_id)

    def reconcile(self, task: Task) -> None:
        task_id = task.version.task_id
        status = task.status.status
        if status in self.RECOVERABLE:
            self.received(task_id)
        else:
            self.acknowledge(task_id, status)
            if status not in self.TERMINAL:
                self.blocked.add(task_id)

    def fetch(self) -> TaskDelivery | None:
        if self.blocked:
            return None
        now = time.monotonic()
        for task_id, delivery in self.pending.items():
            if task_id in self.unreceipted or self.offered_until.get(task_id, 0) > now:
                continue
            self.offered_until[task_id] = now + self.fetch_lease_s
            self.last_task = delivery
            return delivery
        return None
