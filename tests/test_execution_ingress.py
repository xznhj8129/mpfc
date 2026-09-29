"""Task specification parsing, status sync, queueing, and frame math."""

from __future__ import annotations

import collections
import math
import threading
import time
from types import SimpleNamespace

import pytest
from anduril import AgentRequest, Entity, Quaternion, Task
from anduril.core import ApiError

from lib.mavlink_models import GlobalPositionInt
from plugins.execution_ingress.execution_ingress import (
    ORBIT_SPECIFICATION_URL,
    TRANSIT_SPECIFICATION_URL,
    Destination,
    ExecutionIngress,
    _attitude_enu_quaternion,
    _quaternion_product,
    _TaskCancelled,
)


def _bare_ingress() -> ExecutionIngress:
    return ExecutionIngress.__new__(ExecutionIngress)


def _task(specification: dict) -> Task:
    return Task.model_validate(
        {
            "version": {"taskId": "task-1", "definitionVersion": 1, "statusVersion": 1},
            "status": {"status": "STATUS_CREATED"},
            "specification": specification,
        }
    )


def test_transit_destination_from_waypoint() -> None:
    task = _task(
        {
            "@type": TRANSIT_SPECIFICATION_URL,
            "plan": {
                "route": {
                    "path": [
                        {
                            "waypoint": {
                                "llaPoint": {
                                    "lat": 36.53,
                                    "lon": -83.21,
                                    "alt": 120.0,
                                    "altitudeReference": "ALTITUDE_REFERENCE_HEIGHT_ABOVE_WGS84",
                                }
                            }
                        }
                    ]
                }
            },
        }
    )
    ingress = _bare_ingress()
    destination = ingress._transit_destination(task.specification)
    assert destination.latitude_deg == 36.53
    assert destination.longitude_deg == -83.21
    assert destination.altitude_m == 120.0
    assert destination.altitude_reference == "hae"


def test_orbit_spec_from_sample_schema() -> None:
    task = _task(
        {
            "@type": ORBIT_SPECIFICATION_URL,
            "objective": {
                "lla": {
                    "latitudeDegrees": 21.12,
                    "longitudeDegrees": -11.40,
                    "altitudeHaeM": 900.0,
                }
            },
            "orbitRadius": 1000.0,
            "orbitHeight": 100.0,
            "orbitDirection": "ORBIT_CLOCKWISE",
        }
    )
    ingress = _bare_ingress()
    orbit = ingress._orbit_spec(task.specification)
    assert orbit.latitude_deg == 21.12
    assert orbit.altitude_hae_m == 900.0
    assert orbit.radius_m == 1000.0
    assert orbit.height_m == 100.0
    assert orbit.direction == "ORBIT_CLOCKWISE"


def test_arrival_metrics_relative_altitude() -> None:
    ingress = _bare_ingress()
    ingress.home_altitude_hae_m = None
    location = GlobalPositionInt(
        latitude_deg=36.530440,
        longitude_deg=-83.216383,
        altitude_m=300.0,
        relative_altitude_m=100.0,
        vx_m_s=0.0,
        vy_m_s=0.0,
        vz_m_s=0.0,
        heading_deg=0.0,
    )
    destination = Destination(
        latitude_deg=36.531340,
        longitude_deg=-83.216383,
        altitude_m=100.0,
        altitude_reference="relative",
    )
    horizontal_m, altitude_error_m = ingress._arrival_metrics(location, destination)
    assert 90.0 < horizontal_m < 110.0
    assert altitude_error_m == 0.0


# --- fixtures --------------------------------------------------------------


class _FakeTasks:
    def __init__(self) -> None:
        self.status_version = 1
        self.updates: list[tuple[str, int, str]] = []
        self.fail_versions: set[int] = set()
        self.returned_version: int | None = None
        self.get_failures = 0

    def _task(self, task_id: str) -> Task:
        return Task.model_validate(
            {
                "version": {
                    "taskId": task_id,
                    "definitionVersion": 1,
                    "statusVersion": self.status_version,
                },
                "status": {"status": "STATUS_CREATED"},
            }
        )

    def update_task_status(self, *, task_id, status_version, new_status, author):
        self.updates.append((task_id, status_version, new_status.status))
        if status_version in self.fail_versions:
            raise ApiError(status_code=409, body={"error": "stale status version"})
        if self.returned_version is not None:
            self.status_version = self.returned_version
        else:
            self.status_version = status_version
        return self._task(task_id)

    def get_task(self, *, task_id, request_options=None):
        if self.get_failures:
            self.get_failures -= 1
            raise ApiError(status_code=503, body={"error": "endpoint unavailable"})
        return self._task(task_id)


def _ingress_stub(tasks: _FakeTasks | None = None):
    ingress = ExecutionIngress.__new__(ExecutionIngress)
    ingress.client_id = "execution_ingress"
    ingress.status_versions = {}
    ingress.task_states = {}
    ingress.queued_executes = collections.deque()
    ingress.active_task_id = None
    ingress.active_specification_url = None
    ingress.active_cancel_requested = False
    ingress.active_complete_requested = False
    ingress.active_lock = threading.Lock()
    ingress.entity_id = "mpfc-uav"
    ingress.lattice_endpoint = "127.0.0.1:18443"
    ingress.lattice_attempts = 1
    ingress.lattice_retry_delay_s = 0.0
    ingress.last_lattice_ok_at = time.monotonic()
    ingress.failsafe_triggered = False
    ingress.link_loss_timeout_s = 0.0
    ingress.failsafe_action = "rtl"
    ingress.response_timeout_s = 1.0
    ingress.lifecycle_state = "IDLE"
    ingress.lifecycle_topic = "DIAG/test/LIFECYCLE"
    fake_tasks = tasks if tasks is not None else _FakeTasks()
    ingress.lattice = SimpleNamespace(tasks=fake_tasks)
    ingress.client = SimpleNamespace(publish=lambda *args, **kwargs: None)
    return ingress, fake_tasks


def _orbit_agent_request(task_id: str, status_version: int = 1) -> AgentRequest:
    return AgentRequest.model_validate(
        {
            "executeRequest": {
                "task": {
                    "version": {
                        "taskId": task_id,
                        "definitionVersion": 1,
                        "statusVersion": status_version,
                    },
                    "status": {"status": "STATUS_CREATED"},
                    "specification": {
                        "@type": ORBIT_SPECIFICATION_URL,
                        "objective": {
                            "lla": {
                                "latitudeDegrees": 36.530440,
                                "longitudeDegrees": -83.216383,
                                "altitudeHaeM": 500.0,
                            }
                        },
                        "orbitRadius": 100.0,
                        "orbitHeight": 50.0,
                        "orbitDirection": "ORBIT_CLOCKWISE",
                    },
                }
            }
        }
    )


# --- status version sync ---------------------------------------------------


def test_status_version_seed_adopt_and_stale_resync() -> None:
    ingress, tasks = _ingress_stub()
    ingress._seed_status_version("t1", 4)
    assert ingress.status_versions["t1"] == 4

    assert ingress._update_status("t1", "STATUS_ACK") is True
    assert tasks.updates[-1] == ("t1", 5, "STATUS_ACK")
    assert ingress.status_versions["t1"] == 5

    # The version returned by the endpoint is adopted, not the requested one.
    tasks.returned_version = 9
    assert ingress._update_status("t1", "STATUS_EXECUTING") is True
    assert tasks.updates[-1][1] == 6
    assert ingress.status_versions["t1"] == 9

    # A stale update re-fetches (authoritative 12) and retries from 13.
    tasks.returned_version = None
    tasks.status_version = 12
    tasks.fail_versions = {10}
    assert ingress._update_status("t1", "STATUS_DONE_OK") is True
    assert tasks.updates[-2][1] == 10
    assert tasks.updates[-1][1] == 13
    assert ingress.status_versions["t1"] == 13


def test_status_update_failure_is_logged_not_swallowed(capsys) -> None:
    ingress, tasks = _ingress_stub()
    ingress._seed_status_version("t1", 1)
    tasks.fail_versions = {2}
    tasks.get_failures = 1  # resync fetch fails too
    assert ingress._update_status("t1", "STATUS_ACK") is False
    out = capsys.readouterr().out
    assert "[LATTICE_ERROR]" in out
    assert "operation=update_task_status" in out
    assert "task_id=t1" in out
    assert "expected_version=2" in out
    assert "[TASK_STATUS_FAILED]" in out


# --- queued task state machine ---------------------------------------------


def test_queued_receipt_cancel_and_duplicate() -> None:
    ingress, tasks = _ingress_stub()
    ingress.active_task_id = "active-1"
    request = _orbit_agent_request("queued-1", status_version=5)

    ingress._handle_execute(request)
    assert ingress.task_states["queued-1"] == "queued"
    assert len(ingress.queued_executes) == 1
    assert tasks.updates[-1] == ("queued-1", 6, "STATUS_ACK")

    # A duplicate delivery must not queue or acknowledge twice.
    ingress._handle_execute(request)
    assert len(ingress.queued_executes) == 1
    assert len(tasks.updates) == 1

    # Cancelling the queued task drops the queued execute.
    ingress._handle_cancel(
        AgentRequest.model_validate({"cancelRequest": {"taskId": "queued-1"}})
    )
    assert ingress.task_states["queued-1"] == "terminal"
    assert tasks.updates[-2][2] == "STATUS_CANCEL_REQUESTED"
    assert tasks.updates[-1][2] == "STATUS_DONE_NOT_OK"

    ingress.active_task_id = None
    ingress._take_queued_execute()
    assert ingress.active_task_id is None

    # A redelivered terminal task never executes again.
    ingress._handle_execute(request)
    assert len(ingress.queued_executes) == 0
    assert ingress.task_states["queued-1"] == "terminal"


def test_duplicate_of_active_task_is_dropped() -> None:
    ingress, tasks = _ingress_stub()
    ingress.task_states["active-1"] = "active"
    ingress.active_task_id = "active-1"
    ingress._handle_execute(_orbit_agent_request("active-1", status_version=7))
    assert tasks.updates == []  # no ack/executing for the duplicate
    assert ingress.task_states["active-1"] == "active"


# --- attitude frame math ---------------------------------------------------


def _rotate_flu_forward(quaternion: Quaternion):
    vector = Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)
    conjugate = Quaternion(
        w=quaternion.w, x=-quaternion.x, y=-quaternion.y, z=-quaternion.z
    )
    rotated = _quaternion_product(
        _quaternion_product(quaternion, vector), conjugate
    )
    return (rotated.x, rotated.y, rotated.z)


def test_attitude_known_headings() -> None:
    north = _attitude_enu_quaternion(
        SimpleNamespace(roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0)
    )
    east = _attitude_enu_quaternion(
        SimpleNamespace(roll_rad=0.0, pitch_rad=0.0, yaw_rad=math.pi / 2.0)
    )
    fx, fy, fz = _rotate_flu_forward(north)
    assert abs(fx) < 1e-9
    assert abs(fy - 1.0) < 1e-9  # yaw 0 -> body forward points north
    assert abs(fz) < 1e-9
    fx, fy, fz = _rotate_flu_forward(east)
    assert abs(fx - 1.0) < 1e-9  # yaw 90 -> body forward points east
    assert abs(fy) < 1e-9
    assert abs(fz) < 1e-9


# --- home HAE datum --------------------------------------------------------


def test_home_hae_missing_is_a_visible_failure() -> None:
    ingress, _ = _ingress_stub()
    ingress.home_altitude_hae_m = None
    ingress.home_altitude_timeout_s = 0.0
    ingress.uav = SimpleNamespace(location=lambda: None)
    with pytest.raises(RuntimeError, match="home altitude HAE unavailable"):
        ingress._resolve_home_altitude_hae()


def test_home_hae_derived_from_telemetry_and_used_for_hae_tasks() -> None:
    ingress, _ = _ingress_stub()
    ingress.home_altitude_hae_m = None
    ingress.home_altitude_timeout_s = 1.0
    location = GlobalPositionInt(
        latitude_deg=36.530440,
        longitude_deg=-83.216383,
        altitude_m=138.0,
        relative_altitude_m=100.0,
        vx_m_s=0.0,
        vy_m_s=0.0,
        vz_m_s=0.0,
        heading_deg=0.0,
    )
    ingress.uav = SimpleNamespace(location=lambda: location)
    ingress._resolve_home_altitude_hae()
    assert ingress.home_altitude_hae_m == pytest.approx(38.0)

    command_altitude_m, reference = ingress._command_altitude(
        Destination(
            latitude_deg=36.531340,
            longitude_deg=-83.216383,
            altitude_m=138.0,
            altitude_reference="hae",
        )
    )
    assert reference == "relative"
    assert command_altitude_m == pytest.approx(100.0)


def test_objective_without_altitude_uses_logged_home_datum(capsys) -> None:
    ingress, _ = _ingress_stub()
    ingress.home_altitude_hae_m = 38.0
    ingress._target_entity = lambda entity_id: Entity(
        entity_id=entity_id,
        location={
            "position": {"latitudeDegrees": 36.53, "longitudeDegrees": -83.21}
        },
    )
    destination = ingress._resolve_objective({"entityId": "track-1"})
    assert destination.altitude_m == pytest.approx(38.0)
    assert destination.altitude_reference == "hae"
    assert "[OBJECTIVE_DATUM]" in capsys.readouterr().out


# --- link/manager-loss failsafe --------------------------------------------


def test_link_failsafe_commands_rtl_once_when_armed() -> None:
    ingress, _ = _ingress_stub()
    ingress.link_loss_timeout_s = 10.0
    ingress.last_lattice_ok_at = time.monotonic() - 11.0
    commands: list[str] = []
    ingress.uav = SimpleNamespace(
        flight_control=lambda: SimpleNamespace(
            heartbeat=SimpleNamespace(armed=True)
        ),
        return_to_launch_command=lambda: "rtl-cmd",
        land_command=lambda: "land-cmd",
        execute=lambda command, timeout_s=None: commands.append(command),
    )
    ingress._check_link_failsafe()
    ingress._check_link_failsafe()
    assert commands == ["rtl-cmd"]


def test_link_failsafe_skips_grounded_vehicle() -> None:
    ingress, _ = _ingress_stub()
    ingress.link_loss_timeout_s = 10.0
    ingress.last_lattice_ok_at = time.monotonic() - 11.0
    commands: list[str] = []
    ingress.uav = SimpleNamespace(
        flight_control=lambda: SimpleNamespace(
            heartbeat=SimpleNamespace(armed=False)
        ),
        return_to_launch_command=lambda: "rtl-cmd",
        land_command=lambda: "land-cmd",
        execute=lambda command, timeout_s=None: commands.append(command),
    )
    ingress._check_link_failsafe()
    assert commands == []


def test_orbit_hold_exit_is_reported_as_cancelled() -> None:
    ingress, _ = _ingress_stub()
    ingress.state_timeout_s = 1.0
    ingress.response_timeout_s = 1.0
    ingress.execution_timeout_s = 5.0
    ingress.progress_interval_s = 0.0
    ingress.arrival_radius_m = 3.0
    ingress.arrival_altitude_tolerance_m = 2.0
    ingress.poll_interval_s = 0.01
    ingress.active_cancel_requested = False
    ingress._wait_for_location = lambda timeout: object()
    ingress._prepare_vehicle_for_move = lambda destination: None
    ingress._command_altitude = lambda destination: (100.0, "relative")
    ingress._arrival_metrics = lambda location, destination: (0.0, 0.0)
    ingress._pump_with_ingress = lambda deadline=None: None

    def cancel_on_arrival(task_id, status, *, progress=None, error=None):
        if progress == 1.0:
            ingress.active_cancel_requested = True
        return True

    ingress._update_status = cancel_on_arrival
    ingress.uav = SimpleNamespace(
        location=lambda: object(),
        execute=lambda command, timeout_s=None: None,
        go_to_command=lambda *args, **kwargs: "goto",
    )
    with pytest.raises(_TaskCancelled):
        ingress._execute_orbit("orbit-cancel", ingress._orbit_spec(
            _orbit_agent_request("orbit-cancel").execute_request.task.specification
        ))


def test_start_execute_reports_cancel_requested_and_done_not_ok() -> None:
    ingress, tasks = _ingress_stub()
    request = _orbit_agent_request("cancel-me", status_version=3)

    def cancelled(task_id, orbit):
        raise _TaskCancelled()

    ingress._execute_orbit = cancelled
    ingress._start_execute("cancel-me", request)
    statuses = [update[2] for update in tasks.updates]
    assert statuses == [
        "STATUS_ACK",
        "STATUS_EXECUTING",
        "STATUS_CANCEL_REQUESTED",
        "STATUS_DONE_NOT_OK",
    ]
    assert ingress.task_states["cancel-me"] == "terminal"


def test_entity_publishes_agl_and_hae_when_home_known() -> None:
    ingress, _ = _ingress_stub()
    ingress.home_altitude_hae_m = 38.0
    ingress.uav = SimpleNamespace(
        location=lambda: GlobalPositionInt(
            latitude_deg=36.530440,
            longitude_deg=-83.216383,
            altitude_m=138.0,
            relative_altitude_m=100.0,
            vx_m_s=0.0,
            vy_m_s=0.0,
            vz_m_s=0.0,
            heading_deg=0.0,
        ),
        attitude=lambda: None,
    )
    ingress.state = {}
    captured: dict = {}
    ingress.lattice = SimpleNamespace(
        entities=SimpleNamespace(
            publish_entity=lambda **kwargs: captured.update(kwargs)
        )
    )
    ingress._publish_entity_state()
    position = captured["location"]["position"]
    assert position["altitude_agl_meters"] == pytest.approx(100.0)
    assert position["altitude_hae_meters"] == pytest.approx(138.0)
