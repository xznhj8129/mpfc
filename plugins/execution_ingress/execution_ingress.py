#!/usr/bin/env python3
"""Lattice agent execution ingress for MPFC.

MPFC is a Lattice agent: it publishes its Entity (with a task catalog), listens
as an agent for its own entity id, accepts or rejects execute requests, executes
supported task specifications against the vehicle through MAVLink command
records, and reports task status with ``StatusUpdate``/``TaskStatus`` through the
official ``anduril`` SDK.  No relay or control-node message shapes are involved.

Supported task specifications:

* ``anduril.tasks.v2.Transit`` -- fly the requested route and report arrival.
* the Anduril sample auto-reconnaissance ``Orbit`` task -- fly to the orbit
  centre and hold; the task stays executing until the manager completes or
  cancels it.  This is the one local schema kept because the sample flow
  publishes it (see ``tasks/sim_asset_tasks.proto`` in the sample repo).

Task status semantics follow the Lattice 14-state lifecycle; progress is the
local ``{"progress": 0..1}`` document on ``TaskStatus.progress`` (ENG-2).
"""
from __future__ import annotations

import collections
import math
import os
import queue
import threading
import time
import traceback
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Any, Deque, Dict, Tuple

import httpx
from anduril import (
    Aliases,
    Classification,
    ClassificationInformation,
    Entity,
    EntityIdsSelector,
    Enu,
    GoogleProtobufAny,
    Health,
    Lattice,
    Location as LatticeLocation,
    MilView,
    Ontology,
    Position,
    PowerLevel,
    PowerSource,
    PowerState,
    Principal,
    Provenance,
    Quaternion,
    RequestTimeoutError,
    System,
    TaskCatalog,
    TaskDefinition,
    TaskError,
    TaskStatus,
)
from anduril.core import ApiError

from lib.common import apply_cfg, build_envelope
from lib.lattice_bus import get_record_state
from lib.mavlink_models import (
    BatteryStatus,
    GlobalPositionInt,
    VehicleControl,
)
from lib.plugin_base import PluginBase
from lib.provisioning import asset_id
from lib.uav_client import UavClient

TRANSIT_SPECIFICATION_URL = "type.googleapis.com/anduril.tasks.v2.Transit"
ORBIT_SPECIFICATION_URL = (
    "type.googleapis.com/anduril.sample_app_auto_reconnaissance.v1.Orbit"
)
SUPPORTED_SPECIFICATIONS = (TRANSIT_SPECIFICATION_URL, ORBIT_SPECIFICATION_URL)
MPFC_PROGRESS_TYPE_URL = "type.googleapis.com/mpfc.TaskProgress"
EARTH_RADIUS_M = 6371008.8
ENTITY_EXPIRY_S = 15.0
SQRT_HALF = math.sqrt(0.5)
# Lattice exchanges retry these; anything else is a programming error.
LATTICE_ERRORS = (ApiError, httpx.HTTPError)
# A long-poll that simply has no manager request waiting is not an error.
LATTICE_POLL_TIMEOUTS = (RequestTimeoutError, httpx.ReadTimeout, httpx.ReadError)


@dataclass(frozen=True)
class Destination:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    altitude_reference: str


@dataclass(frozen=True)
class OrbitSpec:
    """Minimal local parse of the sample-app Orbit task (published schema)."""

    latitude_deg: float
    longitude_deg: float
    altitude_hae_m: float
    radius_m: float
    height_m: float
    direction: str


def _field(node: Any, *names: str) -> Any:
    if node is None:
        return None
    if isinstance(node, dict):
        mapping = node
    elif hasattr(node, "model_extra"):
        mapping = dict(node.model_extra or {})
    else:
        return None
    for name in names:
        if name in mapping and mapping[name] is not None:
            return mapping[name]
    return None


def _float_field(node: Any, *names: str) -> float | None:
    value = _field(node, *names)
    if value is None:
        return None
    return float(value)


def _standard_altitude_reference(raw: Any, default: str) -> str:
    if raw is None:
        return default
    name = str(raw).upper()
    if "WGS84" in name or "ELLIPSOID" in name:
        return "hae"
    if "EGM96" in name or "MSL" in name or "MEAN_SEA" in name:
        return "asl"
    if "AGL" in name or "GROUND" in name:
        return "agl"
    if "SEA_FLOOR" in name:
        return "asf"
    return default


def _distance_m(a_lat: float, a_lon: float, b_lat: float, b_lon: float) -> float:
    lat1 = math.radians(float(a_lat))
    lat2 = math.radians(float(b_lat))
    dlat = lat2 - lat1
    dlon = math.radians(float(b_lon) - float(a_lon))
    hav = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(hav)))


def _quaternion_product(a: Quaternion, b: Quaternion) -> Quaternion:
    """Hamilton product ``a * b`` (rotation ``a`` applied after rotation ``b``)."""
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )


def _attitude_enu_quaternion(attitude: Any) -> Quaternion | None:
    """Convert a MAVLink attitude record to the Lattice body-FLU -> ENU quaternion.

    MAVLink euler angles build ``q(FRD->NED) = q_z(yaw) * q_y(pitch) * q_x(roll)``.
    Lattice ``attitude_enu`` is the body FLU frame expressed in ENU, so compose
    ``q(NED->ENU) * q(FRD->NED) * q(FLU->FRD)``:

    * ``q(FLU->FRD)`` is 180 deg about body x (forward),
    * ``q(NED->ENU)`` is 180 deg about the horizontal axis bisecting north/east.
    """
    if attitude is None:
        return None
    roll = float(attitude.roll_rad)
    pitch = float(attitude.pitch_rad)
    yaw = float(attitude.yaw_rad)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    # q(FRD->NED) = q_z(yaw) * q_y(pitch) * q_x(roll)
    ned_frd = Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )
    enu_ned = Quaternion(w=0.0, x=SQRT_HALF, y=SQRT_HALF, z=0.0)
    frd_flu = Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)
    return _quaternion_product(_quaternion_product(enu_ned, ned_frd), frd_flu)


class ExecutionIngress(PluginBase):
    """Lattice agent task consumer backed by MPFC's UAV service."""

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
        self.state_publish_interval_s = float(cfg.get("state_publish_interval_s", 1.0))
        self.home_altitude_hae_m = cfg.get("home_altitude_hae_m")
        if self.home_altitude_hae_m is not None:
            self.home_altitude_hae_m = float(self.home_altitude_hae_m)
        self.home_altitude_timeout_s = float(
            cfg.get("home_altitude_timeout_s", self.state_timeout_s)
        )
        self.lattice_attempts = max(1, int(cfg.get("lattice_attempts", 3)))
        self.lattice_retry_delay_s = max(
            0.0, float(cfg.get("lattice_retry_delay_s", 1.0))
        )
        # The agent listen is a long poll (the server may hold it for minutes);
        # it must not expire on the command-response timeout or each expiry
        # leaves a stale delivery attempt behind.
        self.lattice_listen_timeout_s = float(
            cfg.get("lattice_listen_timeout_s", 300.0)
        )
        self.link_loss_timeout_s = float(cfg.get("link_loss_timeout_s", 60.0))
        self.failsafe_action = str(cfg.get("failsafe_action", "rtl")).strip().lower()
        if self.failsafe_action not in ("rtl", "land"):
            raise ValueError(
                f"invalid failsafe_action {self.failsafe_action!r}; expected rtl or land"
            )

        self.lattice_endpoint = str(
            cfg.get("lattice_endpoint") or os.environ.get("LATTICE_ENDPOINT") or ""
        )
        self.lattice_client_id = str(
            cfg.get("lattice_client_id") or os.environ.get("LATTICE_CLIENT_ID") or ""
        )
        self.lattice_client_secret = str(
            cfg.get("lattice_client_secret")
            or os.environ.get("LATTICE_CLIENT_SECRET")
            or ""
        )
        sandboxes_token = str(
            cfg.get("sandboxes_token") or os.environ.get("SANDBOXES_TOKEN") or ""
        )
        if not self.lattice_endpoint:
            raise ValueError(
                "execution_ingress requires a Lattice endpoint "
                "(cfg lattice_endpoint or LATTICE_ENDPOINT)"
            )
        headers = (
            {"anduril-sandbox-authorization": f"Bearer {sandboxes_token}"}
            if sandboxes_token
            else None
        )
        base_url = (
            self.lattice_endpoint
            if self.lattice_endpoint.startswith("http")
            else f"https://{self.lattice_endpoint}"
        )
        self.lattice_kwargs = {
            "base_url": base_url,
            "client_id": self.lattice_client_id,
            "client_secret": self.lattice_client_secret,
            "headers": headers,
            "timeout": self.response_timeout_s,
        }
        self.lattice = Lattice(**self.lattice_kwargs)

        self.entity_id = str(cfg.get("asset_id") or asset_id())
        self.uav = UavClient(
            self,
            dict(cfg["interface"]),
            self.response_timeout_s,
            target_id=self.entity_id,
        )
        self.init_bus(
            self.poll_interval_s,
            state_topics=self.uav.state_topics(),
            response_topic=self.uav.response_topic,
        )

        self.status_versions: dict[str, int] = {}
        self.task_states: Dict[str, str] = {}
        self.queued_executes: Deque[Tuple[str, Any]] = collections.deque()
        self.active_task_id: str | None = None
        self.active_specification_url: str | None = None
        self.active_cancel_requested = False
        self.active_complete_requested = False
        self.active_lock = threading.Lock()
        self.last_state_publish = 0.0
        self.last_lattice_ok_at = time.monotonic()
        self.failsafe_triggered = False
        self.lifecycle_state = "STARTING"
        self.lifecycle_topic = f"DIAG/{self.client_id}/LIFECYCLE"
        self.agent_requests: "queue.Queue[Any]" = queue.Queue()
        self.listener_stop = threading.Event()
        self.listener_thread: threading.Thread | None = None
        self.listener_error: BaseException | None = None

    # -- diagnostics --------------------------------------------------------

    def _set_lifecycle(self, state: str, detail: str | None = None) -> None:
        self.lifecycle_state = str(state)
        data: dict[str, Any] = {"state": self.lifecycle_state}
        if self.active_task_id is not None:
            data["task_id"] = self.active_task_id
        if detail:
            data["detail"] = str(detail)
        self.client.publish(
            self.lifecycle_topic,
            build_envelope(self.client_id, self.lifecycle_topic, data),
        )
        suffix = f" task={self.active_task_id}" if self.active_task_id else ""
        if detail:
            suffix += f" detail={detail}"
        print(f"[EXECUTION_LIFECYCLE] state={self.lifecycle_state}{suffix}", flush=True)

    # -- Lattice transport policy -------------------------------------------

    def _log_lattice_error(
        self,
        operation: str,
        exc: BaseException,
        *,
        attempt: int | None = None,
        task_id: str | None = None,
        expected_version: int | None = None,
        actual_version: int | None = None,
    ) -> None:
        parts = [f"endpoint={self.lattice_endpoint}", f"operation={operation}"]
        if attempt is not None:
            parts.append(f"attempt={attempt}/{self.lattice_attempts}")
        if task_id is not None:
            parts.append(f"task_id={task_id}")
        if expected_version is not None:
            parts.append(f"expected_version={expected_version}")
        if actual_version is not None:
            parts.append(f"actual_version={actual_version}")
        parts.append(f"cause={type(exc).__name__}: {exc}")
        print("[LATTICE_ERROR] " + " ".join(parts), flush=True)

    def _lattice_call(
        self,
        operation: str,
        call: Any,
        *,
        task_id: str | None = None,
        expected_version: int | None = None,
        poll_timeouts_ok: bool = False,
    ) -> Any:
        """Run one Lattice SDK call with bounded retries and visible failures.

        Every listener, entity-publish, and status-update exchange goes through
        here so transport errors share one diagnostic shape and never tear down
        the runtime.  A long-poll timeout with ``poll_timeouts_ok`` means "no
        manager request"; it is not an error.
        """
        for attempt in range(1, self.lattice_attempts + 1):
            try:
                result = call()
            except LATTICE_POLL_TIMEOUTS as exc:
                if poll_timeouts_ok and attempt == 1:
                    return None
                self._log_lattice_error(
                    operation,
                    exc,
                    attempt=attempt,
                    task_id=task_id,
                    expected_version=expected_version,
                )
            except LATTICE_ERRORS as exc:
                self._log_lattice_error(
                    operation,
                    exc,
                    attempt=attempt,
                    task_id=task_id,
                    expected_version=expected_version,
                )
            else:
                self.last_lattice_ok_at = time.monotonic()
                self.failsafe_triggered = False
                return result
            if attempt < self.lattice_attempts:
                time.sleep(self.lattice_retry_delay_s)
        raise _LatticeTransportError(
            f"{operation} failed after {self.lattice_attempts} attempts "
            f"(endpoint={self.lattice_endpoint})"
        )

    # -- link/manager failsafe ----------------------------------------------

    def _check_link_failsafe(self) -> None:
        """Command the configured safe action when the Lattice link is lost.

        Link loss is measured from the last successful Lattice exchange.  The
        action fires once per outage and only when the vehicle is armed, so a
        manager outage cannot leave an armed aircraft hovering indefinitely.
        """
        if self.link_loss_timeout_s <= 0.0:
            return
        control = self.uav.flight_control()
        if control is None or not control.heartbeat.armed:
            return
        now = time.monotonic()
        link_loss_s = now - self.last_lattice_ok_at
        if link_loss_s <= self.link_loss_timeout_s:
            self.failsafe_triggered = False
            return
        if self.failsafe_triggered:
            return
        self.failsafe_triggered = True
        action = self.failsafe_action
        print(
            f"[FAILSAFE] link_loss_s={link_loss_s:.1f} action={action} "
            f"endpoint={self.lattice_endpoint}",
            flush=True,
        )
        try:
            if action == "land":
                self.uav.execute(
                    self.uav.land_command(), timeout_s=self.response_timeout_s
                )
            else:
                self.uav.execute(
                    self.uav.return_to_launch_command(),
                    timeout_s=self.response_timeout_s,
                )
            print(f"[FAILSAFE] action={action} acknowledged", flush=True)
        except Exception as exc:  # vehicle link diagnostics only
            print(
                f"[FAILSAFE_FAILED] action={action} "
                f"error={type(exc).__name__}: {exc}",
                flush=True,
            )

    # -- home altitude datum -------------------------------------------------

    def _resolve_home_altitude_hae(self) -> None:
        """Resolve the home HAE datum at startup; fail visibly if unavailable.

        HAE task altitudes cannot be commanded without this datum, so it is
        either taken from config or derived from live telemetry
        (``absolute_altitude - relative_altitude``).  There is no silent
        fallback: if neither source is available the plugin faults at startup.
        """
        if self.home_altitude_hae_m is not None:
            print(
                f"[HOME_ALTITUDE] source=config hae_m={self.home_altitude_hae_m:.3f}",
                flush=True,
            )
            return
        deadline = time.monotonic() + self.home_altitude_timeout_s
        while True:
            location = self.uav.location()
            if location is not None:
                self.home_altitude_hae_m = float(location.altitude_m) - float(
                    location.relative_altitude_m
                )
                print(
                    f"[HOME_ALTITUDE] source=telemetry "
                    f"hae_m={self.home_altitude_hae_m:.3f} "
                    f"abs_m={float(location.altitude_m):.3f} "
                    f"rel_m={float(location.relative_altitude_m):.3f}",
                    flush=True,
                )
                return
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "home altitude HAE unavailable: home_altitude_hae_m is not "
                    "configured and no vehicle location telemetry arrived within "
                    f"{self.home_altitude_timeout_s:.1f}s "
                    f"(endpoint={self.lattice_endpoint})"
                )
            self._pump_with_ingress(deadline)

    # -- Lattice agent surface ---------------------------------------------

    def _target_entity(self, entity_id: str) -> Entity | None:
        try:
            return self._lattice_call(
                "get_entity", lambda: self.lattice.entities.get_entity(entity_id)
            )
        except _LatticeTransportError:
            return None

    def _publish_entity_state(self) -> None:
        location = self.uav.location()
        if location is None:
            return
        attitude = self.uav.attitude()
        battery = get_record_state(self.state, "power", BatteryStatus)
        power_state = None
        if battery is not None:
            power_state = PowerState(
                source_id_to_state={
                    "mpfc": PowerSource(
                        power_type="POWER_TYPE_PRIMARY",
                        power_status="POWER_SOURCE_POWER_STATUS_ONLINE",
                        power_level=PowerLevel(
                            voltage=(
                                None
                                if battery.voltage_v is None
                                else float(battery.voltage_v)
                            ),
                            current_amps=(
                                None
                                if battery.current_a is None
                                else float(battery.current_a)
                            ),
                            percent_remaining=(
                                None
                                if battery.remaining_pct is None
                                else float(battery.remaining_pct) / 100.0
                            ),
                        ),
                    )
                }
            )
        # ENG-13: MAVLink relative altitude is home-relative; publish it as the
        # explicit AGL reference.  HAE is published too once the home datum is
        # known so consumers never have to guess which altitude they see.
        relative_alt_m = float(location.relative_altitude_m)
        hae_alt_m = (
            None
            if self.home_altitude_hae_m is None
            else self.home_altitude_hae_m + relative_alt_m
        )
        entity = Entity(
            entity_id=self.entity_id,
            is_live=True,
            expiry_time=datetime.now(timezone.utc)
            + timedelta(seconds=ENTITY_EXPIRY_S),
            aliases=Aliases(name=f"MPFC {self.client_id}"),
            data_classification=Classification(
                default=ClassificationInformation(
                    level="CLASSIFICATION_LEVELS_UNCLASSIFIED"
                )
            ),
            health=Health(
                connection_status="CONNECTION_STATUS_ONLINE",
                health_status="HEALTH_STATUS_HEALTHY",
            ),
            location=LatticeLocation(
                position=Position(
                    latitude_degrees=float(location.latitude_deg),
                    longitude_degrees=float(location.longitude_deg),
                    altitude_agl_meters=relative_alt_m,
                    altitude_hae_meters=hae_alt_m,
                ),
                speed_mps=math.hypot(
                    float(location.vx_m_s), float(location.vy_m_s)
                ),
                velocity_enu=Enu(
                    e=float(location.vx_m_s),
                    n=float(location.vy_m_s),
                    u=float(location.vz_m_s),
                ),
                attitude_enu=_attitude_enu_quaternion(attitude),
            ),
            mil_view=MilView(
                disposition="DISPOSITION_FRIENDLY",
                environment="ENVIRONMENT_AIR",
                platform_type="UAV",
            ),
            ontology=Ontology(template="TEMPLATE_ASSET", platform_type="UAV"),
            provenance=Provenance(
                integration_name=f"mpfc.{self.client_id}",
                data_type="MPFC UAV",
                source_id=self.client_id,
                source_update_time=datetime.now(timezone.utc),
            ),
            power_state=power_state,
            task_catalog=TaskCatalog(
                task_definitions=[
                    TaskDefinition(task_specification_url=url)
                    for url in SUPPORTED_SPECIFICATIONS
                ]
            ),
        )
        self._lattice_call(
            "publish_entity",
            lambda: self.lattice.entities.publish_entity(**entity.model_dump()),
        )

    def _maybe_publish_entity_state(self) -> None:
        """Refresh the Lattice entity on its configured interval.

        Execution runs block the main loop, so the entity must also be
        refreshed from the in-flight pump or the asset expires on the Lattice
        side (ENTITY_EXPIRY_S) for the whole duration of a task.  Transport
        failures are logged by ``_lattice_call`` and never stop the runtime.
        """
        now = time.monotonic()
        if now - self.last_state_publish < self.state_publish_interval_s:
            return
        try:
            self._publish_entity_state()
        except _LatticeTransportError:
            pass
        self.last_state_publish = now

    def _seed_status_version(self, task_id: str, status_version: int) -> None:
        """Seed a task's status version from the delivered task document."""
        current = self.status_versions.get(task_id)
        if current is not None and current >= int(status_version):
            return
        self.status_versions[task_id] = int(status_version)
        print(
            f"[TASK_STATUS_SEED] task_id={task_id} version={int(status_version)}",
            flush=True,
        )

    def _resync_status_version(
        self, task_id: str, expected_version: int | None
    ) -> int | None:
        """Re-fetch a task and adopt its authoritative status version."""
        try:
            task = self._lattice_call(
                "get_task",
                lambda: self.lattice.tasks.get_task(task_id=task_id),
                task_id=task_id,
                expected_version=expected_version,
            )
        except _LatticeTransportError:
            return None
        actual_version = int(task.version.status_version)
        self.status_versions[task_id] = actual_version
        print(
            f"[TASK_STATUS_RESYNC] task_id={task_id} "
            f"expected_version={expected_version} actual_version={actual_version}",
            flush=True,
        )
        return actual_version

    def _next_status_version(self, task_id: str) -> int:
        version = self.status_versions.get(task_id)
        if version is None:
            version = self._resync_status_version(task_id, None)
            if version is None:
                raise _LatticeTransportError(
                    f"status version unavailable for task_id={task_id} "
                    f"(endpoint={self.lattice_endpoint})"
                )
        return version + 1

    def _update_status(
        self,
        task_id: str,
        status: str,
        *,
        progress: float | None = None,
        error: TaskError | None = None,
    ) -> bool:
        """Update one task status; failures are logged, never silently dropped.

        Returns True when the update was accepted.  Version conflicts re-fetch
        the task and retry once from the authoritative version.  Transport
        failures do not fail the task: execution continues and the failure is
        visible with endpoint, operation, task id, and expected/actual version.
        """
        try:
            expected_version = self._next_status_version(task_id)
        except _LatticeTransportError:
            print(
                f"[TASK_STATUS_FAILED] task_id={task_id} status={status} "
                f"cause=status version unavailable "
                f"endpoint={self.lattice_endpoint}",
                flush=True,
            )
            return False
        progress_document = None
        if progress is not None:
            progress_document = GoogleProtobufAny(
                type=MPFC_PROGRESS_TYPE_URL,
                progress=max(0.0, min(1.0, float(progress))),
            )
        new_status = TaskStatus(
            status=status,
            task_error=error,
            progress=progress_document,
        )

        def _call(status_version: int) -> Any:
            return self.lattice.tasks.update_task_status(
                task_id=task_id,
                status_version=status_version,
                new_status=new_status,
                author=Principal(system=System(entity_id=self.entity_id)),
            )

        try:
            updated = self._lattice_call(
                "update_task_status",
                lambda: _call(expected_version),
                task_id=task_id,
                expected_version=expected_version,
            )
        except _LatticeTransportError:
            actual_version = self._resync_status_version(task_id, expected_version)
            if actual_version is None or actual_version == expected_version:
                print(
                    f"[TASK_STATUS_FAILED] task_id={task_id} status={status} "
                    f"expected_version={expected_version} "
                    f"actual_version={actual_version if actual_version is not None else 'unknown'} "
                    f"endpoint={self.lattice_endpoint}",
                    flush=True,
                )
                return False
            retry_version = actual_version + 1
            try:
                updated = self._lattice_call(
                    "update_task_status",
                    lambda: _call(retry_version),
                    task_id=task_id,
                    expected_version=retry_version,
                )
            except _LatticeTransportError:
                print(
                    f"[TASK_STATUS_FAILED] task_id={task_id} status={status} "
                    f"expected_version={retry_version} "
                    f"actual_version={actual_version} "
                    f"endpoint={self.lattice_endpoint}",
                    flush=True,
                )
                return False
        actual_version = int(updated.version.status_version)
        if actual_version != expected_version:
            print(
                f"[TASK_STATUS_RESYNC] task_id={task_id} "
                f"expected_version={expected_version} actual_version={actual_version}",
                flush=True,
            )
        self.status_versions[task_id] = actual_version
        print(
            f"[TASK_STATUS] task_id={task_id} status={status} "
            f"version={actual_version} progress={progress}",
            flush=True,
        )
        return True

    def _listener_main(self) -> None:
        listener_kwargs = dict(self.lattice_kwargs)
        listener_kwargs["timeout"] = self.lattice_listen_timeout_s
        listener_client = Lattice(**listener_kwargs)
        selector = EntityIdsSelector(entity_ids=[self.entity_id])
        while not self.listener_stop.is_set():
            try:
                request = self._lattice_call(
                    "listen_as_agent",
                    lambda: listener_client.tasks.listen_as_agent(
                        agent_selector=selector
                    ),
                    poll_timeouts_ok=True,
                )
            except _LatticeTransportError:
                self.listener_error = _LatticeTransportError(
                    f"listen_as_agent failed endpoint={self.lattice_endpoint}"
                )
                time.sleep(max(1.0, self.lattice_retry_delay_s))
                continue
            if request is not None:
                self.agent_requests.put(request)

    # -- task specification parsing ----------------------------------------

    def _resolve_objective(self, objective: Any) -> Destination:
        if not isinstance(objective, dict):
            raise ValueError("task objective must be an object")
        entity_id = _field(objective, "entityId", "entity_id")
        if entity_id:
            entity = self._target_entity(str(entity_id))
            if entity is None or entity.location is None or entity.location.position is None:
                raise ValueError(f"objective entity {entity_id} has no known position")
            position = entity.location.position
            if position.altitude_hae_meters is not None:
                altitude_m = float(position.altitude_hae_meters)
                altitude_reference = "hae"
            elif position.altitude_agl_meters is not None:
                altitude_m = float(position.altitude_agl_meters)
                altitude_reference = "agl"
            else:
                # An objective with no published altitude has no datum of its
                # own; use the explicit home datum (logged, never silent).
                if self.home_altitude_hae_m is None:
                    raise ValueError(
                        f"objective entity {entity_id} has no altitude and the "
                        "home HAE datum is unavailable"
                    )
                altitude_m = float(self.home_altitude_hae_m)
                altitude_reference = "hae"
                print(
                    f"[OBJECTIVE_DATUM] entity={entity_id} has no published "
                    f"altitude; using home HAE datum hae_m={altitude_m:.3f}",
                    flush=True,
                )
            return Destination(
                latitude_deg=float(position.latitude_degrees),
                longitude_deg=float(position.longitude_degrees),
                altitude_m=altitude_m,
                altitude_reference=altitude_reference,
            )
        point = _field(objective, "point")
        if point:
            lla = _field(point, "lla")
            if not isinstance(lla, dict):
                raise ValueError("point objective has no lla")
            return self._lla_destination(lla)
        # Sample-app Orbit objectives carry the lla oneof directly.
        lla = _field(objective, "lla")
        if isinstance(lla, dict):
            return self._lla_destination(lla)
        raise ValueError(f"unsupported task objective {objective!r}")

    @staticmethod
    def _lla_destination(lla: dict) -> Destination:
        latitude = _float_field(lla, "latitudeDegrees", "latitude_degrees", "lat")
        longitude = _float_field(lla, "longitudeDegrees", "longitude_degrees", "lon")
        altitude = _float_field(
            lla, "altitudeHaeM", "altitude_hae_meters", "alt", "altitudeM"
        )
        if latitude is None or longitude is None:
            raise ValueError(f"objective lla missing lat/lon: {lla}")
        if altitude is None:
            raise ValueError(f"objective lla missing altitude: {lla}")
        reference = _standard_altitude_reference(
            _field(lla, "altitudeReference", "altitude_reference"), "hae"
        )
        return Destination(
            latitude_deg=latitude,
            longitude_deg=longitude,
            altitude_m=altitude,
            altitude_reference=reference,
        )

    def _transit_destination(self, specification: Any) -> Destination:
        plan = _field(specification, "plan")
        route = _field(plan, "route")
        path = _field(route, "path")
        if not isinstance(path, list) or not path:
            raise ValueError("Transit task has no route path")
        segment = path[-1]
        waypoint = _field(segment, "waypoint")
        if waypoint:
            lla = _field(waypoint, "llaPoint", "lla_point")
            if isinstance(lla, dict):
                return self._lla_destination(lla)
        loiter = _field(segment, "loiter")
        if loiter:
            center = _field(loiter, "center", "loiterCenter")
            if isinstance(center, dict):
                return self._lla_destination(center)
        raise ValueError("Transit task final path segment has no point")

    def _orbit_spec(self, specification: Any) -> OrbitSpec:
        objective = _field(specification, "objective")
        destination = self._resolve_objective(objective)
        radius = _float_field(specification, "orbitRadius", "orbit_radius")
        height = _float_field(specification, "orbitHeight", "orbit_height")
        if radius is None:
            raise ValueError("Orbit task missing orbitRadius")
        if height is None:
            raise ValueError("Orbit task missing orbitHeight")
        direction = str(
            _field(specification, "orbitDirection", "orbit_direction")
            or "ORBIT_CLOCKWISE"
        )
        return OrbitSpec(
            latitude_deg=destination.latitude_deg,
            longitude_deg=destination.longitude_deg,
            altitude_hae_m=destination.altitude_m,
            radius_m=radius,
            height_m=height,
            direction=direction,
        )

    # -- execution ----------------------------------------------------------

    def _wait_for_location(self, timeout_s: float) -> GlobalPositionInt:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            location = self.uav.location()
            if location is not None:
                return location
            if time.monotonic() > deadline:
                raise RuntimeError("timed out waiting for UAV position telemetry")
            self._pump_with_ingress(deadline)

    def _wait_response(self, request_id: str, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            if request_id in self.bus.responses:
                return self.bus.responses.pop(request_id)
            if time.monotonic() > deadline:
                raise RuntimeError(f"timeout waiting for UAV result id={request_id}")
            self._pump_with_ingress(deadline)

    def _wait_until(self, predicate: Any, timeout_s: float, error: str) -> None:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            if predicate():
                return
            if time.monotonic() > deadline:
                raise RuntimeError(error)
            self._pump_with_ingress(deadline)

    def _vehicle_control(self) -> VehicleControl:
        control = self.uav.flight_control()
        if control is None:
            raise RuntimeError("timed out waiting for UAV vehicle state")
        return control

    def _arrival_altitude_m(self, location: GlobalPositionInt, reference: str) -> float:
        if reference in ("relative", "agl"):
            return float(location.relative_altitude_m)
        if reference in ("asl", "amsl"):
            return float(location.altitude_m)
        if reference == "hae":
            if self.home_altitude_hae_m is None:
                raise RuntimeError(
                    "destination altitude reference is HAE but home_altitude_hae_m "
                    "is not configured; cannot resolve the vehicle altitude datum"
                )
            return float(location.relative_altitude_m) + float(self.home_altitude_hae_m)
        raise RuntimeError(f"unsupported altitude reference {reference!r}")

    def _command_altitude(self, destination: Destination) -> tuple[float, str]:
        """Convert a destination altitude to a vehicle-resolvable command altitude.

        The vehicle reports altitude relative to its home position (MAVLink
        ``relative_alt``); HAE is resolved against the configured home HAE so
        no ambiguous altitude ever reaches the vehicle.  ENG-13.
        """
        reference = destination.altitude_reference
        if reference == "hae":
            if self.home_altitude_hae_m is None:
                raise RuntimeError(
                    "destination altitude reference is HAE but home_altitude_hae_m "
                    "is not configured; cannot resolve the vehicle altitude datum"
                )
            return destination.altitude_m - float(self.home_altitude_hae_m), "relative"
        if reference in ("relative", "agl"):
            return destination.altitude_m, "relative"
        if reference in ("asl", "amsl"):
            return destination.altitude_m, "asl"
        raise RuntimeError(f"unsupported altitude reference {reference!r}")

    def _arrival_metrics(
        self, location: GlobalPositionInt, destination: Destination
    ) -> tuple[float, float]:
        horizontal_m = _distance_m(
            location.latitude_deg,
            location.longitude_deg,
            destination.latitude_deg,
            destination.longitude_deg,
        )
        observed_altitude_m = self._arrival_altitude_m(
            location, destination.altitude_reference
        )
        return horizontal_m, abs(observed_altitude_m - destination.altitude_m)

    def _relative_altitude(self) -> float | None:
        location = self.uav.location()
        if location is None:
            return None
        return float(location.relative_altitude_m)

    def _prepare_vehicle_for_move(self, destination: Destination) -> None:
        control = self._vehicle_control()
        if control.heartbeat.armed and self._in_air():
            return
        if not self.auto_takeoff_for_move:
            raise RuntimeError(
                "Transit requires an airborne vehicle when auto_takeoff_for_move is false"
            )

        takeoff_altitude_m = self.takeoff_altitude_m
        if destination.altitude_reference == "relative" and destination.altitude_m > 0.0:
            takeoff_altitude_m = min(takeoff_altitude_m, destination.altitude_m)
        if takeoff_altitude_m <= 0.0:
            raise RuntimeError("Transit automatic takeoff altitude must be positive")

        self.uav.execute(
            self.uav.takeoff_altitude_command(takeoff_altitude_m),
            timeout_s=self.response_timeout_s,
        )
        self._wait_until(
            lambda: self._readiness().arm_ready,
            self.state_timeout_s,
            "timed out waiting for UAV arm readiness",
        )

        control = self._vehicle_control()
        if not control.heartbeat.armed:
            self.uav.execute(self.uav.arm_command(True), timeout_s=self.response_timeout_s)
            self._wait_until(
                lambda: self._vehicle_control().heartbeat.armed,
                self.state_timeout_s,
                "timed out waiting for UAV armed state",
            )

        self._wait_until(
            lambda: self._readiness().takeoff_ready,
            self.state_timeout_s,
            "timed out waiting for UAV takeoff readiness",
        )
        self.uav.execute(
            self.uav.takeoff_command(takeoff_altitude_m),
            timeout_s=self.response_timeout_s,
        )
        self._wait_until(
            lambda: (
                self._in_air()
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

    def _readiness(self):
        control = self.uav.flight_control()
        if control is None:
            return _EmptyReadiness()
        return control.readiness

    def _in_air(self) -> bool:
        location = self.uav.location()
        if location is None:
            return False
        return float(location.relative_altitude_m) > 0.5

    def _execute_transit(self, task_id: str, destination: Destination) -> None:
        self._wait_for_location(self.state_timeout_s)
        self._prepare_vehicle_for_move(destination)
        current = self._wait_for_location(self.state_timeout_s)
        initial_horizontal_m, initial_altitude_error_m = self._arrival_metrics(
            current, destination
        )
        horizontal_denominator = max(initial_horizontal_m, self.arrival_radius_m, 0.01)
        altitude_denominator = max(
            initial_altitude_error_m,
            self.arrival_altitude_tolerance_m,
            0.01,
        )

        command_altitude_m, command_reference = self._command_altitude(destination)
        self.uav.execute(
            self.uav.go_to_command(
                destination.latitude_deg,
                destination.longitude_deg,
                command_altitude_m,
                altitude_reference=command_reference,
            ),
            timeout_s=self.response_timeout_s,
        )

        deadline = time.monotonic() + self.execution_timeout_s
        last_progress_publish = 0.0
        while True:
            if self.active_cancel_requested:
                raise _TaskCancelled()
            current = self.uav.location()
            if current is not None:
                horizontal_m, altitude_error_m = self._arrival_metrics(
                    current, destination
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
                    self._update_status(task_id, "STATUS_DONE_OK", progress=1.0)
                    return
                now = time.monotonic()
                if now - last_progress_publish >= self.progress_interval_s:
                    self._update_status(
                        task_id, "STATUS_EXECUTING", progress=progress
                    )
                    last_progress_publish = now

            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"Transit timed out after {self.execution_timeout_s:.1f}s without arrival"
                )
            self._pump_with_ingress(
                min(deadline, time.monotonic() + self.poll_interval_s)
            )

    def _execute_orbit(self, task_id: str, orbit: OrbitSpec) -> None:
        # The sample Orbit task asks the asset to orbit the objective.  MPFC
        # arms/takes off when needed and flies to the objective at
        # centre-HAE + orbit height (explicit datum); the available MAVLink
        # command set has no orbit-radius/direction command in this toolchain,
        # so the agent holds on the objective until the manager completes or
        # cancels the task.  The hold/orbit state is only declared after the
        # vehicle has actually arrived at the hold point.
        print(
            f"[TASK_ORBIT] task_id={task_id} lat={orbit.latitude_deg} "
            f"lon={orbit.longitude_deg} hae_m={orbit.altitude_hae_m} "
            f"radius_m={orbit.radius_m} height_m={orbit.height_m} "
            f"direction={orbit.direction}",
            flush=True,
        )
        orbit_destination = Destination(
            latitude_deg=orbit.latitude_deg,
            longitude_deg=orbit.longitude_deg,
            altitude_m=orbit.altitude_hae_m + orbit.height_m,
            altitude_reference="hae",
        )
        self._wait_for_location(self.state_timeout_s)
        self._prepare_vehicle_for_move(orbit_destination)
        current = self._wait_for_location(self.state_timeout_s)
        initial_horizontal_m, initial_altitude_error_m = self._arrival_metrics(
            current, orbit_destination
        )
        horizontal_denominator = max(initial_horizontal_m, self.arrival_radius_m, 0.01)
        altitude_denominator = max(
            initial_altitude_error_m, self.arrival_altitude_tolerance_m, 0.01
        )
        command_altitude_m, command_reference = self._command_altitude(orbit_destination)
        self.uav.execute(
            self.uav.go_to_command(
                orbit.latitude_deg,
                orbit.longitude_deg,
                command_altitude_m,
                altitude_reference=command_reference,
            ),
            timeout_s=self.response_timeout_s,
        )
        self._update_status(task_id, "STATUS_EXECUTING", progress=0.0)

        # Ingress: wait for arrival at the hold point before declaring orbit.
        deadline = time.monotonic() + self.execution_timeout_s
        last_progress_publish = 0.0
        while True:
            if self.active_cancel_requested:
                raise _TaskCancelled()
            current = self.uav.location()
            if current is not None:
                horizontal_m, altitude_error_m = self._arrival_metrics(
                    current, orbit_destination
                )
                if (
                    horizontal_m <= self.arrival_radius_m
                    and altitude_error_m <= self.arrival_altitude_tolerance_m
                ):
                    print(
                        f"[TASK_ORBIT_ARRIVED] task_id={task_id} "
                        f"horizontal_m={horizontal_m:.2f} "
                        f"altitude_error_m={altitude_error_m:.2f}",
                        flush=True,
                    )
                    self._update_status(task_id, "STATUS_EXECUTING", progress=1.0)
                    break
                now = time.monotonic()
                if now - last_progress_publish >= self.progress_interval_s:
                    remaining_fraction = max(
                        horizontal_m / horizontal_denominator,
                        altitude_error_m / altitude_denominator,
                    )
                    self._update_status(
                        task_id,
                        "STATUS_EXECUTING",
                        progress=max(0.0, min(1.0, 1.0 - remaining_fraction)),
                    )
                    last_progress_publish = now
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"Orbit ingress timed out after {self.execution_timeout_s:.1f}s "
                    "without arrival"
                )
            self._pump_with_ingress(
                min(deadline, time.monotonic() + self.poll_interval_s)
            )

        # Keep the agent alive on the task: report executing until the manager
        # sends complete/cancel or a telemetry-driven progress source exists.
        # Pump on a deadline instead of sleeping a fixed interval: the pump
        # blocks only when the bus is idle, so full-rate telemetry keeps the
        # published entity current throughout the task.  Exiting the hold is
        # either a cancel or a complete request; both are reported by the
        # execute handler, so surface it as a task interruption.
        while not self.active_cancel_requested:
            self._pump_with_ingress(time.monotonic() + self.poll_interval_s)
        raise _TaskCancelled()

    def _handle_execute(self, request: Any) -> None:
        task = request.execute_request.task
        task_id = str(task.version.task_id)
        specification = task.specification
        spec_url = "" if specification is None else str(specification.type or "")
        self._seed_status_version(task_id, int(task.version.status_version))
        state = self.task_states.get(task_id)
        if state is not None:
            print(
                f"[TASK_DUPLICATE] task_id={task_id} state={state} dropped",
                flush=True,
            )
            return
        print(
            f"[TASK_EXECUTE] task_id={task_id} spec={spec_url} "
            f"display_name={task.display_name!r}",
            flush=True,
        )
        if spec_url not in SUPPORTED_SPECIFICATIONS:
            self.task_states[task_id] = "terminal"
            self._update_status(
                task_id,
                "STATUS_DONE_NOT_OK",
                error=TaskError(
                    code="ERROR_CODE_REJECTED",
                    message=f"unsupported task specification {spec_url!r}",
                ),
            )
            return
        if self.active_task_id is not None:
            self.task_states[task_id] = "queued"
            self.queued_executes.append((task_id, request))
            # Machine receipt for the queued task; it starts when the active
            # task ends unless a cancel/complete for it arrives first.
            self._update_status(task_id, "STATUS_ACK")
            print(
                f"[TASK_QUEUED] task_id={task_id} spec={spec_url} "
                f"active={self.active_task_id}",
                flush=True,
            )
            return
        self._start_execute(task_id, request)

    def _start_execute(self, task_id: str, request: Any) -> None:
        self.task_states[task_id] = "active"
        task = request.execute_request.task
        specification = task.specification
        spec_url = "" if specification is None else str(specification.type or "")
        self._update_status(task_id, "STATUS_ACK")
        try:
            if spec_url == TRANSIT_SPECIFICATION_URL:
                destination = self._transit_destination(specification)
                orbit = None
            else:
                destination = None
                orbit = self._orbit_spec(specification)
        except Exception as exc:
            self.task_states[task_id] = "terminal"
            self._update_status(
                task_id,
                "STATUS_DONE_NOT_OK",
                error=TaskError(code="ERROR_CODE_REJECTED", message=str(exc)),
            )
            return
        self._update_status(task_id, "STATUS_EXECUTING", progress=0.0)
        with self.active_lock:
            self.active_task_id = task_id
            self.active_specification_url = spec_url
            self.active_cancel_requested = False
            self.active_complete_requested = False
        self._set_lifecycle("EXECUTING")
        try:
            if spec_url == TRANSIT_SPECIFICATION_URL:
                self._execute_transit(task_id, destination)
            else:
                self._execute_orbit(task_id, orbit)
        except _TaskCancelled:
            with self.active_lock:
                completed = self.active_complete_requested
            if not completed:
                self._update_status(task_id, "STATUS_CANCEL_REQUESTED")
                self._update_status(
                    task_id,
                    "STATUS_DONE_NOT_OK",
                    error=TaskError(
                        code="ERROR_CODE_CANCELLED", message="task cancelled"
                    ),
                )
        except Exception as exc:
            self._update_status(
                task_id,
                "STATUS_DONE_NOT_OK",
                error=TaskError(code="ERROR_CODE_FAILED", message=str(exc)),
            )
            print(
                f"[EXECUTION_FAILED] task_id={task_id} error={exc}\n"
                f"{traceback.format_exc().strip()}",
                flush=True,
            )
        finally:
            with self.active_lock:
                self.active_task_id = None
                self.active_specification_url = None
                self.active_cancel_requested = False
                self.active_complete_requested = False
            # Terminal for the rest of the process lifetime: a redelivered task
            # id is dropped rather than executed twice.
            self.task_states[task_id] = "terminal"
            self._set_lifecycle("IDLE")

    def _take_queued_execute(self) -> None:
        """Start the oldest queued task that has not been cancelled/dropped."""
        while self.queued_executes:
            task_id, request = self.queued_executes.popleft()
            if self.task_states.get(task_id) != "queued":
                print(
                    f"[TASK_QUEUED_DROPPED] task_id={task_id} state="
                    f"{self.task_states.get(task_id)}",
                    flush=True,
                )
                continue
            self._start_execute(task_id, request)
            return

    def _handle_cancel(self, request: Any) -> None:
        task_id = str(request.cancel_request.task_id)
        print(f"[TASK_CANCEL] task_id={task_id}", flush=True)
        with self.active_lock:
            matches = self.active_task_id == task_id
            if matches:
                self.active_cancel_requested = True
        if matches:
            # The execution loop reports CANCEL_REQUESTED/DONE_NOT_OK.
            return
        state = self.task_states.get(task_id)
        if state == "terminal":
            print(
                f"[TASK_CANCEL_TERMINAL] task_id={task_id} ignored (already terminal)",
                flush=True,
            )
            return
        if state == "queued":
            # Drop the queued execute; it must never start after cancellation.
            self.task_states[task_id] = "terminal"
            print(f"[TASK_QUEUED_CANCELLED] task_id={task_id}", flush=True)
        self._update_status(task_id, "STATUS_CANCEL_REQUESTED")
        self._update_status(
            task_id,
            "STATUS_DONE_NOT_OK",
            error=TaskError(code="ERROR_CODE_CANCELLED", message="task cancelled"),
        )

    def _handle_complete(self, request: Any) -> None:
        task_id = str(request.complete_request.task_id)
        print(f"[TASK_COMPLETE] task_id={task_id}", flush=True)
        with self.active_lock:
            if self.active_task_id == task_id:
                self.active_cancel_requested = True
                self.active_complete_requested = True
                matches = True
            else:
                matches = False
        state = self.task_states.get(task_id)
        if not matches and state == "terminal":
            print(
                f"[TASK_COMPLETE_TERMINAL] task_id={task_id} ignored (already terminal)",
                flush=True,
            )
            return
        if not matches and state == "queued":
            self.task_states[task_id] = "terminal"
            print(f"[TASK_QUEUED_COMPLETED] task_id={task_id}", flush=True)
        self._update_status(task_id, "STATUS_DONE_OK", progress=1.0)

    def _handle_request(self, request: Any) -> None:
        if request.execute_request is not None:
            self._handle_execute(request)
        elif request.cancel_request is not None:
            self._handle_cancel(request)
        elif request.complete_request is not None:
            self._handle_complete(request)

    # -- runtime ------------------------------------------------------------

    def _pump_with_ingress(self, deadline: float | None = None) -> None:
        """Pump the node bus while vehicle work is in progress.

        Cancel and complete requests are serviced here so a task in flight can
        be interrupted; execute requests for other tasks are acknowledged and
        queued by the task state machine rather than starting a second
        execution on top of the active one.
        """
        self._maybe_publish_entity_state()
        self._pump_once(deadline)
        self._check_link_failsafe()
        while True:
            try:
                request = self.agent_requests.get_nowait()
            except queue.Empty:
                break
            try:
                self._handle_request(request)
            except Exception as exc:
                print(
                    f"[TASK_HANDLING_FAILED] error={exc}\n"
                    f"{traceback.format_exc().strip()}",
                    flush=True,
                )

    def run(self) -> None:
        self.listener_stop.clear()
        self.listener_thread = threading.Thread(
            target=self._listener_main, name="lattice-agent-listen", daemon=True
        )
        self.listener_thread.start()
        self.send_online()
        self._set_lifecycle("IDLE")
        try:
            self._resolve_home_altitude_hae()
            while True:
                self._maybe_publish_entity_state()
                try:
                    request = self.agent_requests.get_nowait()
                except queue.Empty:
                    request = None
                if request is not None:
                    try:
                        self._handle_request(request)
                    except Exception as exc:
                        print(
                            f"[TASK_HANDLING_FAILED] error={exc}\n"
                            f"{traceback.format_exc().strip()}",
                            flush=True,
                        )
                elif self.active_task_id is None:
                    self._take_queued_execute()
                self._pump_once(
                    time.monotonic() + self.poll_interval_s
                )
                self._check_link_failsafe()
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
            self.listener_stop.set()
            if self.listener_thread is not None:
                self.listener_thread.join(timeout=1.0)
            if self.lifecycle_state != "FAULTED":
                try:
                    self._set_lifecycle("STOPPING")
                except Exception:
                    pass
            self.stop()


class _LatticeTransportError(RuntimeError):
    """A Lattice exchange failed after bounded retries (diagnostics logged)."""


class _TaskCancelled(Exception):
    pass


class _EmptyReadiness:
    arm_ready = False
    takeoff_ready = False
    problems: tuple[str, ...] = ()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    ExecutionIngress(cfg, bus_config).run()
