#!/usr/bin/env python3
"""
Lattice agent interface plugin: connection, task translation, entity publishing.

Usage:
    from plugins.lattice.lattice import run_plugin
    run_plugin(cfg, bus_config)

The plugin wraps the official anduril-lattice-sdk client and is the agent
surface of MPFC:

* it owns the API connection and token lifecycle declared in
  protocols/lattice.yaml;
* it listens as the configured asset, decodes execute requests into the
  protocol task shape, and rejects unsupported or malformed tasks visibly;
* it performs versioned task status updates seeded from the delivered task,
  adopting the version returned by the endpoint and resyncing on mismatch;
* it encodes the published vehicle state into a Lattice entity and refreshes
  it on the configured interval.

No mission policy and no vehicle command logic live here; cores drive tasks
and missions over the bus.
"""

import collections
import queue
import threading
import time
import traceback
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Any, Callable, Deque, Dict

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
from anduril.oauth.types.get_token_response import GetTokenResponse

from lib.altitude import home_hae_from_telemetry, relative_to_hae
from lib.common import (
    build_event_topics,
    build_request_topic,
    build_response_topic,
    build_state_scheduler_topics,
    build_state_topics,
    build_topic_base,
)
from lib.lattice_specs import (
    ORBIT_SPECIFICATION_URL,
    SUPPORTED_SPECIFICATION_URLS,
    TRANSIT_SPECIFICATION_URL,
)
from lib.plugin_base import PluginBase
from lib.reference_frames import attitude_enu_quaternion
from lib.state_scheduler import StateScheduler
from protocols.namespace_loader import load_protocol_namespace

LATTICE = load_protocol_namespace("lattice")
UAV = load_protocol_namespace("uav")

MPFC_PROGRESS_TYPE_URL = "type.googleapis.com/mpfc.TaskProgress"

ENTITY_EXPIRY_S = 15.0
DEFAULT_TOKEN_EXPIRES_IN_S = 3600
TOKEN_EXPIRY_BUFFER_S = 120.0

LATTICE_ERRORS = (ApiError, httpx.HTTPError)
LATTICE_POLL_TIMEOUTS = (RequestTimeoutError, httpx.ReadTimeout, httpx.ReadError)

# Bus status/error keys -> official Lattice API status and error codes.
TASK_STATUS_BY_KEY = {
    LATTICE.Enums.LatticeTaskStatus.Ack: "STATUS_ACK",
    LATTICE.Enums.LatticeTaskStatus.Executing: "STATUS_EXECUTING",
    LATTICE.Enums.LatticeTaskStatus.CancelRequested: "STATUS_CANCEL_REQUESTED",
    LATTICE.Enums.LatticeTaskStatus.DoneOk: "STATUS_DONE_OK",
    LATTICE.Enums.LatticeTaskStatus.DoneNotOk: "STATUS_DONE_NOT_OK",
}
TASK_ERROR_CODE_BY_KEY = {
    LATTICE.Enums.LatticeTaskErrorCode.Rejected: "ERROR_CODE_REJECTED",
    LATTICE.Enums.LatticeTaskErrorCode.Cancelled: "ERROR_CODE_CANCELLED",
    LATTICE.Enums.LatticeTaskErrorCode.Failed: "ERROR_CODE_FAILED",
}
ALTITUDE_REFERENCE_HAE = LATTICE.Enums.LatticeAltitudeReference.Hae
ALTITUDE_REFERENCE_AGL = LATTICE.Enums.LatticeAltitudeReference.Agl
ALTITUDE_REFERENCE_RELATIVE = LATTICE.Enums.LatticeAltitudeReference.Relative
ALTITUDE_REFERENCE_ASL = LATTICE.Enums.LatticeAltitudeReference.Asl


class LatticeConnectionError(RuntimeError):
    def __init__(self, operation: str, endpoint: str, detail: str) -> None:
        self.operation = operation
        self.endpoint = endpoint
        self.detail = detail
        super().__init__(f"lattice {operation} failed endpoint={endpoint} detail={detail}")


def _field(node: Any, *names: str) -> Any:  # Read a field from SDK models, extra fields, or dicts.
    if node is None:
        return None
    for name in names:
        if isinstance(node, dict):
            value = node.get(name)
        else:
            value = getattr(node, name, None)
            if value is None and hasattr(node, "model_extra"):
                value = (node.model_extra or {}).get(name)
        if value is not None:
            return value
    return None


def _float_field(node: Any, *names: str) -> float | None:  # Read an optional float field.
    value = _field(node, *names)
    if value is None:
        return None
    return float(value)


def _altitude_reference(raw: Any, default: str) -> str:  # Normalize an SDK altitude reference to a protocol key.
    if raw is None:
        return default
    name = str(raw).upper()
    if "WGS84" in name or "ELLIPSOID" in name:
        return ALTITUDE_REFERENCE_HAE
    if "EGM96" in name or "MSL" in name or "MEAN_SEA" in name:
        return ALTITUDE_REFERENCE_ASL
    if "AGL" in name or "GROUND" in name:
        return ALTITUDE_REFERENCE_AGL
    return default


@dataclass(frozen=True)
class DecodedTask:
    task_id: str
    specification_type: str
    display_name: str
    latitude_deg: float | None
    longitude_deg: float | None
    altitude_m: float | None
    altitude_reference: str | None
    orbit_radius_m: float | None
    orbit_height_m: float | None
    orbit_direction: str | None
    received_at_ms: int

    def payload(self) -> Dict[str, Any]:  # Bus payload keyed by protocols/lattice.yaml field names.
        return {
            "TaskId": self.task_id,
            "SpecificationType": self.specification_type,
            "DisplayName": self.display_name,
            "LatitudeDeg": self.latitude_deg,
            "LongitudeDeg": self.longitude_deg,
            "AltitudeM": self.altitude_m,
            "AltitudeReference": self.altitude_reference,
            "OrbitRadiusM": self.orbit_radius_m,
            "OrbitHeightM": self.orbit_height_m,
            "OrbitDirection": self.orbit_direction,
            "ReceivedAtMs": self.received_at_ms,
        }


class TaskDecoder:
    """Decode official Lattice task specifications into the bus task shape."""

    def __init__(
        self,
        resolve_entity: Callable[[str], Any] | None = None,
        resolve_home_hae: Callable[[], float | None] | None = None,
    ) -> None:
        self._resolve_entity = resolve_entity
        self._resolve_home_hae = resolve_home_hae

    def decode(self, task: Any) -> DecodedTask:  # Decode one SDK Task document.
        version = _field(task, "version")
        task_id = "" if version is None else str(_field(version, "task_id") or "")
        specification = _field(task, "specification")
        spec_url = "" if specification is None else str(_field(specification, "type") or "")
        if not task_id:
            raise ValueError("task document has no task_id")
        if spec_url not in SUPPORTED_SPECIFICATION_URLS:
            raise ValueError(f"unsupported task specification {spec_url!r}")
        display_name = _field(task, "display_name")
        if spec_url == TRANSIT_SPECIFICATION_URL:
            destination = self._transit_destination(specification)
            orbit_radius_m = orbit_height_m = orbit_direction = None
        else:
            orbit = self._orbit_spec(specification)
            destination = orbit["destination"]
            orbit_radius_m = orbit["radius_m"]
            orbit_height_m = orbit["height_m"]
            orbit_direction = orbit["direction"]
        return DecodedTask(
            task_id=task_id,
            specification_type=spec_url,
            display_name="" if display_name is None else str(display_name),
            latitude_deg=destination["latitude_deg"],
            longitude_deg=destination["longitude_deg"],
            altitude_m=destination["altitude_m"],
            altitude_reference=destination["altitude_reference"],
            orbit_radius_m=orbit_radius_m,
            orbit_height_m=orbit_height_m,
            orbit_direction=orbit_direction,
            received_at_ms=int(time.time() * 1000),
        )

    def _resolve_objective(self, objective: Any) -> Dict[str, Any]:  # Resolve a task objective to lat/lon/altitude.
        if not isinstance(objective, dict):
            raise ValueError("task objective must be an object")
        entity_id = _field(objective, "entityId", "entity_id")
        if entity_id:
            if self._resolve_entity is None:
                raise ValueError(f"objective entity {entity_id} cannot be resolved by this interface")
            entity = self._resolve_entity(str(entity_id))
            if entity is None:
                raise ValueError(f"objective entity {entity_id} not found")
            location = _field(entity, "location")
            position = _field(location, "position") if location is not None else None
            if position is None:
                raise ValueError(f"objective entity {entity_id} has no known position")
            latitude = _float_field(position, "latitude_degrees", "latitudeDegrees")
            longitude = _float_field(position, "longitude_degrees", "longitudeDegrees")
            if latitude is None or longitude is None:
                raise ValueError(f"objective entity {entity_id} has no usable position")
            altitude = _float_field(position, "altitude_hae_meters", "altitudeHaeMeters")
            reference = ALTITUDE_REFERENCE_HAE
            if altitude is None:
                altitude = _float_field(position, "altitude_agl_meters", "altitudeAglMeters")
                reference = ALTITUDE_REFERENCE_AGL
            if altitude is None:
                # An objective with no published altitude has no datum of its
                # own; use the explicit home HAE datum, logged, never silent.
                home_hae = None if self._resolve_home_hae is None else self._resolve_home_hae()
                if home_hae is None:
                    raise ValueError(
                        f"objective entity {entity_id} has no altitude and the home HAE datum is unavailable"
                    )
                altitude = home_hae
                reference = ALTITUDE_REFERENCE_HAE
                print(
                    f"[OBJECTIVE_DATUM] entity={entity_id} has no published altitude; "
                    f"using home HAE datum hae_m={altitude:.3f}",
                    flush=True,
                )
            return {
                "latitude_deg": latitude,
                "longitude_deg": longitude,
                "altitude_m": altitude,
                "altitude_reference": reference,
            }
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
    def _lla_destination(lla: dict) -> Dict[str, Any]:  # Decode a LLA waypoint.
        latitude = _float_field(lla, "latitudeDegrees", "latitude_degrees", "lat")
        longitude = _float_field(lla, "longitudeDegrees", "longitude_degrees", "lon")
        altitude = _float_field(lla, "altitudeHaeM", "altitude_hae_meters", "alt", "altitudeM")
        if latitude is None or longitude is None:
            raise ValueError(f"objective lla missing lat/lon: {lla}")
        if altitude is None:
            raise ValueError(f"objective lla missing altitude: {lla}")
        reference = _altitude_reference(
            _field(lla, "altitudeReference", "altitude_reference"), ALTITUDE_REFERENCE_HAE
        )
        return {
            "latitude_deg": latitude,
            "longitude_deg": longitude,
            "altitude_m": altitude,
            "altitude_reference": reference,
        }

    def _transit_destination(self, specification: Any) -> Dict[str, Any]:  # Decode a Transit final waypoint.
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

    def _orbit_spec(self, specification: Any) -> Dict[str, Any]:  # Decode a sample-app Orbit task.
        objective = _field(specification, "objective")
        destination = self._resolve_objective(objective)
        radius = _float_field(specification, "orbitRadius", "orbit_radius")
        height = _float_field(specification, "orbitHeight", "orbit_height")
        if radius is None:
            raise ValueError("Orbit task missing orbitRadius")
        if height is None:
            raise ValueError("Orbit task missing orbitHeight")
        direction = str(_field(specification, "orbitDirection", "orbit_direction") or "ORBIT_CLOCKWISE")
        return {
            "destination": destination,
            "radius_m": radius,
            "height_m": height,
            "direction": direction,
        }


class TaskStatusVersions:
    """Server-driven status version bookkeeping for agent tasks."""

    def __init__(self) -> None:
        self._versions: Dict[str, int] = {}

    def seed(self, task_id: str, status_version: int) -> bool:  # Seed from a delivered task; True when changed.
        current = self._versions.get(task_id)
        if current is not None and current >= int(status_version):
            return False
        self._versions[task_id] = int(status_version)
        return True

    def adopt(self, task_id: str, status_version: int) -> None:  # Adopt the endpoint-authoritative version.
        self._versions[task_id] = int(status_version)

    def get(self, task_id: str) -> int | None:  # Current version, None when unseeded.
        return self._versions.get(task_id)

    def next_version(self, task_id: str) -> int | None:  # Version for the next update, None when unseeded.
        version = self._versions.get(task_id)
        if version is None:
            return None
        return version + 1


class LatticeConfig:
    def __init__(self, cfg: Dict[str, Any]) -> None:  # Parse Lattice connection settings.
        self.base_url = str(cfg["base_url"]).rstrip("/")
        self.tls_verify = cfg.get("tls_verify", True)
        self.ca_bundle = cfg.get("ca_bundle")
        self.client_id = cfg.get("client_id")
        self.client_secret = cfg.get("client_secret")
        self.token = cfg.get("token")
        token_endpoint = cfg.get("token_endpoint")
        self.token_endpoint = (
            str(token_endpoint).rstrip("/") if token_endpoint else f"{self.base_url}/api/v1/oauth/token"
        )
        self.timeout_s = float(cfg.get("timeout_s", 10.0))
        self.max_retries = int(cfg.get("max_retries", 2))
        self.reconnect_secs = float(cfg.get("reconnect_secs", 5.0))
        self.listen_timeout_s = float(cfg.get("listen_timeout_s", 300.0))
        self.lattice_attempts = max(1, int(cfg.get("lattice_attempts", 3)))
        self.lattice_retry_delay_s = max(0.0, float(cfg.get("lattice_retry_delay_s", 1.0)))
        if not self.token and not (self.client_id and self.client_secret):
            raise ValueError("lattice config requires either token or both client_id and client_secret")

    def verify(self) -> bool | str:  # TLS verification setting for httpx.
        if self.ca_bundle:
            return str(self.ca_bundle)
        return bool(self.tls_verify)


class LatticeConnection:
    def __init__(self, config: LatticeConfig, http_client: httpx.Client | None = None) -> None:  # Initialize connection state.
        self.config = config
        self._http = http_client
        self._owns_http = http_client is None
        self._client: Lattice | None = None
        self._token: str | None = config.token
        self._token_expires_at = float("inf") if config.token else 0.0
        self._token_acquisitions = 0
        self._token_lock = threading.Lock()
        self._next_attempt = 0.0
        self.connected = False
        self.connect_attempts = 0
        self.last_error: LatticeConnectionError | None = None

    @property
    def client(self) -> Lattice:  # SDK client, valid only while connected.
        if self._client is None:
            raise LatticeConnectionError("client", self.config.base_url, "not connected")
        return self._client

    @property
    def token_acquisitions(self) -> int:  # Number of successful token acquisitions.
        return self._token_acquisitions

    def token_valid(self) -> bool:  # True while a usable token is cached.
        return self._token is not None and time.time() < self._token_expires_at

    def token_expires_at_ms(self) -> int:  # Token expiry as epoch milliseconds; 0 when not tracked.
        if self._token_expires_at == float("inf"):
            return 0
        return int(self._token_expires_at * 1000)

    def token_value(self) -> str:  # Bearer token for SDK requests, refreshed when stale.
        if self._token is None or time.time() >= self._token_expires_at:
            with self._token_lock:
                if self._token is None or time.time() >= self._token_expires_at:
                    self._refresh_token()
        return self._token

    def build_client(self, timeout_s: float | None = None) -> Lattice:  # Build an SDK client sharing this transport and token.
        return Lattice(
            base_url=self.config.base_url,
            token=self.token_value,
            timeout=self.config.timeout_s if timeout_s is None else float(timeout_s),
            max_retries=self.config.max_retries,
            httpx_client=self._http_client(),
        )

    def connect(self) -> None:  # Acquire a token when needed and build the SDK client.
        if self.connected:
            return
        self.connect_attempts += 1
        try:
            if self._token is None or time.time() >= self._token_expires_at:
                self._refresh_token()
            if self._client is None:
                self._client = self.build_client()
        except LatticeConnectionError as exc:
            self.last_error = exc
            self._next_attempt = time.monotonic() + self.config.reconnect_secs
            raise
        except Exception as exc:
            error = LatticeConnectionError("connect", self.config.base_url, f"{type(exc).__name__}: {exc}")
            self.last_error = error
            self._next_attempt = time.monotonic() + self.config.reconnect_secs
            raise error from exc
        self.connected = True
        self.last_error = None

    def close(self) -> None:  # Drop the SDK client and release the transport.
        self._client = None
        self.connected = False
        if self._owns_http and self._http is not None and not self._http.is_closed:
            self._http.close()

    def ensure_connected(self, now: float | None = None) -> bool:  # Reconnect hook with retry backoff.
        if self.connected:
            return True
        current = time.monotonic() if now is None else now
        if current < self._next_attempt:
            return False
        try:
            self.connect()
        except LatticeConnectionError:
            return False
        return True

    def _http_client(self) -> httpx.Client:  # Owned httpx client carrying the TLS settings.
        if self._http is None or self._http.is_closed:
            self._http = httpx.Client(
                verify=self.config.verify(),
                timeout=self.config.timeout_s,
                follow_redirects=True,
            )
            self._owns_http = True
        return self._http

    def _refresh_token(self) -> None:  # Acquire a bearer token from the token endpoint.
        endpoint = self.config.token_endpoint
        data = {
            "client_id": self.config.client_id,
            "client_secret": self.config.client_secret,
            "grant_type": "client_credentials",
        }
        try:
            response = self._http_client().post(
                endpoint,
                data=data,
                headers={"content-type": "application/x-www-form-urlencoded"},
            )
        except httpx.HTTPError as exc:
            raise LatticeConnectionError("token-acquisition", endpoint, f"{type(exc).__name__}: {exc}") from exc
        if not 200 <= response.status_code < 300:
            raise LatticeConnectionError(
                "token-acquisition", endpoint, f"HTTP {response.status_code}: {response.text.strip()}"
            )
        try:
            token_response = GetTokenResponse.model_validate(response.json())
        except ValueError as exc:
            raise LatticeConnectionError("token-acquisition", endpoint, f"invalid token response: {exc}") from exc
        expires_in = token_response.expires_in if token_response.expires_in is not None else DEFAULT_TOKEN_EXPIRES_IN_S
        self._token = token_response.access_token
        self._token_expires_at = time.time() + float(expires_in) - TOKEN_EXPIRY_BUFFER_S
        self._token_acquisitions += 1


class LatticeTransportError(RuntimeError):
    """A bounded Lattice exchange failed after retries; causes are logged."""


class LatticePlugin(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Initialize Lattice agent interface plugin.
        super().__init__(cfg, bus_config)
        # Read config explicitly: cfg["client_id"] is the Lattice API credential id,
        # while self.client_id is the bus client id set by RuntimeBase.
        self.topic_ns = cfg["topic_ns"]
        self.state_intervals = cfg["state_intervals"]
        self.loop_interval_s = float(cfg["loop_interval_s"])
        self.asset_id = str(cfg["asset_id"])
        self.state_publish_interval_s = float(cfg.get("state_publish_interval_s", 1.0))
        self.entity_expiry_s = float(cfg.get("entity_expiry_s", ENTITY_EXPIRY_S))
        self.interface = dict(cfg["interface"])
        self.vehicle_state_keys = list(cfg["vehicle_state_keys"])
        self.lattice_config = LatticeConfig(cfg)
        self.connection = LatticeConnection(self.lattice_config)

        base = build_topic_base(self.client_id, self.topic_ns)
        self.state_scheduler = StateScheduler(
            self.client,
            self.client_id,
            build_state_scheduler_topics(base, self.state_intervals),
        )
        self.event_topics = build_event_topics(
            base,
            [
                LATTICE.Event.Link.Connected,
                LATTICE.Event.Link.Disconnected,
                LATTICE.Event.Auth.TokenAcquired,
                LATTICE.Event.Task.Received,
                LATTICE.Event.Task.DuplicateRejected,
                LATTICE.Event.Task.CancelRequested,
                LATTICE.Event.Task.CompleteRequested,
                LATTICE.Event.Task.StatusUpdated,
                LATTICE.Event.System.Error,
            ],
        )
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)

        interface_base = build_topic_base(self.interface["id"], self.interface["topic_ns"])
        self.vehicle_state_topics = build_state_topics(interface_base, self.vehicle_state_keys)
        self.init_bus(float(cfg["bus_poll_interval_s"]), state_topics=self.vehicle_state_topics)

        self.decoder = TaskDecoder(
            resolve_entity=self._get_target_entity,
            resolve_home_hae=lambda: self.entity_home_hae_m,
        )
        self.status_versions = TaskStatusVersions()
        self.pending_tasks: Deque[DecodedTask] = collections.deque()
        self.seen_task_ids: set[str] = set()
        self.last_task: DecodedTask | None = None
        self.agent_requests: "queue.Queue[Any]" = queue.Queue()
        self.listener_stop = threading.Event()
        self.listener_thread: threading.Thread | None = None
        self.listener_error: LatticeConnectionError | None = None
        self.last_lattice_ok_at = time.monotonic()
        self.entity_publish_count = 0
        self.last_entity_publish_ms = 0
        self.last_entity_fields: Dict[str, Any] = {}
        self.entity_home_hae_m: float | None = None

        self.connect_enabled = True
        self._published_connected = False
        self._published_token_acquisitions = 0
        self._reported_error: LatticeConnectionError | None = None
        self._sync_connection_state()
        self._sync_task_state()
        self._sync_entity_state()

    # -- connection state ----------------------------------------------------

    def _status_data(self) -> Dict[str, Any]:  # Build connection status payload.
        return {
            "Connected": self.connection.connected,
            "Endpoint": self.lattice_config.base_url,
            "ConnectAttempts": self.connection.connect_attempts,
            "TokenValid": self.connection.token_valid(),
        }

    def _record_error(self, error: LatticeConnectionError) -> None:  # Surface an error with operation and endpoint context.
        self._reported_error = error
        self.state_scheduler.update(LATTICE.State.System.LastError, error.detail)
        self.state_scheduler.update(LATTICE.State.System.LastErrorOperation, error.operation)
        self.state_scheduler.update(LATTICE.State.System.LastErrorEndpoint, error.endpoint)
        self._publish_event(
            LATTICE.Event.System.Error,
            {"Operation": error.operation, "Endpoint": error.endpoint, "Detail": error.detail},
        )
        print(
            f"[PLUGIN] {self.client_id} error operation={error.operation} endpoint={error.endpoint} "
            f"detail={error.detail}",
            flush=True,
        )

    def _sync_connection_state(self) -> None:  # Publish connection state and transition events.
        endpoint = self.lattice_config.base_url
        connected = self.connection.connected
        self.state_scheduler.update(LATTICE.State.Link.Connected, connected)
        if connected != self._published_connected:
            self._published_connected = connected
            if connected:
                self._publish_event(LATTICE.Event.Link.Connected, {"Endpoint": endpoint})
            else:
                self._publish_event(LATTICE.Event.Link.Disconnected, {"Endpoint": endpoint})
            print(f"[PLUGIN] {self.client_id} connected={connected} endpoint={endpoint}", flush=True)
        error = self.connection.last_error or self.listener_error
        if error is not None and error is not self._reported_error:
            self._record_error(error)
        elif error is None:
            self._reported_error = None
        if self.connection.token_acquisitions != self._published_token_acquisitions:
            self._published_token_acquisitions = self.connection.token_acquisitions
            self._publish_event(
                LATTICE.Event.Auth.TokenAcquired,
                {"ExpiresAtMs": self.connection.token_expires_at_ms()},
            )
        self.state_scheduler.update(LATTICE.State.Link.Endpoint, endpoint)
        self.state_scheduler.update(LATTICE.State.Link.ConnectAttempts, self.connection.connect_attempts)
        self.state_scheduler.update(LATTICE.State.Auth.TokenValid, self.connection.token_valid())
        self.state_scheduler.update(
            LATTICE.State.Link.LastOkAgeS, time.monotonic() - self.last_lattice_ok_at
        )

    def _sync_task_state(self) -> None:  # Publish decoded task queue state.
        self.state_scheduler.update(LATTICE.State.Task.PendingCount, len(self.pending_tasks))
        if self.last_task is not None:
            self.state_scheduler.update(LATTICE.State.Task.LastTask, self.last_task.payload())

    def _sync_entity_state(self) -> None:  # Publish last entity encode state.
        if not self.last_entity_fields:
            return
        for key, value in self.last_entity_fields.items():
            self.state_scheduler.update(key, value)

    # -- Lattice exchange policy ---------------------------------------------

    def _lattice_call(
        self,
        operation: str,
        call: Callable[[], Any],
        *,
        task_id: str | None = None,
        expected_version: int | None = None,
        poll_timeouts_ok: bool = False,
    ) -> Any:
        """Run one Lattice SDK call with bounded retries and visible failures.

        A long-poll timeout with ``poll_timeouts_ok`` means "no manager
        request"; it is not an error.  Every exhausted call raises
        ``LatticeTransportError`` with endpoint and operation context already
        logged as ``[LATTICE_ERROR]``.
        """
        attempts = self.lattice_config.lattice_attempts
        for attempt in range(1, attempts + 1):
            try:
                result = call()
            except LATTICE_POLL_TIMEOUTS as exc:
                if poll_timeouts_ok and attempt == 1:
                    return None
                self._log_lattice_error(
                    operation, exc, attempt, task_id, expected_version
                )
            except LATTICE_ERRORS as exc:
                self._log_lattice_error(
                    operation, exc, attempt, task_id, expected_version
                )
            except LatticeConnectionError as exc:
                self._log_lattice_error(
                    operation, exc, attempt, task_id, expected_version
                )
            else:
                self.last_lattice_ok_at = time.monotonic()
                return result
            if attempt < attempts:
                time.sleep(self.lattice_config.lattice_retry_delay_s)
        raise LatticeTransportError(
            f"operation={operation} endpoint={self.lattice_config.base_url} attempts={attempts}"
        )

    def _log_lattice_error(
        self,
        operation: str,
        exc: BaseException,
        attempt: int,
        task_id: str | None,
        expected_version: int | None,
    ) -> None:
        parts = [
            f"endpoint={self.lattice_config.base_url}",
            f"operation={operation}",
            f"attempt={attempt}/{self.lattice_config.lattice_attempts}",
        ]
        if task_id is not None:
            parts.append(f"task_id={task_id}")
        if expected_version is not None:
            parts.append(f"expected_version={expected_version}")
        parts.append(f"cause={type(exc).__name__}: {exc}")
        print("[LATTICE_ERROR] " + " ".join(parts), flush=True)

    def _surface_transport_error(self, operation: str, exc: LatticeTransportError) -> None:
        error = LatticeConnectionError(operation, self.lattice_config.base_url, str(exc))
        self._record_error(error)

    # -- agent listen --------------------------------------------------------

    def _start_listener(self) -> None:  # Start the agent listen thread.
        self.listener_stop.clear()
        self.listener_thread = threading.Thread(
            target=self._listener_main, name="lattice-agent-listen", daemon=True
        )
        self.listener_thread.start()

    def _stop_listener(self) -> None:  # Stop the agent listen thread.
        self.listener_stop.set()
        if self.listener_thread is not None:
            self.listener_thread.join(timeout=1.0)

    def _listener_main(self) -> None:  # Long-poll the agent listen and buffer requests.
        selector = EntityIdsSelector(entity_ids=[self.asset_id])
        while not self.listener_stop.is_set():
            try:
                client = self.connection.build_client(timeout_s=self.lattice_config.listen_timeout_s)
                request = self._lattice_call(
                    "listen_as_agent",
                    lambda: client.tasks.listen_as_agent(agent_selector=selector),
                    poll_timeouts_ok=True,
                )
            except LatticeTransportError as exc:
                self.listener_error = LatticeConnectionError(
                    "listen_as_agent", self.lattice_config.base_url, str(exc)
                )
                time.sleep(max(1.0, self.lattice_config.lattice_retry_delay_s))
                continue
            except LatticeConnectionError as exc:
                self.listener_error = exc
                time.sleep(max(1.0, self.lattice_config.lattice_retry_delay_s))
                continue
            self.listener_error = None
            if request is not None:
                self.agent_requests.put(request)

    def _drain_agent_requests(self) -> None:  # Decode buffered agent requests on the main loop.
        while True:
            try:
                request = self.agent_requests.get_nowait()
            except queue.Empty:
                return
            try:
                execute_request = _field(request, "execute_request")
                cancel_request = _field(request, "cancel_request")
                complete_request = _field(request, "complete_request")
                if execute_request is not None:
                    self._register_execute(execute_request)
                elif cancel_request is not None:
                    task_id = str(_field(cancel_request, "task_id") or "")
                    print(f"[TASK_CANCEL_REQUEST] task_id={task_id}", flush=True)
                    self._publish_event(LATTICE.Event.Task.CancelRequested, {"TaskId": task_id})
                elif complete_request is not None:
                    task_id = str(_field(complete_request, "task_id") or "")
                    print(f"[TASK_COMPLETE_REQUEST] task_id={task_id}", flush=True)
                    self._publish_event(LATTICE.Event.Task.CompleteRequested, {"TaskId": task_id})
            except Exception as exc:
                print(
                    f"[TASK_HANDLING_FAILED] endpoint={self.lattice_config.base_url} "
                    f"error={type(exc).__name__}: {exc}\n{traceback.format_exc().strip()}",
                    flush=True,
                )

    def _register_execute(self, execute_request: Any) -> None:  # Decode one execute request into the pending queue.
        task = _field(execute_request, "task")
        if task is None:
            raise ValueError("execute request has no task")
        version = _field(task, "version")
        task_id = "" if version is None else str(_field(version, "task_id") or "")
        status_version = 0 if version is None else int(_field(version, "status_version") or 0)
        if self.status_versions.seed(task_id, status_version):
            print(f"[TASK_STATUS_SEED] task_id={task_id} version={status_version}", flush=True)
        if task_id in self.seen_task_ids:
            print(f"[TASK_DUPLICATE] task_id={task_id} dropped", flush=True)
            self._publish_event(LATTICE.Event.Task.DuplicateRejected, {"TaskId": task_id})
            return
        self.seen_task_ids.add(task_id)
        try:
            decoded = self.decoder.decode(task)
        except ValueError as exc:
            print(
                f"[TASK_REJECTED] task_id={task_id} cause={exc} "
                f"endpoint={self.lattice_config.base_url}",
                flush=True,
            )
            self._update_status(
                task_id,
                LATTICE.Enums.LatticeTaskStatus.DoneNotOk,
                error_code=LATTICE.Enums.LatticeTaskErrorCode.Rejected,
                error_message=str(exc),
            )
            return
        self.pending_tasks.append(decoded)
        self.last_task = decoded
        self._sync_task_state()
        self._publish_event(LATTICE.Event.Task.Received, decoded.payload())
        print(
            f"[TASK_RECEIVED] task_id={task_id} spec={decoded.specification_type} "
            f"pending={len(self.pending_tasks)}",
            flush=True,
        )

    # -- task status ---------------------------------------------------------

    def _get_target_entity(self, entity_id: str) -> Any:  # Fetch a Lattice entity for objective resolution.
        try:
            return self._lattice_call(
                "get_entity", lambda: self.connection.client.entities.get_entity(entity_id)
            )
        except LatticeTransportError:
            return None

    def _resync_status_version(self, task_id: str, expected_version: int | None) -> int | None:
        """Re-fetch a task and adopt its endpoint-authoritative status version."""
        try:
            task = self._lattice_call(
                "get_task",
                lambda: self.connection.client.tasks.get_task(task_id=task_id),
                task_id=task_id,
                expected_version=expected_version,
            )
        except LatticeTransportError:
            return None
        actual_version = int(_field(_field(task, "version"), "status_version") or 0)
        self.status_versions.adopt(task_id, actual_version)
        print(
            f"[TASK_STATUS_RESYNC] task_id={task_id} expected_version={expected_version} "
            f"actual_version={actual_version}",
            flush=True,
        )
        return actual_version

    def _update_status(
        self,
        task_id: str,
        status_key: str,
        *,
        progress: float | None = None,
        error_code: str | None = None,
        error_message: str | None = None,
    ) -> tuple[bool, int | None]:
        """Update one task status; failures are logged, never silently dropped.

        Version conflicts re-fetch the task and retry once from the
        authoritative version.  Returns ``(accepted, version)``.
        """
        status = TASK_STATUS_BY_KEY.get(status_key)
        if status is None:
            raise ValueError(f"unknown task status key {status_key!r}")
        error = None
        if error_code is not None:
            sdk_error_code = TASK_ERROR_CODE_BY_KEY.get(error_code)
            if sdk_error_code is None:
                raise ValueError(f"unknown task error code key {error_code!r}")
            error = TaskError(code=sdk_error_code, message=error_message or "")
        expected_version = self.status_versions.next_version(task_id)
        if expected_version is None:
            expected_version = self._resync_status_version(task_id, None)
            if expected_version is not None:
                expected_version = expected_version + 1
        if expected_version is None:
            print(
                f"[TASK_STATUS_FAILED] task_id={task_id} status={status} "
                f"cause=status version unavailable endpoint={self.lattice_config.base_url}",
                flush=True,
            )
            return False, None
        progress_document = None
        if progress is not None:
            progress_document = GoogleProtobufAny(
                type=MPFC_PROGRESS_TYPE_URL, progress=max(0.0, min(1.0, float(progress)))
            )
        new_status = TaskStatus(status=status, task_error=error, progress=progress_document)

        def _call(status_version: int) -> Any:
            return self.connection.client.tasks.update_task_status(
                task_id=task_id,
                status_version=status_version,
                new_status=new_status,
                author=Principal(system=System(entity_id=self.asset_id)),
            )

        try:
            updated = self._lattice_call(
                "update_task_status",
                lambda: _call(expected_version),
                task_id=task_id,
                expected_version=expected_version,
            )
        except LatticeTransportError as exc:
            actual_version = self._resync_status_version(task_id, expected_version)
            if actual_version is None or actual_version == expected_version:
                print(
                    f"[TASK_STATUS_FAILED] task_id={task_id} status={status} "
                    f"expected_version={expected_version} "
                    f"actual_version={actual_version if actual_version is not None else 'unknown'} "
                    f"endpoint={self.lattice_config.base_url}",
                    flush=True,
                )
                self._surface_transport_error("update_task_status", exc)
                return False, None
            retry_version = actual_version + 1
            try:
                updated = self._lattice_call(
                    "update_task_status",
                    lambda: _call(retry_version),
                    task_id=task_id,
                    expected_version=retry_version,
                )
            except LatticeTransportError as retry_exc:
                print(
                    f"[TASK_STATUS_FAILED] task_id={task_id} status={status} "
                    f"expected_version={retry_version} actual_version={actual_version} "
                    f"endpoint={self.lattice_config.base_url}",
                    flush=True,
                )
                self._surface_transport_error("update_task_status", retry_exc)
                return False, None
        actual_version = int(_field(_field(updated, "version"), "status_version") or 0)
        if actual_version != expected_version:
            print(
                f"[TASK_STATUS_RESYNC] task_id={task_id} expected_version={expected_version} "
                f"actual_version={actual_version}",
                flush=True,
            )
        self.status_versions.adopt(task_id, actual_version)
        print(
            f"[TASK_STATUS] task_id={task_id} status={status} version={actual_version} "
            f"progress={progress}",
            flush=True,
        )
        self._publish_event(
            LATTICE.Event.Task.StatusUpdated,
            {
                "TaskId": task_id,
                "Status": status,
                "ErrorCode": None if error is None else error.code,
                "Version": actual_version,
            },
        )
        return True, actual_version

    # -- entity publishing ---------------------------------------------------

    def _publish_entity_state(self) -> bool:  # Encode and publish the vehicle state as a Lattice entity.
        position = self.state.get(UAV.State.Navigation.Position)
        if position is None:
            return False
        attitude = self.state.get(UAV.State.Attitude.AttitudeRad)
        relative_alt_m = float(position["RelAltM"])
        absolute_alt_m = float(position["AbsAltM"])
        if self.entity_home_hae_m is None:
            self.entity_home_hae_m = home_hae_from_telemetry(absolute_alt_m, relative_alt_m)
            print(
                f"[ENTITY_HAE] source=telemetry hae_m={self.entity_home_hae_m:.3f} "
                f"abs_m={absolute_alt_m:.3f} rel_m={relative_alt_m:.3f}",
                flush=True,
            )
        hae_alt_m = relative_to_hae(relative_alt_m, self.entity_home_hae_m)
        attitude_enu = None
        if attitude is not None:
            w, x, y, z = attitude_enu_quaternion(
                attitude["Roll"], attitude["Pitch"], attitude["Yaw"]
            )
            attitude_enu = Quaternion(w=w, x=x, y=y, z=z)
        entity = Entity(
            entity_id=self.asset_id,
            is_live=True,
            expiry_time=datetime.now(timezone.utc) + timedelta(seconds=self.entity_expiry_s),
            aliases=Aliases(name=f"MPFC {self.client_id}"),
            data_classification=Classification(
                default=ClassificationInformation(level="CLASSIFICATION_LEVELS_UNCLASSIFIED")
            ),
            health=Health(
                connection_status="CONNECTION_STATUS_ONLINE",
                health_status="HEALTH_STATUS_HEALTHY",
            ),
            location=LatticeLocation(
                position=Position(
                    latitude_degrees=float(position["LatDeg"]),
                    longitude_degrees=float(position["LonDeg"]),
                    altitude_agl_meters=relative_alt_m,
                    altitude_hae_meters=hae_alt_m,
                ),
                attitude_enu=attitude_enu,
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
            task_catalog=TaskCatalog(
                task_definitions=[
                    TaskDefinition(task_specification_url=url) for url in SUPPORTED_SPECIFICATION_URLS
                ]
            ),
        )
        try:
            self._lattice_call(
                "publish_entity",
                lambda: self.connection.client.entities.publish_entity(**entity.model_dump()),
            )
        except LatticeTransportError as exc:
            self._surface_transport_error("publish_entity", exc)
            return False
        self.entity_publish_count += 1
        self.last_entity_publish_ms = int(time.time() * 1000)
        self.last_entity_fields = {
            LATTICE.State.Entity.LatitudeDeg: float(position["LatDeg"]),
            LATTICE.State.Entity.LongitudeDeg: float(position["LonDeg"]),
            LATTICE.State.Entity.AltitudeAglM: relative_alt_m,
            LATTICE.State.Entity.AltitudeHaeM: hae_alt_m,
            LATTICE.State.Entity.PublishCount: self.entity_publish_count,
            LATTICE.State.Entity.LastPublishAtMs: self.last_entity_publish_ms,
        }
        if attitude_enu is not None:
            self.last_entity_fields.update(
                {
                    LATTICE.State.Entity.AttitudeEnuW: float(attitude_enu.w),
                    LATTICE.State.Entity.AttitudeEnuX: float(attitude_enu.x),
                    LATTICE.State.Entity.AttitudeEnuY: float(attitude_enu.y),
                    LATTICE.State.Entity.AttitudeEnuZ: float(attitude_enu.z),
                }
            )
        self._sync_entity_state()
        return True

    def _maybe_publish_entity_state(self) -> None:  # Publish the entity on its configured interval.
        if self.state.get(UAV.State.Navigation.Position) is None:
            return
        if not self.connection.connected:
            return
        now = time.time()
        if now - (self.last_entity_publish_ms / 1000.0) < self.state_publish_interval_s:
            return
        self._publish_entity_state()

    # -- bus request handling ------------------------------------------------

    def _handle_fetch(self, request_id: str, action: str) -> None:  # Fetch a decoded task for a core.
        decoded = self.pending_tasks.popleft() if self.pending_tasks else None
        data: Dict[str, Any] = {
            "PendingCount": len(self.pending_tasks),
            "TaskId": "" if decoded is None else decoded.task_id,
        }
        if decoded is not None:
            data.update(decoded.payload())
        self._sync_task_state()
        self.enqueue_response(request_id, action, True, data)

    def _handle_report(self, request_id: str, action: str, params: Dict[str, Any]) -> None:  # Report one task status.
        task_id = str(params["TaskId"])
        status_key = str(params["Status"])
        progress = params.get("Progress")
        error_code = params.get("ErrorCode")
        error_message = params.get("ErrorMessage")
        try:
            accepted, version = self._update_status(
                task_id,
                status_key,
                progress=None if progress is None else float(progress),
                error_code=None if error_code is None else str(error_code),
                error_message=None if error_message is None else str(error_message),
            )
        except ValueError as exc:
            self.enqueue_response(
                request_id,
                action,
                False,
                {
                    "error": str(exc),
                    "Operation": "update_task_status",
                    "Endpoint": self.lattice_config.base_url,
                },
            )
            return
        if not accepted:
            self.enqueue_response(
                request_id,
                action,
                False,
                {
                    "error": f"status update failed task_id={task_id} status={status_key}",
                    "Operation": "update_task_status",
                    "Endpoint": self.lattice_config.base_url,
                },
            )
            return
        self.enqueue_response(
            request_id,
            action,
            True,
            {"TaskId": task_id, "Status": TASK_STATUS_BY_KEY[status_key], "Version": version},
        )

    def _handle_request(self, request: Dict[str, Any]) -> None:  # Handle a bus REQUEST action.
        request_id = str(request["request_id"])
        action = request["action"]
        params = request.get("params") or {}
        try:
            if action == LATTICE.Action.Connection.Connect:
                self.connect_enabled = True
                self.connection.connect()
                self._sync_connection_state()
                self.enqueue_response(request_id, action, True, self._status_data())
                return
            if action == LATTICE.Action.Connection.Disconnect:
                self.connect_enabled = False
                self.connection.close()
                self._sync_connection_state()
                self.enqueue_response(request_id, action, True, self._status_data())
                return
            if action == LATTICE.Action.Connection.Status:
                self.enqueue_response(request_id, action, True, self._status_data())
                return
            if action == LATTICE.Action.Task.Fetch:
                self._handle_fetch(request_id, action)
                return
            if action == LATTICE.Action.Task.Report:
                self._handle_report(request_id, action, params)
                return
            self.enqueue_response(request_id, action, False, {"error": f"unknown action {action}"})
        except LatticeConnectionError as exc:
            self._record_error(exc)
            self._sync_connection_state()
            self.enqueue_response(
                request_id,
                action,
                False,
                {"error": str(exc), "Operation": exc.operation, "Endpoint": exc.endpoint},
            )
        except LatticeTransportError as exc:
            self._surface_transport_error(action, exc)
            self.enqueue_response(
                request_id,
                action,
                False,
                {
                    "error": str(exc),
                    "Operation": action,
                    "Endpoint": self.lattice_config.base_url,
                },
            )

    # -- runtime -------------------------------------------------------------

    def run(self) -> None:  # Run Lattice agent interface loop.
        self._start_listener()
        self.send_online()
        self._sync_connection_state()
        self._sync_task_state()
        self._sync_entity_state()
        try:
            while True:
                if self.connect_enabled:
                    self.connection.ensure_connected()
                self._drain_agent_requests()
                self._maybe_publish_entity_state()
                self._sync_connection_state()
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
            self._stop_listener()
            self.stop()

    def stop(self) -> None:  # Close the Lattice connection and stop plugin.
        self.connection.close()
        super().stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    LatticePlugin(cfg, bus_config).run()
