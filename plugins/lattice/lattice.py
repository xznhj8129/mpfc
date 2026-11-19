#!/usr/bin/env python3
"""
Lattice agent interface plugin: connection, native task delivery, entity publishing.

Usage:
    from plugins.lattice.lattice import run_plugin
    run_plugin(cfg, bus_config)

The plugin wraps the official anduril-lattice-sdk client and is the agent
surface of MPFC:

* it owns the API connection and token lifecycle declared in
  protocols/lattice.yaml;
* it listens as the configured asset and delivers intact native task documents;
* it recovers unstarted work from the persistent Lattice task authority and
  blocks ambiguous execution after restart;
* it performs versioned task status updates seeded from the delivered task,
  adopting the version returned by the endpoint and resyncing on mismatch;
* it encodes the published vehicle state into a Lattice entity and refreshes
  it on the configured interval.

No mission policy and no vehicle command logic live here; cores drive tasks
and missions over the bus.
"""

import math
import queue
import re
import threading
import time
import traceback
from datetime import datetime, timedelta, timezone
from enum import Enum
from typing import Any, Callable, Dict

import httpx
import numpy as np
from anduril import (
    Aliases,
    Entity,
    ErrorEllipse,
    EntityIdsSelector,
    GoogleProtobufAny,
    Lattice,
    Location as LatticeLocation,
    LocationUncertainty,
    Media,
    MediaItem,
    MilView,
    Ontology,
    Position,
    Principal,
    Provenance,
    RequestTimeoutError,
    RtspSettings,
    System,
    Task,
    TaskError,
    TaskStatus,
)
from anduril.core import ApiError
from anduril.oauth.types.get_token_response import GetTokenResponse
from anduril.types.entity_manager_t_mat3 import EntityManagerTMat3
from froggeolib import GimbalCamera, GPSposition

from lib.common import (
    build_event_topics,
    build_request_topic,
    build_response_topic,
    build_state_scheduler_topics,
    build_state_topics,
    build_topic_base,
)
from lib.lattice_fields import _field
from plugins.lattice.tasks import TaskDeliveries
from plugins.lattice.picture import VehiclePicture, VehiclePublication
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from protocols.namespace_loader import load_protocol_namespace

LATTICE = load_protocol_namespace("lattice")
UAV = load_protocol_namespace("uav")


class PlatformActivity(str, Enum):
    returning = "RETURN TO BASE (RTB)"
    landing = "LANDING"
    airborne = "AIRBORNE"
    armed = "ARMED"
    grounded = "ON GROUND"


ACTIVITY_MODES = {
    "BOXNAVRTH": PlatformActivity.returning,
    "RETURN_TO_LAUNCH": PlatformActivity.returning,
    "LAND": PlatformActivity.landing,
}


def platform_activity(state: Dict[str, Any]) -> PlatformActivity | None:
    """Describe observed autopilot state; task intent alone is not vehicle activity."""
    armed = state.get(UAV.State.Flight.IsArmed)
    in_air = state.get(UAV.State.Flight.IsInAir)
    if armed is False and in_air is False:
        return PlatformActivity.grounded
    modes = state.get(UAV.State.Flight.ActiveModeNames) or []
    for mode in modes:
        if mode in ACTIVITY_MODES:
            return ACTIVITY_MODES[mode]
    if in_air is True:
        return PlatformActivity.airborne
    if armed is True:
        return PlatformActivity.armed
    return None

MPFC_PROGRESS_TYPE_URL = "type.googleapis.com/mpfc.TaskProgress"

ENTITY_EXPIRY_S = 15.0
# Published geospatial contacts use the TAK marker lifetime, not detector-box expiry.
TRACK_EXPIRY_S = 86400.0
DEFAULT_TOKEN_EXPIRES_IN_S = 3600
TOKEN_EXPIRY_BUFFER_S = 120.0
# Lattice allows 4-36 printable ASCII; MediaMTX path names and the REST path allow only these.
INGRESS_ID_RE = re.compile(r"^[A-Za-z0-9_.~:-]{4,36}$")
HTTP_CONFLICT = 409
HTTP_UNAUTHORIZED = 401

LATTICE_ERRORS = (ApiError, httpx.HTTPError)
LATTICE_POLL_TIMEOUTS = (RequestTimeoutError, httpx.ReadTimeout, httpx.ReadError)

# Bus status/error keys -> official Lattice API status and error codes.
TASK_STATUS_BY_KEY = {
    LATTICE.Enums.LatticeTaskStatus.MachineReceipt: "STATUS_MACHINE_RECEIPT",
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

class MediaSource:
    """One vehicle video source registered as a Lattice Video API ingress stream."""

    def __init__(self, asset_id: str, cfg: Dict[str, Any]) -> None:
        self.source = str(cfg["source"])
        self.url = str(cfg["url"])
        self.ingress_id = f"{asset_id}.{self.source}"
        if not INGRESS_ID_RE.match(self.ingress_id):
            raise ValueError(f"invalid ingress id {self.ingress_id!r}: 4-36 chars of A-Z a-z 0-9 _ . ~ : -")
        self.registered = False

    def media_item(self) -> MediaItem:  # Entity reference to the registered ingress.
        return MediaItem(
            item_identifier=self.ingress_id,
            type="MEDIA_TYPE_VIDEO",
        )


class DetectionCamera:
    """Geolocate detector boxes using the calibrated stabilized camera and level ground."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.image_w = float(cfg["image_w"])
        self.image_h = float(cfg["image_h"])
        self.projection = GimbalCamera(**cfg)

    def image_center(self, detection: Dict[str, Any]) -> tuple[float, float]:
        return ((float(detection["x1"]) + float(detection["x2"])) / 2 / self.image_w,
                (float(detection["y1"]) + float(detection["y2"])) / 2 / self.image_h)

    def ground_point(self, position: Dict[str, Any], yaw_rad: float, detection: Dict[str, Any]) -> tuple[float, float] | None:
        norm_x, norm_y = self.image_center(detection)
        point = self.projection.ground_point(
            GPSposition(float(position["LatDeg"]), float(position["LonDeg"])),
            float(position["RelAltM"]),
            math.degrees(yaw_rad) % 360.0,
            norm_x,
            norm_y,
        )
        return (point.lat, point.lon) if point is not None else None

    def uncertainty(self, position: Dict[str, Any], yaw_rad: float, detection: Dict[str, Any]) -> LocationUncertainty | None:
        norm_x, norm_y = self.image_center(detection)
        covariance = self.projection.ground_covariance(float(position['RelAltM']), math.degrees(yaw_rad), norm_x, norm_y)
        if covariance is None:
            return None
        east, cross, north = covariance
        values, axes = np.linalg.eigh([[east, cross], [cross, north]])
        scale = -2 * math.log(1 - .9)
        return LocationUncertainty(
            position_enu_cov=EntityManagerTMat3(mxx=east, mxy=cross, myy=north),
            position_error_ellipse=ErrorEllipse(probability=.9,
                semi_major_axis_m=math.sqrt(max(0, float(values[1])) * scale),
                semi_minor_axis_m=math.sqrt(max(0, float(values[0])) * scale),
                orientation_d=math.degrees(math.atan2(float(axes[0, 1]), float(axes[1, 1]))) % 180))


class LatticeConnectionError(RuntimeError):
    def __init__(self, operation: str, endpoint: str, detail: str) -> None:
        self.operation = operation
        self.endpoint = endpoint
        self.detail = detail
        super().__init__(f"lattice {operation} failed endpoint={endpoint} detail={detail}")


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

    def invalidate_token(self) -> bool:  # Drop a rejected token so the next request re-authenticates; False for a static token.
        if self.config.token:
            return False
        with self._token_lock:
            self._token = None
            self._token_expires_at = 0.0
        return True

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
        self.callsign = self.asset_id if cfg["callsign"] is None else str(cfg["callsign"])
        self.state_publish_interval_s = float(cfg.get("state_publish_interval_s", 1.0))
        self.entity_expiry_s = float(cfg.get("entity_expiry_s", ENTITY_EXPIRY_S))
        self.interface = dict(cfg["interface"])
        self.vehicle_state_keys = list(cfg["vehicle_state_keys"])
        self.accept_tasks = bool(cfg["accept_tasks"])
        self.media_sources = [MediaSource(self.asset_id, item) for item in cfg["media"]]
        detections_cfg = cfg["detections"]
        self.detection_topic: str | None = None
        if detections_cfg is not None:
            source = detections_cfg["source"]
            self.detection_topic = f"{build_topic_base(source['id'], source['topic_ns'])}/STATE/detections"
            self.camera = DetectionCamera(detections_cfg["camera"])
            self.track_expiry_s = float(detections_cfg.get("track_expiry_s", TRACK_EXPIRY_S))
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
        if self.detection_topic is not None:
            self.client.subscribe(self.detection_topic)

        self.status_versions = TaskStatusVersions()
        self.deliveries = TaskDeliveries(float(cfg.get("task_fetch_lease_s", 30.0)))
        self.recovery_ready = False
        self.next_task_recovery_at = 0.0
        self.next_receipt_retry_at = 0.0
        self.agent_requests: "queue.Queue[Any]" = queue.Queue()
        self.listener_stop = threading.Event()
        self.listener_thread: threading.Thread | None = None
        self.listener_error: LatticeConnectionError | None = None
        self.last_lattice_ok_at = time.monotonic()
        self.entity_publish_count = 0
        self.last_entity_publish_ms = 0
        self.last_entity_fields: Dict[str, Any] = {}
        home_hae = cfg.get("home_altitude_hae_m")
        self.entity_home_hae_m: float | None = None if home_hae is None else float(home_hae)
        self.picture = VehiclePicture(VehiclePublication(
            asset_id=self.asset_id, callsign=self.callsign, integration_name=f"mpfc.{self.client_id}",
            expiry_s=self.entity_expiry_s, telemetry_freshness_s=cfg["telemetry_freshness_s"],
            home_altitude_hae_m=self.entity_home_hae_m,
            relationships=cfg["relationships"], group_details=cfg["group_details"],
        ))

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

    def _sync_task_state(self) -> None:
        self.state_scheduler.update(LATTICE.State.Task.PendingCount, self.deliveries.pending_count)
        if self.deliveries.last_task is not None:
            self.state_scheduler.update(LATTICE.State.Task.LastTask, self.deliveries.last_task.payload())

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
                # The server forgets issued tokens on restart; a 401 means re-authenticate, not retry the same token.
                if isinstance(exc, ApiError) and exc.status_code == HTTP_UNAUTHORIZED and self.connection.invalidate_token():
                    print(f"[PLUGIN] {self.client_id} token rejected operation={operation}; re-authenticating", flush=True)
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

    def _register_execute(self, execute_request: Any) -> None:
        task = _field(execute_request, "task")
        if task is None:
            raise ValueError("execute request has no task")
        task = Task.model_validate(task)
        admitted = self.deliveries.admit(task)
        task_id = task.version.task_id
        if not admitted:
            print(f"[TASK_DUPLICATE] task_id={task_id} retained; no duplicate bus admission", flush=True)
            self._publish_event(LATTICE.Event.Task.DuplicateRejected, {"TaskId": task_id})
            return
        self.status_versions.seed(task_id, task.version.status_version)
        self._receive_task(task_id)

    def _receive_task(self, task_id: str) -> None:
        accepted, _ = self._update_status(task_id, LATTICE.Enums.LatticeTaskStatus.MachineReceipt)
        if accepted:
            self.deliveries.received(task_id)
            self._publish_event(LATTICE.Event.Task.Received, self.deliveries.pending[task_id].payload())
            print(f"[TASK_RECEIVED] task_id={task_id} pending={self.deliveries.pending_count}", flush=True)
        self._sync_task_state()

    def _recover_tasks(self) -> None:
        """Reconcile acceptance from the persistent native task authority before offering work."""
        now = time.monotonic()
        if self.recovery_ready or now < self.next_task_recovery_at or not self.connection.connected:
            return
        self.next_task_recovery_at = now + self.lattice_config.reconnect_secs
        recovered = []
        page_token = None
        seen_tokens = set()
        try:
            while True:
                page = self._lattice_call("query_tasks", lambda: self.connection.client.tasks.query_tasks(page_token=page_token))
                recovered.extend(page.tasks or [])
                page_token = page.next_page_token
                if not page_token:
                    break
                if page_token in seen_tokens:
                    raise RuntimeError("Lattice task query repeated a page token")
                seen_tokens.add(page_token)
        except LatticeTransportError as exc:
            self._surface_transport_error("query_tasks", exc)
            return
        for task in recovered:
            assignee = _field(_field(_field(task, "relations"), "assignee"), "system")
            if _field(assignee, "entity_id") != self.asset_id or task.is_executed_elsewhere:
                continue
            self.status_versions.seed(task.version.task_id, task.version.status_version)
            self.deliveries.restore(task)
            if task.status.status in TaskDeliveries.RECOVERABLE:
                print(f"[TASK_RECOVERED] task_id={task.version.task_id} status={task.status.status}", flush=True)
            if task.status.status == "STATUS_CANCEL_REQUESTED":
                # The manager cancelled while no edge was listening; the request is not redelivered.
                print(f"[TASK_RECOVERED_CANCEL] task_id={task.version.task_id}", flush=True)
                self._publish_event(LATTICE.Event.Task.CancelRequested, {"TaskId": task.version.task_id})
        self.recovery_ready = True
        if self.deliveries.blocked:
            self._record_error(LatticeConnectionError("task_recovery", self.lattice_config.base_url,
                f"Unresolved tasks {sorted(self.deliveries.blocked)}; flight blocked. Reconcile their terminal status before restarting MPFC."))
        self._sync_task_state()

    def _retry_receipts(self) -> None:
        now = time.monotonic()
        if not self.connection.connected or now < self.next_receipt_retry_at:
            return
        self.next_receipt_retry_at = now + self.lattice_config.reconnect_secs
        for task_id in list(self.deliveries.unreceipted):
            self._receive_task(task_id)

    # -- task status ---------------------------------------------------------

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
        self.resynced_task = task
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
        result: GoogleProtobufAny | None = None,
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
        expected_version = self.status_versions.get(task_id)
        if expected_version is None:
            expected_version = self._resync_status_version(task_id, None)
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
        new_status = TaskStatus(status=status, task_error=error, progress=progress_document, result=result)

        try:
            updated = self._lattice_call(
                "update_task_status",
                lambda: self._submit_status(task_id, expected_version, new_status),
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
            remote_status = self.resynced_task.status
            if (remote_status.model_dump(exclude_none=True) == new_status.model_dump(exclude_none=True)
                    and self.resynced_task.last_updated_by == Principal(system=System(entity_id=self.asset_id))):
                self.deliveries.acknowledge(task_id, status)
                return True, actual_version
            resolves_cancel = (remote_status.status == "STATUS_CANCEL_REQUESTED"
                               and status == "STATUS_DONE_NOT_OK" and error is not None
                               and error.code == "ERROR_CODE_CANCELLED")
            if (not resolves_cancel and str(remote_status.status) not in
                    {"STATUS_SENT", "STATUS_MACHINE_RECEIPT", "STATUS_ACK", "STATUS_EXECUTING"}):
                self.deliveries.reconcile(self.resynced_task)
                return False, actual_version
            if status == "STATUS_MACHINE_RECEIPT" and remote_status.status != "STATUS_SENT":
                self.deliveries.reconcile(self.resynced_task)
                return False, actual_version
            retry_version = actual_version
            try:
                updated = self._lattice_call(
                    "update_task_status",
                    lambda: self._submit_status(task_id, retry_version, new_status),
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
        if actual_version != expected_version + 1:
            print(
                f"[TASK_STATUS_RESYNC] task_id={task_id} expected_version={expected_version} "
                f"actual_version={actual_version}",
                flush=True,
            )
        self.status_versions.adopt(task_id, actual_version)
        self.deliveries.acknowledge(task_id, status)
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

    def _submit_status(self, task_id: str, version: int, new_status: TaskStatus) -> Any:
        return self.connection.client.tasks.update_task_status(task_id=task_id, status_version=version,
            new_status=new_status, author=Principal(system=System(entity_id=self.asset_id)))

    # -- entity publishing ---------------------------------------------------

    def _create_ingress(self, item: MediaSource) -> bool:  # Create the ingress; False when the id already exists.
        try:
            self.connection.client.video.create_ingress_stream(
                ingress_id=item.ingress_id, title=item.ingress_id, rtsp=RtspSettings(url=item.url)
            )
        except ApiError as exc:
            if exc.status_code == HTTP_CONFLICT:
                return False
            raise
        return True

    def _register_media(self) -> None:  # Register configured video sources as ingress streams once per connection.
        if not self.connection.connected:
            for item in self.media_sources:
                item.registered = False
            return
        for item in self.media_sources:
            if item.registered:
                continue
            try:
                created = self._lattice_call("create_ingress_stream", lambda: self._create_ingress(item))
                if not created:
                    existing = self._lattice_call(
                        "get_ingress_stream", lambda: self.connection.client.video.get_ingress_stream(item.ingress_id)
                    ).ingress_stream
                    existing_url = None if existing.rtsp is None else existing.rtsp.url
                    if existing_url != item.url:
                        print(
                            f"[PLUGIN] {self.client_id} ingress {item.ingress_id} url changed "
                            f"from={existing_url} to={item.url}; re-creating",
                            flush=True,
                        )
                        self._lattice_call(
                            "delete_ingress_stream",
                            lambda: self.connection.client.video.delete_ingress_stream(item.ingress_id),
                        )
                        self._lattice_call("create_ingress_stream", lambda: self._create_ingress(item))
            except LatticeTransportError as exc:
                self._surface_transport_error("register_media", exc)
                continue
            item.registered = True
            print(f"[PLUGIN] {self.client_id} ingress registered id={item.ingress_id} url={item.url}", flush=True)

    def _publish_entity_state(self) -> bool:
        entity = self.picture.entity(
            self.state, self.bus.state_observed_at,
            Media(media=[item.media_item() for item in self.media_sources if item.registered]),
            self.accept_tasks, platform_activity(self.state),
        )
        try:
            self._lattice_call("publish_entity", lambda: self.connection.client.entities.publish_entity(
                **entity.model_dump(exclude_unset=True, exclude_none=True)))
        except LatticeTransportError as exc:
            self._surface_transport_error("publish_entity", exc)
            return False
        self.entity_publish_count += 1
        self.last_entity_publish_ms = int(time.time() * 1000)
        self.last_entity_fields = {
            LATTICE.State.Entity.PublishCount: self.entity_publish_count,
            LATTICE.State.Entity.LastPublishAtMs: self.last_entity_publish_ms,
        }
        if entity.location is not None:
            position = entity.location.position
            self.last_entity_fields[LATTICE.State.Entity.LatitudeDeg] = position.latitude_degrees
            self.last_entity_fields[LATTICE.State.Entity.LongitudeDeg] = position.longitude_degrees
            if position.altitude_hae_meters is not None:
                self.last_entity_fields[LATTICE.State.Entity.AltitudeHaeM] = position.altitude_hae_meters
            attitude = entity.location.attitude_enu
            if attitude is not None:
                self.last_entity_fields.update({
                    LATTICE.State.Entity.AttitudeEnuW: attitude.w,
                    LATTICE.State.Entity.AttitudeEnuX: attitude.x,
                    LATTICE.State.Entity.AttitudeEnuY: attitude.y,
                    LATTICE.State.Entity.AttitudeEnuZ: attitude.z,
                })
        self._sync_entity_state()
        return True

    def _publish_track(self, envelope: Dict[str, Any]) -> None:  # Geolocate one detection and publish it as a track entity.
        detection = envelope["data"]
        position = self.state.get(UAV.State.Navigation.Position)
        attitude = self.state.get(UAV.State.Attitude.AttitudeRad)
        now = datetime.now(timezone.utc)
        navigation_fresh = self.picture.fresh(self.bus.state_observed_at, UAV.State.Navigation.Position, now)
        attitude_fresh = self.picture.fresh(self.bus.state_observed_at, UAV.State.Attitude.AttitudeRad, now)
        if ("tracker_id" not in detection or position is None or attitude is None or not self.connection.connected
                or not navigation_fresh or not attitude_fresh):
            print(
                f"[TRACK_SKIPPED] class={detection.get('class_name')} tracker_id={detection.get('tracker_id')} "
                f"position_fresh={navigation_fresh} attitude_fresh={attitude_fresh} connected={self.connection.connected}",
                flush=True,
            )
            return
        point = self.camera.ground_point(position, float(attitude["Yaw"]), detection)
        if point is None:
            print(f"[TRACK_SKIPPED] class={detection['class_name']} tracker_id={detection['tracker_id']} "
                  f"camera_pitch_deg={self.camera.projection.pitch_deg} reason=image_ray_above_horizon", flush=True)
            return
        lat, lon = point
        uncertainty = self.camera.uncertainty(position, float(attitude['Yaw']), detection)
        observed_at = datetime.fromtimestamp(envelope["time"] / 1000.0, timezone.utc)
        track_id = f"{self.asset_id}.trk.{detection['tracker_id']}"
        entity = Entity(
            entity_id=track_id,
            is_live=True,
            expiry_time=observed_at + timedelta(seconds=self.track_expiry_s),
            aliases=Aliases(name=f"{detection['class_name']} {detection['tracker_id']}"),
            location=LatticeLocation(
                position=Position(latitude_degrees=lat, longitude_degrees=lon, altitude_hae_meters=self.entity_home_hae_m)
            ),
            location_uncertainty=uncertainty,
            mil_view=MilView(disposition="DISPOSITION_UNKNOWN", environment="ENVIRONMENT_LAND"),
            ontology=Ontology(template="TEMPLATE_TRACK"),
            provenance=Provenance(
                integration_name=f"mpfc.{self.client_id}",
                data_type=f"MPFC detection {detection['class_name']} conf={detection['confidence']}",
                source_id=self.asset_id,
                source_update_time=observed_at,
            ),
        )
        try:
            self._lattice_call("publish_track", lambda: self.connection.client.entities.publish_entity(**entity.model_dump()))
        except LatticeTransportError as exc:
            self._surface_transport_error("publish_track", exc)

    def _maybe_publish_entity_state(self) -> None:  # Publish the entity on its configured interval.
        if not self.connection.connected:
            return
        now = time.time()
        if now - (self.last_entity_publish_ms / 1000.0) < self.state_publish_interval_s:
            return
        self._publish_entity_state()

    # -- bus request handling ------------------------------------------------

    def _handle_fetch(self, request_id: str, action: str) -> None:
        if not self.recovery_ready:
            self.enqueue_response(request_id, action, False, {"error": "Task recovery has not completed"})
            return
        task = self.deliveries.fetch()
        data = {"PendingCount": self.deliveries.pending_count, "TaskId": "" if task is None else task.task_id,
                "RecoveryTaskIds": sorted(self.deliveries.blocked)}
        if task is not None:
            data.update(task.payload())
        self._sync_task_state()
        self.enqueue_response(request_id, action, True, data)

    def _handle_entity_get(self, request_id: str, action: str, params: Dict[str, Any]) -> None:
        entity = self._lattice_call("get_entity", lambda: self.connection.client.entities.get_entity(params["EntityId"]))
        self.enqueue_response(request_id, action, True,
            {"EntityJson": entity.model_dump_json(by_alias=True, exclude_unset=True)})

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
                result=GoogleProtobufAny.model_validate_json(params["ResultJson"]) if params.get("ResultJson") else None,
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
            if action == LATTICE.Action.Entity.Get:
                self._handle_entity_get(request_id, action, params)
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
        if self.accept_tasks:
            self._start_listener()
        self.send_online()
        self._sync_connection_state()
        self._sync_task_state()
        self._sync_entity_state()
        try:
            while True:
                if self.connect_enabled:
                    self.connection.ensure_connected()
                if self.accept_tasks:
                    self._recover_tasks()
                    self._drain_agent_requests()
                    self._retry_receipts()
                self._register_media()
                self._maybe_publish_entity_state()
                self._sync_connection_state()
                self.state_scheduler.flush()
                self.flush_queue(self.response_queue, self.response_topic)
                deadline = time.monotonic() + self.loop_interval_s
                while time.monotonic() < deadline:
                    topic, payload = self._pump_once(deadline)
                    if topic is not None and topic == self.detection_topic:
                        self._publish_track(payload)
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
