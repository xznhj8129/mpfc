#!/usr/bin/env python3
"""
Lattice interface plugin: owns the Lattice API connection for MPFC.

Usage:
    from plugins.lattice.lattice import run_plugin
    run_plugin(cfg, bus_config)

The plugin wraps the official anduril-lattice-sdk client and exposes only
the connection and token lifecycle over the bus messages declared in
protocols/lattice.yaml. Task handling, mission policy, and entity
semantics are out of scope for this interface.
"""

import threading
import time
import traceback
from typing import Any, Dict

import httpx
from anduril import Lattice
from anduril.oauth.types.get_token_response import GetTokenResponse

from lib.common import (
    build_event_topics,
    build_request_topic,
    build_response_topic,
    build_state_scheduler_topics,
    build_topic_base,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from protocols.namespace_loader import load_protocol_namespace

LATTICE = load_protocol_namespace("lattice")

DEFAULT_TOKEN_EXPIRES_IN_S = 3600
TOKEN_EXPIRY_BUFFER_S = 120.0


class LatticeConnectionError(RuntimeError):
    def __init__(self, operation: str, endpoint: str, detail: str) -> None:
        self.operation = operation
        self.endpoint = endpoint
        self.detail = detail
        super().__init__(f"lattice {operation} failed endpoint={endpoint} detail={detail}")


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

    def connect(self) -> None:  # Acquire a token when needed and build the SDK client.
        if self.connected:
            return
        self.connect_attempts += 1
        try:
            if self._token is None or time.time() >= self._token_expires_at:
                self._refresh_token()
            if self._client is None:
                self._client = Lattice(
                    base_url=self.config.base_url,
                    token=self.token_value,
                    timeout=self.config.timeout_s,
                    max_retries=self.config.max_retries,
                    httpx_client=self._http_client(),
                )
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


class LatticePlugin(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Initialize Lattice interface plugin.
        super().__init__(cfg, bus_config)
        # Read config explicitly: cfg["client_id"] is the Lattice API credential id,
        # while self.client_id is the bus client id set by RuntimeBase.
        self.topic_ns = cfg["topic_ns"]
        self.state_intervals = cfg["state_intervals"]
        self.loop_interval_s = float(cfg["loop_interval_s"])
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
                LATTICE.Event.System.Error,
            ],
        )
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(float(cfg["bus_poll_interval_s"]))

        self.connect_enabled = True
        self._published_connected = False
        self._published_token_acquisitions = 0
        self._reported_error: LatticeConnectionError | None = None
        self._sync_connection_state()

    def _status_data(self) -> Dict[str, Any]:  # Build connection status payload.
        return {
            "Connected": self.connection.connected,
            "Endpoint": self.lattice_config.base_url,
            "ConnectAttempts": self.connection.connect_attempts,
            "TokenValid": self.connection.token_valid(),
        }

    def _record_error(self, error: LatticeConnectionError) -> None:  # Surface an error with operation and endpoint context.
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
        error = self.connection.last_error
        if error is not None and error is not self._reported_error:
            self._reported_error = error
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

    def _handle_request(self, request: Dict[str, Any]) -> None:  # Handle a bus REQUEST action.
        request_id = str(request["request_id"])
        action = request["action"]
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

    def run(self) -> None:  # Run Lattice connection loop.
        self.send_online()
        self._sync_connection_state()
        try:
            while True:
                if self.connect_enabled:
                    self.connection.ensure_connected()
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
            self.stop()

    def stop(self) -> None:  # Close the Lattice connection and stop plugin.
        self.connection.close()
        super().stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    LatticePlugin(cfg, bus_config).run()
