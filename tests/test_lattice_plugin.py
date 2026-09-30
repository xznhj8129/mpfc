#!/usr/bin/env python3
"""Unit tests for the Lattice interface config, token, and connection lifecycle."""

import time
from urllib.parse import parse_qs

import httpx
import pytest
from anduril import Lattice
from anduril.core.api_error import ApiError

from plugins.lattice.lattice import (
    LatticeConfig,
    LatticeConnection,
    LatticeConnectionError,
)
from protocols.namespace_loader import load_protocol_namespace


def make_config(**overrides):
    cfg = {
        "base_url": "https://lattice.test:8443/",
        "tls_verify": True,
        "ca_bundle": None,
        "client_id": "api-client",
        "client_secret": "api-secret",
        "token": None,
        "token_endpoint": None,
        "timeout_s": 5.0,
        "max_retries": 0,
        "reconnect_secs": 0.0,
    }
    cfg.update(overrides)
    return LatticeConfig(cfg)


def make_token_handler(requests, expires_in=3600):
    def handler(request):
        requests.append(request)
        return httpx.Response(
            200,
            json={
                "access_token": f"token-{len(requests)}",
                "token_type": "Bearer",
                "expires_in": expires_in,
            },
        )

    return handler


def make_connection(handler, config=None):
    http_client = httpx.Client(transport=httpx.MockTransport(handler))
    return LatticeConnection(config or make_config(), http_client=http_client)


def test_protocol_contract_tokens() -> None:
    lattice = load_protocol_namespace("lattice")
    assert lattice.State.Link.Connected == "Connected"
    assert lattice.State.Auth.TokenValid == "TokenValid"
    assert lattice.State.System.LastErrorOperation == "LastErrorOperation"
    assert lattice.Action.Connection.Connect == "Connect"
    assert lattice.Action.Connection.Status == "Status"
    assert lattice.Event.Auth.TokenAcquired == "TokenAcquired"
    assert lattice.Event.System.Error == "Error"


def test_config_url_and_token_endpoint() -> None:
    config = make_config()
    assert config.base_url == "https://lattice.test:8443"
    assert config.token_endpoint == "https://lattice.test:8443/api/v1/oauth/token"
    assert make_config(token_endpoint="https://auth.test/oauth/token/").token_endpoint == (
        "https://auth.test/oauth/token"
    )


def test_config_tls_verify_mapping() -> None:
    assert make_config().verify() is True
    assert make_config(tls_verify=False).verify() is False
    assert make_config(tls_verify=False, ca_bundle="/etc/ssl/ca.pem").verify() == "/etc/ssl/ca.pem"


def test_config_requires_credentials() -> None:
    with pytest.raises(ValueError):
        make_config(client_id=None, client_secret=None)
    static = make_config(client_id=None, client_secret=None, token="static-token")
    assert static.token == "static-token"


def test_connect_acquires_token_and_builds_sdk_client() -> None:
    requests = []
    connection = make_connection(make_token_handler(requests))
    connection.connect()
    assert connection.connected is True
    assert connection.connect_attempts == 1
    assert connection.token_valid() is True
    assert connection.token_acquisitions == 1
    assert connection.token_value() == "token-1"
    assert len(requests) == 1
    request = requests[0]
    assert str(request.url) == "https://lattice.test:8443/api/v1/oauth/token"
    form = parse_qs(request.content.decode())
    assert form["client_id"] == ["api-client"]
    assert form["client_secret"] == ["api-secret"]
    assert form["grant_type"] == ["client_credentials"]
    assert isinstance(connection.client, Lattice)
    assert connection.client._client_wrapper.get_base_url() == "https://lattice.test:8443"
    headers = connection.client._client_wrapper.get_headers()
    assert headers["Authorization"] == "Bearer token-1"


def test_sdk_api_request_uses_connection_token_and_transport() -> None:
    requests = []

    def handler(request):
        requests.append(request)
        if request.url.path == "/api/v1/oauth/token":
            return httpx.Response(
                200,
                json={"access_token": "token-1", "token_type": "Bearer", "expires_in": 3600},
            )
        return httpx.Response(404, json={"error": "not found"})

    connection = make_connection(handler)
    connection.connect()
    with pytest.raises(ApiError):
        connection.client.tasks.get_task("task-1")
    assert len(requests) == 2
    api_request = requests[1]
    assert str(api_request.url) == "https://lattice.test:8443/api/v1/tasks/task-1"
    assert api_request.headers["Authorization"] == "Bearer token-1"


def test_token_error_surfaces_operation_and_endpoint() -> None:
    def handler(request):
        return httpx.Response(401, text="unauthorized")

    connection = make_connection(handler)
    with pytest.raises(LatticeConnectionError) as excinfo:
        connection.connect()
    error = excinfo.value
    assert error.operation == "token-acquisition"
    assert error.endpoint == "https://lattice.test:8443/api/v1/oauth/token"
    assert "HTTP 401" in str(error)
    assert connection.connected is False
    assert connection.last_error is error


def test_transport_error_is_wrapped() -> None:
    def handler(request):
        raise httpx.ConnectError("connection refused")

    connection = make_connection(handler)
    with pytest.raises(LatticeConnectionError) as excinfo:
        connection.connect()
    assert excinfo.value.operation == "token-acquisition"
    assert "ConnectError" in excinfo.value.detail
    assert connection.connected is False


def test_static_token_skips_token_endpoint() -> None:
    def handler(request):
        raise AssertionError(f"unexpected request url={request.url}")

    config = make_config(client_id=None, client_secret=None, token="static-token")
    connection = make_connection(handler, config=config)
    connection.connect()
    assert connection.connected is True
    assert connection.token_acquisitions == 0
    assert connection.token_value() == "static-token"
    assert connection.token_valid() is True


def test_short_lived_token_is_refreshed_on_use() -> None:
    requests = []
    connection = make_connection(make_token_handler(requests, expires_in=1))
    connection.connect()
    assert connection.token_acquisitions == 1
    assert connection.token_valid() is False  # expires_in is inside the refresh buffer
    assert connection.token_value() == "token-2"
    assert connection.token_acquisitions == 2
    assert len(requests) == 2


def test_close_and_reconnect_reuses_valid_token() -> None:
    requests = []
    connection = make_connection(make_token_handler(requests))
    connection.connect()
    connection.close()
    assert connection.connected is False
    assert connection.ensure_connected() is True
    assert connection.connected is True
    assert connection.connect_attempts == 2
    assert len(requests) == 1  # cached token was still valid


def test_ensure_connected_backoff_and_recovery() -> None:
    attempts = []

    def handler(request):
        attempts.append(request)
        if len(attempts) == 1:
            return httpx.Response(503, text="unavailable")
        return httpx.Response(
            200,
            json={"access_token": "token-ok", "token_type": "Bearer", "expires_in": 3600},
        )

    connection = make_connection(handler, config=make_config(reconnect_secs=60.0))
    assert connection.ensure_connected() is False
    assert connection.connected is False
    assert connection.last_error is not None
    assert connection.last_error.operation == "token-acquisition"
    assert connection.ensure_connected() is False  # still inside the retry backoff
    assert len(attempts) == 1
    assert connection.ensure_connected(now=time.monotonic() + 61.0) is True
    assert connection.connected is True
    assert connection.last_error is None
    assert connection.connect_attempts == 2
