#!/usr/bin/env python3
"""
Shared synchronous core base class.
Provides bus connection, auto diag subscription/ping-pong, ONLINE/STOPPED events, and a helper to fetch the next non-diag message.
"""

from typing import Any, Dict

from lib.common import CONTROL_SHUTDOWN_TOPIC, RuntimeBase, build_envelope


class CoreBase(RuntimeBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Initialize core base and bus.
        super().__init__(cfg, bus_config)

    def _log_starting(self) -> None:  # Log STARTING event.
        print(f"[CORE] {self.client_id} starting", flush=True)

    def _log_online(self) -> None:  # Log ONLINE event.
        print(f"[CORE] {self.client_id} online", flush=True)

    def _log_stopped(self) -> None:  # Log STOPPED event.
        print(f"[CORE] {self.client_id} stopped", flush=True)

    def publish_shutdown(self) -> None:  # Publish CONTROL/SHUTDOWN.
        try:
            self.client.publish(CONTROL_SHUTDOWN_TOPIC, build_envelope(self.client_id, CONTROL_SHUTDOWN_TOPIC, {}))
        except Exception:
            pass

    def finish(self, exit_code: int = 0) -> None:  # Shutdown and exit.
        self.publish_shutdown()
        self.stop()
        raise SystemExit(exit_code)

    def _on_control_shutdown(self) -> None:  # Handle CONTROL/SHUTDOWN.
        print(f"[CORE] {self.client_id} shutdown", flush=True)
        self.finish(0)
