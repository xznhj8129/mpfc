#!/usr/bin/env python3
"""
Usage:
    MAIN_CONFIG=./config/config_hello.json python main.py
    python examples/run_config_hello.py
MAIN_CONFIG must point to a config shaped like config/config_hello.json.
"""

import asyncio
import importlib
import multiprocessing as mp
import os
import socket
import time
import sys
from pathlib import Path
from typing import Any, Dict, List

from lib.common import load_json
from lib.message_bus import run_server

CONFIG_ENV = "MAIN_CONFIG"
BUS_READY_INTERVAL_S = 0.1
BUS_READY_TIMEOUT_S = 5.0

def _run_bus(bus_config: Dict[str, Any]) -> None:
    asyncio.run(run_server(bus_config["endpoint"], bus_config["log_file"]))


def start_from_config(config_path: Path) -> None:
    config = load_json(config_path)
    # TODO: Verbosely print config file
    base_dir = config_path.parent
    repo_root = base_dir.parent if base_dir.name == "config" else base_dir
    raw_bus_config = config["bus_config"]

    schema_path = Path(raw_bus_config["schema_path"])
    if not schema_path.is_absolute():
        schema_path = base_dir / schema_path

    log_file = Path(raw_bus_config["log_file"])
    if not log_file.is_absolute():
        log_file = repo_root / log_file

    endpoint_cfg = raw_bus_config["endpoint"]
    etype = endpoint_cfg.get("type")
    if etype == "tcp":
        endpoint = {"type": "tcp", "host": endpoint_cfg.get("host"), "port": endpoint_cfg.get("port")}
    elif etype == "unix":
        endpoint_path = Path(endpoint_cfg.get("path"))
        if not endpoint_path.is_absolute():
            endpoint_path = base_dir / endpoint_path
        endpoint = {"type": "unix", "path": str(endpoint_path)}
    else:
        endpoint = endpoint_cfg
    bus_config = {"schema_path": str(schema_path), "log_file": str(log_file), "endpoint": endpoint}

    log_fd = os.open(str(log_file), os.O_WRONLY | os.O_CREAT | os.O_APPEND)
    log_handle = os.fdopen(log_fd, "a", buffering=1)
    console_out = os.fdopen(os.dup(sys.__stdout__.fileno()), "w", buffering=1)
    console_err = os.fdopen(os.dup(sys.__stderr__.fileno()), "w", buffering=1)

    class Tee:
        def __init__(self, streams: list) -> None:
            self.streams = streams

        def write(self, data: str) -> None:
            for stream in self.streams:
                stream.write(data)
                stream.flush()

        def flush(self) -> None:
            for stream in self.streams:
                stream.flush()

    sys.stdout = Tee([console_out, log_handle])
    sys.stderr = Tee([console_err, log_handle])

    core_config = config["core"]
    core_name = core_config["name"]
    core_cfg = core_config["cfg"]
    core_module = importlib.import_module(f"flight_cores.{core_name}")
    core_runner = getattr(core_module, "run_core")

    plugins_raw = config["plugins"]
    plugin_entries: List[Dict[str, Any]] = []
    for entry in plugins_raw:
        plugin_name = entry["plugin"]
        plugin_cfg = entry["cfg"]
        plugin_module = importlib.import_module(f"plugins.{plugin_name}")
        plugin_runner = getattr(plugin_module, "run_plugin")
        plugin_entries.append({"cfg": plugin_cfg, "runner": plugin_runner})

    bus_proc = mp.Process(target=_run_bus, args=(bus_config,), name="bus")
    bus_proc.start()
    if bus_config["endpoint"]["type"] == "tcp":
        start_time = time.monotonic()
        while True:
            if not bus_proc.is_alive():
                break
            try:
                with socket.create_connection(
                    (bus_config["endpoint"]["host"], bus_config["endpoint"]["port"]), timeout=BUS_READY_INTERVAL_S
                ):
                    break
            except OSError:
                if time.monotonic() - start_time > BUS_READY_TIMEOUT_S:
                    break
                time.sleep(BUS_READY_INTERVAL_S)
    else:
        endpoint_path = Path(bus_config["endpoint"]["path"])
        start_time = time.monotonic()
        while True:
            if not bus_proc.is_alive():
                break
            if endpoint_path.exists():
                break
            if time.monotonic() - start_time > BUS_READY_TIMEOUT_S:
                break
            time.sleep(BUS_READY_INTERVAL_S)

    core_proc = mp.Process(target=core_runner, args=(core_cfg, bus_config), name=f"core-{core_cfg.get('id','?')}")
    core_proc.start()
    plugin_procs = []
    for entry in plugin_entries:
        plugin_cfg = entry["cfg"]
        proc = mp.Process(
            target=entry["runner"], args=(plugin_cfg, bus_config), name=f"plugin-{plugin_cfg.get('id','?')}"
        )
        proc.start()
        plugin_procs.append(proc)

    all_procs = [bus_proc, core_proc] + plugin_procs
    try:
        while True:
            dead = [proc for proc in all_procs if not proc.is_alive()]
            if dead:
                names = ", ".join(f"{p.name}({p.exitcode})" for p in dead)
                print(f"[MAIN] detected exit: {names}, terminating remaining", flush=True)
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("[MAIN] interrupt received, terminating", flush=True)
    finally:
        for proc in all_procs:
            if proc.is_alive():
                proc.terminate()
            proc.join(timeout=5.0)


def main() -> None:
    config_path = Path(os.environ[CONFIG_ENV]).resolve()
    start_from_config(config_path)


if __name__ == "__main__":
    main()
