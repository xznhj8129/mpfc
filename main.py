#!/usr/bin/env python3
"""
Usage:
    MAIN_CONFIG=./config/config_hello.json python main.py
    python examples/run_config_hello.py
MAIN_CONFIG must point to a config shaped like config/config_hello.json.
"""

import asyncio
import multiprocessing as mp
import os
import socket
import time
from pathlib import Path
from typing import Any, Dict, List

from flight_cores.test_core import run_core
from lib.common import load_json
from lib.message_bus import run_server
from plugins.example_hello import run_plugin

CONFIG_ENV = "MAIN_CONFIG"
BUS_READY_INTERVAL_S = 0.1
BUS_READY_TIMEOUT_S = 5.0


def _run_bus(bus_config: Dict[str, Any]) -> None:
    asyncio.run(run_server(bus_config["endpoint"], bus_config["log_file"]))


def start_from_config(config_path: Path) -> None:
    config = load_json(config_path)
    base_dir = config_path.parent
    if "bus_config" not in config:
        raise KeyError("config missing bus_config")
    raw_bus_config = config["bus_config"]
    if not isinstance(raw_bus_config, dict):
        raise TypeError("bus_config must be an object")
    for required_key in ("schema_path", "log_file", "endpoint"):
        if required_key not in raw_bus_config:
            raise KeyError(f"bus_config missing {required_key}")

    schema_path = Path(raw_bus_config["schema_path"])
    if not schema_path.is_absolute():
        schema_path = base_dir / schema_path
    if not schema_path.is_file():
        raise FileNotFoundError(f"schema file not found: {schema_path}")

    log_file = Path(raw_bus_config["log_file"])
    if not log_file.is_absolute():
        log_file = base_dir / log_file
    log_parent = log_file.parent
    if log_parent and not log_parent.exists():
        raise FileNotFoundError(f"log directory does not exist: {log_parent}")

    endpoint_cfg = raw_bus_config["endpoint"]
    if not isinstance(endpoint_cfg, dict):
        raise TypeError("bus_config.endpoint must be an object")
    etype = endpoint_cfg.get("type")
    if etype == "tcp":
        host = endpoint_cfg.get("host")
        port = endpoint_cfg.get("port")
        if not isinstance(host, str) or not host:
            raise ValueError("bus_config.endpoint.host is required for tcp")
        if not isinstance(port, int) or port < 1 or port > 65535:
            raise ValueError("bus_config.endpoint.port must be int 1-65535 for tcp")
        endpoint = {"type": "tcp", "host": host, "port": port}
    elif etype == "unix":
        path = endpoint_cfg.get("path")
        if not isinstance(path, str) or not path:
            raise ValueError("bus_config.endpoint.path is required for unix")
        endpoint_path = Path(path)
        if not endpoint_path.is_absolute():
            endpoint_path = base_dir / endpoint_path
        endpoint = {"type": "unix", "path": str(endpoint_path)}
    else:
        raise ValueError("bus_config.endpoint.type must be 'tcp' or 'unix'")
    bus_config = {"schema_path": str(schema_path), "log_file": str(log_file), "endpoint": endpoint}

    if "core" not in config:
        raise KeyError("config missing core object")
    core_config = config["core"]
    if not isinstance(core_config, dict):
        raise TypeError("core must be an object")
    for required_key in ("name", "cfg"):
        if required_key not in core_config:
            raise KeyError(f"core missing {required_key}")
    core_name = core_config["name"]
    if core_name != "test_core":
        raise ValueError(f"hello config requires core.name='test_core', got '{core_name}'")
    core_cfg = core_config["cfg"]
    if not isinstance(core_cfg, dict):
        raise TypeError("core cfg must be an object")

    if "plugins" not in config:
        raise KeyError("config missing plugins list")
    plugins_raw = config["plugins"]
    if not isinstance(plugins_raw, list):
        raise TypeError("plugins must be a list")
    plugin_cfgs: List[Dict[str, Any]] = []
    for entry in plugins_raw:
        if not isinstance(entry, dict):
            raise TypeError("plugin entry must be an object")
        for required_key in ("plugin", "cfg"):
            if required_key not in entry:
                raise KeyError(f"plugin entry missing {required_key}")
        name = entry["plugin"]
        if name != "example_hello":
            raise ValueError(f"hello config only supports example_hello plugins, got '{name}'")
        cfg = entry["cfg"]
        if not isinstance(cfg, dict):
            raise TypeError("plugin cfg must be an object")
        plugin_cfgs.append(cfg)

    bus_proc = mp.Process(target=_run_bus, args=(bus_config,), name="bus")
    bus_proc.start()
    if bus_config["endpoint"]["type"] == "tcp":
        start_time = time.monotonic()
        while True:
            if not bus_proc.is_alive():
                raise RuntimeError(f"bus process exited early with code={bus_proc.exitcode}")
            try:
                with socket.create_connection(
                    (bus_config["endpoint"]["host"], bus_config["endpoint"]["port"]), timeout=BUS_READY_INTERVAL_S
                ):
                    break
            except OSError:
                if time.monotonic() - start_time > BUS_READY_TIMEOUT_S:
                    raise TimeoutError("bus did not start before timeout")
                time.sleep(BUS_READY_INTERVAL_S)
    else:
        endpoint_path = Path(bus_config["endpoint"]["path"])
        start_time = time.monotonic()
        while True:
            if not bus_proc.is_alive():
                raise RuntimeError(f"bus process exited early with code={bus_proc.exitcode}")
            if endpoint_path.exists():
                break
            if time.monotonic() - start_time > BUS_READY_TIMEOUT_S:
                raise TimeoutError("bus did not create unix socket before timeout")
            time.sleep(BUS_READY_INTERVAL_S)

    core_proc = mp.Process(target=run_core, args=(bus_config, core_cfg), name="core")
    core_proc.start()
    plugin_procs = []
    for plugin_cfg in plugin_cfgs:
        proc = mp.Process(target=run_plugin, args=(bus_config, plugin_cfg), name=f"plugin-{plugin_cfg.get('id','?')}")
        proc.start()
        plugin_procs.append(proc)

    all_procs = [bus_proc, core_proc] + plugin_procs
    try:
        while True:
            for proc in all_procs:
                if not proc.is_alive():
                    raise RuntimeError(f"process {proc.name} exited code={proc.exitcode}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("[MAIN] interrupt received, terminating", flush=True)
    finally:
        for proc in all_procs:
            if proc.is_alive():
                proc.terminate()
            proc.join(timeout=5.0)
            if proc.is_alive():
                raise RuntimeError(f"process {proc.name} refused to terminate")


def main() -> None:
    env_value = os.environ.get(CONFIG_ENV)
    if not env_value:
        raise EnvironmentError(f"Set {CONFIG_ENV} to a config file path (e.g., config/config_hello.json)")
    config_path = Path(env_value).resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"config file not found: {config_path}")
    start_from_config(config_path)


if __name__ == "__main__":
    main()
