#!/usr/bin/env python3
"""
Usage:
    MAIN_CONFIG=./flight_cores/test_core/config.yaml python main.py
    MAIN_CONFIG=./flight_cores/test_takeoff_land/config.yaml python main.py --mav_port=14551 --takeoff_altitude_m=12.5
MAIN_CONFIG must point to a YAML config shaped like flight_cores/test_core/config.yaml.
"""

import importlib
import multiprocessing as mp
import os
import time
import sys
import queue
import json
import re
import signal
from pathlib import Path
from typing import Any, Dict, List

REPO_ROOT = Path(__file__).resolve().parent

from lib.common import CONTROL_SHUTDOWN_TOPIC, build_envelope, connect_bus_client, load_config
from lib.mqtt_bus_client import MqttPublishError

CONFIG_ENV = "MAIN_CONFIG"
PLACEHOLDER_RE = re.compile(r"<([A-Za-z0-9_]+)>")

# Local vehicle classification (N1-N3).  Tokens are MPFC-local routing values;
# the MAVLink mapping is the classification used on the wire where one exists.
AUTOPILOT_TOKENS = {
    "BETAFLIGHT": "MAV_AUTOPILOT_GENERIC",
    "INAV": "MAV_AUTOPILOT_GENERIC",
    "ARDUPILOT": "MAV_AUTOPILOT_ARDUPILOTMEGA",
    "PX4": "MAV_AUTOPILOT_PX4",
    "CUSTOM": "MAV_AUTOPILOT_GENERIC",
}
AIRFRAME_TOKENS = {
    "FIXED_WING": "MAV_TYPE_FIXED_WING",
    "COPTER": "MAV_TYPE_QUADROTOR",
    "VTOL": "MAV_TYPE_VTOL_QUADROTOR",
    "TAILSITTER": "MAV_TYPE_VTOL_TAILSITTER_QUADROTOR",
    "FLYING_WING": "MAV_TYPE_FIXED_WING",
}
TELEMETRY_TOKENS = ("NONE", "MSP", "MAVLINK", "CRSF", "MANUAL_ENTRY")


def parse_runtime_overrides(argv: list[str]) -> Dict[str, str]:
    overrides: Dict[str, str] = {}
    for arg in argv:
        if not arg.startswith("--"):
            raise RuntimeError(f"invalid argument {arg} expected --key=value")
        raw = arg[2:]
        if "=" not in raw:
            raise RuntimeError(f"invalid argument {arg} expected --key=value")
        key, value = raw.split("=", 1)
        if not key:
            raise RuntimeError(f"invalid argument {arg} empty override key")
        overrides[key] = value
    return overrides


def cast_override_value(raw: str, existing: Any) -> Any:
    existing_type = type(existing)
    if existing_type is bool:
        low = raw.lower()
        if low in {"1", "true", "yes", "on"}:
            return True
        if low in {"0", "false", "no", "off"}:
            return False
        raise RuntimeError(f"invalid bool override value {raw}")
    if existing_type is int:
        return int(raw)
    if existing_type is float:
        return float(raw)
    if existing_type is str:
        return raw
    if existing_type is list or existing_type is dict:
        return json.loads(raw)
    raise RuntimeError(f"unsupported override type {existing_type.__name__}")


def apply_runtime_overrides(config: Dict[str, Any], overrides: Dict[str, str]) -> None:
    core_cfg = config["core"]["cfg"]
    mission_cfg = core_cfg.get("mission") if type(core_cfg.get("mission")) is dict else None
    for key, raw_value in overrides.items():
        if key == "my_name":
            config["my_name"] = cast_override_value(raw_value, config["my_name"])
            if "my_name" in core_cfg:
                core_cfg["my_name"] = cast_override_value(raw_value, core_cfg["my_name"])
            continue
        if key in core_cfg:
            core_cfg[key] = cast_override_value(raw_value, core_cfg[key])
            continue
        if mission_cfg is not None and key in mission_cfg:
            mission_cfg[key] = cast_override_value(raw_value, mission_cfg[key])
            continue
        raise RuntimeError(f"override key not found in core.cfg or core.cfg.mission {key}")


def collect_named_scalars(node: Any, out: Dict[str, str]) -> None:
    if type(node) is dict:
        for key, value in node.items():
            if type(value) in {str, int, float, bool}:
                out[key] = str(value)
            else:
                collect_named_scalars(value, out)
        return
    if type(node) is list:
        for value in node:
            collect_named_scalars(value, out)


def resolve_placeholders(node: Any, values: Dict[str, str]) -> Any:
    if type(node) is dict:
        return {key: resolve_placeholders(value, values) for key, value in node.items()}
    if type(node) is list:
        return [resolve_placeholders(value, values) for value in node]
    if type(node) is str:
        missing = [match for match in PLACEHOLDER_RE.findall(node) if match not in values]
        if missing:
            missing_keys = ",".join(missing)
            raise RuntimeError(f"unknown placeholder(s) {missing_keys} in value {node}")
        for key, value in values.items():
            node = node.replace(f"<{key}>", value)
        return node
    return node


def deep_merge_dict(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged: Dict[str, Any] = dict(base)
    for key, value in override.items():
        if key in merged and type(merged[key]) is dict and type(value) is dict:
            merged[key] = deep_merge_dict(merged[key], value)
        else:
            merged[key] = value
    return merged


def apply_plugin_config_templates(config: Dict[str, Any], repo_root: Path) -> None:
    merged_plugins: list[Dict[str, Any]] = []
    for plugin_entry in config["plugins"]:
        plugin_name = plugin_entry["plugin"]
        template_path = repo_root / "plugins" / plugin_name / "config_template.yaml"
        template = load_config(template_path)
        template_entry = {key: value for key, value in template.items() if key != "_notes"}
        merged_plugins.append(deep_merge_dict(template_entry, plugin_entry))
    config["plugins"] = merged_plugins


def _resolve_token(raw_value: Any, tokens: Any, kind: str) -> str:
    if type(raw_value) is not str:
        raise RuntimeError(
            f"invalid {kind} value type type={type(raw_value).__name__}"
        )
    token = raw_value.strip()
    if "." in token:
        prefix, token = token.rsplit(".", 1)
        if prefix != kind:
            raise RuntimeError(
                f"wrong {kind} prefix expected={kind} value={raw_value}"
            )
    table = tokens if isinstance(tokens, dict) else {name: name for name in tokens}
    if token not in table:
        raise RuntimeError(
            f"unknown {kind} value value={raw_value}"
        )
    return token


def resolve_enum_value(raw_value: Any, tokens: Any, kind: str) -> str:
    return _resolve_token(raw_value, tokens, kind)


def resolve_enum_list(raw_values: Any, tokens: Any, kind: str) -> list[str]:
    if type(raw_values) is not list:
        raise RuntimeError(
            f"invalid {kind} list type type={type(raw_values).__name__}"
        )
    return [_resolve_token(raw_value, tokens, kind) for raw_value in raw_values]


def resolve_vehicle_config(config: Dict[str, Any]) -> Dict[str, str] | None:
    vehicle = config.get("vehicle")
    if vehicle is None:
        return None
    if type(vehicle) is not dict:
        raise RuntimeError(f"invalid vehicle config type {type(vehicle).__name__}")
    vehicle["autopilot"] = resolve_enum_value(vehicle["autopilot"], AUTOPILOT_TOKENS, "Autopilot")
    if "airframe" not in vehicle:
        raise RuntimeError("vehicle config missing airframe")
    vehicle["airframe"] = resolve_enum_value(vehicle["airframe"], AIRFRAME_TOKENS, "Airframe")
    vehicle["telem_type"] = resolve_enum_value(vehicle["telem_type"], TELEMETRY_TOKENS, "Telemetry")
    return vehicle


def resolve_plugin_supports(config: Dict[str, Any]) -> None:
    for plugin_entry in config["plugins"]:
        plugin_cfg = plugin_entry["cfg"]
        supports = plugin_cfg.get("supports")
        if supports is None:
            continue
        if type(supports) is not dict:
            raise RuntimeError(
                f"invalid supports config type plugin={plugin_entry['plugin']} "
                f"type={type(supports).__name__}"
            )
        supports["autopilot"] = resolve_enum_list(supports["autopilot"], AUTOPILOT_TOKENS, "Autopilot")
        supports["telem_type"] = resolve_enum_list(supports["telem_type"], TELEMETRY_TOKENS, "Telemetry")


def plugin_supports_vehicle(plugin_cfg: Dict[str, Any], vehicle: Dict[str, str]) -> bool:
    supports = plugin_cfg.get("supports")
    if type(supports) is not dict:
        return False
    return (
        vehicle["autopilot"] in supports["autopilot"]
        and vehicle["telem_type"] in supports["telem_type"]
    )


def configure_runtime_plugins(config: Dict[str, Any], vehicle: Dict[str, str] | None) -> None:
    plugins_raw = config["plugins"]
    core_cfg = config["core"]["cfg"]
    if vehicle is not None:
        core_cfg["vehicle"] = dict(vehicle)

    # When a UAV controller is present it is the stable program-facing UAV API.
    # The supervisor selects exactly one endpoint adapter and injects that
    # backend into the controller; programs never bind to MAVSDK/MSP directly.
    controller_plugins = [
        entry for entry in plugins_raw if entry.get("cfg", {}).get("is_controller")
    ]
    if len(controller_plugins) > 1:
        raise RuntimeError(
            f"multiple controller plugins configured count={len(controller_plugins)}"
        )
    if controller_plugins:
        if vehicle is None:
            raise RuntimeError("vehicle config required when is_controller=true")
        controller_cfg = controller_plugins[0]["cfg"]
        matching_backends = [
            entry
            for entry in plugins_raw
            if entry.get("cfg", {}).get("is_interface")
            and entry["cfg"].get("topic_ns") == controller_cfg["topic_ns"]
            and plugin_supports_vehicle(entry["cfg"], vehicle)
        ]
        if len(matching_backends) != 1:
            backend_names = [entry["plugin"] for entry in matching_backends]
            raise RuntimeError(
                f"expected exactly one backend interface match "
                f"autopilot={vehicle['autopilot']} "
                f"telem_type={vehicle['telem_type']} matches={backend_names}"
            )
        backend_cfg = matching_backends[0]["cfg"]
        controller_cfg["backend"] = {
            "id": backend_cfg["id"],
            "topic_ns": backend_cfg["topic_ns"],
        }
        controller_cfg["backend_state_keys"] = list(
            backend_cfg["state_intervals"].keys()
        )
        controller_cfg["backend_event_keys"] = list(
            backend_cfg.get("event_keys", [])
        )
        controller_cfg["vehicle"] = dict(vehicle)
        expected_interface = {
            "id": controller_cfg["id"],
            "topic_ns": controller_cfg["topic_ns"],
        }
        if "interface" in core_cfg and core_cfg["interface"] != expected_interface:
            raise RuntimeError(
                f"core interface must target controller expected={expected_interface} "
                f"actual={core_cfg['interface']}"
            )
        core_cfg["interface"] = expected_interface
        return

    if "interface" in core_cfg:
        return
    interface_plugins = [
        entry for entry in plugins_raw if entry.get("cfg", {}).get("is_interface")
    ]
    if vehicle is not None:
        interface_plugins = [
            entry
            for entry in interface_plugins
            if plugin_supports_vehicle(entry["cfg"], vehicle)
        ]
    if len(interface_plugins) == 1:
        intf_cfg = interface_plugins[0]["cfg"]
        core_cfg["interface"] = {
            "id": intf_cfg["id"],
            "topic_ns": intf_cfg["topic_ns"],
        }
        return
    if len(interface_plugins) > 1:
        raise RuntimeError(
            "multiple interface plugins require explicit core.cfg.interface or one matching vehicle backend"
        )
    if vehicle is not None:
        raise RuntimeError(
            f"no interface plugin supports autopilot={vehicle['autopilot']} "
            f"telem_type={vehicle['telem_type']}"
        )


def start_from_config(config_path: Path, overrides: Dict[str, str]) -> None:
    repo_root = REPO_ROOT
    config = load_config(config_path)
    apply_runtime_overrides(config, overrides)
    apply_plugin_config_templates(config, repo_root)
    placeholder_values: Dict[str, str] = {}
    collect_named_scalars(config, placeholder_values)
    config = resolve_placeholders(config, placeholder_values)
    vehicle = resolve_vehicle_config(config)
    resolve_plugin_supports(config)
    configure_runtime_plugins(config, vehicle)

    runtime_mode = str(config.get("runtime_mode", "mission")).strip().lower()
    if runtime_mode not in {"mission", "resident"}:
        raise RuntimeError(f"invalid runtime_mode {runtime_mode!r}; expected mission or resident")
    resident_runtime = runtime_mode == "resident"

    config_dump = json.dumps(config, indent=2, sort_keys=True)
    base_dir = config_path.parent
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
        endpoint = {
            "type": "tcp",
            "host": endpoint_cfg.get("host"),
            "port": endpoint_cfg.get("port"),
        }
    elif etype == "unix":
        endpoint_path = Path(endpoint_cfg.get("path"))
        if not endpoint_path.is_absolute():
            endpoint_path = base_dir / endpoint_path
        endpoint = {"type": "unix", "path": str(endpoint_path)}
    else:
        endpoint = endpoint_cfg
    instance_name = config["core"]["cfg"].get("my_name", config["my_name"])
    topic_prefix = f"mpfc/{instance_name}"
    bus_config = {
        "schema_path": str(schema_path),
        "log_file": str(log_file),
        "endpoint": endpoint,
        "topic_prefix": topic_prefix,
    }

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
    print(
        f"[MAIN] runtime_mode={runtime_mode} using config {config_path}:\n{config_dump}",
        flush=True,
    )

    plugins_raw = config["plugins"]
    core_config = config["core"]
    core_name = core_config["name"]
    core_cfg = core_config["cfg"]
    core_module = importlib.import_module(f"flight_cores.{core_name}.{core_name}")
    core_runner = getattr(core_module, "run_core")

    plugin_entries: List[Dict[str, Any]] = []
    for entry in plugins_raw:
        plugin_name = entry["plugin"]
        plugin_cfg = entry["cfg"]
        plugin_module = importlib.import_module(f"plugins.{plugin_name}.{plugin_name}")
        plugin_runner = getattr(plugin_module, "run_plugin")
        plugin_entries.append({"cfg": plugin_cfg, "runner": plugin_runner})

    core_proc = mp.Process(
        target=core_runner,
        args=(core_cfg, bus_config),
        name=f"core-{core_cfg.get('id','?')}",
    )
    core_proc.start()
    plugin_procs = []
    for entry in plugin_entries:
        plugin_cfg = entry["cfg"]
        proc = mp.Process(
            target=entry["runner"],
            args=(plugin_cfg, bus_config),
            name=f"plugin-{plugin_cfg.get('id','?')}",
        )
        proc.start()
        plugin_procs.append(proc)

    all_procs = [proc for proc in [core_proc] + plugin_procs if proc is not None]
    bus_client = connect_bus_client(bus_config, "main")
    bus_client.subscribe("DIAG/#")
    bus_client.subscribe(CONTROL_SHUTDOWN_TOPIC)

    shutdown_requested = False
    shutdown_started_at: float | None = None
    failure_reason: str | None = None
    terminate_requested = False

    def _handle_sigterm(_signum: int, _frame: Any) -> None:
        nonlocal terminate_requested
        terminate_requested = True

    previous_sigterm = signal.signal(signal.SIGTERM, _handle_sigterm)
    try:
        while True:
            try:
                topic, _message = bus_client.receive(timeout=0.2)
            except queue.Empty:
                topic = None

            if terminate_requested:
                shutdown_requested = True
            elif topic == CONTROL_SHUTDOWN_TOPIC:
                shutdown_requested = True
            elif topic and topic.endswith("/ERROR"):
                if failure_reason is None:
                    failure_reason = f"runtime component reported error topic={topic}"
                    print(f"[MAIN] {failure_reason}", flush=True)
                shutdown_requested = True

            dead = [proc for proc in all_procs if not proc.is_alive()]
            if dead and not shutdown_requested:
                names = ", ".join(f"{proc.name}({proc.exitcode})" for proc in dead)
                if (
                    not resident_runtime
                    and core_proc in dead
                    and core_proc.exitcode == 0
                    and all(proc is core_proc or proc.is_alive() for proc in all_procs)
                ):
                    print(
                        f"[MAIN] mission core completed: {core_proc.name}(0); terminating remaining",
                        flush=True,
                    )
                else:
                    failure_reason = f"unexpected runtime component exit: {names}"
                    print(f"[MAIN] {failure_reason}; terminating remaining", flush=True)
                shutdown_requested = True

            if shutdown_requested and shutdown_started_at is None:
                shutdown_started_at = time.monotonic()
                try:
                    bus_client.publish(
                        CONTROL_SHUTDOWN_TOPIC,
                        build_envelope("main", CONTROL_SHUTDOWN_TOPIC, {}),
                    )
                except MqttPublishError:
                    pass

            if not shutdown_requested:
                continue

            alive = [proc for proc in all_procs if proc.is_alive()]
            if not alive:
                break
            if (
                shutdown_started_at is not None
                and time.monotonic() - shutdown_started_at > 5.0
            ):
                for proc in alive:
                    proc.terminate()
                break
    except KeyboardInterrupt:
        print("[MAIN] interrupt received, terminating", flush=True)
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        alive = [proc for proc in all_procs if proc.is_alive()]
        if alive:
            try:
                bus_client.publish(
                    CONTROL_SHUTDOWN_TOPIC,
                    build_envelope("main", CONTROL_SHUTDOWN_TOPIC, {}),
                )
            except MqttPublishError:
                pass
            deadline = time.monotonic() + 5.0
            while alive and time.monotonic() < deadline:
                time.sleep(0.1)
                alive = [proc for proc in all_procs if proc.is_alive()]
        try:
            bus_client.close()
        except Exception:
            pass
        for proc in all_procs:
            if proc.is_alive():
                proc.terminate()
            proc.join(timeout=5.0)

    if failure_reason is not None:
        raise RuntimeError(failure_reason)


def main() -> None:
    config_path = Path(os.environ[CONFIG_ENV]).resolve()
    overrides = parse_runtime_overrides(sys.argv[1:])
    start_from_config(config_path, overrides)


if __name__ == "__main__":
    main()
