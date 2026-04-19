/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/hiveos/README.md

# HiveOS

## WIP
This is pre-0.01 initial development and sketching out, a lot will change quickly. A lot of the current architecture, message formats, schema types, networking and else may change dramatically.

**Bus-first modular flight runtime for UAV systems.**

HiveOS decouples mission logic from protocol complexity. Flight cores express linear, readable mission intent while plugins handle the messy details of MAVLink, MSP, YOLO, CoT, and other integrations — all communicating over a structured MQTT message bus.

**Why not ROS**
HiveOS is simple on purpose: one mission core, plugins for protocol stuff, an MQTT bus, and messages you can read directly. No special build maze, no huge framework stack, no graph debugging rabbit hole. This is intended to just work.

---

## Table of Contents

- [Key Concepts](#key-concepts)
- [Architecture](#architecture)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Flight Cores](#flight-cores)
- [Plugins](#plugins)
- [Bus Contract](#bus-contract)
- [Protocol System](#protocol-system)
- [Lifecycle and Supervision](#lifecycle-and-supervision)
- [Shared Runtime API](#shared-runtime-api)
- [Development Guide](#development-guide)
- [Dependencies](#dependencies)

---

## Key Concepts

| Concept | Role |
|---------|------|
| **Flight Core** | Mission sequencing and decision logic. Linear, readable, policy-driven. One per runtime. |
| **Plugin** | Protocol/device translation. Reusable, stateless from the core's perspective. N per runtime. |
| **Bus** | MQTT message bus with structured topics. All processes are peers — no direct calls between them. |
| **Protocol Namespace** | Typed vocabulary (`UAV.Action.Arm`, `UAV.State.Position`, ...) loaded from JSON schemas. |
| **Supervisor** | `main.py` spawns and monitors all processes, triggers graceful shutdown on failures. |

---

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│  main.py (supervisor)                                    │
│  ┌────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Flight Core │  │  Plugin #1   │  │  Plugin #2   │ ... │
│  │ (mission)   │  │ (MAVLink)    │  │ (CoT/ATAK)   │     │
│  └──────┬─────┘  └──────┬───────┘  └──────┬───────┘     │
│         │               │                 │              │
│  ───────┴───────────────┴─────────────────┴──────────    │
│              MQTT Bus  (hiveos/<instance>/...)            │
└──────────────────────────────────────────────────────────┘
```

**Runtime flow:**

1. Load YAML config from `MAIN_CONFIG` env var
2. Apply CLI overrides (`--key=value`) and merge plugin config templates
3. Resolve `<placeholder>` tokens in config values
4. Spawn one core process + N plugin processes
5. Supervise via `DIAG/#` diagnostics — shutdown on crash or error

---

## Conventions
**Right-Hand**
**Axis Positive:**
**Local 2D: RD**
**Local 3D: FRD**
**World 3D: NED**
**Angles: YPR (Radians)**
**Control: AETR**
**Altitudes are always positive up**

---

## Quick Start

**Prerequisites:** Python 3.10+, an MQTT broker (e.g. Mosquitto) running on `localhost:1883`.

```bash
# Install dependencies
pip install -r requirements.txt

# Run the basic hello-world core
MAIN_CONFIG=flight_cores/test_core/config.yaml python main.py
```

### PX4 SITL (MAVLink / MAVSDK)

```bash
# Launch PX4 SITL + autonomous takeoff/land mission
START_PX4_SITL=1 MY_NAME=uav1 MAV_PORT=14550 ./run_mav_example.sh

# With Gazebo world and QGroundControl output
PX4_GZ_WORLD=baylands START_PX4_SITL=1 QGC_MAVLINK_PORT=14551 ./run_mav_example.sh
```

### Container Runtime

Build a multi-arch image with Docker Buildx:

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64,linux/arm/v7 \
  -t hiveos:latest .
```

For Raspberry Pi Zero 2 W and Raspberry Pi 4B, prefer a 64-bit OS and the `linux/arm64` image. Both boards are 64-bit capable. The `linux/arm/v7` build stays available for 32-bit Pi OS, but the Dockerfile intentionally skips MAVSDK and YOLO there because upstream `grpcio` / Torch support is not dependable on 32-bit ARM.

Run the container with host networking so MQTT, MAVLink UDP, ATAK multicast, and HiveLink UDP behave like a native Linux service:

```bash
docker run --rm -it \
  --network host \
  -v /dev/bus/usb:/dev/bus/usb \
  --device /dev/ttyUSB0 \
  --device /dev/ttyACM0 \
  --device /dev/serial0 \
  --device /dev/gpiomem \
  --device /dev/gpiochip0 \
  --device /dev/gpiochip1 \
  -e MAIN_CONFIG=/opt/hiveos/flight_cores/test_core/config.yaml \
  -v "$PWD:/opt/hiveos" \
  hiveos:latest
```

Notes:

- `--network host` is the recommended Linux mode for MAVLink UDP, ATAK multicast, Meshtastic sidecars, and local GCS tools.
- Add or remove `--device` flags to match your hardware. For cameras also pass `/dev/video0` or the specific V4L device.
- If you use `config/mavlink-router/main.conf`, point `plugins/mavsdk_interface/config_template.yaml` or your runtime config at `udp://:14540`, because the bundled router forwards the FC stream to `127.0.0.1:14540`.
- The image starts Mosquitto, `mavlink-routerd`, and `main.py` together by default. Set `HIVEOS_START_MOSQUITTO=0` or `HIVEOS_START_MAVLINK_ROUTER=0` only when you want to use host-managed services instead.

### MSP (iNav)

```bash
MAIN_CONFIG=flight_cores/example_msp/config.yaml python main.py
```

### YOLO Object Detection

```bash
# Run with default webcam (source: "0") — auto-downloads yolov8n weights
MAIN_CONFIG=flight_cores/yolo_example/config.yaml python main.py

# Run on a video file
MAIN_CONFIG=flight_cores/yolo_example/config.yaml python main.py --source=path/to/video.mp4

# Run with custom weights and GPU
MAIN_CONFIG=flight_cores/yolo_example/config.yaml python main.py --weights=yolov8s.pt --device=0
```

### ATAK (Cursor on Target)

```bash
MAIN_CONFIG=flight_cores/atak_example/config.yaml python main.py
```

---

## Configuration

Main config is a YAML file pointed to by the `MAIN_CONFIG` environment variable.

```yaml
my_name: uav1
core:
  name: test_takeoff_land
  cfg:
    id: px4_core
    poll_interval_s: 0.5
    takeoff_altitude_m: 10.0
bus_config:
  schema_path: config/bus_topics_schema.json
  log_file: hivebus.log
  endpoint:
    type: mqtt
    host: 127.0.0.1
    port: 1883
plugins:
  - plugin: mavsdk_interface
    cfg:
      id: mavsdk
      topic_ns: UAV
      conn_str: "udp://:<mav_port>"
```

### Config Processing

1. **Plugin defaults** — each plugin provides `config_template.yaml`; your `cfg` entries are overrides, not full replacements (recursive merge)
2. **CLI overrides** — `--key=value` applies to keys in `core.cfg` and `my_name`; type-cast from existing value type; unknown keys fail immediately
3. **Placeholder substitution** — `<token>` strings are replaced from scalar config values (e.g. `<mav_port>` from `--mav_port=14552`)

---

## Flight Cores

Cores live in `flight_cores/` and inherit from `CoreBase`. Each implements a `run()` method containing linear mission logic.

| Core | Description |
|------|-------------|
| `test_core` | Hello-world example. Sends messages to plugins in a loop. |
| `test_takeoff_land` | Full autonomous mission — takeoff, fly north, change altitude, RTL, land. Uses GPS distance calculation and attitude monitoring with failsafe handling. |
| `example_msp` | Displays a telemetry snapshot from an iNav flight controller (23 state fields). |
| `atak_example` | Monitors and prints incoming ATAK/CoT datalink events. |
| `yolo_example` | Subscribes to YOLO detector plugin and prints individual detections. |

---

## Plugins

Plugins live in `plugins/` and inherit from `PluginBase`. They translate between the standardized HiveOS bus vocabulary and external protocols/devices.

| Plugin | Description |
|--------|-------------|
| `mavsdk_interface` | MAVLink bridge to PX4 autopilots via MAVSDK. Full flight stack: arm, takeoff, RTL, land, goto. Async telemetry with configurable per-field publish rates. |
| `msp_interface` | MSP bridge to iNav flight controllers. Serial/TCP connection, RC override, waypoint management, comprehensive telemetry. |
| `atak_interface` | ATAK/CoT tactical datalink. Dual UDP/TCP support (multicast + unicast), CoT event serialization via `frogcot`. |
| `hivelink_interface` | Multi-transport datalink (UDP, Meshtastic/LoRa, MQTT) using the HiveLink protocol. |
| `yolo_detector` | YOLO object detection pipeline. Runs ultralytics inference on a video source (camera, file, RTSP) and publishes each detection individually with class, confidence, and bounding box. Optional ByteTrack tracking. |
| `example_hello` | Simple echo responder for testing the request/response message pattern. |

---

## Bus Contract

All communication flows through the MQTT bus using JSON envelopes:

```json
{
  "client": "sender_id",
  "topic": "runtime/topic",
  "time": 1770835200000,
  "data": {}
}
```

Topics are prefixed with `hiveos/<instance_name>/` and follow these patterns:

| Pattern | Direction | Purpose |
|---------|-----------|---------|
| `CONTROL/<command>` | Broadcast | Global commands (e.g. `CONTROL/SHUTDOWN`) |
| `SET/<client_id>` | Any → Target | Runtime parameter mutation |
| `<client_id>/<ns>/REQUEST` | Core → Plugin | Action request |
| `<client_id>/<ns>/RESPONSE` | Plugin → Core | Action result |
| `<client_id>/<ns>/STATE/<field>` | Plugin → Bus | Per-field state stream |
| `<client_id>/<ns>/EVENT/<field>` | Plugin → Bus | Event notification |
| `DIAG/<client_id>/<event>` | Any → Supervisor | Lifecycle diagnostics |

---

## Protocol System

Protocol vocabularies are defined in `protocols/*.json` and loaded dynamically via `namespace_loader.py`:

```python
from protocols.namespace_loader import load_protocol_namespace

UAV = load_protocol_namespace("uav")

# Use typed tokens instead of string literals
self.send_action(interface_id, UAV.Action.Arm)
altitude = self.state.get(UAV.State.AltitudeM)
```

### Available Protocols

| Protocol | Schema | Vocabulary |
|----------|--------|------------|
| **UAV** | `protocols/uav.json` | Flight state (position, attitude, battery, GPS), actions (arm, takeoff, land, RTL, goto), sensor data |
| **ATAK** | `protocols/atak.json` | CoT event state (Rx/Tx), actions (send event, send marker), system counters |

Protocol additions start in the JSON schema, then get implemented in plugins. Cores and plugins reference `UAV.Action.*`, `UAV.State.*`, etc. — never raw strings.

---

## Lifecycle and Supervision

Every process publishes lifecycle diagnostics:

```
DIAG/<id>/STARTING  →  DIAG/<id>/ONLINE  →  DIAG/<id>/STOPPED
```

**Supervisor behavior** (`main.py`):

- Monitors `DIAG/#` for all children
- Responds to `DIAG/<id>/PING` with `DIAG/<id>/PONG` (liveness)
- If any child exits unexpectedly or publishes `DIAG/.../ERROR` → initiates shutdown
- Publishes `CONTROL/SHUTDOWN` → waits 5s grace period → SIGTERM remaining children

**Shared base classes** (`lib/`):

- `RuntimeBase` — lifecycle events, shutdown subscription, diag ping/pong
- `CoreBase` — adds `publish_shutdown()`, clean `finish()` with exit code
- `PluginBase` — adds response queue, error publishing, event helpers

---

## Shared Runtime API

Central primitives in `lib/common.py`:

| Category | Functions |
|----------|-----------|
| **Config** | `load_config()`, `apply_cfg()` |
| **Topic Builders** | `build_request_topic()`, `build_response_topic()`, `build_state_topics()`, `build_event_topics()`, `build_set_topic()` |
| **Bus Machinery** | `BusRouter` (state/response subscription, SET dispatch), `connect_bus_client()` |
| **Wait Primitives** | `wait_until(predicate, timeout)`, `wait_for_state(key, value, timeout)`, `pump_for(duration)` |
| **State Scheduling** | `StateScheduler` — per-field rate-limited publishing with thread-safe flush |
| **Geo Utilities** | GPS distance, bearing, vector math, MGRS/UTM conversion, local tangent plane projection (`lib/geo_utils.py`) |

---

## Development Guide

### Project Structure

```
hiveos/
├── main.py                     # Supervisor entrypoint
├── config/
│   └── bus_topics_schema.json  # Topic pattern definitions
├── protocols/
│   ├── registry.json           # Protocol registry
│   ├── namespace_loader.py     # Dynamic protocol binding
│   ├── uav.json                # UAV command/state vocabulary
│   └── atak.json               # ATAK/CoT vocabulary
├── lib/
│   ├── common.py               # Config, topics, bus, waits
│   ├── core_base.py            # CoreBase class
│   ├── plugin_base.py          # PluginBase class
│   ├── state_scheduler.py      # Rate-limited state publisher
│   ├── mqtt_bus_client.py      # MQTT transport layer
│   ├── geo_utils.py            # GPS/cartography utilities
│   └── uav.py                  # PWM scaling helpers
├── flight_cores/
│   ├── test_core/              # Hello-world example
│   ├── test_takeoff_land/      # Autonomous mission example
│   ├── example_msp/            # MSP telemetry example
│   ├── atak_example/           # ATAK monitor example
│   └── yolo_example/           # YOLO detection example
├── plugins/
│   ├── mavsdk_interface/       # PX4/MAVLink bridge
│   ├── msp_interface/          # iNav/MSP bridge
│   ├── atak_interface/         # ATAK/CoT bridge
│   ├── hivelink_interface/     # Multi-transport datalink
│   ├── yolo_detector/          # YOLO object detection
│   └── example_hello/          # Echo test plugin
├── run_mav_example.sh          # PX4 SITL launcher
├── run_example_msp.sh          # MSP example launcher
├── run_example.sh              # Basic example launcher
└── requirements.txt
```

### Adding a New Protocol Token

1. Add the token to the appropriate `protocols/*.json` schema
2. Load it in your core/plugin: `UAV = load_protocol_namespace("uav")`
3. Reference via `UAV.State.YourField` / `UAV.Action.YourAction` — never raw strings

### Writing a New Flight Core

1. Create `flight_cores/<name>/` with a main module and `config.yaml`
2. Subclass `CoreBase` and implement `run()`
3. Use `self.send_action()`, `self.wait_for_state()`, `self.state` for bus interaction

### Writing a New Plugin

1. Create `plugins/<name>/` with a main module and `config_template.yaml`
2. Subclass `PluginBase` and implement `run()`
3. Register state fields with `add_state_topics()` and a `StateScheduler`
4. Handle incoming requests, publish responses via `enqueue_response()`

### Rules of Thumb

- Protocol tokens in `protocols/*.json` first, implementation second
- Reusable wait/state/request patterns go in `lib/`, not duplicated per core
- Mission-specific policy stays in flight cores
- Device/protocol conversion stays in plugins
- Cores should be linear and readable — push complexity downward

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `mavsdk` | MAVLink communication with PX4 autopilots |
| `pymavlink` | MAVLink message definitions and parsing |
| `hivelink` | Multi-transport datalink protocol |
| `frogproto` | Protocol buffer utilities |
| `paho-mqtt` | MQTT client for the message bus |
| `numpy` | Numerical computing (geo math, filtering) |
| `filterpy` | Kalman filtering |
| `simple-pid` | PID controller |
| `PyYAML` | YAML config file parsing |

```bash
pip install -r requirements.txt
```
