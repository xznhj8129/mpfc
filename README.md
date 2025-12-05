# HiveOS

# User-provided guidance
Program process structure

```. hived
├── message_bus
└── children
    ├── core
    ├── plugin
    ├── plugin
    └── plugin
```

## Daemon
* Main always-on supervisor daemon
* Loads and maintains message_bus: Sole broker, socket server + topic router, every module registers a hardcoded client id and uses JSON envelopes described in `bus_topics_schema.json`.
* Loads and spawns client processes from the config selected via env `MAIN_CONFIG`, maintains bus alive, silent master controller. 
* maintains hardware watchdog /dev/watchdog **WHEN RUN WITH --live ARGUMENT TO ENSURE NOT RESTARTING DEV ENVIRONMENT (MY PC)**

## Message bus
* Runs inside hived (not a separate OS process).
* `bus_topics.json`: Valid topics documentation
* `messages.json`: Message format templates

## Clients
* core: Main operational client, setup, state control and loop of intended actions
* plugins: Baseplate or user provided client code connected to bus and exchanges information, can be blocking

### Cores
* Core is a separate child process started by hived.
* Core connects to the bus just like any other client.
* Loads a flight_core plugin (test_core, test_takeoff_land, etc) in-process.
* Core speaks in intents (“what to do”: takeoff, goto, start mission) on generic bus topics.
* Plugins translate those intents into protocol specifics (“how to do it”: MSP, MAVLink, MAVSDK calls).
* Each core lives under `flight_cores/<name>/` with `<name>.py` as the entry module and a base config next to it (e.g. `flight_cores/test_takeoff_land/config.json`).

#### Core layout
```. flight_cores
├── core_name
│   ├── __init__.py
│   ├── core_name.py    - main body
│   ├── config.json     - configuration file
│   └── helper.py       - optional import
```

#### Core characteristics:

* test_core: non-UAV test/debug code
* test_takeoff_land: drives a simple arm/takeoff/land sequence via the MAVSDK interface plugin
Cores control the master on/off switch. They can send a Control.Shutdown message that all clients will take as a sign to shut down cleanly. Cores encountering an unrecoverable error will also send Control.Shutdown.

### Plugins
Every plugin (msp_interface, pymavlink_interface, hivelink_interface, CV, whatever):
Is a separate process started and supervised by hived.
Has its own id and cfg.
Connects to the bus just like the core does.
Can be blocking or ugly internally without dragging down anyone else.

#### Plugin main types
* **Interface**: Plugin designated to be an API/interface to specific external devices (ie: serial to flight controller), translation layer between intent (ie: takeoff) and API and protocol specific to UAV type (ie: Ardupilot, Pixhawk, INAV, Betaflight; MSP or Mavlink.) flagged as `is_interface`
* **Datalink**: Plugin designated to be router or proxy that exchanges messages wirelessly, can be Hivelink or any other form of exchange interface, flagged as `is_datalink`

#### Plugins layout
* Each plugin lives under `plugins/<name>/` with `<name>.py` as the entry module and a `config_template.json` showing required/optional fields.

```. plugins
├── plugin_name
│   ├── __init__.py
│   ├── plugin_name.py  - main body
│   ├── config_template.json
│   └── helper.py       - optional import by plugin
```

* mavsdk_interface: MavSDK-driven interface that connects directly to the autopilot on-device
* msp_interface: Bus-connected MSP Flight Controller interface using MSPAPI2, ip or serial
* pymavlink_interface: Bus-connected Mavlink router interface that bridges Mavlink between the bus and an external interface (ip or serial), internally transacting parsed frames and encoded frames outwards; does not translate
* hivelink_interface: Bus-connected Hivelink interface


### Common characteristics
- All clients have a configurable `client_id` for IPC
- All clients can be configured from the main config with the `cfg` or `cfg_path` key. Same exact schema, but can be in main config or split, or combined.
- main config -> load cfg_path if provided -> add cfg fields if provided -> pass to plugin
- JSON documentation of topics and payload fields lives in `bus_topics_schema.json`

## Health

### Diagnostics
- All clients will respond to a Diag.<their_id>.PING with a Diag.<their_id>.PONG ASAP
- All clients upon starting (start init()) will send a Diag.<id>.STARTING broadcast
- All clients upon reaching ready state will send a Diag.<id>.ONLINE broadcast
- All clients if they exit without ever reaching a ready state will send a Diag.<id>.FAIL broadcast (with data=JSON exception traceback if applicable)
- All clients upon encountering a recovered exception send a Diag.<id>.ERROR broadcast with data=JSON exception traceback
- All clients upon encountering a running unrecoverable exception send a Diag.<id>.CRASH broadcast with data=JSON exception traceback
- All clients upon normal shutdown send a Diag.<id>.STOPPED broadcast
- Datalinks and interfaces, having to connect to an external device or process, must establish and check connection in their initialization phase and report Diag.<their_id>.FAIL if they cannot within their configured timeout

### Supervision
- hived subscribes to all DIAG topics to maintain a health table and drive supervision decisions.
- Cores must figure out if they have successfully started or failed within 5 seconds, or send a Diag.<their_id>.FAIL and stop (Control.Shutdown).

Global Control.Shutdown topic exists for clean system-wide shutdown.
Only core(s) are allowed to emit Control.Shutdown:
hived keeps a list of allowed_shutdown_clients.
If any other client publishes to Control.Shutdown, hived logs and drops it.
All clients subscribe to Control.Shutdown and exit cleanly when they see a valid one.
hived also listens and tears down all children on Control.Shutdown.
