# HiveOS unified bus bindings

# User-provided guidance
Program process structure

. main: Loads and starts processes as configured in `process_config.json` selected via env `MAIN_CONFIG`, maintains bus alive, silent master controller. 
├── message_bus
├── core
│   ├── flight_core
└── plugins

Processes:
* message_bus: Sole broker, every module registers a hardcoded client id and uses JSON envelopes described in `bus_topics_schema.json`.
* core: Controller program that loads a flight core plugin within itself
* flight_core: main central client, setup, state control and loop specific to UAV type (MSP or Mavlink)
* user_plugin: User provided client code connected to bus and exchanges information, can be blocking

See config/config_*.json for examples and structure (mavsdk_core: config/config_mavsdk.json):

* is_interface: Plugin designated to be the interface to specific external devices (ie: serial to flight controller)
* is_datalink: Plugin designated to be router or proxy that exchanges messages wirelessly, can be Hivelink or any other form of interface, 

Flight cores:
* mavlink_core: MAVLink-specific core flight code
* mavsdk_core: MavSDK-driven core that connects directly to the autopilot on-device
* msp_core: INAV-specific core flight code
* test_core: non-UAV test/debug code
Cores control the master on/off switch. They can send a Control.Shutdown message that all clients will take as a sign to shut down cleanly. Cores encountering an unrecoverable error will also send Control.Shutdown.

Base plugins:
* msp_interface: Bus-connected MSP Flight Controller interface using MSPAPI2, ip or serial
* mav_interface: Bus-connected Mavlink interface that bridges Mavlink between the bus and an external interface (ip or serial), internally transacting parsed frames and encoded frames outwards
* hivelink_interface: Bus-connected Hivelink interface

Core characteristics:
- Cores must figure out if they have successfully started or failed within 5 seconds, or send a Diag.<their_id>.FAIL and stop (Control.Shutdown).

Common characteristics:
- All clients have a configurable `id` for IPC
- All clients can be configured from the main config with the `cfg` or `cfg_path` key. Same exact schema, but can be in main config or split, or combined.
- main config -> load cfg_path if provided -> add cfg fields if provided -> pass to plugin
- JSON schema of topics and payload fields lives in `bus_topics_schema.json`.
- All clients will respond to a Diag.<their_id>.PING with a Diag.<their_id>.PONG ASAP
- All clients upon starting and connecting send a Diag.<id>.ONLINE broadcast
- All clients upon encountering a recovered exception send a Diag.<id>.ERROR broadcast with data=JSON exception traceback
- All clients upon encountering an unrecoverable exception send a Diag.<id>.CRASH broadcast with data=JSON exception traceback
- All clients upon normal shutdown send a Diag.<id>.STOPPED broadcast
