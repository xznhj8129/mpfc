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
* flight_core: main setup, state control and loop specific to UAV type (MSP or Mavlink)
* user_plugin: User provided code connected to bus and exchanges information, can be blocking

See config/config_*.json for examples and structure:

* is_interface: Plugin designated to be the interface to specific external devices (ie: serial to flight controller)
* is_datalink: Plugin designated to be router or proxy that exchanges messages wirelessly, can be Hivelink or any other form of interface, 

Flight cores:
* mavlink_core: MAVLink-specific core flight code
* msp_core: INAV-specific core flight code

Base plugins:
* msp_interface: Bus-connected MSP Flight Controller interface using MSPAPI2, tcp or serial
* mav_interface: Bus-connected Mavlink interface
* hivelink_interface: Bus-connected Hivelink interface

Common characteristics:
- All cores and plugins have a configurable `id` for IPC
- All cores and plugins can be configured from their call from the main config with the `cfg` or `cfg_path` key. Same exact schema, but can be in main config or split, or combined.
- main config -> load cfg_path if provided -> add cfg fields if provided -> pass to plugin
- JSON schema of topics and payload fields lives in `bus_topics_schema.json`.

# Messaging schematic
- Core envelope: `client` (sender id), `topic` (bus route), `time` (epoch ms), `data` (topic payload).
- Flows:
  * MSP mode: `main_uav.py` → `MSP.REQUEST.x` (`data`) → bus
  * bus → `msp_interface` (stub) 
  * `MSP.REPLY.x` → bus → `main_uav.py`.



