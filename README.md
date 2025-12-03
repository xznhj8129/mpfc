# HiveOS unified bus bindings

- Message bus is the sole broker; every module registers a hardcoded client id and uses JSON envelopes described in `bus_topics_schema.json`.
- New entrypoints (all read `process_config.json`; no CLI arguments):
- `main_uav.py` supports two modes (configured in `process_config.json`):
  - `msp`: publishes MSP requests (`msp.requests` list of `{op, data}`) and listens for `MSP.REPLY` plus `Datalink.IN`.
  - `mavlink`: connects to ArduPilot (e.g., SITL via `mavlink.conn_str`), consumes `Datalink.IN` commands (`Command.AP.*`), and publishes `Status.AP.HL_TELEM` over `Datalink.OUT` at `mavlink.hl_rate_hz` using `datalink.destination`/`transport`.
- `hivelink_interface.py` bridges HiveLink `DatalinkInterface` traffic to bus topics `Datalink.OUT` → `Datalink.IN`.
- `uav_interface.py` (stub) subscribes to base `MSP.REQUEST` traffic; MSP transport wiring is pending.
- Envelope template: `{"client": "<id>", "topic": "<bus topic>", "time": <epoch_ms>, "data": <payload>}`.
- MSP prefixes (`MSP.REQUEST` / `MSP.REPLY`) are documented; requests use an `op` field inside the message data to choose the operation.
- Common process configuration lives in `process_config.json` (bus config path, schema path, nodes path, per-process MSP request list, datalink bindings, and MAVLink settings).
- Hardcoded client ids: `main_uav.py` -> `main_uav`, `uav_interface.py` -> `FlightController`, `hivelink_interface.py` -> `hivelink`.
- MAVLink mode config keys (under `main_uav`): `mode`=`mavlink`, `mavlink.conn_str` (e.g., `udp:127.0.0.1:14550` for ArduPilot SITL), `mavlink.hl_rate_hz` (>0), `datalink.destination` (node id or empty for broadcast), `datalink.transport` (`udp`/`meshtastic`/`multicast`).
- MAVLink mode requires `pymavlink` available in the environment.

## Run order
- Start the message bus: `python message_bus.py --config message_bus_config.json`.
- Launch the HiveLink bridge: `python hivelink_interface.py`.
- Launch the MSP/Mavlink bridge (currently raises `NotImplementedError` on MSP requests): `python uav_interface.py`.
- Launch the UAV process in the desired mode (`msp` or `mavlink`) as configured: `python main_uav.py`.
- ArduPilot SITL test (GUIDED/AUTO control, HL telemetry): set `main_uav.mode` to `mavlink`, `mavlink.conn_str` to the SITL endpoint (e.g., `udp:127.0.0.1:14550`), ensure `datalink.destination`/`transport` are valid, then run `python main_uav.py`.
- Use `example_unified_usage.py` to see prebuilt envelopes without contacting the bus; it reads `process_config.json` for the request topic.
