# HiveOS unified bus bindings

- Message bus is the sole broker; every module registers a hardcoded client id and uses JSON envelopes described in `bus_topics_schema.json`.
- New entrypoints (all read `process_config.json`; no CLI arguments):
- `main_uav.py` publishes configured MSP requests (list entries must include `op` and `data`) and listens for MSP replies and `Datalink.IN`.
- `hivelink_interface.py` bridges HiveLink `DatalinkInterface` traffic to bus topics `Datalink.OUT` → `Datalink.IN`.
- `uav_interface.py` (stub) subscribes to base `MSP.REQUEST` traffic; MSP transport wiring is pending.
- Envelope template: `{"client": "<id>", "topic": "<bus topic>", "time": <epoch_ms>, "data": <payload>}`.
- MSP prefixes (`MSP.REQUEST` / `MSP.REPLY`) are documented; requests use an `op` field inside the message data to choose the operation.
- Common process configuration lives in `process_config.json` (bus config path, schema path, nodes path, per-process MSP request list and datalink bindings).
- Hardcoded client ids: `main_uav.py` -> `main_uav`, `uav_interface.py` -> `FlightController`, `hivelink_interface.py` -> `hivelink`.

## Run order
- Start the message bus: `python message_bus.py --config message_bus_config.json`.
- Launch the HiveLink bridge: `python hivelink_interface.py`.
- Launch the MSP/Mavlink bridge (currently raises `NotImplementedError` on MSP requests): `python uav_interface.py`.
- Launch the UAV process that issues configured MSP requests: `python main_uav.py`.
- Use `example_unified_usage.py` to see prebuilt envelopes without contacting the bus; it reads `process_config.json` for the request topic.
