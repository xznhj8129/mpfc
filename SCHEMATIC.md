# Messaging schematic
- Core envelope: `client` (sender id), `topic` (bus route), `time` (epoch ms), `data` (topic payload).
- Message bus routes exact `topic` strings; no wildcards.
- Flows:
  - `main_uav.py` → `MSP.REQUEST` with `data.op` selecting operation → bus → `uav_interface.py` (stub) → `MSP.REPLY` (echoing `data.op`) → bus → `main_uav.py`.
  - Any publisher → `Datalink.OUT` (unencoded HiveLink payload) → bus → `hivelink_interface.py` → encodes → `DatalinkInterface.send`.
  - HiveLink radios → `DatalinkInterface.receive` → `hivelink_interface.py` → `Datalink.IN` (decoded payload) → bus → subscribers.
- JSON schema of topics and payload fields lives in `bus_topics_schema.json`.
- MSP prefixes (`MSP.REQUEST` / `MSP.REPLY`) are listed; operations are selected via the `op` field inside the message data.
- All entrypoints read `process_config.json` (bus config path, schema path, nodes path, per-process topics and datalink bindings); client ids are hardcoded in code, not supplied via CLI.
