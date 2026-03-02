# ATAK Interface Plugin

Bridges ATAK CoT traffic (UDP/TCP) into HiveOS bus topics and accepts bus requests with parsed JSON payloads.

## Bus Topics

Base: `<id>/<topic_ns>`

- REQUEST: `<id>/<topic_ns>/REQUEST`
- RESPONSE: `<id>/<topic_ns>/RESPONSE`
- STATE: `<id>/<topic_ns>/STATE/<Key>`
- EVENT: `<id>/<topic_ns>/EVENT/<Key>`
- `ATAK.State.*`: `LastEvent`, `LastResult`, `LastError`, `RxCount`, `TxCount`, `RxParseErrors`, `TcpClientConnected`
- `ATAK.Event.*`: `ReceivedEvent`, `ReceivedMarker`, `ReceivedGeoChat`, `ParseError`

## REQUEST Actions

- `SendEvent`
  - params:
    - `Uid`: string
    - `Type`: string
    - `How`: string
    - `PositionLatDeg`: float
    - `PositionLonDeg`: float
    - `PositionHaeM`: float
    - `PositionCe`: float
    - `PositionLe`: float
    - `Time` (optional): ISO datetime
    - `Start` (optional): ISO datetime
    - `Stale` (optional): ISO datetime
    - `Callsign` (optional): string
    - `Targets` (optional): list of `{host, port}`
  - response data:
    - `TargetCount`: int
    - `Bytes`: int

- `SendMarker`
  - params:
    - `Callsign`: string
    - `Uid`: string
    - `CotType`: string
    - `PositionLatDeg`: float
    - `PositionLonDeg`: float
    - `PositionAltM`: float
    - `PositionCe`: float
    - `PositionLe`: float
    - `StaleSeconds` (optional): int
    - `Targets` (optional): list of `{host, port}`
  - response data:
    - `TargetCount`: int
    - `Bytes`: int

- `SendGeoChat`
  - params:
    - `Message`: string
    - `ToTeam`: string
    - `PositionLatDeg`: float
    - `PositionLonDeg`: float
    - `PositionAltM`: float
    - `PositionCe`: float
    - `PositionLe`: float
    - `Targets` (optional): list of `{host, port}`
  - response data:
    - `TargetCount`: int
    - `Bytes`: int

## State/Event Emission

- `LastEvent` state and `ReceivedEvent` event are published when inbound CoT parses successfully.
- `LastResult` and `TxCount` are published after outbound CoT sends.
- `RxParseErrors` and `LastError` states plus `ParseError` event are published on parse failures.
- `TcpClientConnected` is published when TCP client link state changes.

## Dependencies

- `frogcot`

## Notes

- Outbound ATAK endpoints are static runtime config: `cfg.cot_output_targets`.
- Changing targets requires editing config and restarting.
- `translator.self_callsign` is used to initialize `frogcot.ATAKClient(...)` for ATAK-originated messages (marker/geochat).
