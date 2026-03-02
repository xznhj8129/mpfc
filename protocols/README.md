# Protocol Domains

`protocols/` defines HiveOS domain contracts independently from plugins and transport.

This is an architecture-first namespace layer:
- runtime bus/base code stays generic;
- each protocol domain owns its message contract;
- adapters/plugins map external protocol specifics into these contracts.

## Goals
- Enable multiple protocols in one core without collisions:
  - `UAV.Action.Flight.SetTakeoffAltitude`
  - `CV.State.Tracking.Angles`
  - `ATAK.Event.Marker.ReceivedMarker`
- Keep one shared bus and one shared envelope format.
- Prevent plugin-local message drift by centralizing schemas.
- Enforce explicit field-level contracts (no opaque object blobs).

## Layout
- `registry.json`: protocol catalog and schema locations.
- `schema_format.json`: canonical schema shape.
- `<domain>.json`: protocol contract per domain (for example `uav.json`, `cv.json`, `atak.json`).

## Rules
- `State`, `Action`, and `Event` are explicitly defined.
- `Query` is derived automatically from `State` keys and is not declared per message.
- Message payloads must be fully field-defined; opaque `object` types are not permitted.

## Status
- Namespace loading is wired through `protocols/namespace_loader.py` and used by cores/plugins for `State`, `Action`, `Event`, and derived `Query`.
- Field-level payload validation against schema `Fields` is not wired yet.
