# example_liftoff

`example_liftoff` is a read-only HiveOS core that subscribes to telemetry from `liftoff_interface` and prints periodic snapshots.

It prints:

- link status (`FcConnected`)
- sensor readiness (`SensorConfig`)
- flight mode and in-air status (`FlightMode`, `IsInAir`)
- altitude and attitude (`AltitudeM`, `AttitudeRad`)
- FC-reported normalized control telemetry (`RcTelemetry`)
- power state (`Battery`, `Analog`)

Run:

```bash
MAIN_CONFIG=flight_cores/example_liftoff/config.yaml python main.py
```
