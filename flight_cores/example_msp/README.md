# example_msp

`example_msp` is a read-only HiveOS core that mirrors the data-read spirit of `mspapi2/example_api.py`.

It subscribes to standardized UAV status fields from `msp_interface` and prints periodic snapshots:

- FC identity and protocol info (`FcInfo`)
- sensor layout (`SensorConfig`)
- mode/rx mapping (`ModeRanges`, `RxConfig`, `ChannelMap`)
- analog and battery state (`Analog`, `Battery`)
- GPS and navigation (`GpsInfo`, `RawGps`, `GpsStatistics`, `WaypointInfo`, `NavState`)
- attitude and IMU (`AttitudeRad`, `AngVelRadS`, `Imu`)
- RC and active modes (`RcChannels`, `ActiveModeNames`, `FlightMode`)

Run:

```bash
./run_example_msp.sh
```
