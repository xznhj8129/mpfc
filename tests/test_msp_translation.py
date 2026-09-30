#!/usr/bin/env python3
"""Unit tests for the MSP translation layer and the INAV platform config."""

from pathlib import Path

from plugins.msp_interface.msp_interface import global_target_params
from protocols.namespace_loader import load_protocol_namespace


def test_global_target_params_maps_protocol_fields() -> None:
    params = global_target_params(
        {"Latitude": 36.530440, "Longitude": -83.216383, "AltitudeM": 60.0}
    )
    assert params == {
        "latitude_deg": 36.530440,
        "longitude_deg": -83.216383,
        "altitude_m": 60.0,
    }


def test_protocol_declares_nav_target_and_actions() -> None:
    uav = load_protocol_namespace("uav")
    assert uav.State.Navigation.Target == "Target"
    assert uav.Action.Navigation.GoTo == "GoTo"
    assert uav.Action.Flight.Arm == "Arm"
    assert uav.Action.Flight.Takeoff == "Takeoff"
    assert uav.Action.Flight.Land == "Land"
    assert uav.Action.Flight.Rtl == "Rtl"


def test_inav_config_resolves_to_msp_backend() -> None:
    import main as mpfc_main

    config = mpfc_main.load_config(Path("flight_cores/lattice_demo/config_inav_sitl.yaml"))
    mpfc_main.apply_plugin_config_templates(config, Path("."))
    placeholders: dict = {}
    mpfc_main.collect_named_scalars(config, placeholders)
    config = mpfc_main.resolve_placeholders(config, placeholders)
    vehicle = mpfc_main.resolve_vehicle_config(config)
    mpfc_main.resolve_plugin_supports(config)
    mpfc_main.configure_runtime_plugins(config, vehicle)

    core_cfg = config["core"]["cfg"]
    assert core_cfg["interface"] == {"id": "uav_controller", "topic_ns": "UAV"}
    assert vehicle == {
        "autopilot": "INAV",
        "uav_type": "Copter",
        "telem_type": "MSPV2",
    }
    controller_cfg = [p["cfg"] for p in config["plugins"] if p["plugin"] == "uav_controller"][0]
    assert controller_cfg["backend"] == {"id": "mspapi", "topic_ns": "UAV"}
    msp_cfg = [p["cfg"] for p in config["plugins"] if p["plugin"] == "msp_interface"][0]
    assert msp_cfg["conn_type"] == "tcp"
    assert msp_cfg["conn_str"] == "127.0.0.1:5761"
    assert msp_cfg["state_intervals"]["Target"] == 0.5
