#!/usr/bin/env python3
"""
Usage:
    python examples/run_config_mavsdk.py
"""

import os
from pathlib import Path

import main

CONFIG_PATH = Path(__file__).resolve().parent.parent / "flight_cores" / "test_takeoff_land" / "config.json"


if __name__ == "__main__":
    os.environ["MAIN_CONFIG"] = str(CONFIG_PATH)
    main.main()
