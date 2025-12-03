#!/usr/bin/env python3
"""
Usage:
    MAIN_UAV_CONFIG=/path/to/process_config_mavlink.json python main_uav.py
    MAIN_UAV_CONFIG=/path/to/process_config_msp.json python main_uav.py
Config path is required via MAIN_UAV_CONFIG; no CLI arguments are used.
"""

import json
import os
import queue
import time
from pathlib import Path
from typing import Any, Dict

from message_bus_client import BusClientSync


def main() -> None:

    # get config here

    plugin = None
    # plugin = get cfg['main_uav']['core']

    plugin.start()

    try:
        # run core here

    finally:
        plugin.stop()
        client.close()


if __name__ == "__main__":
    main()
