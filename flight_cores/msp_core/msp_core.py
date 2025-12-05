#!/usr/bin/env python3
"""
MSP mode plugin for main_uav: publishes configured MSP requests and logs MSP replies.
"""

import time
from typing import Any, Dict, List


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    raise NotImplementedError("msp_core not implemented")
