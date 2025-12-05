"""
Shared action identifiers for the MAVSDK interface and coordinating cores.
"""

from typing import Dict, Any

ACTION_IS_IN_AIR = "is_in_air"
ACTION_IS_ARMED = "is_armed"
ACTION_HOME_OK = "is_home_position_ok"
ACTION_GLOBAL_OK = "is_global_position_ok"
ACTION_GET_ALTITUDE = "get_altitude"
ACTION_GET_POSITION = "get_position"
ACTION_ARM = "arm"
ACTION_SET_TAKEOFF_ALTITUDE = "set_takeoff_altitude"
ACTION_TAKEOFF = "takeoff"
ACTION_LAND = "land"


def build_action_request(request_id: str, action: str, params: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return {"request_id": request_id, "action": action, "params": params or {}}
