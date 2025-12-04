#!/usr/bin/env python3
"""
MSP mode plugin for main_uav: publishes configured MSP requests and logs MSP replies.
"""

import time
from typing import Any, Dict, List

CLIENT_ID = "FlightController"
MSP_REQUEST_TOPIC = "MSP.REQUEST"
MSP_REPLY_TOPIC = "MSP.REPLY"


class MSPPlugin:
    def __init__(self, client_id: str, requests: List[Dict[str, Any]]) -> None:
        self.client_id = client_id
        self.requests = requests

    def start(self, client) -> None:
        for entry in self.requests:
            op = entry["op"]
            payload_data = entry["data"]
            envelope = {
                "client": self.client_id,
                "topic": MSP_REQUEST_TOPIC,
                "time": int(time.time() * 1000),
                "data": {"op": op, "data": payload_data},
            }
            client.publish(MSP_REQUEST_TOPIC, envelope)
            print(f"[MSP_REQUEST] op={op} topic={MSP_REQUEST_TOPIC} envelope_time_ms={envelope['time']}", flush=True)

    def tick(self, client) -> None:
        return

    def handle_bus_message(self, topic: str, payload: Dict[str, Any], message: Dict[str, Any]) -> bool:
        if topic != MSP_REPLY_TOPIC:
            return False
        data_field = payload.get("data")
        op_name = data_field.get("op") if isinstance(data_field, dict) else None
        print(
            f"[MSP_REPLY] topic={topic} op={op_name} src={message.get('src')} issued={payload.get('time')} data={data_field}",
            flush=True,
        )
        return True

    def stop(self) -> None:
        return
