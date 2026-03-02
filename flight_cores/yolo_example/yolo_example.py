#!/usr/bin/env python3
"""
Minimal core that subscribes to yolo_detector detections and prints them.

Usage:
    from flight_cores.yolo_example.yolo_example import run_core
    run_core(cfg, bus_config)
"""

from typing import Any, Dict

from lib.common import build_topic_base
from lib.core_base import CoreBase


class YoloExampleCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.poll_interval_s = float(cfg["poll_interval_s"])
        interface_cfg = cfg["interface"]
        interface_id = interface_cfg["id"]
        interface_ns = interface_cfg["topic_ns"]
        base = build_topic_base(interface_id, interface_ns)

        self.det_topic = f"{base}/STATE/detections"
        self.count_topic = f"{base}/STATE/detection_count"
        self.fps_topic = f"{base}/STATE/inference_fps"

        self.init_bus(self.poll_interval_s)
        self.bus.add_state_topic(self.det_topic, "detections")
        self.bus.add_state_topic(self.count_topic, "detection_count")
        self.bus.add_state_topic(self.fps_topic, "inference_fps")

    def run(self) -> None:
        self.send_online()
        try:
            while True:
                topic, _payload = self._pump_once()
                if topic == self.det_topic:
                    det = self.state.get("detections")
                    print(f"[CORE] {self.client_id} detection: {det}", flush=True)
                elif topic == self.count_topic:
                    count = self.state.get("detection_count")
                    fps = self.state.get("inference_fps", "?")
                    print(f"[CORE] {self.client_id} count={count} fps={fps}", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    YoloExampleCore(cfg, bus_config).run()
