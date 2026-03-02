#!/usr/bin/env python3
"""
Usage:
    from flight_cores.atak_example.atak_example import run_core
    run_core(cfg, bus_config)
"""

from typing import Any, Dict

from lib.common import build_state_topics, build_topic_base
from lib.core_base import CoreBase
from protocols.namespace_loader import load_protocol_namespace

ATAK = load_protocol_namespace("atak")


class AtakExampleCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.poll_interval_s = float(cfg["poll_interval_s"])
        interface_cfg = cfg["interface"]
        interface_id = interface_cfg["id"]
        interface_ns = interface_cfg["topic_ns"]
        base = build_topic_base(interface_id, interface_ns)
        state_topics = build_state_topics(base, [ATAK.State.Rx.LastEvent])
        self.init_bus(self.poll_interval_s, state_topics)
        self.last_event_topic = state_topics[ATAK.State.Rx.LastEvent]

    def run(self) -> None:
        self.send_online()
        try:
            while True:
                topic, _payload = self._pump_once()
                if topic == self.last_event_topic:
                    event = self.state.get(ATAK.State.Rx.LastEvent)
                    print(
                        f"[CORE] {self.client_id} last_event={event}",
                        flush=True,
                    )
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    AtakExampleCore(cfg, bus_config).run()
