#!/usr/bin/env python3
"""Program example that monitors CoT-derived Lattice entity state."""

from __future__ import annotations

from typing import Any, Dict

from anduril import Entity

from lib.common import build_state_topics, build_topic_base
from lib.core_base import CoreBase
from lib.lattice_bus import get_lattice_state
from lib.bus_topics import ENTITY_STATE


class AtakExampleCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.poll_interval_s = float(cfg["poll_interval_s"])
        interface_cfg = cfg["interface"]
        base = build_topic_base(interface_cfg["id"], interface_cfg["topic_ns"])
        state_topics = build_state_topics(base, [ENTITY_STATE])
        self.init_bus(self.poll_interval_s, state_topics)
        self.entity_state_topic = state_topics[ENTITY_STATE]

    def run(self) -> None:
        self.send_online()
        try:
            while True:
                topic, _payload = self._pump_once()
                if topic != self.entity_state_topic:
                    continue
                entity = get_lattice_state(self.state, ENTITY_STATE, Entity)
                print(f"[CORE] {self.client_id} entity_state={entity}", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    AtakExampleCore(cfg, bus_config).run()
