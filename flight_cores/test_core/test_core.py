"""
Example Hello World core
Listens to plugins "hello" and sends "hi" to each
Usage:
    from flight_cores.test_core.test_core import run_core
    run_core(cfg, bus_config)
"""

import queue
import time
from typing import Any, Dict
import json

from lib.common import build_envelope
from lib.core_base import CoreBase

HELLO_TOPIC = "hello"


class HelloCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.send_interval = float(cfg.get("send_interval"))
        self.my_topic = f"{self.client_id}.hello"
        self.client.subscribe(self.my_topic)

    def run(self) -> None:
        clients = ["testclient1", "testclient2"]
        try:
            while True:
                for c in clients:
                    t = f"{c}.{HELLO_TOPIC}"
                    envelope = build_envelope(self.client_id, t, {"message": f"hello {c}"})
                    self.client.publish(t, envelope)
                    print(
                    f"[CORE] id={self.client_id} topic={t} payload={json.dumps(envelope, separators=(',', ':'))}",
                    flush=True,
                )
                
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    HelloCore(cfg, bus_config).run()
