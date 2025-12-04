"""
Example Hello World core
Listens to plugins "hello" and sends "hi" to each
"""

import queue
import time
from typing import Any, Dict
import json

from lib.common import build_envelope, connect_bus_client
from lib.core_base import CoreBase

HELLO_TOPIC = "hello"


class HelloCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.send_interval = float(cfg.get("send_interval"))

        if self.send_interval is None:
            raise ValueError("cfg missing send_interval")
        if self.send_interval <= 0:
            raise ValueError("send_interval must be positive")

        self.my_topic = f"{self.client_id}.hello"
        self.client.subscribe(self.my_topic)

    def run(self) -> None:
        clients = ["testclient1", "testclient2"]
        client = connect_bus_client(self.bus_config, self.client_id)
        try:
            while True:
                for c in clients:
                    t = f"{c}.{HELLO_TOPIC}"
                    envelope = build_envelope(self.client_id, t, {"message": "hello"})
                    self.client.publish(HELLO_TOPIC, envelope)
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