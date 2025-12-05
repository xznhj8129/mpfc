#!/usr/bin/env python3
import json
import time
import traceback
from typing import Any, Dict

from lib.common import build_envelope
from lib.plugin_base import PluginBase

HELLO_TOPIC = "hello"


class HelloPlugin(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.send_interval = float(cfg.get("send_interval"))
        self.my_topic = f"{self.client_id}.hello"
        self.client.subscribe(self.my_topic)

    def run(self) -> None:
        deadline = time.monotonic() + self.send_interval
        try:
            while True:
                topic, payload, message = self.recv_until(deadline)
                if topic is None:
                    deadline = time.monotonic() + self.send_interval
                    continue
                if topic == self.my_topic:
                    sender = payload.get("client") or message.get("src")
                    reply_topic = f"{sender}.{HELLO_TOPIC}"
                    envelope = build_envelope(self.client_id, reply_topic, {"message": f"hello {sender}"})
                    self.client.publish(reply_topic, envelope)
                    print(
                        f"[{self.client_id}] Replied: reply_topic={reply_topic} src={sender} data={payload.get('data')}",
                        flush=True,
                    )
                    deadline = time.monotonic() + self.send_interval
                    continue
                deadline = time.monotonic() + self.send_interval
        except RuntimeError:
            error_topic = f"DIAG.{self.client_id}.ERROR"
            error_payload = build_envelope(
                self.client_id, error_topic, {"event": "ERROR", "traceback": traceback.format_exc().strip()}
            )
            self.client.publish(error_topic, error_payload)
            raise
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    HelloPlugin(cfg, bus_config).run()
