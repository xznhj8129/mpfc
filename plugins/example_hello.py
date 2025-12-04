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

        if self.send_interval is None:
            raise ValueError("cfg missing send_interval")
        if self.send_interval <= 0:
            raise ValueError("send_interval must be positive")

        self.my_topic = f"{self.client_id}.hello"
        self.client.subscribe(self.my_topic)

    def run(self) -> None:
        
        next_send = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                if now >= next_send:
                    envelope = build_envelope(self.client_id, HELLO_TOPIC, {"message": "hello"})
                    self.client.publish(HELLO_TOPIC, envelope)
                    print(
                        f"[PLUGIN_HELLO] id={self.client_id} topic={HELLO_TOPIC} payload={json.dumps(envelope, separators=(',', ':'))}",
                        flush=True,
                    )
                    next_send = now + self.send_interval

                topic, payload, message = self.recv_until(next_send)

                if topic is None:
                    continue
                if topic == self.my_topic:
                    sender = payload.get("client") or message.get("src")
                    print(
                        f"[PLUGIN_RECV] id={self.client_id} reply_topic={topic} src={sender} data={payload.get('data')}",
                        flush=True,
                    )
                    continue
                
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
    HelloPlugin(cfg, bus_config)
