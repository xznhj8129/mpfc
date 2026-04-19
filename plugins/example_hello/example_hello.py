#!/usr/bin/env python3
"""
Usage:
    from plugins.example_hello.example_hello import run_plugin
    run_plugin(cfg, bus_config)
"""

import time
import traceback
from typing import Any, Dict

from lib.common import build_envelope, build_event_topics, build_request_topic, build_response_topic, build_state_topics, build_topic_base
from lib.plugin_base import PluginBase
from protocols.namespace_loader import load_protocol_namespace

HELLO = load_protocol_namespace("hello")


class HelloPlugin(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.send_interval = float(cfg["send_interval"])
        self.topic_ns = cfg["topic_ns"]
        self.poll_interval_s = float(cfg["poll_interval_s"])
        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.state_topics = build_state_topics(base, [HELLO.State.Hello.LastReply])
        self.event_topics = build_event_topics(base, [HELLO.Event.Hello.Pong])
        self.client.subscribe(self.request_topic)

    def run(self) -> None:
        self.send_online()
        deadline = time.monotonic() + self.send_interval

        try:
            while True:
                topic, payload = self.recv_until(deadline)
                self.flush_queue(self.response_queue, self.response_topic)
                if topic is None:
                    deadline = time.monotonic() + self.send_interval
                    continue

                if topic == self.request_topic:
                    sender = payload["client"]
                    request = payload["data"]
                    request_id = request["request_id"]
                    action = request["action"]
                    params = request["params"]
                    if action != HELLO.Action.Hello.Ping:
                        self.enqueue_response(request_id, action, False, {"error": f"unknown action {action}"})
                        self.flush_queue(self.response_queue, self.response_topic)
                        deadline = time.monotonic() + self.send_interval
                        continue

                    reply = {
                        "Sender": self.client_id,
                        "Kind": HELLO.Enums.PingPong.PONG,
                        "Message": f"{params['Message']} -> {HELLO.Enums.PingPong.PONG}",
                    }
                    self.client.publish(
                        self.state_topics[HELLO.State.Hello.LastReply],
                        build_envelope(self.client_id, self.state_topics[HELLO.State.Hello.LastReply], reply),
                    )
                    self._publish_event(HELLO.Event.Hello.Pong, reply)
                    self.enqueue_response(request_id, action, True, reply)
                    self.flush_queue(self.response_queue, self.response_topic)
                    print(
                        f"[{self.client_id}] request_id={request_id} sender={sender} "
                        f"kind_in={params['Kind']} kind_out={reply['Kind']} message={reply['Message']}",
                        flush=True,
                    )
                    deadline = time.monotonic() + self.send_interval
                    continue
                deadline = time.monotonic() + self.send_interval

        except RuntimeError:
            error_topic = f"DIAG/{self.client_id}/ERROR"
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
