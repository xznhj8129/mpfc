"""
Example Hello World core
Listens to plugins "hello" and sends "hi" to each
"""

import queue
import time
from typing import Any, Dict

from lib.common import build_envelope, connect_bus_client

HELLO_TOPIC = "hello"


def run_core(bus_config: Dict[str, Any], cfg: Dict[str, Any]) -> None:
    client_id = cfg.get("id")
    if not client_id:
        raise ValueError("core cfg missing id")
    client = connect_bus_client(bus_config, client_id)
    diag_ping_topic = f"Diag.{client_id}.PING"
    diag_pong_topic = f"Diag.{client_id}.PONG"
    diag_online_topic = f"Diag.{client_id}.ONLINE"
    diag_stopped_topic = f"Diag.{client_id}.STOPPED"
    client.subscribe(HELLO_TOPIC)
    client.subscribe(diag_ping_topic)
    online_payload = build_envelope(client_id, diag_online_topic, {"event": "ONLINE"})
    client.publish(diag_online_topic, online_payload)
    print(f"[CORE_ONLINE] id={client_id} hello_topic={HELLO_TOPIC}", flush=True)
    try:
        while True:
            try:
                message, raw = client.receive(timeout=1.0)
            except queue.Empty:
                continue
            if not isinstance(message, dict):
                raise TypeError("bus message must be a dictionary")
            if "topic" not in message:
                raise KeyError("bus message missing topic")
            if "payload" not in message:
                raise KeyError("bus message missing payload")
            topic = message["topic"]
            payload = message["payload"]
            if not isinstance(payload, dict):
                raise TypeError("bus message payload must be a dictionary")
            if topic == diag_ping_topic:
                pong_payload = build_envelope(
                    client_id, diag_pong_topic, {"ping_time": payload.get("time"), "pong_time": int(time.time() * 1000)}
                )
                client.publish(diag_pong_topic, pong_payload)
                print(f"[CORE_PONG] id={client_id} ping_time={payload.get('time')}", flush=True)
                continue
            if topic != HELLO_TOPIC:
                print(f"[CORE_UNHANDLED] topic={topic} raw={raw}", flush=True)
                continue
            sender_id = payload.get("client")
            if not isinstance(sender_id, str) or not sender_id:
                raise ValueError(f"hello payload missing client id raw={raw}")
            reply_topic = f"{sender_id}.hello"
            reply_envelope = build_envelope(client_id, reply_topic, {"message": "hi"})
            client.publish(reply_topic, reply_envelope)
            print(
                f"[CORE_HELLO] from={sender_id} reply_topic={reply_topic} data={payload.get('data')} raw={raw}",
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        stopped_payload = build_envelope(client_id, diag_stopped_topic, {"event": "STOPPED"})
        client.publish(diag_stopped_topic, stopped_payload)
        client.close()
        print(f"[CORE_STOP] id={client_id}", flush=True)
