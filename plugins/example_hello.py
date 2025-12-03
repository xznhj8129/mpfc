#!/usr/bin/env python3
import json
import queue
import time
from typing import Any, Dict

from lib.common import build_envelope, connect_bus_client

HELLO_TOPIC = "hello"


def run_plugin(bus_config: Dict[str, Any], cfg: Dict[str, Any]) -> None:
    client_id = cfg.get("id")
    interval = cfg.get("send_interval")
    if not client_id:
        raise ValueError("plugin cfg missing id")
    if interval is None:
        raise ValueError("plugin cfg missing send_interval")
    send_interval = float(interval)
    if send_interval <= 0:
        raise ValueError("send_interval must be positive")

    client = connect_bus_client(bus_config, client_id)
    my_topic = f"{client_id}.hello"
    diag_ping_topic = f"Diag.{client_id}.PING"
    diag_pong_topic = f"Diag.{client_id}.PONG"
    diag_online_topic = f"Diag.{client_id}.ONLINE"
    diag_stopped_topic = f"Diag.{client_id}.STOPPED"
    client.subscribe(my_topic)
    client.subscribe(diag_ping_topic)
    online_payload = build_envelope(client_id, diag_online_topic, {"event": "ONLINE"})
    client.publish(diag_online_topic, online_payload)
    print(f"[PLUGIN_ONLINE] id={client_id} send_interval={send_interval}s hello_topic={HELLO_TOPIC}", flush=True)
    next_send = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if now >= next_send:
                envelope = build_envelope(client_id, HELLO_TOPIC, {"message": "hello"})
                client.publish(HELLO_TOPIC, envelope)
                print(
                    f"[PLUGIN_HELLO] id={client_id} topic={HELLO_TOPIC} payload={json.dumps(envelope, separators=(',', ':'))}",
                    flush=True,
                )
                next_send = now + send_interval
            remaining = max(0.0, next_send - time.monotonic())
            try:
                message, raw = client.receive(timeout=remaining)
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
            if topic == my_topic:
                print(
                    f"[PLUGIN_RECV] id={client_id} reply_topic={topic} data={payload.get('data')} src={payload.get('client')}",
                    flush=True,
                )
                continue

            if topic == diag_ping_topic:
                pong_payload = build_envelope(
                    client_id, diag_pong_topic, {"ping_time": payload.get("time"), "pong_time": int(time.time() * 1000)}
                )
                client.publish(diag_pong_topic, pong_payload)
                print(f"[PLUGIN_PONG] id={client_id} ping_time={payload.get('time')}", flush=True)
                continue
            
            print(f"[PLUGIN_UNHANDLED] id={client_id} topic={topic} raw={raw}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stopped_payload = build_envelope(client_id, diag_stopped_topic, {"event": "STOPPED"})
        client.publish(diag_stopped_topic, stopped_payload)
        client.close()
        print(f"[PLUGIN_STOP] id={client_id}", flush=True)
