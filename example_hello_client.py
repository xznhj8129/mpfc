#!/usr/bin/env python3
import argparse
import json
import time

from message_bus_client import BusClientSync

SEND_INTERVAL = 1.0


def connect(args: argparse.Namespace) -> BusClientSync:
    if args.tcp:
        parts = args.tcp.split(":")
        if len(parts) != 2 or not parts[0] or not parts[1]:
            raise ValueError("--tcp must be host:port")
        host, port_text = parts
        port = int(port_text)
        if port < 1 or port > 65535:
            raise ValueError("tcp port must be 1-65535")
        return BusClientSync.connect_tcp(host, port, args.id)
    if not args.unix:
        raise ValueError("either --tcp or --unix is required")
    return BusClientSync.connect_unix(args.unix, args.id)


def run_client(args: argparse.Namespace) -> None:
    client = connect(args)
    my_topic = f"{args.id}.hello"
    dest_topic = f"{args.dest}.hello"
    client.subscribe(my_topic)
    print(f"[SUB] topic={my_topic}", flush=True)

    next_send = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if now >= next_send:
                payload = {"from": args.id, "data": "hello"}
                client.publish(dest_topic, payload)
                print(f"[SEND] topic={dest_topic} payload={json.dumps(payload, separators=(',', ':'))}", flush=True)
                next_send = now + SEND_INTERVAL
            remaining = max(0.0, next_send - time.monotonic())
            try:
                message, raw = client.receive(timeout=remaining)
                print(f"[RECV] raw={raw}", flush=True)
            except Exception:
                # Timeout or reader stop errors fall through to loop for next send.
                pass
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Hello message bus example client (blocking)")
    endpoint = parser.add_mutually_exclusive_group(required=True)
    endpoint.add_argument("--tcp", help="Connect to TCP host:port")
    endpoint.add_argument("--unix", help="Connect to Unix domain socket path")
    parser.add_argument("--id", required=True, help="Client id")
    parser.add_argument("--dest", required=True, help="Destination client id for hello topic")
    args = parser.parse_args()
    run_client(args)


if __name__ == "__main__":
    main()
