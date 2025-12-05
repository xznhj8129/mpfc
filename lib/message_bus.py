#!/usr/bin/env python3
import argparse
import asyncio
import json
import logging
import signal
import sys
from pathlib import Path
from typing import Dict, Set

MAX_LINE_BYTES = 1_048_576
ENCODING = "utf-8"
LOG_FORMAT = "%(asctime)s %(levelname)s %(message)s"
DEFAULT_CONFIG_PATH = Path(__file__).with_name("message_bus_config.json")
HANDSHAKE_OP = "HANDSHAKE"


class MessageBus:
    def __init__(self, logger: logging.Logger) -> None:
        self.logger = logger
        self.clients: Dict[str, asyncio.StreamWriter] = {}
        self.subscriptions: Dict[str, Set[str]] = {}
        self.lock = asyncio.Lock()

    def _decode_json_line(self, line: bytes) -> tuple[str, Dict]:
        text = line.decode(ENCODING).rstrip("\n")
        parsed = json.loads(text)
        return text, parsed

    async def register_client(self, client_id: str, writer: asyncio.StreamWriter) -> None:
        async with self.lock:
            self.clients[client_id] = writer

    async def unregister_client(self, client_id: str) -> None:
        async with self.lock:
            if client_id in self.clients:
                del self.clients[client_id]
            empty_topics = []
            for topic, subscribers in self.subscriptions.items():
                subscribers.discard(client_id)
                if not subscribers:
                    empty_topics.append(topic)
            for topic in empty_topics:
                del self.subscriptions[topic]

    async def add_subscription(self, client_id: str, topic: str) -> None:
        async with self.lock:
            if topic not in self.subscriptions:
                self.subscriptions[topic] = set()
            self.subscriptions[topic].add(client_id)

    async def publish(self, client_id: str, topic: str, message: Dict, line_text: str) -> None:
        payload = message.get("payload")
        data_display = payload
        if isinstance(payload, dict) and "data" in payload:
            data_content = payload["data"]
            if topic == "MAVLINK":
                data_display = {
                    "msgid": data_content["msgid"],
                    "type": data_content["type"],
                    "sysid": data_content["sysid"],
                    "compid": data_content["compid"],
                    "length": data_content["length"],
                }
            else:
                data_display = data_content
        try:
            data_text = json.dumps(data_display, separators=(",", ":"))
        except (TypeError, ValueError):
            data_text = str(data_display)
        async with self.lock:
            targets = tuple(self.subscriptions.get(topic, ()))
            writers = {target_id: self.clients.get(target_id) for target_id in targets}
        if not writers:
            self.logger.info("DROP %s topic=%s reason=no_subscribers", client_id, topic)
            return
        self.logger.info("PUB %s topic=%s targets=%s data=%s", client_id, topic, targets, data_text)
        outgoing = dict(message)
        outgoing["src"] = client_id
        encoded = json.dumps(outgoing, separators=(",", ":")).encode(ENCODING) + b"\n"
        for target_id, target_writer in writers.items():
            if target_writer is None:
                self.logger.error("SUBSCRIBER_MISSING writer for client=%s topic=%s", target_id, topic)
                continue
            target_writer.write(encoded)
            try:
                await target_writer.drain()
            except (ConnectionResetError, BrokenPipeError) as exc:
                self.logger.error("SEND_FAIL src=%s dst=%s topic=%s error=%s", client_id, target_id, topic, exc)
                await self.unregister_client(target_id)
                continue

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername") or "unix-peer"
        client_id = None
        try:
            try:
                first_line = await reader.readline()

            except asyncio.LimitOverrunError as exc:
                self.logger.error("HANDSHAKE_FAIL peer=%s error=%s", peer, exc)
                return
            if not first_line:
                self.logger.error("HANDSHAKE_FAIL peer=%s error=empty_stream", peer)
                return
            try:
                line_text, handshake_message = self._decode_json_line(first_line)
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                self.logger.error("HANDSHAKE_FAIL peer=%s error=%s", peer, exc)
                return
            if handshake_message.get("op") != HANDSHAKE_OP:
                self.logger.error("HANDSHAKE_FAIL peer=%s error=missing_handshake", peer)
                return
            if "client" not in handshake_message or not isinstance(handshake_message["client"], str) or not handshake_message["client"]:
                self.logger.error("HANDSHAKE_FAIL peer=%s error=missing_client_id", peer)
                return
            client_id = handshake_message["client"]

            await self.register_client(client_id, writer)
            self.logger.info("IN %s %s", client_id, line_text)

            while True:
                try:
                    line = await reader.readline()
                except asyncio.LimitOverrunError as exc:
                    self.logger.error("READ_FAIL client=%s error=%s", client_id, exc)
                    break
                if not line:
                    break
                try:
                    line_text, message = self._decode_json_line(line)
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                    self.logger.error("PARSE_FAIL client=%s error=%s", client_id, exc)
                    break
                op = message.get("op")
                if op == "sub":
                    topic = message.get("topic")
                    await self.add_subscription(client_id, topic)
                    self.logger.info("SUB %s topic=%s", client_id, topic)
                elif op == "pub":
                    topic = message.get("topic")
                    await self.publish(client_id, topic, message, line_text)
                else:
                    self.logger.error("OP_UNKNOWN client=%s raw=%s", client_id, line_text)
        except asyncio.CancelledError:
            if client_id is not None:
                self.logger.info("CANCEL client=%s", client_id)
            else:
                self.logger.info("CANCEL_UNIDENTIFIED peer=%s", peer)
        finally:
            if client_id is not None:
                await self.unregister_client(client_id)
            writer.close()
            await writer.wait_closed()
            if client_id is not None:
                self.logger.info("DISCONNECT client=%s", client_id)
            else:
                self.logger.info("DISCONNECT_UNIDENTIFIED peer=%s", peer)


async def run_server(endpoint: Dict, log_file: str) -> None:
    logger = logging.getLogger("message_bus")
    logger.setLevel(logging.INFO)
    log_handlers = [logging.FileHandler(log_file, encoding=ENCODING)]
    formatter = logging.Formatter(LOG_FORMAT)
    for handler in log_handlers:
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    bus = MessageBus(logger)
    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            # Signals may not be available; rely on task cancellation.
            pass

    if endpoint["type"] == "tcp":
        server = await asyncio.start_server(
            bus.handle_client,
            endpoint["host"],
            endpoint["port"],
            limit=MAX_LINE_BYTES,
        )
        logger.info("LISTEN tcp %s:%d", endpoint["host"], endpoint["port"])
    else:
        server = await asyncio.start_unix_server(bus.handle_client, path=endpoint["path"], limit=MAX_LINE_BYTES)
        logger.info("LISTEN unix %s", endpoint["path"])

    async with server:
        await stop_event.wait()
    await bus.lock.acquire()
    try:
        for client_writer in list(bus.clients.values()):
            client_writer.close()
    finally:
        bus.lock.release()


def main() -> None:
    parser = argparse.ArgumentParser(description="HiveOS internal message bus")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="Path to JSON config file")
    args = parser.parse_args()

    config_path = Path(args.config)
    with config_path.open("r", encoding=ENCODING) as handle:
        config_data = json.load(handle)
    endpoint_config = config_data["endpoint"]
    endpoint_type = endpoint_config.get("type")
    if endpoint_type == "tcp":
        host = endpoint_config.get("host")
        port = endpoint_config.get("port")
        endpoint = {"type": "tcp", "host": host, "port": port}
    elif endpoint_type == "unix":
        path = endpoint_config.get("path")
        endpoint = {"type": "unix", "path": path}
    else:
        endpoint = endpoint_config

    log_file = config_data["log_file"]
    log_parent = Path(log_file).parent

    asyncio.run(run_server(endpoint, log_file))


if __name__ == "__main__":
    main()
