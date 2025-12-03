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


class MessageBus:
    def __init__(self, logger: logging.Logger) -> None:
        self.logger = logger
        self.clients: Dict[str, asyncio.StreamWriter] = {}
        self.subscriptions: Dict[str, Set[str]] = {}
        self.lock = asyncio.Lock()

    def _decode_json_line(self, line: bytes) -> tuple[str, Dict]:
        if not line.endswith(b"\n"):
            raise ValueError("message line missing newline terminator")
        text = line.decode(ENCODING).rstrip("\n")
        parsed = json.loads(text)
        if not isinstance(parsed, dict):
            raise ValueError("message is not a JSON object")
        return text, parsed

    async def register_client(self, client_id: str, writer: asyncio.StreamWriter) -> None:
        async with self.lock:
            if client_id in self.clients:
                raise ValueError("duplicate client id")
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
        self.logger.info("IN %s %s", client_id, line_text)
        async with self.lock:
            targets = tuple(self.subscriptions.get(topic, ()))
            writers = {target_id: self.clients.get(target_id) for target_id in targets}
        if not writers:
            self.logger.info("DROP %s topic=%s reason=no_subscribers", client_id, topic)
            return
        outgoing = dict(message)
        outgoing["src"] = client_id
        encoded = json.dumps(outgoing, separators=(",", ":")).encode(ENCODING) + b"\n"
        outgoing_text = encoded.decode(ENCODING).rstrip("\n")
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
            self.logger.info("OUT %s->%s %s", client_id, target_id, outgoing_text)

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername") or "unix-peer"
        client_id = None
        try:
            try:
                first_line = await reader.readline()
            except asyncio.LimitOverrunError as exc:
                self.logger.error("HELLO_FAIL peer=%s error=%s", peer, exc)
                return
            if not first_line:
                self.logger.error("HELLO_FAIL peer=%s error=empty_stream", peer)
                return
            try:
                line_text, hello_message = self._decode_json_line(first_line)
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                self.logger.error("HELLO_FAIL peer=%s error=%s", peer, exc)
                return
            if hello_message.get("op") != "hello":
                self.logger.error("HELLO_FAIL peer=%s error=missing_hello", peer)
                return
            if "client" not in hello_message or not isinstance(hello_message["client"], str) or not hello_message["client"]:
                self.logger.error("HELLO_FAIL peer=%s error=missing_client_id", peer)
                return
            client_id = hello_message["client"]
            try:
                await self.register_client(client_id, writer)
            except ValueError as exc:
                self.logger.error("HELLO_FAIL client=%s error=%s", client_id, exc)
                return
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
                    if not isinstance(topic, str) or not topic:
                        self.logger.error("SUB_FAIL client=%s error=missing_topic raw=%s", client_id, line_text)
                        continue
                    await self.add_subscription(client_id, topic)
                    self.logger.info("IN %s %s", client_id, line_text)
                elif op == "pub":
                    topic = message.get("topic")
                    if not isinstance(topic, str) or not topic:
                        self.logger.error("PUB_FAIL client=%s error=missing_topic raw=%s", client_id, line_text)
                        continue
                    if "payload" not in message:
                        self.logger.error("PUB_FAIL client=%s error=missing_payload raw=%s", client_id, line_text)
                        continue
                    await self.publish(client_id, topic, message, line_text)
                else:
                    self.logger.error("OP_UNKNOWN client=%s raw=%s", client_id, line_text)
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
    log_handlers = [
        logging.FileHandler(log_file, encoding=ENCODING),
    ]
    log_handlers.append(logging.StreamHandler(sys.stdout))
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
    if not config_path.is_file():
        raise FileNotFoundError(f"config file not found: {config_path}")
    with config_path.open("r", encoding=ENCODING) as handle:
        config_data = json.load(handle)
    if not isinstance(config_data, dict):
        raise ValueError("config root must be an object")
    if "endpoint" not in config_data or not isinstance(config_data["endpoint"], dict):
        raise ValueError("config.endpoint is required")
    endpoint_config = config_data["endpoint"]
    endpoint_type = endpoint_config.get("type")
    if endpoint_type == "tcp":
        host = endpoint_config.get("host")
        port = endpoint_config.get("port")
        if not isinstance(host, str) or not host:
            raise ValueError("endpoint.host is required for tcp")
        if not isinstance(port, int) or port < 1 or port > 65535:
            raise ValueError("endpoint.port must be int 1-65535 for tcp")
        endpoint = {"type": "tcp", "host": host, "port": port}
    elif endpoint_type == "unix":
        path = endpoint_config.get("path")
        if not isinstance(path, str) or not path:
            raise ValueError("endpoint.path is required for unix")
        endpoint = {"type": "unix", "path": path}
    else:
        raise ValueError("endpoint.type must be 'tcp' or 'unix'")

    if "log_file" not in config_data or not isinstance(config_data["log_file"], str) or not config_data["log_file"]:
        raise ValueError("log_file is required in config")
    log_file = config_data["log_file"]
    log_parent = Path(log_file).parent
    if log_parent and not log_parent.exists():
        raise FileNotFoundError(f"log directory does not exist: {log_parent}")

    asyncio.run(run_server(endpoint, log_file))


if __name__ == "__main__":
    main()
