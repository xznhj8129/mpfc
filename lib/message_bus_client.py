#!/usr/bin/env python3
"""
Unified HiveOS bus client: async and sync APIs sharing the same message envelope.

Async usage:
    client = await BusClientAsync.connect_tcp("127.0.0.1", 7777, "node")
    await client.subscribe("topic")
    await client.publish("topic", {"data": 1})
    await client.receive_loop(handler)

Sync usage:
    client = BusClientSync.connect_tcp("127.0.0.1", 7777, "node")
    client.subscribe("topic")
    client.publish("topic", {"data": 1})
    message, raw = client.receive(timeout=1.0)
"""

import asyncio
import contextlib
import json
import logging
import queue
import socket
import threading
from typing import Awaitable, Callable, Dict, Optional, Tuple

MAX_LINE_BYTES = 1_048_576
ENCODING = "utf-8"
LOG_FORMAT = "%(asctime)s %(levelname)s %(message)s"
HANDSHAKE_OP = "HANDSHAKE"


def _build_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(LOG_FORMAT))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


class _Common:
    @staticmethod
    def _encode(message: Dict) -> bytes:
        encoded = json.dumps(message, separators=(",", ":")).encode(ENCODING) + b"\n"
        return encoded

    @staticmethod
    def _decode_line(line: bytes) -> Tuple[Dict, str]:
        raw = line.decode(ENCODING).rstrip("\n")
        message = json.loads(raw)
        return message, raw

    @staticmethod
    def _validate_endpoint(host: Optional[str], port: Optional[int]) -> Tuple[str, int]:
        return host, port


class BusClientAsync(_Common):
    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.reader = reader
        self.writer = writer

    @classmethod
    async def connect_tcp(cls, host: str, port: int, client_id: str) -> "BusClientAsync":
        host, port = cls._validate_endpoint(host, port)
        reader, writer = await asyncio.open_connection(host, port, limit=MAX_LINE_BYTES)
        client = cls(reader, writer)
        await client._send({"op": HANDSHAKE_OP, "client": client_id})
        return client

    @classmethod
    async def connect_unix(cls, path: str, client_id: str) -> "BusClientAsync":
        reader, writer = await asyncio.open_unix_connection(path=path, limit=MAX_LINE_BYTES)
        client = cls(reader, writer)
        await client._send({"op": HANDSHAKE_OP, "client": client_id})
        return client

    async def _send(self, message: Dict) -> None:
        encoded = self._encode(message)
        self.writer.write(encoded)
        await self.writer.drain()

    async def subscribe(self, topic: str) -> None:
        await self._send({"op": "sub", "topic": topic})

    async def publish(self, topic: str, payload: Dict) -> None:
        await self._send({"op": "pub", "topic": topic, "payload": payload})

    async def receive_loop(self, handler: Callable[[Dict, str], Awaitable[None]]) -> None:
        while True:
            line = await self.reader.readline()
            if not line:
                return
            message, raw = self._decode_line(line)
            await handler(message, raw)

    async def close(self) -> None:
        self.writer.close()
        await self.writer.wait_closed()


class BusClientSync(_Common):
    def __init__(self, sock: socket.socket, client_id: str, logger: logging.Logger) -> None:
        self.sock = sock
        self.client_id = client_id
        self.logger = logger
        self.stop_event = threading.Event()
        self.inbox: "queue.Queue[Tuple[Dict, str]]" = queue.Queue()
        self.reader_error: Optional[BaseException] = None
        self.reader = self.sock.makefile("rb")
        self.reader_thread = threading.Thread(target=self._reader_loop, name=f"bus-reader-{client_id}", daemon=True)

    @classmethod
    def connect_tcp(cls, host: str, port: int, client_id: str, logger: Optional[logging.Logger] = None) -> "BusClientSync":
        host, port = cls._validate_endpoint(host, port)
        sock = socket.create_connection((host, port))
        log = logger or _build_logger("message_bus_sync_client")
        client = cls(sock, client_id, log)
        client._send_line({"op": HANDSHAKE_OP, "client": client_id})
        client._start_reader()
        return client

    @classmethod
    def connect_unix(cls, path: str, client_id: str, logger: Optional[logging.Logger] = None) -> "BusClientSync":
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(path)
        log = logger or _build_logger("message_bus_sync_client")
        client = cls(sock, client_id, log)
        client._send_line({"op": HANDSHAKE_OP, "client": client_id})
        client._start_reader()
        return client

    def subscribe(self, topic: str) -> None:
        self._ensure_alive()
        self._send_line({"op": "sub", "topic": topic})

    def publish(self, topic: str, payload: Dict) -> None:
        self._ensure_alive()
        self._send_line({"op": "pub", "topic": topic, "payload": payload})

    def receive(self, timeout: Optional[float] = None) -> Tuple[Dict, str]:
        if self.stop_event.is_set() and self.inbox.empty():
            self._ensure_alive()
        try:
            return self.inbox.get(timeout=timeout)
        except queue.Empty as exc:
            self._ensure_alive()
            raise exc

    def close(self) -> None:
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        try:
            self.reader.close()
        except (OSError, ValueError):
            self.logger.warning("reader close failed", exc_info=True)
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            self.logger.warning("socket shutdown failed", exc_info=True)
        try:
            self.sock.close()
        except OSError:
            self.logger.warning("socket close failed", exc_info=True)
        if self.reader_thread.is_alive():
            self.reader_thread.join()

    def __enter__(self) -> "BusClientSync":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def _ensure_alive(self) -> None:
        return None

    def _send_line(self, obj: Dict) -> None:
        encoded = self._encode(obj)
        self.sock.sendall(encoded)

    def _reader_loop(self) -> None:
        try:
            while not self.stop_event.is_set():
                line = self.reader.readline(MAX_LINE_BYTES + 1)
                if not line:
                    self.reader_error = RuntimeError("bus connection closed")
                    self.stop_event.set()
                    break
                try:
                    message, raw = self._decode_line(line)
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                    self.reader_error = exc
                    self.stop_event.set()
                    break
                self.inbox.put((message, raw))
        except (OSError, ValueError) as exc:
            self.reader_error = exc
            self.stop_event.set()
        finally:
            if self.reader_error:
                self.logger.error("reader stopped client=%s error=%s", self.client_id, self.reader_error)
            else:
                self.logger.info("reader stopped client=%s", self.client_id)

    def _start_reader(self) -> None:
        if not self.reader_thread.is_alive():
            self.reader_thread.start()
