#!/usr/bin/env python3
"""CoT/ATAK endpoint adapter: Lattice Entity/GeoChat <-> Cursor on Target."""

from __future__ import annotations

import select
import socket
import struct
import time
import traceback
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, Iterable

import frogcot
from anduril import Aliases, Entity, Location, MilView, Ontology, Provenance

from lib.common import apply_cfg, build_envelope, build_request_topic, build_response_topic, build_state_scheduler_topics, build_topic_base
from lib.lattice_bus import decode_json_request, decode_request, pack_lattice
from lib.bus_topics import COT_RAW, ENTITY_STATE
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from .interop_cot import CotPointFields, cot_point_to_location, location_to_cot_point


@dataclass(frozen=True)
class Endpoint:
    host: str
    port: int


class DatagramReceiver:
    def __init__(self, bind: Endpoint, recv_buffer_bytes: int, multicast_group: str | None) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((bind.host, bind.port))
        if multicast_group:
            membership = struct.pack("=4sl", socket.inet_aton(multicast_group), socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        sock.setblocking(False)
        self.socket = sock
        self.recv_buffer_bytes = recv_buffer_bytes

    def recv(self) -> tuple[bytes, tuple[str, int]]:
        return self.socket.recvfrom(self.recv_buffer_bytes)

    def close(self) -> None:
        self.socket.close()


class DatagramSender:
    def __init__(self, multicast_ttl: int) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, multicast_ttl)
        self.socket = sock

    def send(self, payload: bytes, targets: Iterable[Endpoint]) -> None:
        for endpoint in targets:
            self.socket.sendto(payload, (endpoint.host, endpoint.port))

    def close(self) -> None:
        self.socket.close()


class TcpListener:
    def __init__(self, bind: Endpoint, recv_buffer_bytes: int) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((bind.host, bind.port))
        srv.listen(5)
        srv.setblocking(False)
        self.server = srv
        self.recv_buffer_bytes = recv_buffer_bytes
        self.buffers: Dict[socket.socket, bytearray] = {}

    def sockets(self) -> list[socket.socket]:
        return [self.server] + list(self.buffers.keys())

    def owns(self, sock: socket.socket) -> bool:
        return sock in self.buffers

    def accept_ready(self) -> None:
        conn, _ = self.server.accept()
        conn.setblocking(False)
        self.buffers[conn] = bytearray()

    def recv_ready(self, sock: socket.socket) -> list[tuple[bytes, tuple[str, int]]]:
        data = sock.recv(self.recv_buffer_bytes)
        if not data:
            self.close_conn(sock)
            return []
        buf = self.buffers[sock]
        buf.extend(data)
        messages: list[tuple[bytes, tuple[str, int]]] = []
        marker = b"</event>"
        while True:
            idx = buf.find(marker)
            if idx == -1:
                break
            end = idx + len(marker)
            chunk = bytes(buf[:end])
            del buf[:end]
            try:
                addr = sock.getpeername()
            except OSError:
                addr = ("tcp", 0)
            messages.append((chunk, addr))
        return messages

    def close_conn(self, sock: socket.socket) -> None:
        try:
            sock.close()
        finally:
            self.buffers.pop(sock, None)

    def close(self) -> None:
        for sock in list(self.buffers.keys()):
            self.close_conn(sock)
        self.server.close()


class TcpClientReceiver:
    def __init__(self, endpoint: Endpoint, recv_buffer_bytes: int, reconnect_secs: float) -> None:
        self.endpoint = endpoint
        self.recv_buffer_bytes = recv_buffer_bytes
        self.reconnect_secs = reconnect_secs
        self.sock: socket.socket | None = None
        self.buffer = bytearray()
        self.next_attempt = time.monotonic()

    def socket(self) -> socket.socket | None:
        return self.sock

    def ensure_connected(self) -> None:
        now = time.monotonic()
        if self.sock is not None or now < self.next_attempt:
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((self.endpoint.host, self.endpoint.port))
            sock.setblocking(False)
            self.sock = sock
            self.next_attempt = now + self.reconnect_secs
        except OSError:
            self.sock = None
            self.next_attempt = now + self.reconnect_secs

    def recv_ready(self) -> list[tuple[bytes, tuple[str, int]]]:
        if self.sock is None:
            return []
        try:
            data = self.sock.recv(self.recv_buffer_bytes)
        except OSError:
            self._close()
            return []
        if not data:
            self._close()
            return []
        self.buffer.extend(data)
        marker = b"</event>"
        messages: list[tuple[bytes, tuple[str, int]]] = []
        while True:
            idx = self.buffer.find(marker)
            if idx == -1:
                break
            end = idx + len(marker)
            chunk = bytes(self.buffer[:end])
            del self.buffer[:end]
            messages.append((chunk, (self.endpoint.host, self.endpoint.port)))
        return messages

    def _close(self) -> None:
        if self.sock is None:
            return
        try:
            self.sock.close()
        finally:
            self.sock = None
            self.buffer.clear()
            self.next_attempt = time.monotonic() + self.reconnect_secs

    def close(self) -> None:
        self._close()


class CotTranslator:
    def __init__(
        self,
        stale_seconds: int,
        default_ce: float,
        default_le: float,
        self_callsign: str,
        self_cottype: str,
    ) -> None:
        self.stale_seconds = int(stale_seconds)
        self.default_ce = float(default_ce)
        self.default_le = float(default_le)
        self.self_callsign = str(self_callsign)
        self.self_cottype = str(self_cottype)
        self.client = frogcot.ATAKClient(self.self_callsign, cottype=self.self_cottype, is_self=True)

    def parse_event(self, xml_text: str) -> frogcot.Event:
        return frogcot.xml_to_cot(xml_text)

    def marker_xml(self, callsign: str, uid: str, cottype: str, point: CotPointFields) -> bytes:
        payload = {
            "lat": float(point.lat_deg),
            "lon": float(point.lon_deg),
            "alt": float(point.hae_m),
            "ce": self.default_ce if point.ce_m is None else float(point.ce_m),
            "le": self.default_le if point.le_m is None else float(point.le_m),
        }
        return self.client.cot_marker(
            str(callsign),
            str(uid),
            str(cottype),
            payload,
            staletime=self.stale_seconds,
        )

    def geochat_xml(self, message: str, to_team: str, point: CotPointFields) -> bytes:
        payload = {
            "lat": float(point.lat_deg),
            "lon": float(point.lon_deg),
            "alt": float(point.hae_m),
            "ce": self.default_ce if point.ce_m is None else float(point.ce_m),
            "le": self.default_le if point.le_m is None else float(point.le_m),
        }
        xml_bytes = self.client.geochat(str(message), to_team=str(to_team), pos=payload)
        if xml_bytes is None:
            raise RuntimeError("geochat generation failed")
        return xml_bytes


class AtakInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)

        base = build_topic_base(self.client_id, self.topic_ns)
        self.state_scheduler = StateScheduler(
            self.client,
            self.client_id,
            build_state_scheduler_topics(base, self.state_intervals),
        )
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(float(cfg["bus_poll_interval_s"]))

        listen_cfg = cfg["listen"]
        self.listen_endpoint = Endpoint(listen_cfg["host"], int(listen_cfg["port"]))
        self.multicast_group = cfg["multicast_group"]
        self.recv_buffer_bytes = int(cfg["recv_buffer_bytes"])
        self.sender = DatagramSender(int(cfg["multicast_ttl"]))
        self.receivers = [DatagramReceiver(self.listen_endpoint, self.recv_buffer_bytes, self.multicast_group)]

        tcp_listen_cfg = cfg["tcp_listen"]
        if bool(tcp_listen_cfg["enabled"]):
            endpoint = Endpoint(tcp_listen_cfg["host"], int(tcp_listen_cfg["port"]))
            self.tcp_listener: TcpListener | None = TcpListener(endpoint, self.recv_buffer_bytes)
        else:
            self.tcp_listener = None

        tcp_connect_cfg = cfg["tcp_connect"]
        if bool(tcp_connect_cfg["enabled"]):
            endpoint = Endpoint(tcp_connect_cfg["host"], int(tcp_connect_cfg["port"]))
            self.tcp_client: TcpClientReceiver | None = TcpClientReceiver(
                endpoint,
                self.recv_buffer_bytes,
                float(tcp_connect_cfg["reconnect_secs"]),
            )
        else:
            self.tcp_client = None

        self.cot_output_targets = self._parse_targets(cfg["cot_output_targets"])
        translator_cfg = cfg["translator"]
        self.translator = CotTranslator(
            int(translator_cfg["stale_seconds"]),
            float(translator_cfg["default_ce"]),
            float(translator_cfg["default_le"]),
            translator_cfg["self_callsign"],
            translator_cfg["self_cottype"],
        )
        self.loop_interval_s = float(cfg["loop_interval_s"])
        self.rx_count = 0
        self.tx_count = 0
        self.rx_parse_errors = 0
        self.tcp_client_connected: bool | None = None

    def _parse_targets(self, raw_targets: list[Dict[str, Any]]) -> list[Endpoint]:
        return [Endpoint(entry["host"], int(entry["port"])) for entry in raw_targets]

    def _publish_model(self, key: str, model: Any) -> None:
        if key not in self.state_scheduler.topics:
            return
        if type(model) is dict:
            payload = model
        else:
            payload = pack_lattice(model)
        self.state_scheduler.update(key, payload)

    def _publish_diag(self, name: str, data: Dict[str, Any]) -> None:
        topic = f"DIAG/{self.client_id}/{name}"
        self.client.publish(topic, build_envelope(self.client_id, topic, data))

    def _sync_tcp_client_connected(self) -> None:
        connected = self.tcp_client is not None and self.tcp_client.socket() is not None
        if connected == self.tcp_client_connected:
            return
        self.tcp_client_connected = connected
        print(f"[PLUGIN] {self.client_id} tcp_client_connected={connected}", flush=True)

    def _event_to_entity(self, event: Any, source: tuple[str, int]) -> Entity:
        uid = str(event.unique_id)
        timestamp = event.time.timestamp() if event.time is not None else time.time()
        point = CotPointFields(
            lat_deg=float(event.point.latitude),
            lon_deg=float(event.point.longitude),
            hae_m=float(event.point.height_above_ellipsoid),
            ce_m=None if event.point.circular_error is None else float(event.point.circular_error),
            le_m=None if event.point.linear_error is None else float(event.point.linear_error),
        )
        location, uncertainty = cot_point_to_location(point)
        return Entity(
            entity_id=uid,
            is_live=True,
            expiry_time=datetime.fromtimestamp(
                timestamp + float(self.translator.stale_seconds), tz=timezone.utc
            ),
            aliases=Aliases(name=uid),
            location=location,
            location_uncertainty=uncertainty,
            mil_view=MilView(disposition="DISPOSITION_UNKNOWN", environment="ENVIRONMENT_AIR"),
            ontology=Ontology(template="TEMPLATE_TRACK"),
            provenance=Provenance(
                integration_name="CoT",
                data_type=str(event.event_type),
                source_id=f"{source[0]}:{source[1]}",
                source_update_time=datetime.fromtimestamp(timestamp, tz=timezone.utc),
                source_description=f"cot_uid:{uid}",
            ),
        )

    def _handle_inbound(self, payload: bytes, source: tuple[str, int]) -> None:
        try:
            xml_text = payload.decode("utf-8").strip()
            if not xml_text:
                return
            self._publish_model(
                COT_RAW,
                {
                    "format": "XML",
                    "content_type": "application/cot+xml",
                    "text": xml_text,
                },
            )
            event = self.translator.parse_event(xml_text)
            entity = self._event_to_entity(event, source)
            self._publish_model(ENTITY_STATE, entity)
            self.rx_count += 1
            print(
                f"[PLUGIN] {self.client_id} rx uid={event.unique_id} type={event.event_type} "
                f"source={source[0]}:{source[1]} rx_count={self.rx_count}",
                flush=True,
            )
        except (UnicodeDecodeError, ValueError, KeyError, TypeError, AttributeError) as exc:
            self.rx_parse_errors += 1
            error = f"{exc.__class__.__name__}: {exc}"
            self._publish_diag(
                "RX_PARSE_ERROR",
                {"error": error, "count": self.rx_parse_errors},
            )
            print(
                f"[PLUGIN] {self.client_id} rx_parse_errors={self.rx_parse_errors} last_error={error}",
                flush=True,
            )

    def _send_xml(self, xml_bytes: bytes, targets: list[Endpoint]) -> Dict[str, Any]:
        self.sender.send(xml_bytes, targets)
        self.tx_count += 1
        return {"target_count": len(targets), "bytes_sent": len(xml_bytes), "tx_count": self.tx_count}

    def _entity_xml(self, entity: Entity) -> bytes:
        if entity.location is None:
            raise ValueError("Lattice Entity requires location for CoT marker translation")
        point = location_to_cot_point(entity.location, entity.location_uncertainty)
        uid = str(entity.entity_id or "")
        if not uid:
            raise ValueError("Lattice Entity requires entity_id for CoT marker translation")
        callsign = entity.aliases.name if entity.aliases is not None and entity.aliases.name else uid
        return self.translator.marker_xml(
            callsign=callsign,
            uid=uid,
            cottype=self.translator.self_cottype,
            point=point,
        )

    def _geo_chat_xml(self, payload: Dict[str, Any]) -> bytes:
        message = payload.get("message")
        if not isinstance(message, str):
            raise ValueError("geo chat payload requires message text")
        destination = payload.get("to_team") or payload.get("destination") or "All Chat Rooms"
        point = CotPointFields(
            lat_deg=float(payload["lat"]),
            lon_deg=float(payload["lon"]),
            hae_m=float(payload["alt"]),
            ce_m=None if payload.get("ce") is None else float(payload["ce"]),
            le_m=None if payload.get("le") is None else float(payload["le"]),
        )
        return self.translator.geochat_xml(message, str(destination), point)

    def _handle_request(self, request: Dict[str, Any]) -> None:
        request_id = str(request.get("request_id", "unknown"))
        if "model" in request:
            request_id, model = decode_request(request, Entity)
            try:
                xml_bytes = self._entity_xml(model)
                result = self._send_xml(xml_bytes, self.cot_output_targets)
                self.enqueue_response(request_id, "Entity", True, result)
            except (ValueError, TypeError, RuntimeError) as exc:
                self.enqueue_response(request_id, "Entity", False, {"error": str(exc)})
            return
        request_id, payload = decode_json_request(request)
        try:
            if "message" in payload:
                xml_bytes = self._geo_chat_xml(payload)
                kind = "GeoChat"
            elif payload.get("format") == "XML" and isinstance(payload.get("text"), str):
                xml_bytes = payload["text"].encode("utf-8")
                kind = "CotXml"
            else:
                raise ValueError("unsupported local CoT request payload")
            result = self._send_xml(xml_bytes, self.cot_output_targets)
            self.enqueue_response(request_id, kind, True, result)
        except (ValueError, TypeError, RuntimeError) as exc:
            self.enqueue_response(request_id, "CoT", False, {"error": str(exc)})

    def _poll_network(self, timeout_s: float) -> None:
        if self.tcp_client is not None:
            self.tcp_client.ensure_connected()
        self._sync_tcp_client_connected()

        sockets: list[socket.socket] = []
        receiver_by_fileno: Dict[int, DatagramReceiver] = {}
        for receiver in self.receivers:
            sockets.append(receiver.socket)
            receiver_by_fileno[receiver.socket.fileno()] = receiver
        if self.tcp_listener is not None:
            sockets.extend(self.tcp_listener.sockets())
        if self.tcp_client is not None and self.tcp_client.socket() is not None:
            sockets.append(self.tcp_client.socket())
        if not sockets:
            time.sleep(timeout_s)
            self._sync_tcp_client_connected()
            return

        readable, _, _ = select.select(sockets, [], [], timeout_s)
        for sock in readable:
            if self.tcp_listener is not None and sock is self.tcp_listener.server:
                self.tcp_listener.accept_ready()
                continue
            if self.tcp_listener is not None and self.tcp_listener.owns(sock):
                for payload, source in self.tcp_listener.recv_ready(sock):
                    self._handle_inbound(payload, source)
                continue
            if self.tcp_client is not None and self.tcp_client.socket() is sock:
                for payload, source in self.tcp_client.recv_ready():
                    self._handle_inbound(payload, source)
                continue
            receiver = receiver_by_fileno[sock.fileno()]
            payload, source = receiver.recv()
            self._handle_inbound(payload, source)
        self._sync_tcp_client_connected()

    def run(self) -> None:
        self.send_online()
        self._sync_tcp_client_connected()
        try:
            while True:
                self._poll_network(self.loop_interval_s)
                self.state_scheduler.flush()
                self.flush_queue(self.response_queue, self.response_topic)
                while True:
                    topic, payload = self._pump_once()
                    if topic is None:
                        break
                    if topic == self.request_topic:
                        self._handle_request(payload["data"])
                        self.flush_queue(self.response_queue, self.response_topic)
        except (KeyboardInterrupt, SystemExit):
            pass
        except RuntimeError:
            self.publish_error(traceback.format_exc().strip())
            raise
        finally:
            self.stop()

    def stop(self) -> None:
        for receiver in self.receivers:
            receiver.close()
        self.sender.close()
        if self.tcp_listener is not None:
            self.tcp_listener.close()
        if self.tcp_client is not None:
            self.tcp_client.close()
        super().stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    AtakInterface(cfg, bus_config).run()
