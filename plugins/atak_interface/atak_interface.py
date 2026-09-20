#!/usr/bin/env python3
"""CoT/ATAK endpoint adapter: OCCID <-> Cursor on Target."""

from __future__ import annotations

import select
import socket
import struct
import time
import traceback
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Iterable

import frogcot

from lib.common import apply_cfg, build_envelope, build_request_topic, build_response_topic, build_state_scheduler_topics, build_topic_base
from lib.occid_bus import decode_occid_request, occid, pack_occid
from lib.occid_topics import COT_RAW, ENTITY_STATE
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from interop.cot import CotPointFields, cot_point_to_location_state, global_position_to_cot_point, location_state_to_cot_point


COT_SUBJECT_PREFIX = "external:cot:"


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
        self._subject_cot_uids: dict[str, str] = {}

    def _parse_targets(self, raw_targets: list[Dict[str, Any]]) -> list[Endpoint]:
        return [Endpoint(entry["host"], int(entry["port"])) for entry in raw_targets]

    def _publish_model(self, key: str, model: Any) -> None:
        if key in self.state_scheduler.topics:
            self.state_scheduler.update(key, pack_occid(model))

    def _publish_diag(self, name: str, data: Dict[str, Any]) -> None:
        topic = f"DIAG/{self.client_id}/{name}"
        self.client.publish(topic, build_envelope(self.client_id, topic, data))

    def _sync_tcp_client_connected(self) -> None:
        connected = self.tcp_client is not None and self.tcp_client.socket() is not None
        if connected == self.tcp_client_connected:
            return
        self.tcp_client_connected = connected
        print(f"[PLUGIN] {self.client_id} tcp_client_connected={connected}", flush=True)

    @staticmethod
    def _uid_text(value: Any) -> str:
        return str(uuid.UUID(bytes=bytes(value.root)))

    def _record_id(self, uid: str, timestamp: float) -> Any:
        return occid.UID(
            root=uuid.uuid5(uuid.NAMESPACE_URL, f"record:cot:{uid}:{int(timestamp * 1000)}").bytes
        )

    def _subject_id(self, uid: str) -> Any:
        """Create a local OCCID identity for an unresolved external CoT identity."""
        subject = occid.UID(
            root=uuid.uuid5(uuid.NAMESPACE_URL, f"{COT_SUBJECT_PREFIX}{uid}").bytes
        )
        self._subject_cot_uids[self._uid_text(subject)] = uid
        return subject

    def _cot_uid_for_subject(self, subject_id: Any) -> str:
        """Map an OCCID subject to a CoT UID without treating the IDs as aliases."""
        text = self._uid_text(subject_id)
        cached = self._subject_cot_uids.get(text)
        if cached is not None:
            return cached
        return f"occid:uid:{text}"

    def _event_to_entity_state(self, event: Any, source: tuple[str, int]) -> Any:
        uid = str(event.unique_id)
        timestamp = event.time.timestamp() if event.time is not None else time.time()
        point = CotPointFields(
            lat_deg=float(event.point.latitude),
            lon_deg=float(event.point.longitude),
            hae_m=float(event.point.height_above_ellipsoid),
            ce_m=None if event.point.circular_error is None else float(event.point.circular_error),
            le_m=None if event.point.linear_error is None else float(event.point.linear_error),
        )
        location = cot_point_to_location_state(point)
        stamp = occid.Timestamp(utime=timestamp, tz=0)
        return occid.EntityState(
            record=occid.Record(
                uid=self._record_id(uid, timestamp),
                id=occid.IntID(root=0),
                created_ts=stamp,
                updated_ts=stamp,
                origin_system="CoT",
                provenance=[f"{source[0]}:{source[1]}", str(event.event_type), f"cot_uid:{uid}"],
            ),
            subject_uid=self._subject_id(uid),
            timestamp=stamp,
            position=location,
            link_states={},
        )

    def _handle_inbound(self, payload: bytes, source: tuple[str, int]) -> None:
        try:
            xml_text = payload.decode("utf-8").strip()
            if not xml_text:
                return
            self._publish_model(
                COT_RAW,
                occid.ProtocolPayload(
                    format=occid.ProtocolPayloadFormat.XML,
                    content_type="application/cot+xml",
                    text=xml_text,
                ),
            )
            event = self.translator.parse_event(xml_text)
            entity_state = self._event_to_entity_state(event, source)
            self._publish_model(ENTITY_STATE, entity_state)
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

    def _entity_state_xml(self, state: Any) -> bytes:
        if state.position is None:
            raise ValueError("EntityState requires position for CoT marker translation")
        point = location_state_to_cot_point(state.position)
        uid = self._cot_uid_for_subject(state.subject_id)
        return self.translator.marker_xml(
            callsign=uid,
            uid=uid,
            cottype=self.translator.self_cottype,
            point=point,
        )

    def _human_text_xml(self, message: Any) -> bytes:
        if message.position is None:
            raise ValueError("HumanTextMessage requires position for ATAK geochat translation")
        point = global_position_to_cot_point(message.position)
        destination = message.destination_group
        if destination is None and message.destination_uid is not None:
            destination = self._uid_text(message.destination_uid)
        if destination is None:
            destination = self._uid_text(message.dst)
        return self.translator.geochat_xml(message.message, str(destination), point)

    def _handle_request(self, request: Dict[str, Any]) -> None:
        request_id, model = decode_occid_request(request)
        try:
            if isinstance(model, occid.EntityState):
                xml_bytes = self._entity_state_xml(model)
            elif isinstance(model, occid.HumanTextMessage):
                xml_bytes = self._human_text_xml(model)
            elif isinstance(model, occid.ProtocolPayload):
                if model.format != occid.ProtocolPayloadFormat.XML or model.text is None:
                    raise ValueError("CoT ProtocolPayload requires XML text")
                xml_bytes = model.text.encode("utf-8")
            else:
                raise ValueError(f"unsupported OCCID model for CoT translation {type(model).__name__}")
            result = self._send_xml(xml_bytes, self.cot_output_targets)
            self.enqueue_response(request_id, type(model).__name__, True, result)
        except (ValueError, TypeError, RuntimeError) as exc:
            self.enqueue_response(request_id, type(model).__name__, False, {"error": str(exc)})

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
