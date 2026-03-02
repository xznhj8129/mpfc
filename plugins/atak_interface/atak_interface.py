#!/usr/bin/env python3
"""
ATAK interface plugin that bridges CoT traffic to HiveOS bus topics.
Usage:
    from plugins.atak_interface.atak_interface import run_plugin
    run_plugin(cfg, bus_config)
"""

import datetime
import select
import socket
import struct
import time
import traceback
from dataclasses import dataclass
from typing import Any, Dict, Iterable

import frogcot

from lib.common import (
    apply_cfg,
    build_event_topics,
    build_request_topic,
    build_response_topic,
    build_state_scheduler_topics,
    build_topic_base,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler
from protocols.namespace_loader import load_protocol_namespace

ATAK = load_protocol_namespace("atak")


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
    def __init__(self, stale_seconds: int, default_ce: float, default_le: float, self_callsign: str, self_cottype: str) -> None:
        self.stale_seconds = stale_seconds
        self.default_ce = default_ce
        self.default_le = default_le
        self.client = frogcot.ATAKClient(self_callsign, cottype=self_cottype, is_self=True)

    def parse_event(self, xml_text: str) -> frogcot.Event:
        return frogcot.xml_to_cot(xml_text)

    def marker_xml(self, callsign: str, uid: str, cottype: str, position: Dict[str, Any], stale_seconds: int | None) -> bytes:
        stale = self.stale_seconds if stale_seconds is None else stale_seconds
        payload = {
            "lat": float(position["LatDeg"]),
            "lon": float(position["LonDeg"]),
            "alt": float(position["AltM"]),
            "ce": float(position["Ce"]),
            "le": float(position["Le"]),
        }
        return self.client.cot_marker(callsign, uid, cottype, payload, staletime=stale)

    def geochat_xml(self, message: str, to_team: str, position: Dict[str, Any]) -> bytes:
        payload = {
            "lat": float(position["LatDeg"]),
            "lon": float(position["LonDeg"]),
            "alt": float(position["AltM"]),
            "ce": float(position["Ce"]),
            "le": float(position["Le"]),
        }
        xml_bytes = self.client.geochat(message, to_team=to_team, pos=payload)
        if xml_bytes is None:
            raise RuntimeError("geochat generation failed")
        return xml_bytes

    def marker_event_json(
        self, callsign: str, uid: str, cottype: str, position: Dict[str, Any], stale_seconds: int | None
    ) -> Dict[str, Any]:
        stale = self.stale_seconds if stale_seconds is None else stale_seconds
        now = datetime.datetime.now(datetime.timezone.utc)
        stale_time = now + datetime.timedelta(seconds=stale)
        event = frogcot.Event(
            point=frogcot.Point(
                latitude=float(position["LatDeg"]),
                longitude=float(position["LonDeg"]),
                height_above_ellipsoid=float(position["AltM"]),
                circular_error=float(position["Ce"]),
                linear_error=float(position["Le"]),
            ),
            detail={"contact": {"callsign": callsign}},
            version=2,
            event_type=cottype,
            unique_id=uid,
            time=now,
            start=now,
            stale=stale_time,
            how="h-g-i-g-o",
        )
        return event.to_dict()

    def event_json_to_xml(self, event_data: Dict[str, Any]) -> bytes:
        if "point" in event_data and "event_type" in event_data:
            event = frogcot.Event.from_dict(dict(event_data))
            return frogcot.cot_to_xml(event).encode("utf-8")

        now = datetime.datetime.now(datetime.timezone.utc)
        stale = now + datetime.timedelta(seconds=self.stale_seconds)
        if "Time" in event_data:
            now = datetime.datetime.fromisoformat(str(event_data["Time"]).replace("Z", "+00:00"))
        start = now
        if "Start" in event_data:
            start = datetime.datetime.fromisoformat(str(event_data["Start"]).replace("Z", "+00:00"))
        if "Stale" in event_data:
            stale = datetime.datetime.fromisoformat(str(event_data["Stale"]).replace("Z", "+00:00"))
        point_data = event_data["Position"]
        detail = event_data.get("Detail")
        if detail is not None:
            detail = dict(detail)
        if event_data.get("Callsign") and detail is None:
            detail = {"contact": {"callsign": event_data["Callsign"]}}

        event = frogcot.Event(
            point=frogcot.Point(
                latitude=float(point_data["LatDeg"]),
                longitude=float(point_data["LonDeg"]),
                height_above_ellipsoid=float(point_data.get("HaeM", point_data.get("AltM", 0.0))),
                circular_error=float(point_data.get("Ce", self.default_ce)),
                linear_error=float(point_data.get("Le", self.default_le)),
            ),
            detail=detail,
            version=int(event_data.get("Version", 2)),
            event_type=str(event_data["Type"]),
            access=event_data.get("Access"),
            quality_of_service=event_data.get("Qos"),
            unique_id=str(event_data["Uid"]),
            time=now,
            start=start,
            stale=stale,
            how=str(event_data.get("How", "m-g")),
        )
        return frogcot.cot_to_xml(event).encode("utf-8")


class AtakInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:  # Initialize ATAK bridge plugin.
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)

        base = build_topic_base(self.client_id, self.topic_ns)
        self.state_scheduler = StateScheduler(
            self.client,
            self.client_id,
            build_state_scheduler_topics(base, self.state_intervals),
        )
        self.event_topics = build_event_topics(
            base,
            [
                ATAK.Event.Cot.ReceivedEvent,
                ATAK.Event.Marker.ReceivedMarker,
                ATAK.Event.GeoChat.ReceivedGeoChat,
                ATAK.Event.System.ParseError,
            ],
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
            tcp_listen_endpoint = Endpoint(tcp_listen_cfg["host"], int(tcp_listen_cfg["port"]))
            self.tcp_listener: TcpListener | None = TcpListener(tcp_listen_endpoint, self.recv_buffer_bytes)
        else:
            self.tcp_listener = None

        tcp_connect_cfg = cfg["tcp_connect"]
        if bool(tcp_connect_cfg["enabled"]):
            tcp_connect_endpoint = Endpoint(tcp_connect_cfg["host"], int(tcp_connect_cfg["port"]))
            self.tcp_client: TcpClientReceiver | None = TcpClientReceiver(
                tcp_connect_endpoint, self.recv_buffer_bytes, float(tcp_connect_cfg["reconnect_secs"])
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
        self.last_error = ""
        self.last_rx_event: Dict[str, Any] = {}
        self.last_tx_event: Dict[str, Any] = {}
        self.last_tx_result: Dict[str, Any] = {}
        self.tcp_client_connected: bool | None = None

    def _parse_targets(self, raw_targets: list[Dict[str, Any]]) -> list[Endpoint]:  # Parse list of send targets.
        targets: list[Endpoint] = []
        for entry in raw_targets:
            targets.append(Endpoint(entry["host"], int(entry["port"])))
        return targets

    def _update_state(self, key: str, value: Any) -> None:  # Update state only when field is configured.
        if key not in self.state_scheduler.topics:
            return
        self.state_scheduler.update(key, value)

    def _sync_tcp_client_connected(self) -> None:
        connected = self.tcp_client is not None and self.tcp_client.socket() is not None
        if connected == self.tcp_client_connected:
            return
        self.tcp_client_connected = connected
        self._update_state(ATAK.State.Link.TcpClientConnected, connected)

    def _handle_inbound(self, payload: bytes, source: tuple[str, int]) -> None:  # Parse inbound CoT and update state.
        try:
            xml_text = payload.decode("utf-8").strip()
            if not xml_text:
                return
            event = self.translator.parse_event(xml_text)
            callsign = None
            if event.detail and "contact" in event.detail:
                contact = event.detail["contact"]
                callsign = contact.get("@callsign") or contact.get("callsign")
            summary = {
                "Uid": event.unique_id,
                "Type": event.event_type,
                "How": event.how,
                "Time": event.time.isoformat(),
                "Start": event.start.isoformat(),
                "Stale": event.stale.isoformat(),
                "PositionLatDeg": event.point.latitude,
                "PositionLonDeg": event.point.longitude,
                "PositionHaeM": event.point.height_above_ellipsoid,
                "PositionCe": event.point.circular_error,
                "PositionLe": event.point.linear_error,
                "SourceHost": source[0],
                "SourcePort": source[1],
            }
            if callsign:
                summary["Callsign"] = callsign
            if event.detail is not None:
                summary["DetailXml"] = str(event.detail)
            self.rx_count += 1
            self.last_rx_event = summary
            self._update_state(ATAK.State.Rx.LastEvent, self.last_rx_event)
            self._publish_event(ATAK.Event.Cot.ReceivedEvent, self.last_rx_event)
            if str(event.event_type).startswith("a-"):
                marker_event = {
                    "Uid": event.unique_id,
                    "CotType": event.event_type,
                    "PositionLatDeg": event.point.latitude,
                    "PositionLonDeg": event.point.longitude,
                    "PositionHaeM": event.point.height_above_ellipsoid,
                    "PositionCe": event.point.circular_error,
                    "PositionLe": event.point.linear_error,
                }
                if callsign:
                    marker_event["Callsign"] = callsign
                self._publish_event(ATAK.Event.Marker.ReceivedMarker, marker_event)
            if event.event_type == "b-t-f":
                geochat_event = {"Uid": event.unique_id, "Message": ""}
                if callsign:
                    geochat_event["FromCallsign"] = callsign
                self._publish_event(ATAK.Event.GeoChat.ReceivedGeoChat, geochat_event)
            self._update_state(ATAK.State.System.RxCount, self.rx_count)
            print(
                f"[PLUGIN] {self.client_id} rx uid={summary['Uid']} type={summary['Type']} source={source[0]}:{source[1]}",
                flush=True,
            )
        except (UnicodeDecodeError, ValueError, KeyError, TypeError) as exc:
            self.rx_parse_errors += 1
            self.last_error = f"{exc.__class__.__name__}: {exc}"
            self._update_state(ATAK.State.System.RxParseErrors, self.rx_parse_errors)
            self._update_state(ATAK.State.System.LastError, self.last_error)
            self._publish_event(ATAK.Event.System.ParseError, {"Error": self.last_error})
            print(
                f"[PLUGIN] {self.client_id} rx_parse_errors={self.rx_parse_errors} last_error={self.last_error}",
                flush=True,
            )

    def _send_xml(self, xml_bytes: bytes, targets: list[Endpoint], tx_event: Dict[str, Any]) -> Dict[str, Any]:  # Send CoT payload.
        self.sender.send(xml_bytes, targets)
        self.tx_count += 1
        self.last_tx_event = tx_event
        self.last_tx_result = {"TargetCount": len(targets), "Bytes": len(xml_bytes)}
        self._update_state(ATAK.State.Tx.LastResult, self.last_tx_result)
        self._update_state(ATAK.State.System.TxCount, self.tx_count)
        return self.last_tx_result

    def _handle_request(self, request: Dict[str, Any]) -> None:  # Handle bus REQUEST action.
        request_id = str(request["request_id"])
        action = request["action"]
        params = request.get("params") or {}

        if action == ATAK.Action.Cot.SendEvent:
            event_data = {
                "Uid": str(params["Uid"]),
                "Type": str(params["Type"]),
                "How": str(params["How"]),
                "Position": {
                    "LatDeg": float(params["PositionLatDeg"]),
                    "LonDeg": float(params["PositionLonDeg"]),
                    "HaeM": float(params["PositionHaeM"]),
                    "Ce": float(params["PositionCe"]),
                    "Le": float(params["PositionLe"]),
                },
            }
            if "Time" in params:
                event_data["Time"] = str(params["Time"])
            if "Start" in params:
                event_data["Start"] = str(params["Start"])
            if "Stale" in params:
                event_data["Stale"] = str(params["Stale"])
            if "Callsign" in params:
                event_data["Callsign"] = str(params["Callsign"])
            targets = self.cot_output_targets
            if "Targets" in params:
                targets = self._parse_targets(params["Targets"])
            xml_bytes = self.translator.event_json_to_xml(event_data)
            result = self._send_xml(xml_bytes, targets, event_data)
            self.enqueue_response(request_id, action, True, result)
            return

        if action == ATAK.Action.Marker.SendMarker:
            stale_seconds = None
            if "StaleSeconds" in params:
                stale_seconds = int(params["StaleSeconds"])
            position = {
                "LatDeg": float(params["PositionLatDeg"]),
                "LonDeg": float(params["PositionLonDeg"]),
                "AltM": float(params["PositionAltM"]),
                "Ce": float(params["PositionCe"]),
                "Le": float(params["PositionLe"]),
            }
            marker_event = self.translator.marker_event_json(
                callsign=str(params["Callsign"]),
                uid=str(params["Uid"]),
                cottype=str(params["CotType"]),
                position=position,
                stale_seconds=stale_seconds,
            )
            xml_bytes = self.translator.marker_xml(
                callsign=str(params["Callsign"]),
                uid=str(params["Uid"]),
                cottype=str(params["CotType"]),
                position=position,
                stale_seconds=stale_seconds,
            )
            targets = self.cot_output_targets
            if "Targets" in params:
                targets = self._parse_targets(params["Targets"])
            result = self._send_xml(xml_bytes, targets, marker_event)
            self.enqueue_response(request_id, action, True, result)
            return

        if action == ATAK.Action.GeoChat.SendGeoChat:
            message = str(params["Message"])
            to_team = str(params["ToTeam"])
            position = {
                "LatDeg": float(params["PositionLatDeg"]),
                "LonDeg": float(params["PositionLonDeg"]),
                "AltM": float(params["PositionAltM"]),
                "Ce": float(params["PositionCe"]),
                "Le": float(params["PositionLe"]),
            }
            geochat_event = {
                "Type": "b-t-f",
                "How": "m-g",
                "Message": message,
                "ToTeam": to_team,
                "PositionLatDeg": position["LatDeg"],
                "PositionLonDeg": position["LonDeg"],
                "PositionHaeM": position["AltM"],
                "PositionCe": position["Ce"],
                "PositionLe": position["Le"],
            }
            xml_bytes = self.translator.geochat_xml(message, to_team, position)
            targets = self.cot_output_targets
            if "Targets" in params:
                targets = self._parse_targets(params["Targets"])
            result = self._send_xml(xml_bytes, targets, geochat_event)
            self.enqueue_response(request_id, action, True, result)
            return

        self.enqueue_response(request_id, action, False, {"error": f"unknown action {action}"})

    def _poll_network(self, timeout_s: float) -> None:  # Poll UDP/TCP sockets for inbound CoT payloads.
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
                messages = self.tcp_listener.recv_ready(sock)
                for payload, source in messages:
                    self._handle_inbound(payload, source)
                continue
            if self.tcp_client is not None and self.tcp_client.socket() is sock:
                messages = self.tcp_client.recv_ready()
                for payload, source in messages:
                    self._handle_inbound(payload, source)
                continue
            receiver = receiver_by_fileno[sock.fileno()]
            payload, source = receiver.recv()
            self._handle_inbound(payload, source)
        self._sync_tcp_client_connected()

    def run(self) -> None:  # Run ATAK interface loop.
        self.send_online()
        self._sync_tcp_client_connected()
        self._update_state(ATAK.State.System.RxCount, self.rx_count)
        self._update_state(ATAK.State.System.TxCount, self.tx_count)
        self._update_state(ATAK.State.System.RxParseErrors, self.rx_parse_errors)
        self._update_state(ATAK.State.System.LastError, self.last_error)
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
        except (KeyboardInterrupt, SystemExit):
            pass
        except RuntimeError:
            self.publish_error(traceback.format_exc().strip())
            raise
        finally:
            self.stop()

    def stop(self) -> None:  # Close ATAK sockets and stop plugin.
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
