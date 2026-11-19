"""One field session owns a broker, normal MPFC observers, and local video publishers."""
import os
import sys
from contextlib import ExitStack
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import yaml
from anduril import SrtSettings
from anduril.errors import NotFoundError

from field_launcher.config import Asset, FieldSettings, VideoSource
from field_launcher.media import MediaBindings
from field_launcher.process import ManagedProcess
from field_launcher.registry import FieldRegistry
from lib.deployment import NodeDeployment, NodeSettings

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Delivery:
    source: str
    received: bool
    detail: str


@dataclass
class VideoPublisher:
    asset: Asset
    source: VideoSource
    ingress_id: str
    process: ManagedProcess


class FieldSession:
    def __init__(self, settings: FieldSettings, selected: list[Asset]):
        self.settings, self.selected = settings, selected
        self.registry = FieldRegistry(settings)
        self.folder = settings.state_directory / uuid4().hex
        self.folder.mkdir(parents=True, mode=0o700)
        self.cleanup = ExitStack()
        self.cleanup.callback(self.registry.close)
        self.processes: list[ManagedProcess] = []
        self.observers: dict[str, ManagedProcess] = {}
        self.videos: list[VideoPublisher] = []
        self.bindings: dict[str, MediaBindings] = {}
        self.broker: ManagedProcess | None = None
        self.started = datetime.now(timezone.utc)

    def __enter__(self):
        return self

    def __exit__(self, *exception):
        # Capture stops before any network cleanup, even when the remote server is unavailable.
        with ExitStack() as stopping:
            stopping.callback(self.cleanup.__exit__, *exception)
            for process in self.processes:
                stopping.callback(process.stop)

    def launch(self, command, label, environment=None):
        process = ManagedProcess(command, ROOT, self.folder / (label + ".log"), environment)
        process.start()
        self.processes.append(process)
        return process

    def start(self):
        self.registry.prepare_group()
        if any(asset.telemetry for asset in self.selected):
            cfg = self.folder / "mosquitto.conf"
            cfg.write_text(f"listener {self.settings.mqtt_port} 127.0.0.1\nallow_anonymous true\npersistence false\n")
            self.broker = self.launch(["mosquitto", "-c", str(cfg)], "mqtt")
        for index, asset in enumerate(self.selected):
            self.registry.ensure_identity(asset.entity_id, asset.name)
            if asset.telemetry:
                self.start_observer(asset, index)
            for source in asset.videos:
                self.start_video(asset, source)

    def start_observer(self, asset: Asset, index: int):
        connection = self.settings.connection
        telemetry = asset.telemetry
        node = NodeSettings(template=Path("config/receive_only_lattice.yaml"), runtime_root=ROOT,
            asset_id=asset.entity_id, name=f"field-{index}", endpoint=connection.url, ca_bundle=str(connection.ca),
            token=connection.token.get_secret_value() if connection.token else None,
            client_id=connection.client_id,
            client_secret=connection.client_secret.get_secret_value() if connection.client_secret else None,
            mqtt_port=self.settings.mqtt_port, connection=telemetry.connection,
            target_sysid=telemetry.target_sysid, target_compid=telemetry.target_compid,
            relationships=self.registry.membership())
        config = NodeDeployment(ROOT).prepare(node)
        config["bus_config"]["log_file"] = str(self.folder / f"bus-{index}.log")
        path = self.folder / f"observer-{index}.yaml"
        path.write_text(yaml.safe_dump(config))
        path.chmod(0o600)
        self.observers[asset.entity_id] = self.launch([sys.executable, "main.py"], f"observer-{index}",
            dict(os.environ, MAIN_CONFIG=str(path)))

    def start_video(self, asset: Asset, source: VideoSource):
        ingress_id = str(uuid4())
        transport = self.registry.sdk.video.create_ingress_stream(ingress_id=ingress_id,
            title=source.name, srt=SrtSettings()).srt
        self.cleanup.callback(self.registry.archive_stream, ingress_id)
        if asset.entity_id not in self.bindings:
            self.bindings[asset.entity_id] = MediaBindings(self.registry, asset.entity_id)
        bindings = self.bindings[asset.entity_id]
        bindings.add(ingress_id)
        self.cleanup.callback(bindings.remove, ingress_id)
        command = ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "warning",
            *source.input.input_arguments(), "-an", "-c:v", "libx264", "-preset", "ultrafast",
            "-tune", "zerolatency", "-profile:v", "baseline", "-pix_fmt", "yuv420p", "-bf", "0",
            "-b:v", f"{source.bitrate_kbps}k", "-g", "30", "-f", "mpegts", transport.url]
        process = self.launch(command, "video-" + ingress_id)
        self.videos.append(VideoPublisher(asset, source, ingress_id, process))

    def deliveries(self) -> list[Delivery]:
        if self.broker and self.broker.process.poll() is not None:
            raise RuntimeError(f"Field broker exited; log={self.broker.log}")
        deliveries = []
        for asset in self.selected:
            observer = self.observers.get(asset.entity_id)
            if observer is not None:
                running = observer.maintain()
                try:
                    entity = self.registry.sdk.entities.get_entity(asset.entity_id)
                except NotFoundError:
                    entity = None
                observed = entity.provenance.source_update_time if entity and entity.provenance else None
                connected = bool(entity and entity.health and
                    entity.health.connection_status == "CONNECTION_STATUS_ONLINE" and
                    entity.expiry_time and entity.expiry_time > datetime.now(timezone.utc) and
                    observed and observed >= self.started)
                received = running and connected
                detail = ("Telemetry received; position received" if entity and entity.location else
                          "Telemetry received; position unavailable") if received else "Waiting for telemetry"
                deliveries.append(Delivery(asset.name, received, detail))
        received_video_assets = set()
        for video in self.videos:
            running = video.process.maintain()
            stream = self.registry.sdk.video.get_ingress_stream(ingress_id=video.ingress_id).ingress_stream
            received = running and stream.status == "STREAM_STATUS_LIVE"
            if received:
                received_video_assets.add(video.asset.entity_id)
            deliveries.append(Delivery(f"{video.asset.name} / {video.source.name}", received,
                "Video received remotely" if received else f"Waiting for video; log={video.process.log.name}"))
        for asset in self.selected:
            if asset.telemetry is None:
                received = asset.entity_id in received_video_assets
                self.registry.publish_video_source(asset.entity_id, asset.name, received)
        return deliveries
