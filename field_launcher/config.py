"""Device preparation contract; no connection or camera setup is required at launch."""
from pathlib import Path
from typing import Annotated, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, SecretStr, model_validator


class Configuration(BaseModel):
    model_config = ConfigDict(extra="forbid")


class Connection(Configuration):
    url: str
    ca: Path
    token: SecretStr | None = None
    client_id: str | None = None
    client_secret: SecretStr | None = None

    @model_validator(mode="after")
    def credentials(self):
        if not self.token and not (self.client_id and self.client_secret):
            raise ValueError("Field setup requires a token or OAuth client credentials")
        return self


class Telemetry(Configuration):
    connection: str
    target_sysid: int = Field(ge=1, le=255)
    target_compid: int = Field(default=1, ge=1, le=255)


class USBCamera(Configuration):
    kind: Literal["v4l2"]
    device: Path

    def input_arguments(self) -> list[str]:
        return ["-f", "v4l2", "-i", str(self.device)]


class RTSPCamera(Configuration):
    kind: Literal["rtsp"]
    url: str

    def input_arguments(self) -> list[str]:
        return ["-rtsp_transport", "tcp", "-i", self.url]


class VideoFile(Configuration):
    """A recorded source, replayed in real time through the same encoder and remote ingress."""
    kind: Literal["file"]
    path: Path

    def input_arguments(self) -> list[str]:
        return ["-re", "-stream_loop", "-1", "-i", str(self.path)]


class VideoSource(Configuration):
    name: str = Field(min_length=1, max_length=64)
    input: Annotated[USBCamera | RTSPCamera | VideoFile, Field(discriminator="kind")]
    bitrate_kbps: int = Field(default=1500, ge=100, le=20000)


class Asset(Configuration):
    entity_id: str = Field(min_length=1)
    name: str = Field(min_length=1, max_length=64)
    telemetry: Telemetry | None = None
    videos: list[VideoSource] = Field(default_factory=list)

    @model_validator(mode="after")
    def has_source(self):
        if self.telemetry is None and not self.videos:
            raise ValueError(f"{self.entity_id} has no information or video source")
        return self


class Phone(Configuration):
    tak_uid: str = Field(min_length=1)
    callsign: str = Field(min_length=1, max_length=64)
    ingress_id: str = Field(min_length=4, max_length=36, pattern=r"^[A-Za-z0-9_.~-]+$")

    @property
    def entity_id(self) -> str:
        return "tak:" + self.tak_uid


class FieldSettings(Configuration):
    connection: Connection
    group_id: str = Field(min_length=1)
    group_name: str = Field(min_length=1)
    assets: list[Asset] = Field(default_factory=list)
    phone: Phone | None = None
    mqtt_port: int = Field(default=18884, ge=1, le=65535)
    state_directory: Path = Path("field-state")
    poll_seconds: float = Field(default=1, gt=0)

    @model_validator(mode="after")
    def unique_identities(self):
        ids = [self.group_id, *(asset.entity_id for asset in self.assets)]
        if self.phone:
            ids.append(self.phone.entity_id)
        if len(ids) != len(set(ids)):
            raise ValueError("Group, aircraft and operator must have distinct entity IDs")
        return self

    @classmethod
    def load(cls, path: Path):
        path = path.resolve()
        settings = cls.model_validate(yaml.safe_load(path.read_text()))
        settings.connection.ca = (path.parent / settings.connection.ca).resolve()
        settings.state_directory = (path.parent / settings.state_directory).resolve()
        for asset in settings.assets:
            for video in asset.videos:
                if isinstance(video.input, USBCamera):
                    video.input.device = (path.parent / video.input.device).resolve()
                elif isinstance(video.input, VideoFile):
                    video.input.path = (path.parent / video.input.path).resolve()
        return settings
