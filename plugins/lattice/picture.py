"""Vehicle observations to native Lattice entities; location validity is independent of media.

Freshness uses the backend's observation time through the shared bus. Expired location is omitted.
The edge process may remain live with offline telemetry and working video. Connectivity does not
assert aircraft health. Configured native relationships are owned by this publisher.
"""
from datetime import datetime, timedelta, timezone
from typing import Any, Mapping

from anduril import (Aliases, Classification, ClassificationInformation, Entity, GroupDetails,
                     Health, Location, Media, MilView, Ontology, Position, Provenance,
                     Quaternion, Relationships, Status, TaskCatalog, TaskDefinition)
from pydantic import BaseModel, ConfigDict, Field

from lib.altitude import relative_to_hae
from lib.lattice_specs import SUPPORTED_SPECIFICATION_URLS
from lib.reference_frames import attitude_enu_quaternion
from protocols.namespace_loader import load_protocol_namespace

UAV = load_protocol_namespace("uav")


class PositionTelemetry(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    latitude: float = Field(alias="LatDeg", ge=-90, le=90)
    longitude: float = Field(alias="LonDeg", ge=-180, le=180)
    relative_altitude_m: float = Field(alias="RelAltM")

    def position(self, home_hae_m: float | None) -> Position:
        altitude = None if home_hae_m is None else relative_to_hae(self.relative_altitude_m, home_hae_m)
        return Position(latitude_degrees=self.latitude, longitude_degrees=self.longitude, altitude_hae_meters=altitude)


class AttitudeTelemetry(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    roll: float = Field(alias="Roll")
    pitch: float = Field(alias="Pitch")
    yaw: float = Field(alias="Yaw")

    def quaternion(self) -> Quaternion:
        w, x, y, z = attitude_enu_quaternion(self.roll, self.pitch, self.yaw)
        return Quaternion(w=w, x=x, y=y, z=z)


class VehiclePublication(BaseModel):
    asset_id: str = Field(min_length=1)
    callsign: str
    integration_name: str
    expiry_s: float = Field(gt=0)
    telemetry_freshness_s: float = Field(gt=0)
    home_altitude_hae_m: float | None = None
    relationships: Relationships | None = None
    group_details: GroupDetails | None = None


class VehiclePicture:
    def __init__(self, config: VehiclePublication):
        self.config = config

    def fresh(self, observed_at: Mapping[str, datetime], key: str, now: datetime) -> bool:
        observed = observed_at.get(key)
        return observed is not None and 0 <= (now - observed).total_seconds() <= self.config.telemetry_freshness_s

    def entity(self, state: Mapping[str, Any], observed_at: Mapping[str, datetime], media: Media,
               accept_tasks: bool, activity: str | None) -> Entity:
        now = datetime.now(timezone.utc)
        connected = state.get(UAV.State.System.FcConnected) is True and self.fresh(
            observed_at, UAV.State.System.FcConnected, now)
        location = None
        position_time = observed_at.get(UAV.State.Navigation.Position)
        if connected and state.get(UAV.State.Navigation.Position) is not None and self.fresh(
                observed_at, UAV.State.Navigation.Position, now):
            position = PositionTelemetry.model_validate(state[UAV.State.Navigation.Position])
            attitude = None
            if state.get(UAV.State.Attitude.AttitudeRad) is not None and self.fresh(
                    observed_at, UAV.State.Attitude.AttitudeRad, now):
                attitude = AttitudeTelemetry.model_validate(state[UAV.State.Attitude.AttitudeRad]).quaternion()
            location = Location(position=position.position(self.config.home_altitude_hae_m), attitude_enu=attitude)
        observed = position_time if location is not None else now
        expiry = min(now + timedelta(seconds=self.config.expiry_s),
                     position_time + timedelta(seconds=self.config.telemetry_freshness_s)) if location is not None else (
                         now + timedelta(seconds=self.config.expiry_s))
        return Entity(
            entity_id=self.config.asset_id, aliases=Aliases(name=self.config.callsign),
            is_live=True, expiry_time=expiry,
            data_classification=Classification(default=ClassificationInformation(level="CLASSIFICATION_LEVELS_UNCLASSIFIED")),
            health=Health(connection_status="CONNECTION_STATUS_ONLINE" if connected else "CONNECTION_STATUS_OFFLINE",
                          update_time=observed_at.get(UAV.State.System.FcConnected)),
            status=Status(platform_activity=activity) if connected and activity is not None else None,
            location=location,
            mil_view=MilView(disposition="DISPOSITION_FRIENDLY", environment="ENVIRONMENT_AIR", platform_type="UAV"),
            ontology=Ontology(template="TEMPLATE_ASSET", platform_type="UAV"),
            provenance=Provenance(integration_name=self.config.integration_name, data_type="MPFC UAV",
                                  source_id=self.config.asset_id, source_update_time=observed),
            task_catalog=TaskCatalog(task_definitions=[TaskDefinition(task_specification_url=url)
                                                       for url in SUPPORTED_SPECIFICATION_URLS] if accept_tasks else []),
            media=media, relationships=self.config.relationships, group_details=self.config.group_details,
        )
