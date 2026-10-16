"""Task interpretation owned by the Lattice demo flight core."""
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict
from lib.lattice_fields import _field, _float_field
from lib.lattice_specs import TRANSIT_SPECIFICATION_URL, SUPPORTED_SPECIFICATION_URLS
from protocols.namespace_loader import load_protocol_namespace

LATTICE = load_protocol_namespace("lattice")

ALTITUDE_REFERENCE_HAE = LATTICE.Enums.LatticeAltitudeReference.Hae
ALTITUDE_REFERENCE_AGL = LATTICE.Enums.LatticeAltitudeReference.Agl
ALTITUDE_REFERENCE_RELATIVE = LATTICE.Enums.LatticeAltitudeReference.Relative
ALTITUDE_REFERENCE_ASL = LATTICE.Enums.LatticeAltitudeReference.Asl
ALTITUDE_REFERENCE_BY_WIRE = {
    "ALTITUDE_REFERENCE_HEIGHT_ABOVE_WGS84": ALTITUDE_REFERENCE_HAE,
    "ALTITUDE_REFERENCE_HEIGHT_ABOVE_EGM96": ALTITUDE_REFERENCE_ASL,
}


def _altitude_reference(raw: Any, default: str) -> str:  # Normalize an SDK altitude reference to a protocol key.
    if raw is None:
        return default
    if str(raw) not in ALTITUDE_REFERENCE_BY_WIRE:
        raise ValueError(f"unsupported altitude reference {raw!r}")
    return ALTITUDE_REFERENCE_BY_WIRE[str(raw)]



@dataclass(frozen=True)
class DecodedTask:
    task_id: str
    specification_type: str
    display_name: str
    latitude_deg: float | None
    longitude_deg: float | None
    altitude_m: float | None
    altitude_reference: str | None
    orbit_radius_m: float | None
    orbit_height_m: float | None
    orbit_direction: str | None
    orbit_entity_id: str | None
    received_at_ms: int
    task_json: str

    def payload(self) -> Dict[str, Any]:  # Flight-core interpretation; native TaskJson remains intact.
        return {
            "TaskId": self.task_id,
            "SpecificationType": self.specification_type,
            "DisplayName": self.display_name,
            "LatitudeDeg": self.latitude_deg,
            "LongitudeDeg": self.longitude_deg,
            "AltitudeM": self.altitude_m,
            "AltitudeReference": self.altitude_reference,
            "OrbitRadiusM": self.orbit_radius_m,
            "OrbitHeightM": self.orbit_height_m,
            "OrbitDirection": self.orbit_direction,
            "OrbitEntityId": self.orbit_entity_id,
            "ReceivedAtMs": self.received_at_ms,
            "TaskJson": self.task_json,
        }


class TaskDecoder:
    """Interpret native Lattice task specifications for this flight core."""

    def __init__(
        self,
        resolve_entity: Callable[[str], Any] | None = None,
    ) -> None:
        self._resolve_entity = resolve_entity

    def decode(self, task: Any) -> DecodedTask:  # Decode one SDK Task document.
        version = _field(task, "version")
        task_id = "" if version is None else str(_field(version, "task_id") or "")
        specification = _field(task, "specification")
        spec_url = "" if specification is None else str(_field(specification, "type") or "")
        if not task_id:
            raise ValueError("task document has no task_id")
        if spec_url not in SUPPORTED_SPECIFICATION_URLS:
            raise ValueError(f"unsupported task specification {spec_url!r}")
        display_name = _field(task, "display_name")
        if spec_url == TRANSIT_SPECIFICATION_URL:
            destination = self._transit_destination(specification)
            orbit_radius_m = orbit_height_m = orbit_direction = orbit_entity_id = None
        else:
            orbit = self._orbit_spec(specification)
            destination = orbit["destination"]
            orbit_radius_m = orbit["radius_m"]
            orbit_height_m = orbit["height_m"]
            orbit_direction = orbit["direction"]
            orbit_entity_id = orbit["entity_id"]
        return DecodedTask(
            task_id=task_id,
            specification_type=spec_url,
            display_name="" if display_name is None else str(display_name),
            latitude_deg=destination["latitude_deg"],
            longitude_deg=destination["longitude_deg"],
            altitude_m=destination["altitude_m"],
            altitude_reference=destination["altitude_reference"],
            orbit_radius_m=orbit_radius_m,
            orbit_height_m=orbit_height_m,
            orbit_direction=orbit_direction,
            orbit_entity_id=orbit_entity_id,
            received_at_ms=int(time.time() * 1000),
            task_json=(task.model_dump_json(by_alias=True, exclude_unset=True)
                       if hasattr(task, "model_dump_json") else json.dumps(task)),
        )

    def resolve_objective(self, objective: Any) -> Dict[str, Any]:  # Resolve a task objective to lat/lon/altitude.
        if not isinstance(objective, dict):
            raise ValueError("task objective must be an object")
        entity_id = _field(objective, "entityId", "entity_id")
        if entity_id:
            if self._resolve_entity is None:
                raise ValueError(f"objective entity {entity_id} cannot be resolved by this interface")
            entity = self._resolve_entity(str(entity_id))
            if entity is None:
                raise ValueError(f"objective entity {entity_id} not found")
            location = _field(entity, "location")
            position = _field(location, "position") if location is not None else None
            if position is None:
                raise ValueError(f"objective entity {entity_id} has no known position")
            latitude = _float_field(position, "latitude_degrees", "latitudeDegrees")
            longitude = _float_field(position, "longitude_degrees", "longitudeDegrees")
            if latitude is None or longitude is None:
                raise ValueError(f"objective entity {entity_id} has no usable position")
            altitude = _float_field(position, "altitude_hae_meters", "altitudeHaeMeters")
            reference = ALTITUDE_REFERENCE_HAE
            if altitude is None:
                altitude = _float_field(position, "altitude_agl_meters", "altitudeAglMeters")
                reference = ALTITUDE_REFERENCE_AGL
            if altitude is None:
                raise ValueError(f"objective entity {entity_id} has no known altitude")
            return {
                "latitude_deg": latitude,
                "longitude_deg": longitude,
                "altitude_m": altitude,
                "altitude_reference": reference,
            }
        point = _field(objective, "point")
        if point:
            lla = _field(point, "lla")
            if not isinstance(lla, dict):
                raise ValueError("point objective has no lla")
            return self._lla_destination(lla)
        # Sample-app Orbit objectives carry the lla oneof directly.
        lla = _field(objective, "lla")
        if isinstance(lla, dict):
            return self._lla_destination(lla)
        raise ValueError(f"unsupported task objective {objective!r}")

    @staticmethod
    def _lla_destination(lla: dict) -> Dict[str, Any]:  # Decode a LLA waypoint.
        latitude = _float_field(lla, "latitudeDegrees", "latitude_degrees", "lat")
        longitude = _float_field(lla, "longitudeDegrees", "longitude_degrees", "lon")
        altitude = _float_field(lla, "altitudeHaeM", "altitude_hae_meters", "alt", "altitudeM")
        if latitude is None or longitude is None:
            raise ValueError(f"objective lla missing lat/lon: {lla}")
        if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
            raise ValueError(f"objective lla has invalid coordinates: {lla}")
        if _field(lla, "is2d"):
            raise ValueError("This core requires a three-dimensional destination")
        if altitude is None:
            raise ValueError(f"objective lla missing altitude: {lla}")
        if not math.isfinite(altitude):
            raise ValueError("objective altitude must be finite")
        if "alt" in lla and _field(lla, "altitudeReference", "altitude_reference") is None:
            raise ValueError("Native LLA altitudeReference is required")
        reference = _altitude_reference(
            _field(lla, "altitudeReference", "altitude_reference"), ALTITUDE_REFERENCE_HAE
        )
        return {
            "latitude_deg": latitude,
            "longitude_deg": longitude,
            "altitude_m": altitude,
            "altitude_reference": reference,
        }

    def _transit_destination(self, specification: Any) -> Dict[str, Any]:  # Decode a Transit final waypoint.
        plan = _field(specification, "plan")
        route = _field(plan, "route")
        path = _field(route, "path")
        if not isinstance(path, list) or not path:
            raise ValueError("Transit task has no route path")
        if len(path) != 1:
            raise ValueError("This core supports a single Transit waypoint, not a multi-segment route")
        segment = path[0]
        waypoint = _field(segment, "waypoint")
        if waypoint:
            point = _field(waypoint, "llaPoint", "lla_point")
            lla = _field(point, "lla")
            if isinstance(lla, dict):
                return self._lla_destination(lla)
        loiter = _field(segment, "loiter")
        if loiter:
            raise ValueError("Transit loiter segments are not supported by this core")
        raise ValueError("Transit task final path segment has no point")

    def _orbit_spec(self, specification: Any) -> Dict[str, Any]:  # Decode a sample-app Orbit task.
        objective = _field(specification, "objective")
        destination = self.resolve_objective(objective)
        radius = _float_field(specification, "orbitRadius", "orbit_radius")
        height = _float_field(specification, "orbitHeight", "orbit_height")
        if radius is None:
            raise ValueError("Orbit task missing orbitRadius")
        if height is None:
            raise ValueError("Orbit task missing orbitHeight")
        direction = str(_field(specification, "orbitDirection", "orbit_direction") or "ORBIT_CLOCKWISE")
        return {
            "destination": destination,
            "radius_m": radius,
            "height_m": height,
            "direction": direction,
            "entity_id": _field(objective, "entityId", "entity_id") or None,
        }


