"""Polygon coverage planning from native Lattice GEO entities."""
import math
from dataclasses import dataclass
from enum import Enum

from anduril import Entity, Position
from geographiclib.geodesic import Geodesic
from froggeolib import GimbalCamera
from shapely.geometry import LineString, Polygon


class SearchHeadingMode(str, Enum):
    DIRECTION_OF_TRAVEL = "direction_of_travel"
    LANE = "lane"


@dataclass(frozen=True)
class AreaSearchPlan:
    area_entity_id: str
    area_square_meters: float
    waypoints: tuple[Position, ...]
    headings_deg: tuple[float, ...]

    @classmethod
    def build(cls, entity: Entity, height_m: float, spacing_m: float, camera: GimbalCamera) -> "AreaSearchPlan":
        polygon = entity.geo_shape.polygon if entity.geo_shape else None
        if polygon is None or not polygon.rings:
            raise ValueError(f"Search area {entity.entity_id} requires a polygon")
        if height_m <= 0 or spacing_m <= 0:
            raise ValueError(f"Search height and lane spacing must be positive: height={height_m}, spacing={spacing_m}")
        if spacing_m > camera.near_swath_m(height_m):
            raise ValueError(f"Lane spacing {spacing_m} exceeds camera's near-edge swath {camera.near_swath_m(height_m):.1f} m")
        standoff_m = camera.search_standoff_m(height_m)
        rings = [[vertex.position for vertex in ring.positions] for ring in polygon.rings]
        origin = rings[0][0]
        if any(point.altitude_hae_meters is None for ring in rings for point in ring):
            raise ValueError(f"Search area {entity.entity_id} has no ground HAE on every vertex")
        ground_hae = origin.altitude_hae_meters
        if any(abs(point.altitude_hae_meters - ground_hae) > 1 for ring in rings for point in ring):
            raise ValueError("Area search currently requires a level polygon; terrain following is not implemented")

        def local(point):
            vector = Geodesic.WGS84.Inverse(origin.latitude_degrees, origin.longitude_degrees,
                                           point.latitude_degrees, point.longitude_degrees)
            azimuth = math.radians(vector['azi1'])
            return vector['s12'] * math.sin(azimuth), vector['s12'] * math.cos(azimuth)

        outlines = [[local(point) for point in ring] for ring in rings]
        region = Polygon(outlines[0], outlines[1:])
        if not region.is_valid or region.area <= 0:
            raise ValueError(f"Search area {entity.entity_id} is not a valid polygon")
        min_x, min_y, max_x, max_y = region.bounds
        lanes = max(1, math.ceil((max_y - min_y) / spacing_m))
        coordinates = []
        headings = []
        for lane in range(lanes):
            y = min_y + (lane + .5) * (max_y - min_y) / lanes
            section = region.intersection(LineString([(min_x - 1, y), (max_x + 1, y)]))
            segments = [section] if section.geom_type == 'LineString' else list(section.geoms)
            segments = sorted((segment for segment in segments if segment.length > 0), key=lambda segment: segment.bounds[0])
            points = [point for segment in segments for point in (segment.coords[0], segment.coords[-1])]
            direction = -1 if lane % 2 else 1
            ordered = list(reversed(points)) if lane % 2 else points
            coordinates.extend((x - direction * standoff_m, y) for x, y in ordered)
            headings.extend([270.0 if lane % 2 else 90.0] * len(ordered))
        waypoints = []
        for x, y in coordinates:
            point = Geodesic.WGS84.Direct(origin.latitude_degrees, origin.longitude_degrees,
                                         math.degrees(math.atan2(x, y)), math.hypot(x, y))
            waypoints.append(Position(latitude_degrees=point['lat2'], longitude_degrees=point['lon2'],
                                      altitude_hae_meters=ground_hae + height_m))
        return cls(entity.entity_id, region.area, tuple(waypoints), tuple(headings))
