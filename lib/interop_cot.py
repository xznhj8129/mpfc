"""Deterministic Cursor-on-Target <-> Lattice record conversions (MPFC-local).

  The caller owns XML
parsing/serialization, external identity mapping, provenance, network sessions,
and publication policy; this module only maps the CoT point representation to
and from Lattice location structures.
"""
from __future__ import annotations

from dataclasses import dataclass

from anduril import (
    ErrorEllipse,
    Location,
    LocationUncertainty,
    Position,
)

from lib.interop_common import require_finite


@dataclass(frozen=True)
class CotPointFields:
    """Protocol-native CoT point fields in engineering units."""

    lat_deg: float
    lon_deg: float
    hae_m: float
    ce_m: float | None = None
    le_m: float | None = None


def cot_point_to_location(
    point: CotPointFields,
) -> tuple[Location, LocationUncertainty | None]:
    ce = None if point.ce_m is None else require_finite(point.ce_m, "ce_m")
    if point.le_m is not None:
        require_finite(point.le_m, "le_m")
    uncertainty = None
    if ce is not None:
        # ENG-12: CoT circular error maps to the position error ellipse (equal
        # axes when circular); CoT linear error has no Lattice field and stays
        # on the raw local CoT record.
        uncertainty = LocationUncertainty(
            position_error_ellipse=ErrorEllipse(
                semi_major_axis_m=ce, semi_minor_axis_m=ce
            )
        )
    location = Location(
        position=Position(
            latitude_degrees=require_finite(point.lat_deg, "lat_deg"),
            longitude_degrees=require_finite(point.lon_deg, "lon_deg"),
            altitude_hae_meters=require_finite(point.hae_m, "hae_m"),
        )
    )
    return location, uncertainty


def location_to_cot_point(
    location: Location,
    uncertainty: LocationUncertainty | None = None,
) -> CotPointFields:
    if location is None or location.position is None:
        raise ValueError("Lattice location requires a global position for CoT conversion")
    position = location.position
    if position.altitude_hae_meters is None:
        raise ValueError(
            "CoT HAE requires altitude_hae_meters on the Lattice position"
        )
    ellipse = None
    if uncertainty is not None:
        ellipse = uncertainty.position_error_ellipse
    return CotPointFields(
        lat_deg=require_finite(position.latitude_degrees, "latitude_degrees"),
        lon_deg=require_finite(position.longitude_degrees, "longitude_degrees"),
        hae_m=require_finite(position.altitude_hae_meters, "altitude_hae_meters"),
        ce_m=None
        if ellipse is None or ellipse.semi_major_axis_m is None
        else require_finite(ellipse.semi_major_axis_m, "semi_major_axis_m"),
        le_m=None,
    )
