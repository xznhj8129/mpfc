"""Task specification parsing and arrival math for the Lattice execution ingress."""

from __future__ import annotations

from anduril import Task

from lib.mavlink_models import GlobalPositionInt
from plugins.execution_ingress.execution_ingress import (
    ORBIT_SPECIFICATION_URL,
    TRANSIT_SPECIFICATION_URL,
    Destination,
    ExecutionIngress,
)


def _bare_ingress() -> ExecutionIngress:
    return ExecutionIngress.__new__(ExecutionIngress)


def _task(specification: dict) -> Task:
    return Task.model_validate(
        {
            "version": {"taskId": "task-1", "definitionVersion": 1, "statusVersion": 1},
            "status": {"status": "STATUS_CREATED"},
            "specification": specification,
        }
    )


def test_transit_destination_from_waypoint() -> None:
    task = _task(
        {
            "@type": TRANSIT_SPECIFICATION_URL,
            "plan": {
                "route": {
                    "path": [
                        {
                            "waypoint": {
                                "llaPoint": {
                                    "lat": 36.53,
                                    "lon": -83.21,
                                    "alt": 120.0,
                                    "altitudeReference": "ALTITUDE_REFERENCE_HEIGHT_ABOVE_WGS84",
                                }
                            }
                        }
                    ]
                }
            },
        }
    )
    ingress = _bare_ingress()
    destination = ingress._transit_destination(task.specification)
    assert destination.latitude_deg == 36.53
    assert destination.longitude_deg == -83.21
    assert destination.altitude_m == 120.0
    assert destination.altitude_reference == "hae"


def test_orbit_spec_from_sample_schema() -> None:
    task = _task(
        {
            "@type": ORBIT_SPECIFICATION_URL,
            "objective": {
                "lla": {
                    "latitudeDegrees": 21.12,
                    "longitudeDegrees": -11.40,
                    "altitudeHaeM": 900.0,
                }
            },
            "orbitRadius": 1000.0,
            "orbitHeight": 100.0,
            "orbitDirection": "ORBIT_CLOCKWISE",
        }
    )
    ingress = _bare_ingress()
    orbit = ingress._orbit_spec(task.specification)
    assert orbit.latitude_deg == 21.12
    assert orbit.altitude_hae_m == 900.0
    assert orbit.radius_m == 1000.0
    assert orbit.height_m == 100.0
    assert orbit.direction == "ORBIT_CLOCKWISE"


def test_arrival_metrics_relative_altitude() -> None:
    ingress = _bare_ingress()
    ingress.home_altitude_hae_m = None
    location = GlobalPositionInt(
        latitude_deg=36.530440,
        longitude_deg=-83.216383,
        altitude_m=300.0,
        relative_altitude_m=100.0,
        vx_m_s=0.0,
        vy_m_s=0.0,
        vz_m_s=0.0,
        heading_deg=0.0,
    )
    destination = Destination(
        latitude_deg=36.531340,
        longitude_deg=-83.216383,
        altitude_m=100.0,
        altitude_reference="relative",
    )
    horizontal_m, altitude_error_m = ingress._arrival_metrics(location, destination)
    assert 90.0 < horizontal_m < 110.0
    assert altitude_error_m == 0.0
