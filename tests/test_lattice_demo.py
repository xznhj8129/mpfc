#!/usr/bin/env python3
"""Unit tests for the Lattice demo core helpers and datum invariants."""

import math

import pytest

from flight_cores.lattice_demo.lattice_demo import (
    Destination,
    LatticeDemoError,
    arrival_metrics,
    command_altitude_m,
    observed_altitude_m,
)
from lib.altitude import hae_to_relative, home_hae_from_telemetry, relative_to_hae
from lib.lattice_specs import ORBIT_SPECIFICATION_URL, TRANSIT_SPECIFICATION_URL
from lib.reference_frames import attitude_enu_quaternion


def _quaternion_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quaternion_product(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _rotate(q, vector):
    rotated = _quaternion_product(
        _quaternion_product(q, (0.0, *vector)), _quaternion_conjugate(q)
    )
    return rotated[1], rotated[2], rotated[3]


def test_home_hae_datum_derivation_and_round_trip() -> None:
    home = home_hae_from_telemetry(138.0, 100.0)
    assert home == pytest.approx(38.0)
    assert hae_to_relative(98.0, home) == pytest.approx(60.0)
    assert relative_to_hae(60.0, home) == pytest.approx(98.0)


def test_attitude_enu_quaternion_unit_and_axes() -> None:
    north_q = attitude_enu_quaternion(0.0, 0.0, 0.0)
    assert math.sqrt(sum(component * component for component in north_q)) == pytest.approx(1.0)
    forward = _rotate(north_q, (1.0, 0.0, 0.0))
    assert forward == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)

    east_q = attitude_enu_quaternion(0.0, 0.0, math.pi / 2.0)
    forward_east = _rotate(east_q, (1.0, 0.0, 0.0))
    assert forward_east == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)


def test_destination_from_transit_task() -> None:
    task = {
        "TaskId": "t",
        "SpecificationType": TRANSIT_SPECIFICATION_URL,
        "LatitudeDeg": 36.53,
        "LongitudeDeg": -83.21,
        "AltitudeM": 160.0,
        "AltitudeReference": "Hae",
    }
    destination = Destination.from_task(task)
    assert destination.altitude_m == pytest.approx(160.0)
    assert destination.altitude_reference == "Hae"


def test_destination_from_orbit_task_adds_height() -> None:
    task = {
        "TaskId": "t",
        "SpecificationType": ORBIT_SPECIFICATION_URL,
        "LatitudeDeg": 36.53,
        "LongitudeDeg": -83.21,
        "AltitudeM": 100.0,
        "AltitudeReference": "Hae",
        "OrbitHeightM": 60.0,
    }
    destination = Destination.from_task(task)
    assert destination.altitude_m == pytest.approx(160.0)


def test_destination_missing_altitude_is_rejected() -> None:
    with pytest.raises(LatticeDemoError):
        Destination.from_task({"TaskId": "t", "LatitudeDeg": 1.0, "LongitudeDeg": 2.0})


def test_command_altitude_hae_to_relative() -> None:
    destination = Destination(36.0, -83.0, 160.0, "Hae")
    assert command_altitude_m(destination, 100.0) == pytest.approx(60.0)
    with pytest.raises(LatticeDemoError):
        command_altitude_m(destination, None)


def test_command_altitude_relative_passthrough() -> None:
    destination = Destination(36.0, -83.0, 60.0, "Relative")
    assert command_altitude_m(destination, 100.0) == pytest.approx(60.0)


def test_observed_altitude_references() -> None:
    assert observed_altitude_m("Hae", 100.0, 200.0, 60.0) == pytest.approx(160.0)
    assert observed_altitude_m("Relative", 100.0, 200.0, 60.0) == pytest.approx(60.0)
    assert observed_altitude_m("Asl", 100.0, 200.0, 60.0) == pytest.approx(200.0)
    with pytest.raises(LatticeDemoError):
        observed_altitude_m("Hae", None, 200.0, 60.0)


def test_arrival_metrics_at_and_away_from_destination() -> None:
    destination = Destination(36.530440, -83.216383, 160.0, "Hae")
    horizontal_m, altitude_error_m = arrival_metrics(
        destination, 100.0, 36.530440, -83.216383, 200.0, 60.0
    )
    assert horizontal_m == pytest.approx(0.0, abs=0.5)
    assert altitude_error_m == pytest.approx(0.0)

    horizontal_m, altitude_error_m = arrival_metrics(
        destination, 100.0, 36.531440, -83.216383, 205.0, 60.0
    )
    assert horizontal_m > 100.0
    assert altitude_error_m == pytest.approx(0.0)
