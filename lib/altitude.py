#!/usr/bin/env python3
"""Home altitude datum invariants.

MAVLink reports altitude relative to the home position (``relative_alt``)
while Lattice tasks carry a WGS84 ellipsoid (HAE) altitude.  The home HAE
datum is the one conversion constant between the two; these functions are
the single definition of that arithmetic.

The datum is derived from live telemetry as ``absolute_alt - relative_alt``
and is never guessed: callers must fault visibly when it is unavailable.
"""


def home_hae_from_telemetry(absolute_alt_m: float, relative_alt_m: float) -> float:  # Derive home HAE datum.
    return float(absolute_alt_m) - float(relative_alt_m)


def hae_to_relative(hae_alt_m: float, home_hae_m: float) -> float:  # HAE altitude to home-relative altitude.
    return float(hae_alt_m) - float(home_hae_m)


def relative_to_hae(relative_alt_m: float, home_hae_m: float) -> float:  # Home-relative altitude to HAE.
    return float(home_hae_m) + float(relative_alt_m)
