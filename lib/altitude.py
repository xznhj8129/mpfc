#!/usr/bin/env python3
"""Conversions between home-relative altitude and an explicit WGS84 HAE datum.

MSP/MAVSDK AbsAltM is AMSL and cannot establish home HAE.
"""


def hae_to_relative(hae_alt_m: float, home_hae_m: float) -> float:  # HAE altitude to home-relative altitude.
    return float(hae_alt_m) - float(home_hae_m)


def relative_to_hae(relative_alt_m: float, home_hae_m: float) -> float:  # Home-relative altitude to HAE.
    return float(home_hae_m) + float(relative_alt_m)
