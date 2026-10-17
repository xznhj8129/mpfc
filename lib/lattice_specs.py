#!/usr/bin/env python3
"""Lattice task specification URLs carried by the Lattice path.

Defined once here because both the Lattice interface plugin (entity task catalog) and cores (execution policy) must agree on them.
"""

from lib.lattice_flight_pb2 import Land, ReturnToLaunch, SearchArea

TRANSIT_SPECIFICATION_URL = "type.googleapis.com/anduril.tasks.v2.Transit"
ORBIT_SPECIFICATION_URL = "type.googleapis.com/anduril.sample_app_auto_reconnaissance.v1.Orbit"
LAND_SPECIFICATION_URL = f"type.googleapis.com/{Land.DESCRIPTOR.full_name}"
RETURN_TO_LAUNCH_SPECIFICATION_URL = f"type.googleapis.com/{ReturnToLaunch.DESCRIPTOR.full_name}"
RECOVERY_SPECIFICATIONS = {LAND_SPECIFICATION_URL: Land, RETURN_TO_LAUNCH_SPECIFICATION_URL: ReturnToLaunch}
SEARCH_AREA_SPECIFICATION_URL = f"type.googleapis.com/{SearchArea.DESCRIPTOR.full_name}"
CUSTOM_SPECIFICATIONS = {**RECOVERY_SPECIFICATIONS, SEARCH_AREA_SPECIFICATION_URL: SearchArea}
SUPPORTED_SPECIFICATION_URLS = (TRANSIT_SPECIFICATION_URL, ORBIT_SPECIFICATION_URL, *CUSTOM_SPECIFICATIONS)
