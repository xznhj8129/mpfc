#!/usr/bin/env python3
"""Lattice task specification URLs carried by the Lattice path.

Defined once here because both the Lattice interface plugin (entity task catalog) and cores (execution policy) must agree on them.
"""

TRANSIT_SPECIFICATION_URL = "type.googleapis.com/anduril.tasks.v2.Transit"
ORBIT_SPECIFICATION_URL = "type.googleapis.com/anduril.sample_app_auto_reconnaissance.v1.Orbit"
SUPPORTED_SPECIFICATION_URLS = (TRANSIT_SPECIFICATION_URL, ORBIT_SPECIFICATION_URL)
