"""Provisioned deployment identity for this MPFC node.

The deployment exports one identity file per deployed asset and ``mpfc.sh``
stages it as ``provisioning/<handle>.json`` next to the MPFC checkout.  The
canonical asset identity is the provisioned entity id string (the Lattice
``Entity.entity_id``).  Node identity stays a local provisioning value; no
shared model is involved.
"""
from __future__ import annotations

import json
import os
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROVISIONING = REPO_ROOT / "provisioning" / "UAV1.json"


def provisioning_path() -> Path:
    raw = os.environ.get("MPFC_PROVISIONING")
    return Path(raw) if raw else DEFAULT_PROVISIONING


@lru_cache(maxsize=1)
def deployment_identity() -> Dict[str, Any]:
    path = provisioning_path()
    if not path.is_file():
        raise FileNotFoundError(f"MPFC provisioning identity not found: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if type(data) is not dict:
        raise ValueError(f"invalid provisioning identity root type={type(data).__name__}")
    entity_id = data.get("entity_id") or data.get("entity_uid")
    if not entity_id or type(entity_id) is not str:
        raise ValueError(f"provisioning identity missing entity_id path={path}")
    node_id = data.get("node_id") or data.get("node_uid") or ""
    return {"handle": str(data.get("handle", "")), "entity_id": entity_id, "node_id": str(node_id)}


def asset_id() -> str:
    """Return the canonical Lattice Entity.entity_id for the provisioned asset."""
    return deployment_identity()["entity_id"]


def node_id() -> str:
    """Return the local provisioning node identifier for this deployed node."""
    return deployment_identity()["node_id"]
