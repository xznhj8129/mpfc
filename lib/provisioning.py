"""Provisioned deployment identity for this MPFC node.

Sigma's provisioning service exports one identity file per deployed asset and
``mpfc.sh`` stages it as ``provisioning/<handle>.json`` next to the MPFC
checkout. The canonical asset identity used by OCCID commands and execution
status is the provisioned entity UID.
"""
from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path

from occid import UID
from sigma_sdk.models.identity import DeploymentIdentity
from sigma_sdk.provisioning import load_identity


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROVISIONING = REPO_ROOT / "provisioning" / "UAV1.json"


def provisioning_path() -> Path:
    raw = os.environ.get("MPFC_PROVISIONING")
    return Path(raw) if raw else DEFAULT_PROVISIONING


@lru_cache(maxsize=1)
def deployment_identity() -> DeploymentIdentity:
    path = provisioning_path()
    if not path.is_file():
        raise FileNotFoundError(f"MPFC provisioning identity not found: {path}")
    return load_identity(path)


def asset_uid() -> UID:
    """Return the canonical OCCID Entity UID for the provisioned asset."""
    return deployment_identity().entity_uid


def node_uid() -> UID:
    """Return the canonical OCCID Node UID for this deployed node."""
    return deployment_identity().node_uid
