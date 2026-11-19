"""Prepare a node configuration through MPFC's own configuration contract.

Input is NodeSettings JSON on stdin; output is the prepared native configuration on stdout.
No broker, autopilot or task connection is opened. Core policy stays in MPFC.
"""
import json
import sys
from pathlib import Path

from pydantic import BaseModel, Field
from anduril import Relationships

from main import (apply_plugin_config_templates, configure_runtime_plugins, load_config,
                  plugin_supports_vehicle, resolve_plugin_supports, resolve_vehicle_config)


class NodeSettings(BaseModel):
    template: Path
    runtime_root: Path
    asset_id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    endpoint: str
    ca_bundle: str
    client_id: str | None = None
    client_secret: str | None = None
    token: str | None = None
    mqtt_port: int = Field(ge=1, le=65535)
    connection: str
    target_sysid: int | None = Field(default=None, ge=1, le=255)
    target_compid: int | None = Field(default=None, ge=1, le=255)
    relationships: Relationships | None = None
    home_altitude_hae_m: float | None = None
    remove_launcher: bool = False


class NodeDeployment:
    def __init__(self, root: Path):
        self.root = root

    def prepare(self, settings: NodeSettings) -> dict:
        source = (self.root / settings.template).resolve()
        if not source.is_relative_to(self.root):
            raise ValueError("MPFC template must be inside its source root")
        config = load_config(source)
        apply_plugin_config_templates(config, self.root)
        vehicle = resolve_vehicle_config(config)
        resolve_plugin_supports(config)
        configure_runtime_plugins(config, vehicle)
        config["my_name"] = settings.name
        config["bus_config"].update(schema_path=str(settings.runtime_root / "config/bus_topics_schema.json"))
        config["bus_config"]["endpoint"].update(host="127.0.0.1", port=settings.mqtt_port)
        controllers = [entry["cfg"] for entry in config["plugins"] if entry["cfg"].get("is_controller")]
        interfaces = [entry["cfg"] for entry in config["plugins"]
                      if entry["cfg"].get("is_interface") and vehicle is not None
                      and plugin_supports_vehicle(entry["cfg"], vehicle)]
        if len(interfaces) != 1:
            raise ValueError(f"Node needs exactly one vehicle interface, found {len(interfaces)}")
        interfaces[0]["conn_str"] = settings.connection
        if settings.target_sysid is not None:
            interfaces[0]["target_sysid"] = settings.target_sysid
        if settings.target_compid is not None:
            interfaces[0]["target_compid"] = settings.target_compid
        core = config.get("core")
        if core is not None and "home_altitude_hae_m" in core["cfg"].get("policy", {}):
            if settings.home_altitude_hae_m is None:
                raise ValueError("This flight core requires an explicit WGS84 home HAE")
            core["cfg"]["policy"]["home_altitude_hae_m"] = settings.home_altitude_hae_m
        lattice = [entry["cfg"] for entry in config["plugins"] if entry["plugin"] == "lattice"]
        if len(lattice) != 1:
            raise ValueError("Node needs exactly one Lattice interface")
        lattice[0].update(base_url=settings.endpoint, ca_bundle=settings.ca_bundle,
                          client_id=settings.client_id, client_secret=settings.client_secret, token=settings.token,
                          asset_id=settings.asset_id, home_altitude_hae_m=settings.home_altitude_hae_m)
        if settings.relationships is not None:
            lattice[0]["relationships"] = settings.relationships.model_dump(mode="json", by_alias=True, exclude_none=True)
        if core is None:
            lattice[0]["accept_tasks"] = False
            for controller in controllers:
                controller["receive_only"] = True
        if settings.remove_launcher:
            config["plugins"] = [entry for entry in config["plugins"] if entry["plugin"] != "px4_sitl_gz_launcher"]
        return config


def main() -> None:
    settings = NodeSettings.model_validate_json(sys.stdin.read())
    print(json.dumps(NodeDeployment(Path(__file__).resolve().parents[1]).prepare(settings)))


if __name__ == "__main__":
    main()
