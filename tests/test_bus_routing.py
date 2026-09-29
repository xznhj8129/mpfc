"""Bus codec round-trips for MAVLink records and Lattice documents."""

from __future__ import annotations

from anduril import Entity, Location, MilView, Position

from lib.lattice_bus import entity_document, unpack_lattice
from lib.mavlink_models import (
    CommandLong,
    ControlAxes,
    ControlChannelOverride,
    ControlOverride,
    RcChannels,
    ReceiverConfig,
    RemoteControlState,
    pack_record,
    unpack_record,
)


def test_mavlink_record_round_trip() -> None:
    command = CommandLong(command=400, params=(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    payload = pack_record(command)
    assert payload["_mavlink_message"] == "CommandLong"
    decoded = unpack_record(payload, CommandLong)
    assert decoded == command


def test_nested_record_round_trip() -> None:
    override = ControlOverride(
        roll=0.5,
        throttle=-0.5,
        aux=(ControlChannelOverride(channel_index=0, value=1.0),),
    )
    state = RemoteControlState(
        rc_telemetry=RcChannels(channel_count=4, channels=(1500.0, 1500.0, 1000.0, 1500.0)),
        control_output=ControlAxes(roll=0.1, pitch=0.2, yaw=0.3, throttle=0.4),
        control_override=override,
        receiver_config=ReceiverConfig(),
        channel_map=(),
        mode_ranges=(),
    )
    decoded = unpack_record(pack_record(state), RemoteControlState)
    assert decoded.control_override == override
    assert decoded.rc_telemetry.channels[0] == 1500.0


def test_lattice_entity_document_round_trip() -> None:
    entity = Entity(
        entity_id="asset-01",
        is_live=True,
        location=Location(
            position=Position(
                latitude_degrees=36.530440,
                longitude_degrees=-83.216383,
                altitude_hae_meters=300.0,
            )
        ),
        mil_view=MilView(disposition="DISPOSITION_FRIENDLY", environment="ENVIRONMENT_AIR"),
    )
    document = entity_document(entity)
    assert document["entityId"] == "asset-01"
    assert "entity_id" not in document
    decoded = unpack_lattice(document, Entity)
    assert decoded.entity_id == "asset-01"
