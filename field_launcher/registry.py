"""Native Lattice identities, groups and media associations; no tasking or custom records."""
from datetime import datetime, timedelta, timezone
from urllib.parse import urlsplit

import httpx
from anduril import (Aliases, Entity, GroupChild, GroupParent, Lattice, Media, MediaItem, Ontology,
                     Provenance, Relationship, Relationships, RelationshipType, SrtSettings)
from anduril.errors import NotFoundError
from anduril.core.api_error import ApiError
from anduril.video.errors import NotFoundError as VideoNotFoundError

from field_launcher.config import FieldSettings
from field_launcher.phone import ICUSettings

INTEGRATION = "mpfc-field"
MEDIA_FIELD = "media.media"


class FieldRegistry:
    def __init__(self, settings: FieldSettings):
        self.settings = settings
        connection = settings.connection
        self.http = httpx.Client(verify=str(connection.ca), trust_env=False, timeout=5)
        self.sdk = self.client()

    def client(self) -> Lattice:
        connection = self.settings.connection
        return Lattice(base_url=connection.url, httpx_client=self.http, max_retries=0,
            token=connection.token.get_secret_value() if connection.token else None,
            client_id=connection.client_id,
            client_secret=connection.client_secret.get_secret_value() if connection.client_secret else None)

    def reauthenticate(self, error: ApiError):
        if error.status_code == 401 and self.settings.connection.client_secret:
            # The relay may restart and revoke process-local tokens. A fresh SDK performs native OAuth again.
            self.sdk = self.client()

    def close(self):
        self.http.close()

    @staticmethod
    def provenance(entity_id: str) -> Provenance:
        return Provenance(integration_name=INTEGRATION, source_id=entity_id,
                          source_update_time=datetime.now(timezone.utc))

    def membership(self) -> Relationships:
        return Relationships(relationships=[Relationship(related_entity_id=self.settings.group_id,
            relationship_id=self.settings.group_id,
            relationship_type=RelationshipType(group_parent=GroupParent()))])

    def prepare_group(self):
        members = [asset.entity_id for asset in self.settings.assets]
        if self.settings.phone:
            members.append(self.settings.phone.entity_id)
        group = Entity(entity_id=self.settings.group_id, aliases=Aliases(name=self.settings.group_name),
            no_expiry=True, relationships=Relationships(relationships=[Relationship(
                related_entity_id=member, relationship_id=member,
                relationship_type=RelationshipType(group_child=GroupChild())) for member in members]),
            provenance=self.provenance(self.settings.group_id))
        self.publish(group)

    def publish(self, entity: Entity):
        # Native SDK serialization occurs only at this network boundary.
        return self.sdk.entities.publish_entity(**entity.model_dump(exclude_none=True))

    def archive_stream(self, ingress_id: str):
        self.sdk.video.delete_ingress_stream(ingress_id=ingress_id)

    def ensure_identity(self, entity_id: str, name: str):
        try:
            return self.sdk.entities.get_entity(entity_id)
        except NotFoundError:
            # A provisioned identity has no claimed position, connectivity or task authority.
            return self.publish(Entity(entity_id=entity_id, aliases=Aliases(name=name), is_live=False,
                no_expiry=True, relationships=self.membership(), provenance=self.provenance(entity_id)))

    def attach(self, entity_id: str, ingress_id: str, *, remove=False):
        entity = self.sdk.entities.get_entity(entity_id)
        existing = entity.media or Media()
        items = [item for item in existing.media or [] if item.item_identifier != ingress_id]
        if not remove:
            items.append(MediaItem(type="MEDIA_TYPE_VIDEO", item_identifier=ingress_id))
        self.sdk.entities.override_entity(entity_id=entity_id, field_path=MEDIA_FIELD,
            entity=Entity(media=existing.model_copy(update={"media": items})),
            provenance=self.provenance(entity_id))

    def publish_video_source(self, entity_id: str, name: str, received: bool):
        self.publish(Entity(entity_id=entity_id, aliases=Aliases(name=name), is_live=received,
            expiry_time=datetime.now(timezone.utc) + timedelta(seconds=max(5, self.settings.poll_seconds * 3)),
            ontology=Ontology(template="TEMPLATE_ASSET"), relationships=self.membership(),
            provenance=self.provenance(entity_id)))

    def prepare_phone(self) -> str:
        phone = self.settings.phone
        if phone is None:
            raise ValueError("Field setup has no phone identity")
        self.prepare_group()
        self.ensure_identity(phone.entity_id, phone.callsign)
        try:
            stream = self.sdk.video.get_ingress_stream(ingress_id=phone.ingress_id).ingress_stream
        except VideoNotFoundError:
            stream = None
        if stream is None or stream.status == "STREAM_STATUS_ARCHIVED":
            transport = self.sdk.video.create_ingress_stream(ingress_id=phone.ingress_id,
                title=phone.callsign, srt=SrtSettings()).srt
        else:
            transport = stream.srt
        if transport is None or not transport.url:
            raise ValueError(f"Phone ingress {phone.ingress_id} is not an SRT publisher")
        self.attach(phone.entity_id, phone.ingress_id)
        address = urlsplit(transport.url)
        # OpenTAK ICU Preferences and OnBoardingActivity define these import parameters.
        # Camera2Service prefixes the path with 'publish:' when constructing the SRT stream ID.
        return ICUSettings(address=address.hostname, port=address.port, path=phone.ingress_id).import_uri()
