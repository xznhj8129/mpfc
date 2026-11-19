"""Session-owned media bindings restore the native component override they displaced."""
from anduril import Entity, MediaItem, Override

from field_launcher.registry import FieldRegistry, MEDIA_FIELD


class MediaBindings:
    def __init__(self, registry: FieldRegistry, entity_id: str):
        self.registry, self.entity_id = registry, entity_id
        entity = registry.sdk.entities.get_entity(entity_id)
        self.original_items: list[MediaItem] = entity.media.media or [] if entity.media else []
        overrides = entity.overrides.override or [] if entity.overrides else []
        self.original_override: Override | None = next((item for item in overrides if item.field_path == MEDIA_FIELD), None)
        self.owned: set[str] = set()

    def add(self, ingress_id: str):
        self.registry.attach(self.entity_id, ingress_id)
        self.owned.add(ingress_id)

    def remove(self, ingress_id: str):
        self.registry.attach(self.entity_id, ingress_id, remove=True)
        self.owned.remove(ingress_id)
        if self.owned:
            return
        entity: Entity = self.registry.sdk.entities.get_entity(self.entity_id)
        items = entity.media.media or [] if entity.media else []
        if items != self.original_items:
            return  # Preserve another publisher's independently added media references.
        if self.original_override is None:
            self.registry.sdk.entities.remove_entity_override(self.entity_id, MEDIA_FIELD)
        else:
            self.registry.sdk.entities.override_entity(self.entity_id, MEDIA_FIELD,
                entity=self.original_override.masked_field_value, provenance=self.original_override.provenance)
