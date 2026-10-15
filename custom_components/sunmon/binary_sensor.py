"""Binary sensor platform: shows when values are held after a fault."""

from __future__ import annotations

from homeassistant.components.binary_sensor import (
    BinarySensorDeviceClass,
    BinarySensorEntity,
)
from homeassistant.const import EntityCategory
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .coordinator import SunMonConfigEntry, SunMonCoordinator
from .entity import SunMonEntity


async def async_setup_entry(
    hass: HomeAssistant,
    entry: SunMonConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up the indicator for a config entry."""
    async_add_entities([SunMonValuesHeld(entry.runtime_data)])


class SunMonValuesHeld(SunMonEntity, BinarySensorEntity):
    """On while at least one shown value is a held one, not a fresh reading."""

    _attr_translation_key = "values_held"
    _attr_device_class = BinarySensorDeviceClass.PROBLEM
    _attr_entity_category = EntityCategory.DIAGNOSTIC

    def __init__(self, coordinator: SunMonCoordinator) -> None:
        """Initialise the indicator."""
        super().__init__(coordinator, "values_held")

    @property
    def is_on(self) -> bool:
        """Return true if any channel shows a held value."""
        return bool(self.coordinator.held_channels)

    @property
    def extra_state_attributes(self) -> dict[str, list[int]]:
        """Return the affected channels, numbered 1 to 6 as in the names."""
        return {"channels": [channel + 1 for channel in self.coordinator.held_channels]}
