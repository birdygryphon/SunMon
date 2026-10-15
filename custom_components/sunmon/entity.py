"""Base entity and device info shared by all sensors."""

from __future__ import annotations

from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .const import DOMAIN, MODEL
from .coordinator import SunMonConfigEntry, SunMonCoordinator


def device_info(entry: SunMonConfigEntry) -> DeviceInfo:
    """Return the device all entities of a config entry belong to."""
    return DeviceInfo(
        identifiers={(DOMAIN, entry.entry_id)},
        name=entry.title,
        model=MODEL,
    )


class SunMonEntity(CoordinatorEntity[SunMonCoordinator]):
    """Common device info and naming for the measured values."""

    _attr_has_entity_name = True

    def __init__(self, coordinator: SunMonCoordinator, key: str) -> None:
        """Initialise the entity."""
        super().__init__(coordinator)
        entry = coordinator.config_entry
        self._attr_unique_id = f"{entry.entry_id}_{key}"
        self._attr_device_info = device_info(entry)
