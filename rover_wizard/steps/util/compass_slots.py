"""rover_wizard.steps.util.compass_slots - which compass parameter belongs to which compass.

ArduPilot numbers its compass parameters in two ways (AP_Compass.cpp):

* by detection slot: COMPASS_DEV_ID / _DEV_ID2 / _DEV_ID3 with COMPASS_ORIENT(2/3), COMPASS_EXTERNAL / _EXTERN2 /
  _EXTERN3 and COMPASS_MOT(2/3)_X/Y/Z;
* by priority (firmware 4.1+): COMPASS_PRIO1_ID .. PRIO3_ID name the device IDs in the order they are used, and
  COMPASS_USE / _USE2 / _USE3 belong to those priorities - not to the slots.

So "compass in slot 2" is switched off through the COMPASS_USEk whose COMPASS_PRIOk_ID is COMPASS_DEV_ID2.
"""

from __future__ import annotations

from typing import Optional

# the three compasses that have a full parameter set
COMPASS_SLOTS = (1, 2, 3)


def slot_name(base: str, slot: int) -> str:
    """COMPASS_DEV_ID, COMPASS_DEV_ID2, ... / COMPASS_MOT, COMPASS_MOT2, ... / COMPASS_USE, COMPASS_USE2, ..."""
    return base if slot == 1 else f"{base}{slot}"


def external_name(slot: int) -> str:
    """COMPASS_EXTERNAL, COMPASS_EXTERN2, COMPASS_EXTERN3 (the first one is spelled out)."""
    return "COMPASS_EXTERNAL" if slot == 1 else f"COMPASS_EXTERN{slot}"


def detected_slots(ctx) -> list:
    """Slots that hold a compass (COMPASS_DEV_IDn set)."""
    return [slot for slot in COMPASS_SLOTS if ctx.param_int(slot_name("COMPASS_DEV_ID", slot))]


def priority_of_slot(ctx, slot: int) -> Optional[int]:
    """Priority (1..3) of the compass in `slot`: the k with COMPASS_PRIOk_ID = COMPASS_DEV_IDn. Firmware without
    priorities (before 4.1) numbers COMPASS_USEn by slot. None: the compass has no priority (not used)."""
    if ctx.vehicle.resolve("COMPASS_PRIO1_ID") is None:
        return slot

    device = ctx.param_int(slot_name("COMPASS_DEV_ID", slot))
    for priority in COMPASS_SLOTS:
        if device and ctx.param_int(f"COMPASS_PRIO{priority}_ID") == device:
            return priority
    return None


def primary_compass_slot(ctx) -> int:
    """Parameter slot (1..3) of the compass ArduPilot uses first, which RAW_IMU reports: the slot with priority 1
    (firmware 4.1+), else the first."""
    for slot in COMPASS_SLOTS:
        if ctx.vehicle.resolve("COMPASS_PRIO1_ID") is not None and priority_of_slot(ctx, slot) == 1:
            return slot
    return 1


def is_external(ctx, slot: int) -> bool:
    """Whether the compass in `slot` is external (its orientation is set by COMPASS_ORIENTn)."""
    return ctx.param_int(external_name(slot)) != 0


def use_name(ctx, slot: int) -> Optional[str]:
    """COMPASS_USEk that switches the compass in `slot` on/off (None if it has no priority)."""
    priority = priority_of_slot(ctx, slot)
    return None if priority is None else slot_name("COMPASS_USE", priority)
