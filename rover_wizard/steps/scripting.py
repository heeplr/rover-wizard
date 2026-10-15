"""rover_wizard.steps.scripting - the "Scripting" step."""

from __future__ import annotations

from ..model import Step, bit_field, param_field

# Values from AP_Scripting.
SCRIPTING_ENABLE_CHOICES = [("None", 0), ("Lua scripts", 1)]

SCRIPT_DIR_BITS = [(1, "Run built-in scripts (ROMFS)"), (2, "Run scripts from the SD card (APM/scripts)")]

SCRIPT_DEBUG_BITS = [
    (1, "Message if no scripts are running"),
    (2, "Runtime messages (memory, execution time)"),
    (4, "Don't log scripts to the log"),
    (8, "Log runtime memory and execution time"),
    (16, "Disable scripting pre-arm check"),
]

# SCR_USER1..SCR_USER6
SCRIPT_USER_PARAMETERS = range(1, 7)


def _is_scripting_enabled(ctx) -> bool:
    """Scripting chosen in the form (or enabled on the vehicle)."""
    return bool(ctx.form_int("SCR_ENABLE"))


def scripting_fields() -> list:
    """Scripting on/off, resources, script folders, debug options and the six user parameters."""
    fields = [
        param_field(
            "SCR_ENABLE", "Scripting", kind="choice", choices=SCRIPTING_ENABLE_CHOICES, reboot=True, refresh=True
        ),
        param_field(
            "SCR_HEAP_SIZE", "Memory for scripts", kind="int", unit="bytes", reboot=True, when=_is_scripting_enabled
        ),
        param_field("SCR_VM_I_COUNT", "Instructions per script run", kind="int", when=_is_scripting_enabled),
    ]
    fields += [
        bit_field("SCR_DIR_DISABLE", bit, label, key=f"scr_dir_{bit}", bit_skips=True, when=_is_scripting_enabled)
        for bit, label in SCRIPT_DIR_BITS
    ]
    fields += [
        bit_field("SCR_DEBUG_OPTS", bit, f"Debug: {label}", key=f"scr_debug_{bit}", when=_is_scripting_enabled)
        for bit, label in SCRIPT_DEBUG_BITS
    ]
    fields += [
        param_field(f"SCR_USER{number}", f"Script user parameter {number}", when=_is_scripting_enabled)
        for number in SCRIPT_USER_PARAMETERS
    ]
    return fields


def apply_scripting(ctx):
    """Scripting's other settings appear only after a reboot that follows enabling it."""
    if _is_scripting_enabled(ctx) and not ctx.vehicle.has_parameter("SCR_HEAP_SIZE"):
        ctx.log(
            "Scripting enabled: reboot the flight controller, then the other scripting settings appear here. "
            "Put .lua files into APM/scripts on the SD card."
        )


STEP = Step("scripting", "Scripting", scripting_fields(), [], on_apply=apply_scripting)
