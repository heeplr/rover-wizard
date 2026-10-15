"""rover_wizard.ui.step_catalog - the wizard's steps in order, with the texts shown for them.

The steps themselves (fields, actions, what Apply writes) live in rover_wizard.steps, one module per step. This
catalog puts them in the wizard's order and attaches the text the pilot will read - each step's introduction and help -
from wizard_text.json.
"""

from __future__ import annotations

import json
import os

from ..steps import (
    vehicle_type,
    steering_output,
    motor_output,
    orientation,
    gps,
    accel,
    compass,
    rc,
    wheel_encoders,
    distance_sensors,
    gimbal,
    servo_outputs,
    relays,
    gripper,
    temperature,
    sprayer,
    optical_flow,
    visual_odometry,
    beacon,
    ekf_sources,
    osd,
    remote_id,
    rpm,
    generator,
    serial_ports,
    battery,
    failsafe,
    scripting,
    save,
    predrive,
    direction_check,
    top_speed,
    turning,
    gains,
    verify,
    navigation,
    log_settings,
    final,
)
from ..model import Phase

TEXT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "wizard_text.json")

# The wizard's phases and their steps, in order. Each step module defines one step as STEP.
# The battery monitor comes before the compass: compass-motor compensation needs its current measurement.
PHASES = (
    Phase(
        1,
        "Bench setup (disarmed, indoors OK)",
        drives=False,
        steps=(
            vehicle_type,
            steering_output,
            motor_output,
            battery,
            orientation,
            gps,
            accel,
            compass,
            rc,
            wheel_encoders,
            distance_sensors,
            gimbal,
            servo_outputs,
            relays,
            gripper,
            temperature,
            sprayer,
            optical_flow,
            visual_odometry,
            beacon,
            ekf_sources,
            osd,
            remote_id,
            rpm,
            generator,
            serial_ports,
            failsafe,
            log_settings,
            scripting,
            save,
        ),
    ),
    Phase(
        2,
        "Drive & tune (ARMED, open space)",
        drives=True,
        steps=(predrive, direction_check, top_speed, turning, gains, verify, navigation),
    ),
    Phase(3, "Wrap-up", drives=False, steps=(final,)),
)


def _load_texts() -> dict:
    """Step introductions/help, field help ("step id.field key"), action-button help ("step id.button label") and
    briefings of probes ("step id.field key" of the field the Probe button sits in) and of other runs that move the
    vehicle ("step id.button label")."""
    with open(TEXT_FILE, encoding="utf-8") as text_file:
        return json.load(text_file)


def _in_phase(step, phase: Phase, texts: dict):
    """The step with its phase, its introduction, its help, its fields' help and its action buttons' help attached."""
    step.phase = phase
    entry = texts["steps"][step.id]
    step.intro = entry["intro"]
    step.help = entry["help"]
    for field in step.fields:
        field.help = texts["fields"].get(f"{step.id}.{field.key}", field.help)
        for action in field.inline_actions or []:
            action.briefing = texts["probes"].get(f"{step.id}.{field.key}", action.briefing)
    for action in step.actions:
        action.help = texts["actions"].get(f"{step.id}.{action.label}", action.help)
        action.briefing = texts["probes"].get(f"{step.id}.{action.label}", action.briefing)
    return step


_TEXTS = _load_texts()
STEPS = [_in_phase(module.STEP, phase, _TEXTS) for phase in PHASES for module in phase.steps]
