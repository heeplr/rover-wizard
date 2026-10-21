"""rover_wizard.steps.wheel_encoders - the "Wheel encoders (optional)" step."""

from __future__ import annotations

from ..util import timebase
from ..util.errors import VehicleError
from ..util.values import format_value
from .util.probing import LIVE_DISPLAY_SECONDS
from .util.rover_hardware import has
from ..model import Action, Step, param_field, value_field


def build_wheel_encoder_fields(prefix: str, second: bool) -> list:
    """Configuration fields of one wheel encoder (parameter prefix WENC or WENC2)."""
    # The second encoder is offered for skid steering (one per side) or when it is already configured.
    visible = (lambda ctx: ctx.is_skid() or ctx.number("WENC2_TYPE", 0) > 0) if second else None
    name = "2nd wheel encoder" if second else "Wheel encoder"
    return [
        param_field(
            f"{prefix}_TYPE",
            f"{name}: type",
            kind="choice",
            choices=[("None", 0), ("Quadrature", 1)],
            reboot=True,
            when=visible,
        ),
        param_field(f"{prefix}_CPR", f"{name}: counts per revolution", kind="int", when=visible),
        param_field(f"{prefix}_RADIUS", f"{name}: wheel radius", unit="m", when=visible),
        param_field(f"{prefix}_POS_X", f"{name}: position X (forward)", unit="m", when=visible),
        param_field(f"{prefix}_POS_Y", f"{name}: position Y (right)", unit="m", when=visible),
        param_field(f"{prefix}_PINA", f"{name}: pin A", kind="int", when=visible),
        param_field(f"{prefix}_PINB", f"{name}: pin B", kind="int", when=visible),
    ]


# [m] less than this means the encoder didn't count
ENCODER_MIN_MOVEMENT = 0.05

# wheel-encoder calibration: suggested distance to roll the vehicle
DEFAULT_ROLL_DISTANCE_M = 5.0

# let the last encoder counts arrive before reading
ENCODER_SETTLE_SECONDS = 0.4
# a WHEEL_DISTANCE sent after the settle time must arrive within this long (the link may deliver it late) ...
WHEEL_DISTANCE_WAIT_SECONDS = 5
# ... checked this often
WHEEL_DISTANCE_POLL_SECONDS = 0.05
# WHEEL_DISTANCE older than this means the encoders are not reporting
WHEEL_TELEMETRY_FRESH_SECONDS = 2


def _wheel_distances(ctx) -> list:
    """Distance [m] each wheel encoder has counted so far (from the latest WHEEL_DISTANCE)."""
    message = ctx.vehicle.latest("WHEEL_DISTANCE")
    return list(message.distance)[: int(message.count)]


def _settled_wheel_distances(ctx) -> list:
    """Distances [m] once the encoders' last counts have arrived: from a WHEEL_DISTANCE received at least
    ENCODER_SETTLE_SECONDS after now (an older one may still miss the end of the movement)."""
    settled = timebase.now() + ENCODER_SETTLE_SECONDS
    ctx.sleep(ENCODER_SETTLE_SECONDS)
    # age <= time since `settled`: the latest message arrived after it
    while ctx.vehicle.age("WHEEL_DISTANCE") > timebase.now() - settled:
        if timebase.now() - settled > WHEEL_DISTANCE_WAIT_SECONDS:
            raise VehicleError("WHEEL_DISTANCE telemetry stopped - check the connection, then repeat.")
        ctx.sleep(WHEEL_DISTANCE_POLL_SECONDS)
    return _wheel_distances(ctx)


def action_show_live_wheel_encoder(ctx):
    """Log the encoder distances once per second for 10 seconds."""
    for _ in range(LIVE_DISPLAY_SECONDS):
        if ctx.vehicle.age("WHEEL_DISTANCE") > WHEEL_TELEMETRY_FRESH_SECONDS:
            ctx.log("Distance [m]: no WHEEL_DISTANCE (set WENC_TYPE, reboot, check pins)")
        else:
            ctx.log("Distance [m]: " + ", ".join(f"{distance:.3f}" for distance in _wheel_distances(ctx)))
        ctx.sleep(1)


def action_calibrate_wheel_encoder(ctx):
    """Roll the vehicle a known distance by hand; correct each encoder's radius by measured / reported distance."""
    rolled = ctx.number("roll_m", DEFAULT_ROLL_DISTANCE_M)
    if ctx.vehicle.age("WHEEL_DISTANCE") > WHEEL_TELEMETRY_FRESH_SECONDS:
        raise VehicleError("No WHEEL_DISTANCE telemetry: set WENC_TYPE, reboot and check the wiring/pins first.")
    ctx.pause("Mark a start line and place the vehicle on it, then press Continue.")
    before = _settled_wheel_distances(ctx)
    ctx.pause(f"Roll the vehicle by hand exactly {rolled:g} m FORWARD in a straight line, then press Continue.")
    after = _settled_wheel_distances(ctx)
    for index, (start, end) in enumerate(zip(before, after)):
        name = "WENC_RADIUS" if index == 0 else f"WENC{index + 1}_RADIUS"
        reported = end - start
        radius = ctx.param_value(name)
        if radius is None:
            ctx.log(f"Encoder {index + 1}: {name} does not exist on this firmware - skipped.")
        elif abs(reported) < ENCODER_MIN_MOVEMENT:
            ctx.log(f"Encoder {index + 1}: no movement detected.")
        elif reported < 0:
            ctx.log(f"Encoder {index + 1}: distance DECREASED - swap its PINA/PINB (or the A/B wires), then repeat.")
        else:
            # The encoder reports distance = revolutions x 2*pi*radius, so the true radius scales by
            # (true distance / reported distance).
            corrected = radius * rolled / reported
            ctx.log(
                f"Encoder {index + 1}: measured {reported:.3f} m for {rolled:g} m -> {name} "
                f"{format_value(radius)} -> {corrected:.5f}"
            )
            # radius [m] to 0.01 mm
            ctx.set_values(**{name: round(corrected, 5)})
    ctx.log("Press 'Apply' to write the corrected radius values.")


STEP = Step(
    "wheel_encoders",
    "Wheel encoders (optional)",
    build_wheel_encoder_fields("WENC", False)
    + build_wheel_encoder_fields("WENC2", True)
    + [
        value_field(
            "roll_m",
            "Distance you will roll the vehicle",
            unit="m",
            default=DEFAULT_ROLL_DISTANCE_M,
            min_value=0.5,
            max_value=100,
        )
    ],
    [
        Action("Live encoder distance (10 s)", action_show_live_wheel_encoder),
        Action("Calibrate wheel radius", action_calibrate_wheel_encoder, armed=False),
    ],
    applies=has("has_wenc"),
)
