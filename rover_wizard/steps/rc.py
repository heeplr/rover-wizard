"""rover_wizard.steps.rc - the "Radio control & flight modes" step."""

from __future__ import annotations

from typing import Optional

from ..util.errors import VehicleError
from .util.rover_hardware import has
from .util.outputs import mode_channel
from ..model import Action, Step, param_field
from ..rover_defs import RC_CHANNELS

# MODE1..MODE6 (six positions of the mode switch)
FLIGHT_MODE_SLOTS = range(1, 7)

PILOT_STEER_TYPE_CHOICES = [
    ("Default", 0),
    ("Two paddles (skid-style RC input)", 1),
    ("Direction reversed when backing up", 2),
    ("Direction unchanged when backing up", 3),
]

FLIGHT_MODE_CHOICES = [
    ("Manual", 0),
    ("Acro", 1),
    ("Steering", 3),
    ("Hold", 4),
    ("Loiter", 5),
    ("Follow", 6),
    ("Simple", 7),
    ("Circle", 9),
    ("Auto", 10),
    ("RTL", 11),
    ("SmartRTL", 12),
    ("Guided", 15),
]

# RCx_OPTION values from Rover's RC_Channel parameter definitions.
# Arm/disarm is 153 since firmware 4.2; ArduPilot rejects the old 41 (ARMDISARM_UNUSED: "Invalid channel option").
RC_OPTION_CHOICES = [
    ("Do nothing", 0),
    ("RTL", 4),
    ("Save waypoint", 7),
    ("Camera trigger", 9),
    ("Auto", 16),
    ("Gripper", 19),
    ("Relay 1 on/off", 28),
    ("Lost rover sound", 30),
    ("Motor emergency stop", 31),
    ("Relay 2 on/off", 34),
    ("Relay 3 on/off", 35),
    ("Relay 4 on/off", 36),
    ("SmartRTL", 42),
    ("RC override enable", 46),
    ("Learn cruise speed", 50),
    ("Manual", 51),
    ("Acro", 52),
    ("Steering", 53),
    ("Hold", 54),
    ("Guided", 55),
    ("Loiter", 56),
    ("Follow", 57),
    ("Clear waypoints", 58),
    ("Relay 5 on/off", 66),
    ("Relay 6 on/off", 67),
    ("Visual odometry yaw align", 80),
    ("Disarm", 81),
    ("EKF source set", 90),
    ("Arm/disarm", 153),
    ("Scripting 1", 300),
]

# [us] pulse widths outside this are glitches, not stick positions
RC_PLAUSIBLE_PWM = (800, 2300)

# [us] a channel must move at least this much to be calibrated
RC_MIN_CALIBRATION_RANGE = 200

# [us] a stick held at full deflection must move its channel at least this far from the centre (otherwise the
# channel mapping is wrong and "reversed" would be decided by noise)
RC_MIN_DEFLECTION = RC_MIN_CALIBRATION_RANGE // 2

# stick centres: average of 10 readings ...
RC_CENTRE_SAMPLES = 10

# ... 0.1 s apart
RC_SAMPLE_SPACING = 0.1


def _rc_reading(ctx, channel_count: int) -> list:
    """Current raw pulse widths padded/truncated to `channel_count` channels (0 where nothing was received)."""
    raw = ctx.vehicle.rc_raw() or []
    # Pad with zeros (channels not received), then cut to exactly channel_count entries.
    return (list(raw) + [0] * channel_count)[:channel_count]


def _average_raw_rc_channels(ctx, channel_count: int, sample_count: int = RC_CENTRE_SAMPLES) -> list:
    """Average raw pulse width of every RC channel over `sample_count` readings 0.1 s apart (the stick centres)."""
    sums = [0.0] * channel_count
    for _ in range(sample_count):
        # Accumulate value / count directly, so the result is the mean without a second pass.
        sums = [total + value / sample_count for total, value in zip(sums, _rc_reading(ctx, channel_count))]
        ctx.sleep(RC_SAMPLE_SPACING)
    return sums


def action_calibrate_radio(ctx):
    """Record each channel's min/max/centre and detect reversed steering/throttle from the pilot's stick moves."""
    vehicle = ctx.vehicle
    raw = vehicle.rc_raw()
    if not raw:
        raise VehicleError("No RC_CHANNELS telemetry - is the transmitter ON and the receiver bound/connected?")
    # as many channels as the receiver actually delivers
    channel_count = len(raw)
    # Use the mapping from the form if the pilot just edited it, otherwise the vehicle's.
    mapped_steering, mapped_throttle = vehicle.rc_mapping()
    steering_channel = int(ctx.number("RCMAP_ROLL", mapped_steering))
    throttle_channel = int(ctx.number("RCMAP_THROTTLE", mapped_throttle))
    ctx.log(
        f"Using steering = RC{steering_channel}, throttle = RC{throttle_channel} "
        "(apply the channel mapping first if you changed it)."
    )
    ctx.pause("Transmitter ON. Centre ALL sticks (throttle centre = stopped for a rover), then press Continue.")
    centres = _average_raw_rc_channels(ctx, channel_count)
    lowest = [float("inf")] * channel_count
    highest = [float("-inf")] * channel_count

    def track_extremes():
        """Widen each channel's recorded min/max while the pilot moves the sticks (ignoring glitches)."""
        for index, pwm in enumerate(_rc_reading(ctx, channel_count)):
            if RC_PLAUSIBLE_PWM[0] < pwm < RC_PLAUSIBLE_PWM[1]:
                lowest[index] = min(lowest[index], pwm)
                highest[index] = max(highest[index], pwm)

    ctx.pause(
        "Move EVERY stick and switch to both extremes a few times, release the sticks to centre, "
        "then press Continue.",
        during=track_extremes,
    )
    ctx.pause("Push the THROTTLE stick fully FORWARD, hold it, then press Continue.")
    throttle_reading = _rc_reading(ctx, max(channel_count, throttle_channel))
    ctx.pause("Push the STEERING stick fully to the RIGHT, hold it, then press Continue.")
    steering_reading = _rc_reading(ctx, max(channel_count, steering_channel))
    for index in range(channel_count):
        if highest[index] - lowest[index] < RC_MIN_CALIBRATION_RANGE:
            # channel didn't move (unused, or a switch that wasn't touched)
            continue
        channel = index + 1
        ctx.set_param(f"RC{channel}_MIN", lowest[index], required=False)
        ctx.set_param(f"RC{channel}_MAX", highest[index], required=False)
        ctx.set_param(f"RC{channel}_TRIM", round(centres[index]), required=False)

    for channel, reading, what, key in (
        (throttle_channel, throttle_reading, "throttle", "RCMAP_THROTTLE"),
        (steering_channel, steering_reading, "steering", "RCMAP_ROLL"),
    ):
        deflected = reading[channel - 1]
        if deflected <= 0:
            ctx.log(f"WARNING: could not read the {what} direction.")
            continue
        # "Forward"/"right" should give a pulse above the centre; below it means the channel is reversed.
        if channel > channel_count:
            ctx.log(f"WARNING: RC{channel} is not delivered by the receiver ({channel_count} channels).")
            continue
        if abs(deflected - centres[channel - 1]) < RC_MIN_DEFLECTION:
            _warn_wrong_channel(ctx, channel, reading, centres, what, key)
            continue
        reversed_channel = 1 if deflected < centres[channel - 1] else 0
        ctx.set_param(f"RC{channel}_REVERSED", reversed_channel, required=False)
        ctx.log(f"{what} channel RC{channel}: {'REVERSED' if reversed_channel else 'normal'}")
    ctx.log("RC calibration written.")


def _warn_wrong_channel(ctx, channel: int, reading: list, centres: list, what: str, key: str) -> None:
    """The mapped channel didn't move with the stick: name the channel that did (its direction stays unchanged)."""
    moves = [abs(pwm - centre) if pwm > 0 else 0 for pwm, centre in zip(reading, centres)]
    moved = max(range(len(moves)), key=moves.__getitem__) + 1
    hint = (
        f" RC{moved} moved instead: set the {what} channel ({key}) to {moved}, Apply, and calibrate again."
        if moves[moved - 1] >= RC_MIN_DEFLECTION
        else " No channel moved - check the transmitter."
    )
    ctx.log(f"WARNING: RC{channel} didn't move with the {what.upper()} stick, so its direction wasn't set.{hint}")


def check_rc_channels(ctx) -> Optional[str]:
    """Steering, throttle and the mode switch must be three different channels."""
    steering_channel, throttle_channel = ctx.vehicle.rc_mapping()
    roles = {
        "steering (RCMAP_ROLL)": int(ctx.number("RCMAP_ROLL", steering_channel)),
        "throttle (RCMAP_THROTTLE)": int(ctx.number("RCMAP_THROTTLE", throttle_channel)),
        "flight mode switch (MODE_CH)": int(ctx.number("MODE_CH", mode_channel(ctx))),
    }
    seen = {}
    for role, channel in roles.items():
        if channel in seen:
            return f"The {seen[channel]} and the {role} can't share channel RC{channel}."
        seen[channel] = role
    return None


def clear_options_on_control_channels(ctx) -> None:
    """A switch function on the steering, throttle or mode channel would fire with the stick / mode switch (for
    the mode channel ArduPilot refuses to arm). The wizard hides those RCx_OPTION fields, so it clears them here."""
    steering_channel, throttle_channel = ctx.vehicle.rc_mapping()
    names = {value: name for name, value in RC_OPTION_CHOICES}
    for channel, role in (
        (steering_channel, "steering"),
        (throttle_channel, "throttle"),
        (mode_channel(ctx), "flight mode switch"),
    ):
        option = ctx.param_int(f"RC{channel}_OPTION")
        if option:
            ctx.log(
                f"RC{channel} is the {role} channel: its switch function "
                f"'{names.get(option, option)}' (RC{channel}_OPTION) is removed."
            )
            ctx.set_param(f"RC{channel}_OPTION", 0, required=False)


def _is_free_rc_channel(channel: int):
    """Condition: the channel doesn't steer, throttle or switch flight modes (those can't take a switch function)."""

    def is_free(ctx):
        """Whether this channel may get an RCx_OPTION."""
        steering_channel, throttle_channel = ctx.vehicle.rc_mapping()
        return channel not in (steering_channel, throttle_channel, mode_channel(ctx))

    return is_free


def rc_option_fields() -> list:
    """A switch-function selector for every RC channel that isn't steering, throttle or the mode switch."""
    return [
        param_field(
            f"RC{channel}_OPTION",
            f"RC{channel} switch function",
            kind="choice",
            choices=RC_OPTION_CHOICES,
            when=_is_free_rc_channel(channel),
        )
        for channel in RC_CHANNELS
    ]


STEP = Step(
    "rc",
    "Radio control & flight modes",
    [
        param_field("RCMAP_ROLL", "Steering channel", kind="int", min_value=1, max_value=16),
        param_field("RCMAP_THROTTLE", "Throttle channel", kind="int", min_value=1, max_value=16),
        param_field("PILOT_STEER_TYPE", "Steering input type", kind="choice", choices=PILOT_STEER_TYPE_CHOICES),
        param_field("MODE_CH", "Flight mode switch channel", kind="int", min_value=1, max_value=16),
    ]
    + [
        param_field(f"MODE{i}", f"Flight mode {i}", kind="choice", choices=FLIGHT_MODE_CHOICES)
        for i in FLIGHT_MODE_SLOTS
    ]
    + rc_option_fields(),
    [Action("Calibrate radio", action_calibrate_radio, armed=False)],
    applies=has("has_rc"),
    on_apply=clear_options_on_control_channels,
    check=check_rc_channels,
)
