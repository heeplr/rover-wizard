"""rover_wizard.steps.navigation - the "Navigation parameters" step."""

from __future__ import annotations

from .util.hardware import has
from ..model import Action, Step, param_field

# no cruise speed measured: waypoint speed = 60 % of the top speed
WP_SPEED_FALLBACK_FRACTION = 0.6

# waypoint speed at most 70 % of the top speed
WP_SPEED_MAX_FRACTION = 0.7


def action_suggest_navigation_params(ctx):
    """Propose WP_SPEED (cruise speed, at most 70 % of top speed) and WP_RADIUS (turn radius, >= 1 m, 0.5 m steps)."""
    top_speed = ctx.param_value("SPEED_MAX", 0) or 0
    cruise_speed = ctx.param_value("CRUISE_SPEED", 0) or 0
    turn_radius = ctx.param_value("TURN_RADIUS", 0) or 0
    waypoint_speed = cruise_speed if cruise_speed > 0 else WP_SPEED_FALLBACK_FRACTION * top_speed
    proposal = {}
    if waypoint_speed > 0:
        proposal["WP_SPEED"] = round(
            (
                min(waypoint_speed, WP_SPEED_MAX_FRACTION * top_speed)
                if top_speed
                # to 0.1 m/s
                else waypoint_speed
            ),
            1,
        )
    # Round to the nearest 0.5 m: x2, round, /2.
    proposal["WP_RADIUS"] = round(max(1.0, turn_radius) * 2) / 2
    ctx.set_values(**proposal)
    ctx.log(
        "Suggestions: WP_SPEED ~ cruise speed (<= 70% of top speed), WP_RADIUS ~ turn radius. Review and press Apply."
    )


STEP = Step(
    "navigation",
    "Navigation parameters",
    [
        param_field("WP_SPEED", "Mission speed", unit="m/s"),
        param_field("RTL_SPEED", "RTL speed (0 = WP_SPEED)", unit="m/s"),
        param_field("WP_RADIUS", "Waypoint radius", unit="m"),
        param_field("WP_ACCEL", "Waypoint acceleration", unit="m/s/s"),
        param_field("WP_JERK", "Waypoint jerk", unit="m/s/s/s"),
        # Older firmware (L1 navigation, before Rover 4.2); current firmware doesn't have these, so they stay hidden.
        param_field("WP_OVERSHOOT", "Max overshoot", unit="m"),
        param_field("NAVL1_PERIOD", "L1 period", unit="s"),
        param_field("NAVL1_DAMPING", "L1 damping"),
        param_field(
            ("WP_PIVOT_ANGLE", "PIVOT_TURN_ANGLE"), "Pivot turn angle", unit="deg", when=lambda ctx: ctx.is_skid()
        ),
        param_field(
            ("WP_PIVOT_RATE", "PIVOT_TURN_RATE"), "Pivot turn rate", unit="deg/s", when=lambda ctx: ctx.is_skid()
        ),
        param_field("WP_PIVOT_DELAY", "Wait after a pivot turn", unit="s", when=lambda ctx: ctx.is_skid()),
    ],
    [Action("Fill suggestions from measurements", action_suggest_navigation_params)],
    applies=has("has_gps"),
)
