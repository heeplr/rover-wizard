"""Position sources (EKF): always shown; the heading source is proposed when the default (compass) doesn't fit."""

import pytest

from rover_wizard.steps.ekf_sources import EKF_SOURCE_COMPASS, EKF_YAW_GPS_COMPASS_FALLBACK, EKF_YAW_GSF

from util.helpers import BY_ID, apply, enter, make

# GPS1_TYPE: u-blox moving baseline base
GPS_TYPE_MOVING_BASELINE_BASE = 17


@pytest.fixture
def wizard():
    """A simulated vehicle with GPS and compass; the log goes into a list."""
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, log
    vehicle.disconnect()


def open_step(ctx):
    """Open the step as the UI does (fields loaded, then its on_enter)."""
    enter(ctx, "ekf_sources")
    BY_ID["ekf_sources"].on_enter(ctx)


def test_compass_heading_stays_with_gps_and_compass(wizard):
    vehicle, ctx, log = wizard
    open_step(ctx)
    assert ctx.value("src_yaw") == EKF_SOURCE_COMPASS and not log


def test_without_compass_the_gsf_estimator_is_proposed_and_applied(wizard):
    vehicle, ctx, log = wizard
    ctx.store.set("has_compass", False)
    open_step(ctx)
    assert ctx.value("src_yaw") == EKF_YAW_GSF
    assert "no compass" in log[-1]
    assert vehicle.get("EK3_SRC1_YAW") == EKF_SOURCE_COMPASS, "only a proposal until Apply"
    apply(ctx, "ekf_sources")
    assert vehicle.get("EK3_SRC1_YAW") == EKF_YAW_GSF


def test_heading_gps_is_proposed(wizard):
    vehicle, ctx, log = wizard
    vehicle.set_param("GPS1_TYPE", GPS_TYPE_MOVING_BASELINE_BASE)
    open_step(ctx)
    assert ctx.value("src_yaw") == EKF_YAW_GPS_COMPASS_FALLBACK


def test_gps_can_give_the_vertical_position(wizard):
    vehicle, ctx, log = wizard
    open_step(ctx)
    ctx.set_field_value("src_posz", 3)
    apply(ctx, "ekf_sources")
    assert vehicle.get("EK3_SRC1_POSZ") == 3
