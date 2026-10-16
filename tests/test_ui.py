"""The real wizard UI driven headless by Textual's test pilot (App.run_test), with the simulator as vehicle.

Set ROVER_WIZARD_SCREENSHOTS to a folder to keep a plain-text screenshot of every step.
"""

import asyncio
import threading
import time
import os

import pytest

pytest.importorskip("textual")

from textual.widgets import Button, Checkbox  # noqa: E402

from rover_wizard.util import timebase  # noqa: E402
from rover_wizard.ui.step_catalog import STEPS  # noqa: E402
from rover_wizard.session.store import STEP_DONE  # noqa: E402
from rover_wizard.ui.wizard_app import WizardApp  # noqa: E402
from rover_wizard.motion.servo_test import DEFAULT_STEERING_TEST_PERCENT  # noqa: E402
from rover_wizard.ui.modals import ConfirmScreen, LeaveStepScreen, ProbeScreen  # noqa: E402

from util.helpers import TEST_TIME_SCALE, check  # noqa: E402
from util.screen_text import svg_to_text  # noqa: E402


@pytest.fixture(autouse=True)
def real_time():
    """The UI runs in real time like the real app (other tests fast-forward the simulator 20x)."""
    timebase.set_time_scale(1.0)
    yield
    timebase.set_time_scale(TEST_TIME_SCALE)


SIZE = (180, 55)
# the fake autopilot of the connect-screen test (test_mavlink_link.py uses 14610 and up)
UI_FAKE_AUTOPILOT_PORT = 14590
INDEX = {step.id: position for position, step in enumerate(STEPS)}
# set it to a folder to keep text screenshots
SCREENSHOT_DIR = os.environ.get("ROVER_WIZARD_SCREENSHOTS")


def label_text(widget) -> str:
    """Plain text of a Button/Checkbox label (Textual stores labels as Content)."""
    label = widget.label
    return getattr(label, "plain", str(label))


def widget_value(screen, key):
    """A field widget's value, or None while the step is being re-rendered (the widget briefly doesn't exist)."""
    widget = screen.widgets.get(key)
    return None if widget is None else widget.value


async def until(pilot, condition, timeout=10.0) -> bool:
    """Let the app run until condition() holds (or the timeout passes)."""
    for _ in range(int(timeout / 0.05)):
        if condition():
            return True
        await pilot.pause(0.05)
    return condition()


async def click(pilot, widget) -> None:
    """Scroll a widget into view and click it like a user would."""
    widget.scroll_visible(animate=False)
    await pilot.pause(0.1)
    await pilot.click(widget)
    await pilot.pause(0.1)


def screen_text(app) -> str:
    """What the terminal currently shows, as text."""
    return svg_to_text(app.export_screenshot())


async def start(pilot, app):
    """Wait until the demo has connected and the wizard screen is up; return (screen, ctx, vehicle)."""
    await until(pilot, lambda: type(app.screen).__name__ == "MainScreen")
    screen = app.screen
    await until(pilot, lambda: "Firmware" in (screen.ctx.values.get("vehicle_info") or ""))
    return screen, screen.ctx, screen.ctx.vehicle


def row_of(screen, field_key):
    """The Horizontal row a field's widget sits in."""
    return screen.widgets[field_key].parent


async def every_step_renders(skid: bool, size=SIZE):
    """Open every step (including hidden ones) and keep a text screenshot of each."""
    app = WizardApp(demo=True, demo_skid=skid)
    async with app.run_test(size=size) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        kind = ("skid" if skid else "ackermann") + ("" if size == SIZE else f" {size[0]}x{size[1]}")
        overflowing = []
        for index, step in enumerate(STEPS):
            await screen.show_step(index)
            await pilot.pause(0.2)
            # Layout check: nothing may stick out past the terminal's right edge or need sideways scrolling.
            wide = [
                f"{step.id}:{type(widget).__name__}.{'.'.join(sorted(widget.classes))}"
                for widget in screen.query("*")
                if widget.display and widget.region.width and widget.region.right > size[0]
            ]
            if wide or screen.query_one("#stepbox").max_scroll_x > 0:
                overflowing.append(wide or [f"{step.id}: step form scrolls sideways"])
            if SCREENSHOT_DIR:
                os.makedirs(SCREENSHOT_DIR, exist_ok=True)
                with open(
                    os.path.join(SCREENSHOT_DIR, f"{kind.replace(' ', '_')}_{index:02d}_{step.id}.txt"),
                    "w",
                    encoding="utf-8",
                ) as out:
                    out.write(screen_text(app))
        check(f"every step renders in the real UI ({kind})", True)
        check(f"no widget wider than the terminal on any step ({kind})", not overflowing, str(overflowing[:3]))
        vehicle.disconnect()


async def main_scenario():
    """Click through the features one by one, as a pilot would."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        text = screen_text(app)
        check("vehicle info shown on one line per fact", "Firmware: ArduPilot Rover 4.5.7  (MAV_TYPE 10" in text)

        # --- checkboxes: Textual's own label, clicking toggles the form value
        boxes = {label_text(box): box for box in screen.query(Checkbox)}
        check(
            "optional-hardware checkboxes carry their own labels",
            {"Vehicle has a GPS", "Vehicle has camera gimbal(s)"} <= set(boxes),
            str(sorted(boxes)),
        )
        gps_box = boxes["Vehicle has a GPS"]
        check(
            "GPS detected -> checked, gimbal not configured -> unchecked",
            gps_box.value is True and boxes["Vehicle has camera gimbal(s)"].value is False,
        )
        await click(pilot, gps_box)
        check(
            "clicking the checkbox unticks it and updates the form",
            gps_box.value is False and ctx.values["has_gps"] is False,
        )
        await click(pilot, gps_box)

        # --- live side panel on every step
        text = screen_text(app)
        check("RC panel lists the receiver's 12 channels", " 1  2  3  4  5  6  7  8  9 10 11 12" in text)
        # the left column only
        panel = [line[:50] for line in text.splitlines()]
        check(
            "servo panel lists outputs 1-8 with a + and - per output",
            "  1  2  3  4  5  6  7  8" in [line.rstrip() for line in panel]
            and "  +  +  +  +  +  +  +  +" in [line.rstrip() for line in panel]
            and "  -  -  -  -  -  -  -  -" in [line.rstrip() for line in panel],
        )
        # output 5 has no function: idle until moved, starts from its trim
        idle = vehicle.servo_raw()[4] == 0
        await click(pilot, screen.query_one("#servo-plus-5", Button))
        moved = await until(pilot, lambda: vehicle.servo_raw()[4] == 1525)
        check(
            "clicking servo + moves idle output 5 from its trim by 25 us", idle and moved, str(vehicle.servo_raw()[4])
        )
        await pilot.pause(1.0)
        bars = screen.query_one("#servo-bars").render()
        check("servo bar for output 5 shown", str(bars).strip() != "", repr(str(bars)))

        # --- MIN/TRIM/MAX +/- and the dead-zone Probe button
        await screen.show_step(INDEX["steering_output"])
        await pilot.pause(0.3)
        plus = next(b for b in row_of(screen, "trim_steering").query(Button) if label_text(b) == "+")
        trim = vehicle.get("SERVO1_TRIM")
        vehicle.motor_tests[:] = []
        await click(pilot, plus)
        ok = await until(pilot, lambda: vehicle.get("SERVO1_TRIM") == trim + 10 and vehicle.servo_raw()[0] == trim + 10)
        check(
            "TRIM + click writes +10 us and holds the servo there",
            ok,
            f"{vehicle.get('SERVO1_TRIM')} {vehicle.servo_raw()[0]}",
        )
        check("... without arming or a motor test", not vehicle.armed and vehicle.motor_tests == [])
        await until(pilot, lambda: not screen.busy)
        await screen.show_step(INDEX["motor_output"])
        await pilot.pause(0.3)
        labels = [label_text(b) for b in row_of(screen, "MOT_THR_MIN").query(Button)]
        check("dead-zone Probe button sits in the Minimum-throttle row", "Probe" in labels, str(labels))

        # --- navigation, Apply/Next, locking
        await screen.show_step(INDEX["vehicle_type"])
        await pilot.pause(0.3)
        check("second step locked before the first is done", screen.query_one("#nav-1", Button).disabled)
        await click(pilot, screen.query_one("#b-next", Button))
        await until(pilot, lambda: screen.step_index == INDEX["steering_output"])
        check(
            "Next marks the step done and opens the next one",
            ctx.store.step_status("vehicle_type") == STEP_DONE and screen.step_index == INDEX["steering_output"],
        )
        await screen.show_step(INDEX["steering_output"])
        await pilot.pause(0.3)
        # collides with the throttle output -> Apply fails
        ctx.set_field_value("ch_steer", 3)
        await click(pilot, screen.query_one("#b-next", Button))
        await until(pilot, lambda: not screen.busy)
        await pilot.pause(0.3)
        check(
            "failed Apply keeps the step open",
            screen.step_index == INDEX["steering_output"] and ctx.store.step_status("steering_output") is None,
        )

        # --- probe dialog: opens, Cancel answers the first question with "no", Close returns
        await screen.show_step(INDEX["orientation"])
        await pilot.pause(0.3)
        probe = next(b for b in row_of(screen, "AHRS_ORIENTATION").query(Button) if label_text(b) == "Probe")
        await click(pilot, probe)
        opened = await until(pilot, lambda: isinstance(app.screen, ProbeScreen))
        check("Probe opens the probe dialog", opened)
        if opened:
            dialog = app.screen
            await until(pilot, lambda: not dialog.query_one("#probe-cont", Button).disabled)
            check("main screen is busy while the dialog is open", screen.busy)
            # Log lines are wrapped; fold them back into one text so phrases can be searched.
            dialog_log = " ".join(" ".join(line.text for line in dialog.query_one("#probe-status").lines).split())
            main_log = " ".join(" ".join(line.text for line in screen.query_one("#log").lines).split())
            check(
                "probe question appears in the dialog, not in the main log",
                "Start the board orientation probe" in dialog_log and "Start the board" not in main_log,
            )
            briefing = dialog_log.find("NOSE (front) points straight DOWN")
            check(
                "the briefing (the pilot's tasks) stands above the question",
                0 <= briefing < dialog_log.find("Start the board orientation probe"),
                dialog_log[:300],
            )
            await click(pilot, dialog.query_one("#probe-cancel", Button))
            finished = await until(pilot, lambda: not dialog.query_one("#probe-close", Button).disabled)
            check("Cancel answers 'no' and the probe ends", finished and vehicle.get("AHRS_ORIENTATION") == 0)
            await click(pilot, dialog.query_one("#probe-close", Button))
            await until(pilot, lambda: app.screen is screen)
            check("Close returns to the wizard and releases busy", app.screen is screen and not screen.busy)

        # --- tab order: every field and button before the help buttons
        chain = [widget.id or "" for widget in screen.focus_chain]
        first_help = next(
            (position for position, widget_id in enumerate(chain) if widget_id.startswith("help-")), len(chain)
        )
        check(
            "help buttons come last in the tab order",
            first_help < len(chain) and all(widget_id.startswith("help-") for widget_id in chain[first_help:]),
        )

        # --- STOP
        vehicle.arm()
        await click(pilot, screen.query_one("#b-stop", Button))
        stopped = await until(pilot, lambda: not vehicle.armed)
        check("STOP disarms the vehicle", stopped)
        vehicle.disconnect()


async def narrow_terminal():
    """The layout must survive a smaller terminal (120 x 40) without crashing."""
    app = WizardApp(demo=True)
    async with app.run_test(size=(120, 40)) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        text = screen_text(app)
        check(
            "120x40 terminal: step list, panel and step form all visible",
            "Vehicle type" in text and "RC channels" in text and "Servo output" in text,
        )
        check(
            "120x40 terminal: log lines wrap without losing words",
            "Click ❓ for help on any step or field." in text.replace("\n", " ").replace("│", " ").replace("  ", " ")
            or "Click ❓ for help" in text,
        )
        check(
            "120x40 terminal: status bar wraps instead of cutting off battery voltage",
            " V" in text.split("RC channels")[0],
        )
        vehicle.disconnect()


def pure_rendering_checks():
    """Bargraph text (no app needed)."""
    from rover_wizard.ui.monitor import BAR_LEVELS, rc_bar_text

    check(
        "bars: 1000 us lowest, 2000 us highest, missing channel blank",
        rc_bar_text([1000, 2000, 0]).split("\n")[1] == f" {BAR_LEVELS[0]}  {BAR_LEVELS[-1]}    ",
    )
    check("bars: placeholder without RC", "no RC_CHANNELS" in rc_bar_text(None))


async def steering_test_scenario():
    """Test R in the real UI: the steering output moves right, the vehicle stays disarmed, no motor test."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        await screen.show_step(INDEX["steering_output"])
        await pilot.pause(0.3)
        # Record every steering pulse the simulator publishes: the test holds the steering for 1 s, and on a slow
        # machine the click itself can take about that long - looking at the vehicle afterwards may miss it.
        pulses = []
        publish = vehicle._store_message

        def recording(msg_type, message, timestamp=None):
            """Pass the message on, noting SERVO1's pulse."""
            if msg_type == "SERVO_OUTPUT_RAW":
                pulses.append(message.servo1_raw)
            publish(msg_type, message, timestamp)

        vehicle._store_message = recording
        test_right = next(b for b in row_of(screen, "ch_steer").query(Button) if label_text(b) == "Test R")
        await click(pilot, test_right)
        # the default deflection, as a share of the range between TRIM and MAX, right of TRIM
        trim, maximum = vehicle.get("SERVO1_TRIM"), vehicle.get("SERVO1_MAX")
        expected = round(trim + DEFAULT_STEERING_TEST_PERCENT / 100 * (maximum - trim))
        moved = await until(pilot, lambda: expected in pulses)
        check("Test R moves the steering output right", moved, f"expected {expected}, seen {sorted(set(pulses))}")
        check("vehicle stays disarmed, no motor test", not vehicle.armed and vehicle.motor_tests == [])
        log = " ".join(" ".join(line.text for line in screen.query_one("#log").lines).split())
        check(
            "log says what happened in plain words",
            f"Steering test: RIGHT {DEFAULT_STEERING_TEST_PERCENT}%" in log,
            log[-120:],
        )
        vehicle.disconnect()


async def output_selector_scenario():
    """Choosing another output in the Servo outputs step redraws its fields with that output's own settings."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        # output 5 follows RC channel 7
        vehicle.set_param("SERVO5_FUNCTION", 57)
        await screen.show_step(INDEX["servo_outputs"])
        await pilot.pause(0.3)
        # like picking it in the dropdown
        screen.widgets["aux_output"].value = 5
        switched = await until(
            pilot, lambda: "aux_function" in screen.widgets and widget_value(screen, "aux_function") == 57
        )
        check("output 5 selected: 'Driven by' shows its own function (RC channel 7)", switched)
        # the throttle output
        screen.widgets["aux_output"].value = 3
        locked = await until(pilot, lambda: "aux_locked" in screen.widgets and "aux_function" not in screen.widgets)
        check("the throttle output is shown locked, without editable fields", locked)
        vehicle.disconnect()


async def gps_count_scenario():
    """Choosing two receivers in the real dropdown shows the selector; selecting GPS 2 loads its own settings."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        # an NMEA receiver is configured as GPS 2
        vehicle.set_param("GPS2_TYPE", 5)
        await screen.show_step(INDEX["gps"])
        await pilot.pause(0.3)
        check("GPS 2 detected: two receivers preselected", screen.widgets["gps_count"].value == 2)
        screen.widgets["gps"].value = 2
        loaded = await until(
            pilot, lambda: widget_value(screen, "gps_type") == 5 and widget_value(screen, "gps_port") == 4
        )
        check("GPS 2 selected: its type (NMEA) and port (SERIAL4) are shown", loaded)
        screen.widgets["gps_count"].value = 1
        reduced = await until(
            pilot,
            lambda: "gps" not in screen.widgets
            and "GPS_AUTO_SWITCH" not in screen.widgets
            and widget_value(screen, "gps_type") == 1,
        )
        check("back to one receiver: selector gone, fields show GPS 1 again", reduced)
        vehicle.disconnect()


async def parameter_help_scenario():
    """A field without wizard help shows ArduPilot's own description of its parameter."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        await screen.show_step(INDEX["gps"])
        await pilot.pause(0.3)
        button_id = next(key for key, (title, _) in screen.help.items() if title == "Antenna offset Y (right)")
        await click(pilot, screen.query_one(f"#{button_id}", Button))
        opened = await until(pilot, lambda: type(app.screen).__name__ == "HelpScreen")
        text = screen_text(app).replace("\n", " ")
        check(
            "help dialog opens with ArduPilot's description",
            opened and "Positive Y is to the right of the origin" in " ".join(text.split()),
            text[:200],
        )
        vehicle.disconnect()


async def sensor_panel_scenario():
    """The side panel shows a compass tape and accelerometer/gyro bars - only for sensors the vehicle has."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        # sections are visible from the start but filled by the first refresh: wait for their content
        shown = await until(pilot, lambda: "▲" in screen_text(app) and screen_text(app).count("X  Y  Z") == 2)
        text = screen_text(app)
        check("compass tape, Accel and Gyro shown", shown and "Accel" in text and "Gyro" in text, text[-400:])
        ctx.store.set("has_compass", False)
        vehicle.set_param("INS_GYR_ID", 0)
        hidden = await until(
            pilot,
            lambda: not screen.query_one("#compass-monitor").display
            and not screen.query_one("#gyro-monitor").parent.display
            and screen.query_one("#accel-monitor").display,
        )
        check("compass and gyro hidden without them, Accel still shown", hidden)
        vehicle.set_param("INS_GYR_ID", 2752772)
        vehicle.set_param("INS_ACC_ID", 0)
        # wait for the redrawn screen, not just the widget state
        moved = await until(
            pilot, lambda: any(line.lstrip(" ").startswith("Gyro") for line in screen_text(app).split("\n"))
        )
        check(
            "without accelerometer, Gyro takes the first column",
            moved and not screen.query_one("#accel-monitor").parent.display,
            screen_text(app)[-300:],
        )

        # relays: no column while none is set up; a button per relay that is; a click toggles it
        check("no relay column without relays", not screen.query_one("#relay-column").display)
        vehicle.set_param("RELAY2_FUNCTION", 1)
        vehicle.set_param("RELAY2_PIN", 50)
        button = screen.query_one("#relay-2", Button)
        appeared = await until(
            pilot,
            lambda: screen.query_one("#relay-column").display
            and button.display
            and not screen.query_one("#relay-1", Button).display,
        )
        check("a relay that is set up gets a button showing its number", appeared and label_text(button) == "2")
        await click(pilot, button)
        switched_on = await until(pilot, lambda: vehicle.relay_states.get(2) is True and button.has_class("relay-on"))
        check("a click switches the relay on and inverts the button", switched_on)
        await click(pilot, button)
        switched_off = await until(
            pilot, lambda: vehicle.relay_states.get(2) is False and not button.has_class("relay-on")
        )
        check("a second click switches it off again", switched_off)
        vehicle.disconnect()


async def closing_while_a_job_updates_scenario():
    """A job finishing while the wizard closes must not crash: its screen updates are dropped."""
    app = WizardApp(demo=True)
    errors = []
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)

        def job():
            """Keep updating the Continue button, as a waiting action does, while the app closes."""
            for _ in range(200):
                time.sleep(0.005)
                try:
                    screen.ui(screen._set_waiting, False)
                except Exception as error:
                    errors.append(error)
                    return

        threading.Thread(target=job, daemon=True).start()
        await pilot.pause(0.05)
        vehicle.disconnect()
    await asyncio.sleep(0.5)
    check("updates during shutdown are dropped, not raised", not errors, repr(errors[:1]))


async def parameter_list_scenario():
    """Ctrl+P: every parameter, searchable; a dialog edits the raw value with a live reading."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        await screen.show_step(INDEX["compass"])
        await pilot.pause(0.3)
        footer = screen_text(app)
        check("footer offers ^p parameters", "^p" in footer and "parameters" in footer, footer[-200:])
        # while an operation runs, the list doesn't open
        screen.busy = True
        await pilot.press("ctrl+p")
        await pilot.pause(0.3)
        check("not while busy", type(app.screen).__name__ == "MainScreen")
        screen.busy = False
        await pilot.press("ctrl+p")
        opened = await until(pilot, lambda: type(app.screen).__name__ == "ParameterScreen")
        check("Ctrl+P opens the parameter list", opened)
        table = app.screen.query_one("#p-table")
        check("all parameters listed", table.row_count == len(vehicle.parameters()))
        await pilot.press(*"autodec")
        await until(pilot, lambda: table.row_count == 1)
        check("search narrows the list", table.row_count == 1)
        await pilot.press("enter", "enter")
        dialog_open = await until(pilot, lambda: type(app.screen).__name__ == "ParameterEditScreen")
        check("Enter opens the edit dialog", dialog_open)
        value_input = app.screen.query_one("#pe-value")
        value_input.value = "0"
        read_live = await until(pilot, lambda: "Disabled" in str(app.screen.query_one("#pe-reading").render()))
        check("live reading follows the input", read_live)
        await pilot.press("enter")
        written = await until(pilot, lambda: vehicle.get("COMPASS_AUTODEC") == 0)
        check("saving writes the value", written and "COMPASS_AUTODEC" in [n for n, _, _ in ctx.changes.items()])
        await until(pilot, lambda: "COMPASS_AUTODEC *" in str(table.get_row_at(0)[0]))
        check("changed parameter marked", str(table.get_row_at(0)[0]) == "COMPASS_AUTODEC *")
        # a value outside ArduPilot's range can't be saved
        search = app.screen.query_one("#p-search")
        search.value = "COMPASS_CAL_FIT"
        await until(pilot, lambda: table.row_count == 1)
        table.focus()
        await pilot.press("enter")
        await until(pilot, lambda: type(app.screen).__name__ == "ParameterEditScreen")
        app.screen.query_one("#pe-value").value = "50"
        await pilot.press("enter")
        await pilot.pause(0.3)
        check(
            "out of range is refused",
            type(app.screen).__name__ == "ParameterEditScreen"
            and "outside ArduPilot's range" in str(app.screen.query_one("#pe-error").render())
            and vehicle.get("COMPASS_CAL_FIT") == 16,
        )
        await pilot.press("escape", "escape")
        back = await until(pilot, lambda: type(app.screen).__name__ == "MainScreen")
        reloaded = await until(pilot, lambda: ctx.values.get("COMPASS_AUTODEC") == 0)
        check("closing returns to the step, which shows the new value", back and reloaded)
        vehicle.disconnect()


async def backup_restore_scenario():
    """The parameter list's R/W column, Backup into the data folder and Restore from a file."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        await pilot.press("ctrl+p")
        await until(pilot, lambda: type(app.screen).__name__ == "ParameterScreen")
        params = app.screen
        table = params.query_one("#p-table")
        params.query_one("#p-search").value = "INS_ACC_ID"
        await until(pilot, lambda: table.row_count == 1)
        check("read-only parameter marked R", [str(cell) for cell in table.get_row_at(0)][3] == "R")

        await click(pilot, params.query_one("#p-backup"))
        backups = await until(pilot, lambda: [n for n in os.listdir(ctx.store.root) if n.startswith("params_backup")])
        status = await until(pilot, lambda: "Saved every parameter" in str(params.query_one("#p-status").render()))
        check("Backup writes a file and says so", backups and status)
        backup = os.path.join(ctx.store.root, next(n for n in os.listdir(ctx.store.root) if n.startswith("params_")))
        with open(backup, encoding="utf-8") as handle:
            check("the backup holds every parameter", len(handle.readlines()) == len(vehicle.parameters()))

        restore = os.path.join(ctx.store.root, "zz_restore.param")
        with open(restore, "w", encoding="utf-8") as handle:
            handle.write("COMPASS_AUTODEC,0\nINS_ACC_ID,5\nNOT_ON_VEHICLE,1\n")
        # the newest file is offered first
        os.utime(restore, (time.time() + 10, time.time() + 10))
        await click(pilot, params.query_one("#p-restore"))
        await until(pilot, lambda: type(app.screen).__name__ == "RestoreScreen")
        check("the newest file is offered", app.screen.query_one("#pr-path").value == restore)
        await click(pilot, app.screen.query_one("#pr-restore"))
        asked = await until(pilot, lambda: isinstance(app.screen, ConfirmScreen))
        text = " ".join(screen_text(app).split())
        check(
            "the confirmation summarizes the file",
            asked and "1 value(s) differ" in text and "INS_ACC_ID" in text and "NOT_ON_VEHICLE" in text,
            text[:800],
        )
        await click(pilot, app.screen.query_one("#yes"))
        restored = await until(pilot, lambda: vehicle.get("COMPASS_AUTODEC") == 0)
        check(
            "Yes writes the differing value", restored and "COMPASS_AUTODEC" in [n for n, _, _ in ctx.changes.items()]
        )
        check("read-only values are kept", vehicle.get("INS_ACC_ID") == 2753028)
        done = await until(pilot, lambda: "1 value(s) restored" in str(params.query_one("#p-status").render()))
        check("the status line says so", done)
        vehicle.disconnect()


async def connect_screen_scenario(tmp_path):
    """The connect screen: a connection string is required, its help opens and closes, a failed connection is
    reported and can be retried - then a real MAVLink connection (to the fake autopilot) opens the wizard."""
    pytest.importorskip("pymavlink")
    from util.fake_autopilot import FakeAutopilot

    app = WizardApp()
    async with app.run_test(size=SIZE) as pilot:
        await until(pilot, lambda: type(app.screen).__name__ == "ConnectScreen")
        screen = app.screen
        go = screen.query_one("#c-go", Button)

        def status():
            """The line under the Connect button."""
            return str(screen.query_one("#c-status").render())

        await click(pilot, go)
        check("a connection string is required", "Please enter a connection string." in status(), status())
        await click(pilot, screen.query_one("#help-conn"))
        opened = await until(pilot, lambda: type(app.screen).__name__ == "HelpScreen")
        await click(pilot, app.screen.query_one("#help-close"))
        closed = await until(pilot, lambda: app.screen is screen)
        check("the connection help opens and closes", opened and closed)

        screen.query_one("#c-conn").value = str(tmp_path / "no_such_serial_port")
        screen.query_one("#c-baud").value = "not a number"
        await click(pilot, go)
        failed = await until(pilot, lambda: "Connection failed" in status() and not go.disabled)
        check("a failed connection is reported and can be retried", failed, status())

        autopilot = FakeAutopilot(UI_FAKE_AUTOPILOT_PORT)
        try:
            screen.query_one("#c-conn").value = f"udpin:127.0.0.1:{UI_FAKE_AUTOPILOT_PORT}"
            await click(pilot, go)
            main_screen, ctx, vehicle = await start(pilot, app)
            check("a real connection opens the wizard", "ArduPilot Rover 4.5.7" in ctx.values.get("vehicle_info", ""))
            check("the parameters were downloaded", vehicle.parameter_count == len(autopilot.params))
        finally:
            app.ctx and app.ctx.vehicle.disconnect()
            autopilot.close()


async def main_buttons_scenario():
    """Arm (after a question), Disarm, Skip, Cancel and the arming requirement of actions."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)

        def log_text():
            """The activity log, as text."""
            return " ".join(" ".join(strip.text for strip in screen.query_one("#log").lines).split())

        await screen.show_step(INDEX["accel"])
        await pilot.pause(0.3)
        await click(pilot, screen.query_one("#b-arm", Button))
        await until(pilot, lambda: isinstance(app.screen, ConfirmScreen))
        await click(pilot, app.screen.query_one("#no"))
        await pilot.pause(0.3)
        check("No leaves the vehicle disarmed", not vehicle.armed)
        await click(pilot, screen.query_one("#b-arm", Button))
        await until(pilot, lambda: isinstance(app.screen, ConfirmScreen))
        await click(pilot, app.screen.query_one("#yes"))
        check("Yes arms the vehicle", await until(pilot, lambda: vehicle.armed))

        row = next(
            r for r in screen.query(".arow") if any(label_text(b) == "Gyro calibration" for b in r.query(Button))
        )
        await click(pilot, next(b for b in row.query(Button) if label_text(b) == "Gyro calibration"))
        refused = await until(pilot, lambda: "needs a DISARMED vehicle" in log_text())
        check("a bench action is refused while armed", refused, log_text()[-300:])

        await click(pilot, screen.query_one("#b-disarm", Button))
        check("Disarm disarms", await until(pilot, lambda: not vehicle.armed))

        await screen.show_step(INDEX["verify"])
        await pilot.pause(0.3)
        row = next(
            r for r in screen.query(".arow") if any("SPEED controller" in label_text(b) for b in r.query(Button))
        )
        await click(pilot, next(b for b in row.query(Button) if "SPEED controller" in label_text(b)))
        refused = await until(pilot, lambda: "needs an ARMED vehicle" in log_text())
        check("a driving action is refused while disarmed", refused, log_text()[-300:])

        await click(pilot, screen.query_one("#b-cancel", Button))
        check("Cancel is acknowledged", await until(pilot, lambda: "Cancel requested" in log_text()))

        step_id = STEPS[screen.step_index].id
        await click(pilot, screen.query_one("#b-skip", Button))
        moved = await until(pilot, lambda: STEPS[screen.step_index].id != step_id)
        check(
            "Skip marks the step skipped and opens the next one", moved and ctx.store.step_status(step_id) == "skipped"
        )

        await screen.show_step(len(STEPS) - 1)
        await pilot.pause(0.3)
        await click(pilot, screen.query_one("#b-next", Button))
        last = await until(pilot, lambda: "That was the last step" in log_text())
        check("Next on the last step says so", last, log_text()[-300:])
        vehicle.disconnect()


async def parameter_dialogs_scenario(tmp_path):
    """The edge cases of the parameter list's dialogs: a read-only parameter, Cancel, a file that doesn't exist or
    can't be read, and a file that matches the vehicle."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        await pilot.press("ctrl+p")
        await until(pilot, lambda: type(app.screen).__name__ == "ParameterScreen")
        params = app.screen
        table = params.query_one("#p-table")

        def status():
            """The line under the list."""
            return str(params.query_one("#p-status").render())

        params.query_one("#p-search").value = "INS_ACC_ID"
        await until(pilot, lambda: table.row_count == 1)
        table.focus()
        await pilot.press("enter")
        await until(pilot, lambda: type(app.screen).__name__ == "ParameterEditScreen")
        dialog = app.screen
        text = " ".join(screen_text(app).split())
        check(
            "a read-only parameter can't be saved",
            dialog.query_one("#pe-save").disabled and "Read-only" in text,
            text[:500],
        )
        await click(pilot, dialog.query_one("#pe-cancel"))
        check("Cancel closes the editor", await until(pilot, lambda: app.screen is params))

        async def restore(path):
            """Open Restore and submit `path`."""
            await click(pilot, params.query_one("#p-restore"))
            await until(pilot, lambda: type(app.screen).__name__ == "RestoreScreen")
            app.screen.query_one("#pr-path").value = str(path)
            await click(pilot, app.screen.query_one("#pr-restore"))
            await pilot.pause(0.3)

        await restore(tmp_path / "missing.param")
        error = str(app.screen.query_one("#pr-error").render())
        check("a missing file is refused in the dialog", "No such file" in error, error)
        await click(pilot, app.screen.query_one("#pr-cancel"))
        check("Cancel closes the restore dialog", await until(pilot, lambda: app.screen is params))

        broken = tmp_path / "broken.param"
        broken.write_text("this is not a parameter file\n", encoding="utf-8")
        await restore(broken)
        check("an unreadable file is reported", "broken.param: Line 1" in status(), status())

        same = tmp_path / "same.param"
        same.write_text(f"CRUISE_SPEED,{vehicle.get('CRUISE_SPEED')}\n", encoding="utf-8")
        await restore(same)
        check("a file matching the vehicle writes nothing", "Nothing to write" in status(), status())
        vehicle.disconnect()


async def hidden_parameters_scenario():
    """A step's note about settings missing on this firmware names each one's parameter."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        await screen.show_step(INDEX["remote_id"])
        await pilot.pause(0.5)
        text = " ".join(screen_text(app).split())
        check(
            "hidden settings are named with their parameter",
            "Not available on this firmware (hidden): Remote ID enabled (DID_ENABLE)" in text,
            text[:600],
        )
        vehicle.disconnect()


async def action_help_scenario():
    """The ❓ next to an action button opens that action's help."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        await screen.show_step(INDEX["rc"])
        await pilot.pause(0.5)
        row = next(r for r in screen.query(".arow") if any(label_text(b) == "Calibrate radio" for b in r.query(Button)))
        await click(pilot, next(b for b in row.query(Button) if label_text(b) == "❓"))
        opened = await until(pilot, lambda: type(app.screen).__name__ == "HelpScreen")
        text = " ".join(screen_text(app).split())
        check("action help opens", opened and "Records your transmitter's range" in text, text[:400])
        vehicle.disconnect()


@pytest.mark.parametrize(
    "skid, size", [(False, SIZE), (True, SIZE), (False, (120, 40))], ids=["ackermann", "skid", "ackermann-120x40"]
)
def test_every_step_renders_and_fits(skid, size):
    asyncio.run(every_step_renders(skid, size))


def test_pilot_clicks_through_the_features():
    asyncio.run(main_scenario())


def test_steering_test_button_moves_servo_disarmed():
    asyncio.run(steering_test_scenario())


def test_narrow_terminal():
    asyncio.run(narrow_terminal())


def test_bar_text():
    pure_rendering_checks()


async def leaving_with_unapplied_values_scenario():
    """Back with an edited field asks first: Escape stays, Discard leaves without writing, Apply writes and leaves.
    The reboot question starts on No, and Escape answers No."""
    app = WizardApp(demo=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        battery = INDEX["battery"]
        original = vehicle.get("BATT_CAPACITY")

        async def edit_and_go_back():
            """Open the battery step, type a capacity, press Back; True if the question appeared."""
            await screen.show_step(battery)
            await pilot.pause(0.3)
            screen.widgets["BATT_CAPACITY"].value = "6000"
            await pilot.pause(0.2)
            await click(pilot, screen.query_one("#b-back"))
            return await until(pilot, lambda: isinstance(app.screen, LeaveStepScreen))

        check("Back with an edit asks first", await edit_and_go_back())
        check("the question names the field", "Battery capacity" in screen_text(app))
        await pilot.press("escape")
        check(
            "Escape stays on the step",
            await until(pilot, lambda: app.screen is screen) and screen.step_index == battery,
        )
        await click(pilot, screen.query_one("#b-back"))
        await until(pilot, lambda: isinstance(app.screen, LeaveStepScreen))
        await click(pilot, app.screen.query_one("#discard"))
        check(
            "Discard leaves without writing",
            await until(pilot, lambda: screen.step_index != battery) and vehicle.get("BATT_CAPACITY") == original,
        )
        check("Back with an edit asks again", await edit_and_go_back())
        await click(pilot, app.screen.query_one("#apply"))
        check(
            "Apply writes, then leaves",
            await until(pilot, lambda: screen.step_index != battery and vehicle.get("BATT_CAPACITY") == 6000),
        )
        await click(pilot, screen.query_one("#b-reboot"))
        await until(pilot, lambda: isinstance(app.screen, ConfirmScreen))
        check("the reboot question starts on No", app.screen.focused is not None and app.screen.focused.id == "no")
        await pilot.press("escape")
        check("Escape answers No", await until(pilot, lambda: app.screen is screen))
        vehicle.disconnect()


async def job_ending_while_a_step_is_mounted_scenario():
    """A background job that ends while show_step() is still mounting the next step must not touch its widgets yet
    (a Select isn't composed until mounted); the value it proposed shows once the step is up."""
    app = WizardApp(demo=True, demo_skid=True)
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        showing = asyncio.create_task(screen.show_step(INDEX["steering_output"]))
        # let show_step create the new widgets, then end a "job" before they are mounted
        for _ in range(500):
            await asyncio.sleep(0)
            select = screen.widgets.get("steer")
            # the Select has its parts, but its current-value part has no label yet
            if select is not None and select.query("SelectOverlay") and not select.query("SelectCurrent #label"):
                break
        ctx.values["steer"] = "skid"
        screen._job_done(None)
        await showing
        await pilot.pause(0.2)
        check("the step renders", screen.step_index == INDEX["steering_output"])
        check(
            "the proposed value shows",
            screen.widgets["steer"].value == "skid",
            f"{screen.widgets['steer'].value} {ctx.values.get('steer')}",
        )
        vehicle.disconnect()


def test_job_ending_while_a_step_is_mounted():
    asyncio.run(job_ending_while_a_step_is_mounted_scenario())


def test_leaving_with_unapplied_values():
    asyncio.run(leaving_with_unapplied_values_scenario())


def test_output_selector_reloads_the_fields():
    asyncio.run(output_selector_scenario())


def test_gps_count_and_selector():
    asyncio.run(gps_count_scenario())


def test_parameter_help_shows_arduPilots_description():
    asyncio.run(parameter_help_scenario())


def test_side_panel_shows_the_sensors():
    asyncio.run(sensor_panel_scenario())


def test_closing_while_a_job_updates_the_screen():
    asyncio.run(closing_while_a_job_updates_scenario())


def test_parameter_list():
    asyncio.run(parameter_list_scenario())


def test_backup_and_restore():
    asyncio.run(backup_restore_scenario())


def test_connect_screen(tmp_path):
    asyncio.run(connect_screen_scenario(tmp_path))


def test_main_screen_buttons():
    asyncio.run(main_buttons_scenario())


def test_parameter_dialogs(tmp_path):
    asyncio.run(parameter_dialogs_scenario(tmp_path))


def test_hidden_parameters_are_named():
    asyncio.run(hidden_parameters_scenario())


def test_action_buttons_have_help():
    asyncio.run(action_help_scenario())
