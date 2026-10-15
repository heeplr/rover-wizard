# Vibecode initial human test checklist

---

## 1. Installation and start

- [X] `pip install .` installs; the `rover-wizard` command starts
- [X] `python -m rover_wizard` starts without installing
- [X] `rover-wizard --help` lists `--conn`, `--baud`, `--demo`, `--demo-skid`
- [ ] Without Textual/pymavlink installed: a clear "Missing dependency" message instead of a crash
- [X] `--demo` starts a simulated front-wheel rover; `--demo-skid` a skid-steering one
  - [X] Demo progress is saved under `~/.rover_wizard/demo/`, not mixed with a real vehicle's progress
- [X] `--conn` (and `--baud`) connects immediately, skipping the connect screen

## 2. Connect screen and link

- [X] Connect screen: connection string and baud fields, ❓ help, **Connect**
  - [ ] Serial (e.g. `/dev/ttyACM0`, `COM5`)
  - [ ] UDP (e.g. `udpin:0.0.0.0:14550`)
  - [X] TCP (e.g. `tcp:127.0.0.1:5760`)
  - [X] Empty baud field uses 115200
  - [X] Wrong port / no vehicle: an error message, the screen stays usable, a retry works
  - [ ] A non-Rover vehicle (e.g. Copter) is refused with a message
- [X] Parameter download shows progress; the main screen opens afterwards
- [X] Status bar shows: link state, firmware version, ARMED/disarmed, mode, FRONT-WHEEL/SKID steering, GPS fix + satellites,
  battery voltage
  - [X] "⚠ reboot pending" appears after changing a reboot-requiring parameter and disappears after a reboot
- [X] Link loss (unplug / radio off): the status bar warns; the link reconnects by itself when the vehicle is back
  - [X] Cached parameters stay available during and after the reconnect
- [X] Reboot of the flight controller (any reboot button): the wizard waits and reconnects
- [ ] Non-default ground-station id (`MAV_GCS_SYSID` on current firmware / `SYSID_MYGCS` on older): RC overrides
  (steering tests, automatic drives) still move the vehicle

## 3. Main screen and navigation

- [X] Step list (left) grouped by phase; a step's status mark (done / skipped) is shown and updates
- [X] Locking: a step can only be opened once the steps before it are done or skipped
- [X] Steps that don't apply (hardware not fitted, steering type) are hidden and skipped by Back/Next
- [X] **◀ Back** / **Next ▶**: Next applies the step, marks it done and opens the next step
  - [ ] A failing Apply (e.g. an output collision) shows the error and keeps the step open
- [X] **💾 Apply** writes changed parameters only, logs each change, and redraws the step (new parameters can appear)
  - [X] Unchanged values are not written again
- [X] **Skip** marks the step skipped and moves on
- [X] **Continue ▶** is enabled only while an action waits for the pilot
- [X] **■ Cancel** stops the running action; parameters changed by probes are restored
- [X] **Arm** / **Disarm** work; actions needing an armed or a disarmed vehicle refuse otherwise with a message
- [X] **♻ Reboot FC** reboots and reconnects
- [X] **🛑 STOP** at any time: overrides released, Hold mode, vehicle disarmed, running action stopped
- [ ] While an action runs, other buttons (Apply, Next, Skip, Probe, side-panel buttons) are refused with a "busy" message
- [X] Log panel shows actions, parameter changes and flight-controller messages; long lines wrap without losing text
- [X] Field types: number inputs (with min/max validation), dropdowns, checkboxes (own label), read-only display boxes
  - [ ] An invalid number is rejected with a message
  - [X] A dropdown shows a vehicle value that isn't in its list as "(current)"
- [X] "ℹ Not available on this firmware (hidden): ..." appears for parameters the connected firmware lacks
- [ ] Help: every ❓ opens a dialog; **Close (Esc)** and the Esc key close it
  - [ ] Step help (intro and help text)
  - [ ] A field with wizard guidance shows the guidance followed by ArduPilot's description
  - [ ] A field without guidance shows ArduPilot's description (e.g. "Antenna offset Y")
  - [ ] A bitmask checkbox's help names its bit value
  - [ ] A parameter without ArduPilot description says so
- [ ] Tab order: fields and buttons first, ❓ buttons last
- [ ] Confirmation dialogs: **Yes** / **No**; STOP during a question aborts the action
- [ ] Probe dialog: separate log, **Continue ▶**, **■ Cancel** (answers "no" / stops), **Close** after the end
- [ ] Layout
  - [ ] Wide terminal (≥ 160 columns): normal layout
  - [ ] Narrow terminal (e.g. 120 x 40): compact layout - labels wrap, control buttons in a 3 x 2 grid, status bar wraps,
    nothing cut off
  - [ ] Resizing the terminal while running re-lays out correctly
- [ ] Progress
  - [ ] Quit and restart: done/skipped steps and answers are restored for the same vehicle
  - [ ] A corrupt progress file is moved aside (`*.corrupt-<date>`) and the wizard starts fresh
- [ ] Quitting during an action releases RC overrides and stops the action

## 4. Side panel (always visible)

- [X] **RC channels**: one bar per channel the receiver actually delivers (e.g. 8, 12, 16); bars follow the sticks
  - [X] Transmitter off / no receiver: a placeholder message
- [X] **Servo output**: number, **+**, **-** and a bar per output; outputs shown up to the highest one in use (at least 8)
  - [X] Free output (no function / RCIN / pass-through): + / - step the pulse by 25 us (DO_SET_SERVO); log
    "SERVOn: 1525 us"
  - [X] Steering output: + / - step the stick 10 %; the servo moves, the vehicle stays **disarmed**, no motor test;
    the log shows the reported pulse; the servo returns to centre 3 s after the last click
  - [X] Motor output (wheels lifted!): + / - run a 5 % motor test (arms for 2 s, disarms after); repeated clicks
    during the test work; log shows the pulse and a wheels-off warning
  - [X] Motor output while the pilot has armed the vehicle: refused
  - [ ] Output used by another function (e.g. a gimbal): refused with the function number
  - [ ] Bars update live, also when outputs are moved by the flight controller

## 5. Phase 1 - bench setup

### 5.1 Vehicle type
- [ ] Vehicle info shows firmware, vehicle type, system id and parameter count
- [ ] **Frame class** Rover / Boat (reboot pending)
- [ ] **Vehicle is too heavy to pick up and rotate** switches the compass step to Large Vehicle MagCal
- [ ] Each hardware checkbox starts from the vehicle's configuration; check each pair (configured / not):
  - [ ] GPS: unticked when no GPS type is set, or when the GPS reports "no GPS connected"; a 2nd receiver alone counts
  - [ ] Compass: ticked only if a compass was detected (`COMPASS_DEV_ID*`) and compasses are enabled
  - [ ] RC receiver: ticked with the transmitter on; unticked when the receiver delivers no channels
  - [ ] Battery monitor (any of 16 instances), wheel encoders (1 or 2), rangefinder (any of 10),
    proximity sensor (any of 5), gimbal (1 or 2), temperature (any of 9), RPM (any of 4)
  - [ ] Extra servos/PWM devices, relays, gripper, sprayer, optical flow, visual odometry, beacons, OSD, Remote ID,
    generator
- [ ] Unticking a box hides its step(s) and switches the hardware off (all instances); RC: advice logged instead
- [ ] A saved answer wins over the detection on the next visit
- [ ] **Backup all parameters to file** writes a parameter file and logs its path

### 5.2 Steering output
- [ ] **Steering type** front-wheel / skid: the servo fields appear / disappear immediately (before Apply)
- [ ] Front-wheel: steering servo output number; Apply assigns GroundSteering
  - [ ] Output used by a motor: refused (output collision)
- [ ] Skid: Apply removes any steering-servo assignment
- [ ] **Test L** / **Test R** move the servo by the test deflection, disarmed, no motor test; log in plain words
- [ ] **Reverse output**, **MIN / MAX / TRIM PWM** (+ / - buttons): each click writes ±10 us immediately and
  moves the servo there (MIN = full left, MAX = full right, TRIM = centre; reversed servo swaps the ends)
  - [ ] MIN can't pass TRIM, TRIM stays between MIN and MAX, values stay within 800-2200 us

### 5.3 Motor output
- [ ] **Motor driver / ESC signal**; **Brushed motor PWM frequency** only for brushed types
- [ ] Front-wheel: throttle output; skid: LEFT and RIGHT motor outputs; switching steering type replaces them cleanly
- [ ] **Test ↑** / **Test ↓** per motor (wheels lifted): motor test at the test power, arms for the test and disarms
  after; a second click during the test works; refused when the pilot armed
- [ ] Motor MIN / MAX / TRIM + / -: value written, motor **not** driven; log mentions MOT_SAFE_DISARM
- [ ] **Minimum throttle** ▸ **Probe**: ramp, Continue when the wheel turns, value proposed (vehicle keeps the old
  value until Apply); motor stops at once when the probe ends

### 5.4 Board orientation
- [ ] **Board orientation** dropdown
- [ ] **Probe**: reset, reboot, level + nose-down poses, detected orientation with confidence, apply + reboot
  - [ ] Cancel at each stage restores the original value
  - [ ] Discarding the result restores the original value

### 5.5 GPS
- [ ] **Number of GPS receivers** 1 / 2; with 2: **GPS to configure**, **Using two receivers**, **Primary receiver**
  - [ ] Switching count or receiver reloads the fields for that receiver
  - [ ] Back to 1 receiver: GPS 2 type set to 0
- [ ] **GPS type** with **Probe** (Auto, then NMEA, reboot each); probe for GPS 2 uses GPS 2's telemetry
  - [ ] Cancel restores the original type
- [ ] **Connected to serial port** / **Serial baud rate** (or "Not on a serial port" for DroneCAN)
  - [ ] A port used by another protocol (e.g. telemetry): refused
  - [ ] GPS 1 on a higher port than GPS 2: refused (ArduPilot assigns GPS 1 to the lowest GPS port)
  - [ ] Nonexistent port: refused
- [ ] Antenna offsets X / Y / Z
- [ ] **Show GPS status (10 s)** logs fix, satellites, HDOP

### 5.6 Accelerometer & gyro calibration
- [ ] **Gyro calibration**, **Level calibration** (disarmed)
- [ ] **6-position accelerometer calibration**: each pose prompt, Continue, success message
  - [ ] Failure / timeout / Cancel reported

### 5.7 Compass calibration
- [ ] **Automatic declination**, **Detect orientation during calibration** (COMPASS_AUTO_ROT)
- [ ] **External compass orientation** ▸ **Probe**: magnetic north, then south; detected orientation applied + reboot
  - [ ] Hemisphere from GPS latitude; without fix the wizard asks
  - [ ] Internal compass: refused; near the magnetic equator: refused
  - [ ] Cancel restores the original orientation
- [ ] Normal vehicle: **Start compass calibration (rotate by hand)** with progress, success / failure per compass
- [ ] Heavy vehicle: **Current heading (true north)** + **Large Vehicle MagCal**; needs a GPS fix; reboot pending after

### 5.8 Radio control & flight modes (only with RC receiver)
- [ ] **Calibrate radio**: centre, extremes, direction prompts; values and reversals written
  - [ ] A channel the receiver doesn't deliver is reported
- [ ] Steering / throttle channel, **Steering input type**, **Flight mode switch channel**, **Flight mode 1-6**
- [ ] **RCn switch function** for every channel except steering, throttle and the mode channel

### 5.9 Wheel encoders (only if fitted)
- [ ] Encoder 1 fields; 2nd encoder fields only with a 2nd encoder type
- [ ] **Live encoder distance (10 s)**
- [ ] **Calibrate wheel radius** with the rolled distance; an encoder that didn't count is reported

### 5.10 Distance sensors & object avoidance (rangefinder or proximity sensor fitted)
- [ ] Section visibility: rangefinder fields only with a rangefinder, proximity fields only with a proximity sensor
- [ ] Rangefinders overview lists every configured instance
- [ ] **Rangefinder to configure** 1-9, A: fields reload for that instance
- [ ] **Rangefinder type**: serial types show **Rangefinder serial port**, I2C types **I2C address**, others neither
  - [ ] Serial: Apply sets the port to Rangefinder protocol; a port in use is refused
- [ ] **Pointing direction**, min/max distance (metres; centimetres on older firmware), position X/Y/Z
- [ ] Rangefinder **Probe** (I2C types, then serial ones on the chosen port) on the selected instance; cancel restores
- [ ] Proximity sensors overview; **Proximity sensor to configure** 1-5
- [ ] **Proximity sensor type**; serial types show port and **Serial baud rate**; a port in use is refused
- [ ] Proximity **Probe** (RPLidar baud rates, SF40/C) on the chosen port; cancel restores
- [ ] Type **RangeFinder** without a horizontally pointing rangefinder: warning logged; with one: no warning
- [ ] **Mounting**, **Forward direction correction**, min/max range
- [ ] Avoidance: **Avoid: the fence / proximity sensors / beacon fence**, margin, behaviour, back-up speed,
  acceleration
- [ ] **Show distances (10 s)**

### 5.11 Camera gimbals (if fitted)
- [ ] **Number of gimbals** 1 / 2 and **Gimbal to configure**; back to 1: gimbal 2 switched off
- [ ] **Gimbal type** ▸ **Probe** (serial and MAVLink types; you confirm each tilt); cancel restores
  - [ ] The telemetry port (MAVLink2) is refused for the probe
- [ ] **Connected to serial port** for serial/MAVLink gimbals; a port in use is refused
- [ ] Pitch / yaw limits
- [ ] **Test gimbal (tilt down and back)** on the selected gimbal (also gimbal 2; older firmware falls back for gimbal 1)

### 5.12 Servo outputs (extra devices fitted)
- [ ] Outputs overview; **Output to configure** reloads its settings
- [ ] Output used by steering / motors / gimbal: shown locked with the reason
- [ ] **Driven by**: not used / RC pass-through / GPIO / follows RC channel 1-16 (output follows the stick)
  - [ ] RC channel used for steering, throttle or modes: warning
  - [ ] GPIO: PWM fields hidden; hint to the Relays step
- [ ] MIN / MAX / TRIM / Reverse; **+** / **-** move the output

### 5.13 Relays (if fitted)
- [ ] **Relay to configure** 1-6 reloads; **Function**, **GPIO pin number**, **State at start-up**, **Inverted**
- [ ] **Servo output used as this pin** switches that output to GPIO; an output in use is refused
- [ ] **Switched by RC channel**; moving the switch to another channel removes it from the old one
  - [ ] Steering / throttle / mode channel refused; a channel with another option refused
- [ ] **On** / **Off**; an unconfigured relay is refused

### 5.14 Gripper (if fitted)
- [ ] **Gripper enabled** (other settings appear after Apply), **Gripper type** servo / EPM
- [ ] Servo: **Servo output** (exclusive; output in use refused), grab / release / neutral PWM; EPM: re-grab interval
- [ ] **Operated by RC channel** (same refusals as relays)
- [ ] **Test: grab** / **Test: release**; refused while not configured

### 5.15 Temperature sensors (if fitted)
- [ ] **Sensor to configure** 1-3 reloads; type, I2C bus, I2C address; **Logging**

### 5.16 Sprayer (if fitted)
- [ ] **Sprayer enabled** shows the other fields; pump output and optional spinner output (exclusive)
- [ ] Pump rate, minimum pump rate, start speed, spinner PWM

### 5.17 Optical flow (if fitted)
- [ ] Sensor type; **I2C address** only for PX4Flow; scale corrections, mounting yaw, position

### 5.18 Visual odometry (if fitted)
- [ ] Type, camera position X / Y / Z

### 5.19 Positioning beacons (if fitted)
- [ ] Beacon system, receiver serial port (port in use refused, missing port refused), origin, rotation from north

### 5.20 Position sources (EKF) (shown with optical flow / visual odometry / beacons / wheel encoders, or without GPS)
- [ ] **Source set to configure** 1-3 reloads; horizontal/vertical position and velocity, heading
- [ ] **Source set chosen by RC channel** (same refusals as relays)
- [ ] **Fuse velocities from all sources**
- [ ] **Use this source set now**

### 5.21 On-screen display (if fitted)
- [ ] OSD type, screen-switch channel (RC5-RC16), switching method, units, font, warning levels, message time

### 5.22 Remote ID (if fitted)
- [ ] Standard firmware: all settings listed as not available
- [ ] Remote ID firmware: enabled, MAVLink port (unused port switched to MAVLink2; port with another protocol
  refused), DroneCAN, the three option checkboxes

### 5.23 RPM sensors (if fitted)
- [ ] **Sensor to configure** 1-4 reloads; type
- [ ] GPIO type: pin, servo output used as pin (switched to GPIO), scaling; ESC-telemetry type: ESC bitmask
- [ ] Min / max RPM, minimum quality; **Show RPM (10 s)**

### 5.24 Generator (if fitted)
- [ ] Type, serial port (in use / missing refused), baud rate, **Suppress maintenance-required warnings**

### 5.25 Serial ports
- [ ] Overview of all ports; **Port to configure** reloads; protocol and baud rate (reboot pending)

### 5.26 Battery monitor (if fitted)
- [ ] Monitor type, pins, multiplier, amps per volt, offset, capacity, low / critical voltage and actions
- [ ] **Read battery values**; **Calibrate voltage from multimeter**; **Calibrate current from clamp meter**

### 5.27 Failsafes & arming
- [ ] Failsafe action / timeout, GCS failsafe, crash check, EKF failsafe + threshold
- [ ] Throttle (radio) failsafe + PWM only with an RC receiver
- [ ] **Arming check: ...** checkboxes (performed = ticked)
  - [ ] Older firmware (ARMING_CHECK): "All" shows every box ticked; unticking one writes the individual bits;
    ticking all writes "All" again
  - [ ] Current firmware (ARMING_SKIPCHK): unticking writes the skip bit; unknown bits are kept

### 5.28 Scripting
- [ ] **Scripting** Lua: after Apply + reboot the other settings appear (message tells you)
- [ ] Memory, instructions, script folders (ROMFS / SD card), debug options, user parameters 1-6

### 5.29 Save, reboot & finish Phase 1
- [ ] **Show all changes**, **Backup parameters to file**, **Reboot flight controller**
- [ ] **Revert ALL wizard changes** restores every parameter the wizard changed (confirmation first)

## 6. Phase 2 - drive and tune (armed, open space)

For every driving action test both **Who drives?** options (wizard drives automatically / I drive in Manual mode),
the safety brief, **STOP** during the run, and the automatic stop at the runway / arena limit.

### 6.1 Pre-drive checks
- [ ] Runway length and arena radius are remembered
- [ ] **Run pre-drive checks**: each check reported as passed or failed
  - [ ] GPS 3D fix, satellites and HDOP (only with GPS)
  - [ ] EKF healthy (attitude and horizontal velocity)
  - [ ] Battery voltage reported (only with battery monitor)
  - [ ] RC receiver data present (only with RC receiver)
  - [ ] Ground-station id matches the wizard (RC override accepted)
  - [ ] Vehicle armed
- [ ] **Test RC failsafe** (armed, only with RC receiver)

### 6.2 Direction check
- [ ] **Run direction check**: drives forward, then turns right
  - [ ] Wrong drive direction: offers to reverse the motor output(s); Yes reverses them, No leaves them
  - [ ] Wrong steering direction, skid: offers to swap the LEFT/RIGHT motor outputs
  - [ ] Wrong steering direction, front-wheel: offers to reverse the steering servo

### 6.3 Top speed & acceleration
- [ ] **Measure top speed & acceleration** fills top speed, acceleration and deceleration

### 6.4 Turn radius & turn rate
- [ ] **Measure turn radius** (radius test throttle) and **Measure max turn rate** (max-rate throttle; skid pivots)
- [ ] Results fill turn radius, turn rates, steering rate/acceleration, lateral acceleration

### 6.5 Feed-forward & first PID values
- [ ] **Measure steering feed-forward** proposes FF and starting P / I (and filters)
- [ ] **Measure cruise & speed gains** fills cruise speed / throttle and speed gains
- [ ] **RC channel for QuickTune switch**

### 6.6 Closed-loop verification & fine tuning
- [ ] **Verify & tune SPEED controller**: each round runs a speed step, logs overshoot / rise time / error and writes
  adjusted P and I; stops when tracking is good or after **Max refinement rounds**; Continue between rounds
  - [ ] Speed step above WP_SPEED: refused with explanation; default step stays within WP_SPEED
  - [ ] Runway too short for the step: refused
- [ ] **Verify & tune STEERING-RATE controller** (front-wheel: refused when WP_SPEED is too low to turn)
- [ ] Manual driving variant: tracking report

### 6.7 Navigation parameters (only with GPS)
- [ ] **Fill suggestions from measurements**; mission speed, RTL speed, waypoint radius / acceleration / jerk
- [ ] Skid only: pivot angle, rate and delay
- [ ] Older firmware only: max overshoot, L1 period, L1 damping

## 7. Phase 3 - wrap-up

### 7.1 Logging
- [ ] Log backends (SD card, MAVLink, flash), log content checkboxes, logging while disarmed, new file after disarm,
  rate limit, buffer size, free space

### 7.2 Review & finish
- [ ] **Show all changes**, **Save parameter backup**, **Disarm**, **Reboot flight controller**

## 8. Firmware differences

- [ ] Firmware 4.5.x: old parameter names used transparently (GPS_TYPE / GPS_TYPE2, GPS_POS1_*, SYSID_MYGCS,
  ARMING_CHECK, RNGFND1_MIN_CM / _MAX_CM, NAVL1_*, WP_OVERSHOOT)
- [ ] Current firmware: new names (GPS1_TYPE, MAV_GCS_SYSID, ARMING_SKIPCHK, RNGFND1_MIN / _MAX, MNT1_*, PRX1_*)
- [ ] Parameters missing on the connected firmware are hidden and listed as not available, never written
