# Single EDF with 2-axis thrust vector nozzle (Pixhawk 2.4.8)

Setup, build, upload and test notes for a single electric ducted fan (EDF) with a
2-axis thrust vectoring nozzle, flown with ArduCopter on a Pixhawk 2.4.8
(`Pixhawk1` firmware), a GPS and an RC transmitter.

## Status and limitations

- The firmware supports this airframe: `FRAME_CLASS=8` (Single) with
  `FRAME_TYPE=22` (SingleTVC). See `libraries/AP_Motors/AP_MotorsSingle.cpp`.
- A nozzle cannot produce torque about the thrust axis, so there is **no yaw
  control**. Heading will drift or spin unless yaw authority is added (vanes,
  reaction wheel or a separate yaw thruster). Reliable GPS Loiter is not proven
  until this is solved and tested.
- Static code inspection only. Flight behaviour has not been verified on
  hardware.
- Do not arm untethered until every check below passes.

## Code verification summary

Checked by reading the source. No source files were changed for this setup.

| Item | Result | Where |
|---|---|---|
| Single EDF with 2-axis nozzle | Supported: nozzle roll on output 1, nozzle pitch on output 2, fan on output 5 | `libraries/AP_Motors/AP_MotorsSingle.cpp`, `FRAME_TYPE` in `ArduCopter/Parameters.cpp` |
| Yaw control | Not available. The mixer sets `limit.yaw = true` | `AP_MotorsSingle::output_armed_stabilizing_tvc()` |
| GPS Loiter | Needs a healthy EKF position and passes the GPS pre-arm checks | `AP_Arming_Copter::gps_checks()` in `ArduCopter/AP_Arming_Copter.cpp` |
| TFMini serial rangefinder | Supported as `RNGFND1_TYPE 20` | `libraries/AP_RangeFinder/AP_RangeFinder.h` |
| TFMini I2C rangefinder | Supported as `RNGFND1_TYPE 25` | `libraries/AP_RangeFinder/AP_RangeFinder.h` |
| Downward orientation | `RNGFND1_ORIENT 25` (Pitch270) | `libraries/AP_Math/rotations.h` |

Static inspection cannot prove that a given EDF, nozzle linkage and centre of
mass will hover safely.

## Build

From the repository root:

```sh
git submodule update --init --recursive
./waf configure --board Pixhawk1
./waf copter
```

Output: `build/Pixhawk1/bin/arducopter.apj`

Use the board name that matches your hardware (`./waf list_boards`). The
Pixhawk1 (fmuv3) target is flash-limited, so confirm the build still contains
the features you need.

## Upload

- **Mission Planner**: Setup, Install Firmware, Load custom firmware, then select
  the `.apj`.
- **QGroundControl**: Vehicle Setup, Firmware, Advanced, Custom firmware file.
- **Direct**: `./waf copter --upload` on native Linux, or with USB passthrough
  (`usbipd`) in WSL.

If you build in WSL, copy the file to Windows first, for example
`cp build/Pixhawk1/bin/arducopter.apj /mnt/c/Users/<you>/Desktop/`.

## Wiring (Pixhawk 2.4.8, MAIN OUT)

| MAIN OUT | Function | Parameter |
|---|---|---|
| 1 | Nozzle roll servo | `SERVO1_FUNCTION 33` |
| 2 | Nozzle pitch servo | `SERVO2_FUNCTION 34` |
| 5 | EDF ESC signal | `SERVO5_FUNCTION 37` |

- Each output has signal, V+ and GND pins.
- The Pixhawk does not power the MAIN OUT rail from USB or the power module.
  Feed 5 V to the rail from the ESC BEC or a separate BEC, using one source only.
- On output 5 connect the ESC signal and ground. Leave the ESC red wire
  disconnected if the rail is powered from another source.
- RC receiver to RC IN. GPS to the GPS port (and the compass port if present).
  Power module to POWER, with `BATT_MONITOR` set to match (`0` if none).

## Parameters

Write these, then reboot:

```text
FRAME_CLASS 8
FRAME_TYPE 22
SERVO1_FUNCTION 33
SERVO2_FUNCTION 34
SERVO5_FUNCTION 37
RNGFND1_TYPE 0
```

The SITL defaults are in `Tools/autotest/default_params/copter-edf-tvc.parm`.

### Rangefinder (not used)

A TFMini rangefinder is supported (`RNGFND1_TYPE 20`, `RNGFND1_ORIENT 25`,
UART set to `SERIALx_PROTOCOL 9` and `SERIALx_BAUD 115`) but is not used here. Set `RNGFND1_TYPE 0`
and check `RNGFND2_TYPE` to `RNGFND4_TYPE` are also `0`, otherwise arming fails
with `PreArm: Rangefinder A: Not Detected`. Without a rangefinder, altitude
comes from the barometer and EKF, and surface tracking is inactive.

## Board orientation

Default is `AHRS_ORIENTATION 0`: board flat, label up, arrow toward the front.

For a board mounted vertically with the arrow pointing up (vehicle in hover
attitude), pick the value by which way the board label faces:

| Label faces | `AHRS_ORIENTATION` |
|---|---|
| Rear | `24` (Pitch90) |
| Front | `29` (Roll180Pitch90) |
| Right | `28` (Roll90Pitch90) |
| Left | `30` (Roll270Pitch90) |

"Front" is whichever direction you choose as forward. Keep the nozzle servo
wiring consistent with that choice.

After setting it, reboot, then:

1. Run the accelerometer calibration with the vehicle standing in hover
   posture (thrust pointing down).
2. Redo the compass calibration. An external GPS compass uses `COMPASS_ORIENT`.
3. Check the HUD: nose down must show nose down, right roll must show right
   wing down. If wrong, try another row.

## Testing

### SITL first

```sh
Tools/autotest/autotest.py build.Copter test.Copter.EDFThrustVectoring
```

Or interactively:

```sh
Tools/autotest/sim_vehicle.py -v ArduCopter --model edf-tvc --console --map
```

### Servo and nozzle check (fan or ESC unpowered)

1. Set the outputs above, write params, reboot, and press the safety switch.
2. In Stabilize, disarmed, the stick roll and pitch are passed straight to
   servos 1 and 2. Move the sticks and confirm smooth, unbound nozzle motion.
   Watch `ch1out` and `ch2out` on the Status or Servo/Relay tab.
3. Set `SERVOx_MIN`, `SERVOx_MAX` and `SERVOx_TRIM` so trim points the thrust
   straight down and the nozzle does not hit its stops. Use `SERVOx_REVERSED`
   for wrong directions.
4. Arm in Stabilize (fan unpowered) and tilt the airframe by hand. The nozzle
   must deflect to push it back to level. Roll must move servo 1 and pitch
   servo 2. If wrong, reverse the servo or rewire.
5. Do not use Motor Test for the nozzle servos. It drives them as motors.

### Fan check

- Calibrate the ESC if needed and set `MOT_PWM_MIN` and `MOT_PWM_MAX`.
- With the fan blocked or removed, raise throttle slowly and confirm a smooth
  spin-up.

### Flight progression

1. Tethered hover in Stabilize.
2. AltHold.
3. Loiter, only after AltHold is stable and the GPS has a good lock (8 or more
   satellites, HDOP below 2).
4. Tune the rate PIDs and `MOT_THST_HOVER`, and review the `.bin` logs.

Keep a Stabilize mode on an RC switch and set `FS_THR_ENABLE` and
`FS_GCS_ENABLE` before any untethered flight.

### Takeoff with the transmitter and position hold

- In Loiter, the pilot's throttle stick controls climb rate and the roll and
  pitch sticks move the position target. With the sticks centred the vehicle
  holds position. See `ArduCopter/mode_loiter.cpp`.
- Loiter needs a good GPS and EKF position. Without a fix it cannot arm in
  Loiter and position hold does not work.
- Altitude comes from the barometer, so hover height drifts slightly with
  pressure changes.
- Loiter holds position only with stable heading. With no yaw authority, expect
  heading drift that can degrade position hold.
- Start in Stabilize or AltHold for the first takeoff, then switch to Loiter
  once hovering is steady.

## Firmware check

The firmware version shown in Mission Planner (`V4.8.0-dev (aabea1e3)`) matches
the local checkout (`git rev-parse --short=8 HEAD`), so the board runs this
build. Re-check this after each upload.

The frame type shown as `COAXIAL` in Mission Planner is normal. SingleTVC
reports that MAV type.

## Bring-up observations

Seen on the first power-up, with the fixes:

| Observation | Cause | Fix |
|---|---|---|
| `Sats: 0`, `hdop 1000`, map shows a far-away location | No GPS fix | Move outdoors and wait for a 3D fix |
| `safetyactive True` | Safety switch on | Hold the safety button until solid |
| `PreArm: Rangefinder A: Not Detected` | `RNGFND1_TYPE` still non-zero | Set `RNGFND1_TYPE 0` and reboot |
| `FAILSAFE` on the HUD with `Bat1 0.00v` | No RC link or no power module | Power the transmitter, or set `BATT_MONITOR 0` |
| GPS 3D Fix with 5 satellites and HDOP 3.2 | Marginal for Loiter | Wait for 8 or more satellites and HDOP below 2 |

Parameters are stored on the Pixhawk, so edits in the repository do not change
them. Write params from Mission Planner (Config, Full Parameter List) and reboot.

## Arming and flying with the RC transmitter

Stick layout is mode 2 by default: left stick is throttle (up and down) and yaw
(left and right), right stick is pitch and roll.

### Set up the flight mode switch

- All six flight modes default to Stabilize (`ArduCopter/config.h`). Assign them
  yourself with `FLTMODE1` to `FLTMODE6`, which are selected by the PWM on the
  channel set by `FLTMODE_CH`.
- A suggested three-position layout: Stabilize (0), AltHold (2), Loiter (5).
- Optionally map a switch to Motor Emergency Stop with `RCx_OPTION 31`. Rudder
  disarm does not work reliably in flight.
- Calibrate the radio first (Setup, Mandatory Hardware, Radio Calibration) so
  throttle reads full range and the sticks centre correctly.

### Arm

1. Put the switch in Stabilize for the first flights. Loiter and other GPS modes
   need a position lock before they will arm.
2. Make sure the pre-arm checks are clean: no `PreArm:` message, GPS and
   compass good, safety switch released, no failsafe on the HUD.
3. Move the throttle stick fully down.
4. Hold the yaw stick fully right (rudder arming) for about two seconds. The
   motor-armed indicator comes on and the EDF goes to ground idle.
5. `ARMING_RUDDER` must be `1` (ArmingOnly) or `2` (ArmOrDisarm). The Copter
   default is `2`. Arming by rudder only works with throttle at zero.

Optional: set `RCx_OPTION 153` (ArmDisarm) on a switch to arm and disarm without
the sticks.

### Take off

- **Stabilize**: throttle is direct thrust. Raise it slowly until the vehicle
  lifts. Hover is roughly `MOT_THST_HOVER`. Use tether lines for the first
  flights.
- **AltHold**: centre the throttle stick to hold altitude. Above centre climbs
  and below centre descends.
- **Loiter**: after arming, raise the throttle above mid-stick and the vehicle
  takes off, climbing at the stick-commanded rate. Centre the throttle to hold
  height. The initial climb target is `PILOT_TKO_ALT_M`.

### Hold and move

- **Loiter holds the position** when roll and pitch sticks are centred. Move the
  sticks to translate, release to hold the new position.
- The yaw stick commands a yaw rate. A nozzle has no yaw torque, so this stick
  has no real effect and heading will drift (see Status and limitations).

### Land and disarm

1. In Loiter or AltHold, lower the throttle stick below centre to descend.
2. Close to the ground, switch to Land, or reduce throttle gently in Stabilize.
3. When landed, hold the throttle fully down. The vehicle disarms after
   `DISARM_DELAY` (default 10 s). Rudder disarm: throttle down, yaw stick fully
   left.

### Emergency

- Switch to Stabilize and take manual control.
- Use the Motor Emergency Stop switch if configured. It cuts the fan immediately.
- Keep clear of the airframe. Expect an uncontrolled fall once the fan stops.

## Troubleshooting

### Will not arm

- `PreArm: Rangefinder A: Not Detected`: set `RNGFND1_TYPE 0` and reboot.
- Safety switch active: hold the safety button until it goes solid.
- No GPS lock: wait for a 3D fix with good HDOP, or arm in a mode that does not
  need GPS (Stabilize, AltHold).
- Read the full text on the Messages tab.

### FAILSAFE on the HUD

- Radio failsafe: turn the transmitter on, confirm the receiver is bound, and
  verify the sticks in Radio Calibration. Check `FS_THR_ENABLE`.
- Battery failsafe: a 0.00 V reading means no power module or a wrong
  `BATT_MONITOR`. Set `BATT_MONITOR 0` if none is fitted.

### Disarms automatically

- **Idle auto-disarm**: disarms when throttle is low and the vehicle is landed,
  after `DISARM_DELAY` (default 10 s). Raise throttle within the delay, or set
  `DISARM_DELAY 0` for bench testing. See `ArduCopter/motors.cpp`.
- **Crash check**: message `Crash: Disarming: AngErr=...`. Triggers on lean over
  15 degrees, attitude error over 30 degrees and low acceleration for 2 s. A wrong
  `AHRS_ORIENTATION` for a vertical board commonly causes this. Fix the
  orientation and recalibrate. `FS_CRASH_CHECK 0` disables it for bench tests
  only. See `ArduCopter/crash_check.cpp`.
- A fan that is not spinning keeps the vehicle "landed", so idle auto-disarm
  fires. Test with the fan or ESC powered.
- Radio or battery failsafe: the message names the cause.

### Erratic behaviour

1. Confirm `FRAME_CLASS 8` and `FRAME_TYPE 22`.
2. Confirm the `SERVOx_FUNCTION` values.
3. Calibrate accelerometer and compass, and check `AHRS_ORIENTATION`.
4. Verify nozzle direction by tilting the airframe.
5. Check `MOT_PWM_MIN` and `MOT_PWM_MAX` and calibrate the ESC.
6. Expect heading drift (no yaw control).

For further diagnosis, review the `.bin` log from the DataFlash Logs tab.
