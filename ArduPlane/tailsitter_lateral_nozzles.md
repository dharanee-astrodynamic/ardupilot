# Twin-EDF tailsitter with 2-axis thrust-vectoring nozzles

This document describes the changes that let a twin-motor tailsitter hold
attitude in VTOL modes using only 2-axis vectored nozzles. In this mode there is
no differential thrust and no control surface, flap or spoiler output in hover. It also covers hardware setup, building, SITL testing
and the items still to be investigated.

This work was AI-assisted. It has been compiled and tested in SITL for servo
output behaviour only. It has **not** been flown and SITL does not model nozzle
forces, so the control loops and gains are unvalidated.

## 1. Background

The existing ArduPilot tailsitter code (`ArduPlane/tailsitter.cpp`,
`libraries/AP_Motors/AP_MotorsTailsitter.cpp`) already supports two tilting
motors ("vectored tailsitter") with servo functions `TiltMotorFrontLeft` (75)
and `TiltMotorFrontRight` (76). In the copter-style VTOL controller:

| Copter axis | Body motion (hover) | Actuation before this change |
|---|---|---|
| Pitch | pitch about body Y | both pitch-plane nozzles together (75, 76) |
| Yaw | rotation about the thrust axis (FW roll) | pitch-plane nozzles in opposition (75, 76) |
| Roll | FW yaw (body Z) | differential thrust and/or rudder surface |

Roll was therefore the only axis that could not be produced by nozzles. With
2-axis nozzles, the second (lateral) axis of the two nozzles, moved in the same
direction, creates a yaw moment about body Z and covers that axis.

## 2. Summary of changes

Two new servo functions were added, one lateral-axis servo per nozzle:

| Function | Name | Meaning |
|---|---|---|
| 190 | `TiltMotorLeftLateral` | left nozzle, lateral axis |
| 191 | `TiltMotorRightLateral` | right nozzle, lateral axis |

### Nozzle-only hover

Nozzle-only hover is active when `Q_FRAME_CLASS` is the tailsitter class,
`Q_TAILSIT_VHGAIN` is above 0 and all four nozzle functions (75, 76, 190, 191)
are assigned. It is evaluated at boot. In this mode, while the aircraft is in a
VTOL mode:

- Differential thrust is never used: both motors always receive the same
  throttle.
- The aileron, elevator and rudder demands from the copter controller are
  scaled by an airspeed based factor. At low airspeed the factor is zero, so
  elevons and V-tails (which are mixed from them) stay centred. The factor fades
  in between half and full `AIRSPEED_MIN`, and above that is reduced with the
  square of the airspeed, so the surfaces assist the nozzles in the transitions
  and when the vehicle is still moving fast. Without an airspeed estimate it is
  zero.
- Every other control surface function is centred at the end of the servo output
  stage, including flaps, flaperons, spoilers and airbrakes
  (`SRV_Channel::is_control_surface()`), whatever commands them. The surfaces
  that follow the copter demand are centred only while the factor is zero.
- The forward flight controller keeps control of the surfaces in FW modes, and
  while the nose is raised on the transition from FW to a VTOL mode. The
  surfaces must not be centred in that phase, as they are the only pitch control
  until the copter controller takes over. During that phase the pitch plane
  nozzles also follow the elevator and aileron demand, with the `Q_TAILSIT_VHGAIN`
  gain (or `Q_TAILSIT_VFGAIN` if that is larger).

If a lateral servo function is assigned but the setup above is incomplete
(for example `Q_TAILSIT_VHGAIN` is 0 or servo 76 is missing), arming is refused
with `TAILSIT nozzles need Q_TAILSIT_VHGAIN>0 and servo functions 75, 76, 190, 191, reboot`.

Details of the roll axis, when a lateral servo is assigned:

- The copter roll demand (`_roll_in + _roll_in_ff`) is sent to both lateral
  servos, with the same sign on both.
- Differential thrust for roll is disabled: the thrust split between the two
  motors is not used. Throttle is symmetric.
- If the roll demand reaches +/-1, `motors->limit.roll` is set so the roll rate
  integrator does not wind up.
- In `Tailsitter::output()` the lateral outputs are scaled by the same
  `Q_TAILSIT_VHGAIN` and throttle-based gain scaling (`Q_TAILSIT_GSCMSK`) as the
  existing pitch-plane nozzle outputs. Saturation of a lateral servo also sets
  the roll limit flag.
- In forward flight modes the lateral servos are driven to zero.
- If no lateral servo is assigned, behaviour is unchanged.

### Files changed

| File | Change |
|---|---|
| `libraries/SRV_Channel/SRV_Channel.h` | new enum values `k_tiltMotorLeftLat = 190`, `k_tiltMotorRightLat = 191` |
| `libraries/SRV_Channel/SRV_Channel.cpp` | `SERVOn_FUNCTION` `@Values` entry for 190, 191 (Plane) |
| `libraries/SRV_Channel/SRV_Channel_aux.cpp` | new functions default to the angle (+/-4500) output scale |
| `libraries/AP_Motors/AP_MotorsTailsitter.h/.cpp` | `_has_lateral_vectoring`, `_tilt_lateral`; roll mapped to lateral servos instead of differential thrust; output to the two new functions |
| `ArduPlane/tailsitter.cpp/.h` | lateral outputs in hover (normal path and `TAILSIT_Q_ASSIST_MOTORS_ONLY` path), zero in FW, throttle scaling in `speed_scaling()`, roll limit flag; nozzle-only hover flag, surface demands zeroed, `neutralise_surfaces()` |
| `ArduPlane/servos.cpp` | calls `tailsitter.neutralise_surfaces()` after the other surface mixers |
| `ArduPlane/AP_Arming_Plane.cpp` | pre-arm check for an incomplete nozzle setup |
| `Tools/autotest/quadplane.py` | new SITL test `QuadPlane.TailsitterLateralNozzles` |

The attitude controller, rate PIDs, flight modes and transition logic are not
modified.

## 3. Control allocation (hover)

```text
roll  (copter)  -> lateral servos 190 + 191, same sign
pitch (copter)  -> pitch-plane servos 75 + 76, same sign
yaw   (copter)  -> pitch-plane servos 75 + 76, opposite sign
throttle        -> both EDFs, equal
```

Each output is then scaled by `Q_TAILSIT_VHGAIN` (default 0.5) and the throttle
gain scaler (nozzle force is proportional to thrust). Servo direction and the
physical deflection per unit demand are set by `SERVOn_REVERSED`,
`SERVOn_MIN/MAX/TRIM` and the linkage geometry.

## 4. Hardware setup

Required:

- 2 EDFs with ESCs, mounted side by side, thrust along the fuselage axis.
- 4 nozzle servos: pitch-plane and lateral axis for each nozzle.
- A flight controller with at least 6 PWM outputs (2 ESC + 4 servo).

Suggested output assignment (any free output can be used):

| Output | `SERVOn_FUNCTION` | Purpose |
|---|---|---|
| 1 | 73 `ThrottleLeft` | left EDF ESC |
| 2 | 74 `ThrottleRight` | right EDF ESC |
| 3 | 75 `TiltMotorFrontLeft` | left nozzle, pitch plane |
| 4 | 76 `TiltMotorFrontRight` | right nozzle, pitch plane |
| 5 | 190 `TiltMotorLeftLateral` | left nozzle, lateral axis |
| 6 | 191 `TiltMotorRightLateral` | right nozzle, lateral axis |

Notes:

- Control surfaces (elevator, aileron, rudder, elevon, V-tail, flaps and so on)
  may be fitted for forward flight. They are held at their centre in VTOL
  modes when nozzle-only hover is active. Do not rely on that for surfaces
  that must be moved in hover, such as a flap used as an air brake.
- Put ESC outputs and servo outputs in different timer groups, because they use
  different PWM rates (ESC rate is set by `Q_M_PWM_*`/`SERVO_*` settings; servos
  are normally 50 Hz). Check the board's output groups.
- Nozzle authority scales with thrust. There is no control authority at zero or
  very low throttle (e.g. on the ground before spool-up).

## 5. Parameters

Minimum set:

```text
Q_ENABLE            1
Q_FRAME_CLASS       10      # tailsitter
Q_TAILSIT_ENABLE    1
SERVO1_FUNCTION     73
SERVO2_FUNCTION     74
SERVO3_FUNCTION     75
SERVO4_FUNCTION     76
SERVO5_FUNCTION     190
SERVO6_FUNCTION     191
```

Reboot after changing `SERVOn_FUNCTION` or `Q_*` frame parameters.

Parameters to tune:

| Parameter | Notes |
|---|---|
| `Q_TAILSIT_VHGAIN` | Scales all hover nozzle output (default 0.5). Raise toward 1.0 if the nozzles have limited travel and you rely on the PIDs for gain. |
| `Q_TAILSIT_VHPOW` | Extra pitch-plane deflection at large pitch error. Only affects the pitch axis. |
| `Q_TAILSIT_GSCMSK`, `Q_TAILSIT_GSCMIN`, `Q_TAILSIT_GSCMAX` | Throttle-based gain scaling, applied to all nozzle outputs including the lateral ones. |
| `Q_TAILSIT_VT_R_P` / `VT_P_P` / `VT_Y_P` | Scale applied to control surface outputs only. They do not scale the nozzle outputs. |
| `SERVOn_REVERSED`, `SERVOn_MIN/MAX/TRIM` | Match direction and travel to the nozzle mechanism. |
| `Q_A_RAT_RLL_*`, `Q_A_RAT_PIT_*`, `Q_A_RAT_YAW_*` | Rate PIDs. Roll is the lateral axis, yaw is the differential pitch-plane axis. |

Initial bring-up order:

1. Props off and aircraft restrained. Arm in `QSTABILIZE`/`QHOVER`.
2. Confirm each servo moves in the correct direction for a pitch, roll and yaw
   stick input; use `SERVOn_REVERSED` to fix.
3. Confirm the nozzles are mechanically centred at the trim PWM.
4. Start with low rate gains and tether the aircraft for the first spool-up.

## 6. Building

```sh
# SITL
./waf configure --board sitl
./waf plane

# Hardware (replace with your board)
./waf configure --board <BoardName>
./waf plane
```

Never run `waf` with `sudo`. Use `./waf list_boards` to see available boards.

## 7. Testing in SITL

### Automated test

```sh
Tools/autotest/autotest.py build.Plane test.QuadPlane.TailsitterLateralNozzles
# or, without rebuilding
Tools/autotest/autotest.py --no-clean test.QuadPlane.TailsitterLateralNozzles
```

The test uses the `plane-tailsitter` model and checks:

- with only the lateral servos assigned, arming is refused (incomplete setup);
- with the full setup, arming in `QHOVER` and applying a roll stick input:
  - both lateral servos move away from trim in the same direction;
  - the two throttle outputs stay equal (no differential thrust);
  - aileron, elevator and rudder servos stay at trim.

The test sets `AIRSPEED_MIN` to 40 so the surfaces stay centred in the airflow of the
SITL climb. `test.QuadPlane.TailsitterNozzleSurfaceAssist` sets it to 5 and checks that
the elevator then also moves, together with the nozzles, for a pitch demand.

All outputs are read from the same `SERVO_OUTPUT_RAW` message, so they are
consistent in time. Without the surface changes the surface check fails.

Logs are written to `~/buildlogs/QuadPlane-TailsitterLateralNozzles.txt`.

To check that the existing tailsitter behaviour is unchanged:

```sh
Tools/autotest/autotest.py --no-clean test.QuadPlane.Tailsitter
```

### Interactive SITL

```sh
Tools/autotest/sim_vehicle.py -v ArduPlane -f plane-tailsitter --console --map
```

In MAVProxy:

```text
param set SERVO1_FUNCTION 73
param set SERVO2_FUNCTION 74
param set SERVO3_FUNCTION 75
param set SERVO4_FUNCTION 76
param set SERVO5_FUNCTION 190
param set SERVO6_FUNCTION 191
reboot
```

After the reboot:

```text
mode QHOVER
arm throttle
rc 3 1700
rc 1 1800
```

Optionally assign surfaces (for example `SERVO9_FUNCTION 4`, `SERVO10_FUNCTION 19`,
`SERVO11_FUNCTION 21`) to see them stay centred in hover.

Watch `SERVO_OUTPUT_RAW` (for example `watch SERVO_OUTPUT_RAW`, or
`graph SERVO_OUTPUT_RAW.servo5_raw SERVO_OUTPUT_RAW.servo6_raw`). Servos 5 and 6
should move together while servos 1 and 2 stay equal and any surface servos
stay at trim. Return the sticks to
neutral with `rc 1 1500` and `rc 3 1000`, then `disarm force`.

### What SITL can and cannot show

- It can show: parameter handling, which servo function receives which demand,
  signs, limits and that the build and existing tailsitter tests still work.
- It cannot show: whether the nozzles actually stabilise the aircraft. The
  SITL tailsitter model uses control surfaces and motors, not nozzle force and
  moment. Gain tuning has to be done on the real aircraft, or after adding a
  nozzle model to `libraries/SITL`.

## 8. Items to look into

Open points and risks, roughly by priority:

1. **Flight validation.** No flight testing was done. Verify axis signs, rate
   gains and authority tethered, then free flight. Consider a SITL physics
   model of the nozzles so that tuning and control logic can be tested.
2. **Disarmed and failsafe output.** Check where the lateral servos sit when
   disarmed and on emergency stop. The zero-throttle "flare" path in
   `ArduPlane/servos.cpp` sets the existing tilt servos but does not know about
   the lateral ones.
3. **Forward flight.** The lateral servos are held at zero in FW modes, so there
   is no yaw control from the nozzles in FW. Decide whether `Rudder` demand
   should be mixed to the lateral axis (a forward-flight gain similar to
   `Q_TAILSIT_VFGAIN`).
4. **Transitions.** Both transitions were flown in a Gazebo simulation of a
   twin-EDF airframe with elevons (see the SITL_Models `tailsitter_nozzle`
   model): VTOL to FW, FW to VTOL and a QRTL return and landing. Real aerodynamics
   and nozzle forces still need to be checked. The nozzle
   lateral axis is centred in FW and starts from zero at the transition. Check
   `Tailsitter::relax_attitude_control()` and the `vtol_limit` logic with a
   vectored airframe.
5. **Low-thrust authority.** Nozzles give no control without airflow. Consider
   `Q_M_SPIN_MIN`/`Q_M_SPIN_ARM`, `Q_TAILSIT_MIN_VO`/disk-loading settings and
   takeoff and landing behaviour.
6. **Gain scaling.** `Q_TAILSIT_VHGAIN` and the throttle scaler are shared by
   all axes. If the two nozzle axes have different mechanical gains, a separate
   lateral gain parameter may be needed.
7. **Motor test.** `AP_MotorsTailsitter::_output_test_seq()` does not include
   the lateral servos, so they cannot be exercised with the motor test command.
8. **Logging.** No new log fields were added. Use `ATT`, `RATE`, `RCOU`, `QTUN`
   and `TSIT` to review flights; adding lateral-axis demand to `TSIT` would help
   tuning.
9. **Servo numbering.** 190 and 191 are not allocated upstream. If rebasing on
   upstream ArduPilot, check these numbers have not been used for something
   else, and update the parameter documentation.
10. **Thermal and mechanical limits.** Nozzle linkage travel, slew rate and servo
    bandwidth bound the achievable control rate. EDF exhaust heat on servos and
    linkages should also be considered.
11. **Test coverage.** The autotest checks servo output only, and runs on a model
    that has control surfaces. Add checks for FW zeroing, saturation limiting
    and the `TAILSIT_Q_ASSIST_MOTORS_ONLY` path.

12. **Nozzle-only hover without surfaces to fall back on.** With surfaces and
    differential thrust disabled in hover, any nozzle servo or linkage failure
    leaves no backup control. There is no failsafe for a stuck nozzle. The mode is
    decided at boot, so changing `Q_TAILSIT_VHGAIN` or servo functions needs a
    reboot.
13. **Q assist in FW modes.** In assisted flight in a FW mode (`Q_ASSIST_*`),
    the aileron, elevator and rudder demands are also scaled by the airspeed
    factor of the nozzle-only mode, because the VTOL controller is driving the
    outputs. Flaps and spoilers
    are only centred in VTOL modes and during the nose-down FW transition. With
    the `TAILSIT_Q_ASSIST_MOTORS_ONLY` option, the surfaces stay under the plane
    controller in that case.

## 9. Contribution notes

If these changes are to be proposed upstream, follow the repository
contribution guidelines: one subsystem per commit (for example
`SRV_Channel:`, `AP_Motors:`, `Plane:`, `autotest:`; `git subsystems-split` can
split a combined commit), run `Tools/scripts/check_branch_conventions.py`, state
that the work was AI-assisted, and include real test evidence. Open a
discussion with the maintainers first, since this changes tailsitter control
allocation.
