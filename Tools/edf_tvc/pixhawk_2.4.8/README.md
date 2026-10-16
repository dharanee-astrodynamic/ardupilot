# Single fan with a thrust vectoring nozzle on a Pixhawk 2.4.8

ArduCopter flying one ducted fan whose exhaust is tilted on two axes by a nozzle. It takes off and lands vertically on its
legs. The flight controller is a Pixhawk 2.4.8 (the Pixhawk1 / FMUv3 layout, build targets `Pixhawk1` and `Pixhawk1-1M`).

**Experimental and safety-critical. Never flown.** It was checked in SITL only. Nothing in this document has been run on a
Pixhawk. The change is AI-assisted and is not in upstream ArduPilot.

Files in this folder:

| File | Content |
|------|---------|
| `edf_tvc_pixhawk.param` | starting parameters for a 4S battery and a PWM fan ESC |
| `wiring.svg` | wiring diagram |
| `edf_tvc_pixhawk_nogps.param` | **no GPS**: no horizontal EKF sources, fence off, `DISARM_DELAY` 30. Loaded last. Take-off, hold 0.5 m and land in AltHold: section 7b of `DEPLOY_AND_WIRING.md` |
| `edf_tvc_commands.py` | arm, take off to a height in AltHold, land, status, over MAVLink. Arm, land and status work; the take-off did not in 3 runs (7b) |
| `edf_tvc_pixhawk_lidar.param` | Benewake TFMini Plus as the only height source (`EK3_SRC1_POSZ` 2), loaded last, after measuring `RNGFND1_GNDCLR` |
| `edf_tvc_pixhawk_battery.param` | battery scales and failsafes, loaded after the first file, a reboot and a parameter refresh |
| `DEPLOY_AND_WIRING.md` | build, flash, wiring, set-up, tests and flying on real hardware |

## 1. What was added to ArduCopter

* `FRAME_CLASS 8` (Single) with the new `FRAME_TYPE 22` ("SingleTVC"), in `libraries/AP_Motors/AP_MotorsSingle.*`.
* Motor1 is the nozzle roll servo, Motor2 the nozzle pitch servo, Motor5 the fan. Motors 3, 4 and 6 are not used.
* The nozzle deflection for a requested torque is divided by the thrust (torque is thrust times deflection, bounded so a low
  throttle does not ask for a large deflection). Thrust is never allowed to drop below what the roll and pitch request needs,
  and if the nozzle cannot reach the deflection, roll and pitch are scaled together and flagged as limited.
* **There is no yaw control.** A nozzle gives roll and pitch torque only. The yaw demand is discarded and the yaw limit flag is
  set so the yaw controller does not wind up. The fan reaction torque will turn the airframe.
* The other Single frame types behave as before (the new code runs only for type 22).
* A SITL model (`--model edf-tvc`) and the autotest `Copter.EDFThrustVectoring`.

With the default output functions the mapping on the Pixhawk is already correct, so no `SERVOn_FUNCTION` has to be set:

| Output | Function | Used for |
|--------|----------|----------|
| MAIN 1 | Motor1 (33) | nozzle roll servo |
| MAIN 2 | Motor2 (34) | nozzle pitch servo |
| MAIN 5 | Motor5 (37) | fan ESC |

The MAIN outputs are driven by the IO processor in three timer groups: 1-2, 3-4 and 5-8. The fan output runs at the loop rate
(400 Hz) and the servos stay at 50 Hz, because they are in a different group. MAIN outputs are PWM only: there is no DShot, so
the fan ESC must take a PWM signal at 400 Hz.

## 2. Deploying to hardware

Build, flash, wiring (with pin tables), parameters, bench and fan tests, ESC calibration, flying and troubleshooting are in
[DEPLOY_AND_WIRING.md](DEPLOY_AND_WIRING.md). Build sizes measured here:

| Target | Flash used | Free |
|--------|-----------|------|
| `Pixhawk1` (2 MB) | 1,577,436 B | 503,328 B |
| `Pixhawk1-1M` (1 MB, reduced features) | 967,412 B | 64,768 B |

## 3. Test in SITL

```bash
Tools/autotest/autotest.py build.Copter test.Copter.EDFThrustVectoring
```

It checks the frame and the default output mapping, then takes off, hovers in GUIDED and LOITER within 20 degrees of level,
checks that outputs 3, 4 and 6 are idle, and lands with RTL.

## 3a. Test in Gazebo

`~/run_edf_tvc_pixhawk_sim.sh` starts Gazebo (the `edf_tvc` model, 1.5 kg, 30 N fan) and ArduCopter SITL, flying **these Pixhawk
parameter files** (`edf_tvc_pixhawk.param`, `edf_tvc_pixhawk_battery.param`) plus a small overlay for the simulation
(`~/SITL_Models/Gazebo/config/edf_tvc_pixhawk_sim.param`: sensor offsets, a 4S battery voltage, the model's linear 30 N thrust
curve, and the model's lidar in place of the serial one). See section 17 of [DEPLOY_AND_WIRING.md](DEPLOY_AND_WIRING.md).

```bash
~/run_edf_tvc_pixhawk_sim.sh                # Gazebo GUI, SITL, MAVProxy console and map
HEADLESS=1 ~/run_edf_tvc_pixhawk_sim.sh     # no Gazebo GUI
```

## 4. What was verified, and what was not

Verified:

* `Pixhawk1` and `Pixhawk1-1M` both build (sizes above), and the SITL build.
* The `EDFThrustVectoring` autotest passes.
* **No GPS** (the vehicle has none): in Gazebo, with the files in this folder including `edf_tvc_pixhawk_nogps.param`, arming in AltHold with the throttle low, centring it
  and sending the AltHold take-off command climbed to about 0.5 m above the start, held 10 s (+/-5 cm on the lidar, 0.1 deg of tilt) and landed in Land mode: 3 runs of 3.
  **Not working:** with no RC input at all, with the stick above mid-position, and through `edf_tvc_commands.py takeoff` (3 runs of 3 did not take off). See section 7b.
* In Gazebo, with a GPS, the TFMini Plus lidar as the only height source and the 0.5 m settings, a Guided take-off held a lidar reading of 0.46 to 0.52 m,
  flew 3 m north, 3 m east and back, and landed. **Not solved:** the AltHold throttle-stick take-off was not reliable, and the 0.8 m ceiling overshoots by
  about 0.5 m: see sections 7a and 17 of [DEPLOY_AND_WIRING.md](DEPLOY_AND_WIRING.md).
* In Gazebo, before the lidar was added (barometer height, 6 m), flying the Pixhawk parameter files: take-off to 6 m, 8 m north, 8 m east, back and landing, twice, with 0.1 degrees of tilt
  in hover. This needed the rate gains in the file changed from 0.07 to 0.20: see section 17 of [DEPLOY_AND_WIRING.md](DEPLOY_AND_WIRING.md).
* The existing Single and Coax frames still take off in SITL after the change: a GUIDED take-off to 6 m reached 5.8 m with under 0.5 degrees of tilt on both.
* In SITL, `edf_tvc_pixhawk.param` loads, every name exists and takes the value in the file, the default functions are 33, 34 and 37
  on outputs 1, 2 and 5 and 0 on the rest, and the roll and pitch sticks move outputs 1 and 2 within the `SERVOn_MIN/MAX` limits.

Not verified:

* Anything on a Pixhawk: the flash, the IO processor, the safety switch behaviour, the 400 Hz PWM into your ESC, servo
  directions, the thrust curve, the gains, current draw, vibration, and the real yaw spin.
* The firmware choice for your board (2 MB or 1 MB), the power module rating and the servo rail limit.
* A servo, ESC or fan failure.
* The full flight tests of the existing Single and Coax frames (`FlyEachFrame`) were not rerun after this change.
