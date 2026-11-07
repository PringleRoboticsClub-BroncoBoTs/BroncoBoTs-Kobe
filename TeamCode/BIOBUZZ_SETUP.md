# BIOBUZZ (2026-27) Robot Code: Setup & Tuning

Branch: `biobuzz-integration`. It's built on `program_drivetrain` and uses FTC SDK **12.0**, Pedro Pathing **3.0.2** and FTC Dashboard **0.5.1**.

Requires **Android Studio Narwhal 3 Feature Drop or later**, because SDK 12 uses Gradle 9.1 and AGP 8.13.2. Older Android Studio will offer to downgrade AGP. **Don't accept that.**

## 1. Driver Station robot configuration

Names must match exactly. They're defined in `config/RobotConfig.java`.

| Name | Type | Hardware |
|---|---|---|
| `frontLeft` `frontRight` `backLeft` `backRight` | goBILDA 5202/3/4 motor | drivetrain (same as `program_drivetrain`) |
| `shooterMotor` | goBILDA 5202/3/4 motor | 6000 RPM flywheel motor (encoder cable required) |
| `turretMotor` | goBILDA 5202/3/4 motor | 5203 117 RPM turret motor (encoder cable required) |
| `intakeMotor` | goBILDA 5202/3/4 motor | 1620/1650 RPM intake |
| `armLeft` `armRight` | Servo | front arm pair |
| `gateServo` | Servo | turret ball gate |
| `pinpoint` | goBILDA Pinpoint Odometry Computer (I2C) | odometry |
| `imu` | REV internal IMU | field-centric drive. Control Hub mounted flat, label **up**, USB ports toward the **back**. If you remount it, update `Drivetrain.HUB_LOGO` / `HUB_USB` |
| `Webcam 1` | Webcam | global-shutter USB camera |

## 2. Bring-up order (first time on the robot)

Look for `TODO(tune)` / `TODO(measure)` / `TODO(verify)` in the code. Every one of these is a placeholder.

1. **Drive motor wiring**: put the robot on blocks and run `Tune: Drive Motors`.
   - Hold L1 / R1 / L2 / R2 to spin frontLeft / frontRight / backLeft / backRight. CROSS spins all four.
   - Each button must spin the wheel it names, in the forward direction.
   - **Wrong wheel:** swap those two names in the Driver Station configuration, or swap the cables on the hub.
   - **Right wheel, spinning backwards:** flip that wheel's `*_DIR` setting in `Drivetrain`. Pedro uses the same values. You can try it live on FTC Dashboard (Drivetrain card), then re-init the OpMode.
   - Current config: port 0 `frontRight`, 1 `frontLeft`, 2 `backRight`, 3 `backLeft`.
   - **Stick test** (on the floor, no buttons held). The sticks drive robot-centric at half power:
     - Left stick right must strafe right without turning.
     - Right stick right must turn clockwise in place.
     - If either is wrong, the mecanum wheels are on the wrong corners. Seen from above, the rollers must form an **X**.
     - TRIANGLE switches to field-centric. If only field-centric is wrong, check the IMU: turning the robot counter-clockwise must make `IMU yaw` go **up**.
2. **Servo positions**: run `Tune: Servo Positions`. Align both arm servos first, then set `Arm.STOWED/DEPLOYED`. The gate's home is servo position 0 (`Gate.CLOSED`). It opens `Gate.OPEN_DEG` (30°) to the left, worked out from `Gate.SERVO_RANGE_DEG` (300° for a goBILDA 2000-series servo).
3. **Turret: wiring safety first.** Run `Tune: Turret` with the turret facing forward at INIT.
   - In CALIBRATE mode (motor off), turn the turret by hand:
     - CCW seen from above must read **positive**. If it doesn't, flip `Turret.MOTOR_REVERSED`.
     - Turning it 90° by hand must read about 90. If it doesn't, fix `MOTOR_REVS_PER_TURRET_REV`, the motor-to-turret gearing.
   - Turn it each way until the wires are snug. Set `MIN_DEG` / `MAX_DEG` a few degrees inside those points.
   - **Validate the limits** with `Tune: Turret Limits`. Start it with the turret facing forward.
     - Hold **L1** to turn slowly to `MIN_DEG`, **R1** to turn slowly to `MAX_DEG`, and **CROSS** to come back to 0.
     - Release any button and the turret holds where it is.
     - At each limit, the wires must still have slack. If they don't, tighten the limit on the Dashboard "Turret" card. It takes effect immediately.
     - If the turret turns the wrong way or jams, the test cuts the motor.
   - STEP mode: tune `kP`, then `kD`, then `kS`. Graph `turret/targetDeg` against `turret/angleDeg`.
   - SWEEP mode: the turret must stop at a limit and then **unwind the long way**. It must never cross the dead zone.
4. **Flywheel**:
   1. Run `Tune: Flywheel FF Characterize` and copy the resulting `kV` and `kS` into `Shooter`.
   2. Run `Tune: Flywheel`.
   3. On FTC Dashboard, graph `shooter/targetRpm`, `shooter/rpm` and `shooter/power`.
   4. Use X for step mode. Raise `kP` until the wheel recovers quickly after a shot (hold RB to feed) without oscillating.
   5. Add a small `kI` only if a steady error remains.
5. **Pedro Pathing**:
   1. Open the AutoTune web UI at `http://192.168.43.1:10158`.
   2. Run, in order: Mecanum Tuner, Pinpoint Tuner, Foresight Tuner.
   3. Paste each generated config into `pedro/Constants.java`.
   4. Verify with Tests (Line, Curve, Hold).
6. **Vision + aim**: run `Tune: Vision + Auto-Aim`.
   - Measure `TurretAimer.TURRET_X/Y_IN` and `CAM_FORWARD/LEFT_IN`. Set `CAMERA_ON_TURRET`.
   - Calibrate the camera and fill in `GoalVision.FX/FY/CX/CY`. Without this, distances will be wrong.
   - Set `EXPOSURE_MS` / `GAIN` for the venue.
   - Press A to enable the turret, then push and spin the robot: the turret should stay locked.
7. **Shot table**:
   1. Shoot from several distances. Use `aim/distanceIn` from telemetry.
   2. Record the best RPM at each distance.
   3. Fill in `Shooter.SHOT_DISTANCE_IN` / `SHOT_RPM`.
8. **Auto poses**: measure the RED poses in `BioBuzzAuto`. They're relative to the start pose. BLUE mirrors them automatically.

FTC Dashboard is at `http://192.168.43.1:8080/dash`. All `@Config` values can be edited live there. **Live edits are lost on restart, so copy final values into the code.**

## 3. Controls (`BIOBUZZ TeleOp - RED/BLUE`)

Both the driver and the operator use **Sony PS5 (DualSense) controllers**. The code uses the SDK's PlayStation button names (`cross`, `circle`, `square`, `triangle`, `share`, `options`). The PS button isn't used.

Pairing on the Driver Station: hold **OPTIONS + CROSS** to make a controller gamepad 1, or **OPTIONS + CIRCLE** to make it gamepad 2.

**Gamepad 1 (driver)**

| Control | Action |
|---|---|
| Left stick / right stick X | field-centric drive (same controls as `program_drivetrain`) |
| OPTIONS (hold 0.75 s) | reset field-forward heading |
| R2 / L2 | intake / outtake (reverse intake). Outtake overrides shooting so it can clear a jam |
| R1 (hold) | **shoot**: releases balls only when the flywheel is at speed AND the turret is on target |

**Gamepad 2 (operator)**

| Control | Action |
|---|---|
| CROSS | flywheel on/off (RPM is set automatically from distance) |
| TRIANGLE | auto-aim on/off |
| SQUARE | arm deploy/stow |
| CIRCLE (hold) | force feed (skips the ready checks) |
| L1 (hold) | outtake (reverse intake). Overrides any feed |
| Right stick X | aim trim (auto-aim on) / manual turret (auto-aim off) |
| D-pad up/down | RPM trim ±50 |
| D-pad left | clear trims |
| SHARE (hold 1 s) | re-zero turret. **Only do this with the turret physically facing forward.** |

**Controller feedback (both controllers)**

| Lightbar | Meaning |
|---|---|
| Alliance colour (red/blue) | flywheel off |
| Amber | flywheel spinning up, or turret still aiming |
| Green | **ready**: holding R1 will release a ball now |
| Magenta | turret parked at its wiring limit. Turn the robot toward the goal. |

Both controllers give one rumble blip when the robot becomes ready to shoot.

**Tuning OpModes** use gamepad 1 with the same PS names. Each one's controls are shown in its telemetry.

## 3b. Simple TeleOp (`Simple TeleOp`)

One controller (gamepad 1). There's no turret, vision or auto-aim, and the shooter runs at a fixed power. Use it to test the drive, intake, flywheel and gate on their own.

| Control | Action |
|---|---|
| Left stick | drive (robot-centric: forward is always the robot's front) |
| Right stick X | rotate |
| L2 (hold) | intake |
| L1 (hold) | outtake (wins over intake) |
| R2 (hold) | shooter motor at 35% (`SimpleTeleOp.SHOOTER_POWER`) |
| D-pad up (hold) | open the shooter gate 30° left. Releasing it sends the gate back home. |

The gate homes to 0 at INIT.

## 4. How aiming works

- SDK 12 groups the BIOBUZZ scoring tags into **clusters**. A cluster's origin is the centre of the Cell opening.
- `GoalVision` follows only our alliance's cluster: `RED SCORING` or `BLUE SCORING`.
- `TurretAimer` turns each sighting into a field position of the goal. It uses the Pinpoint pose and turret angle *from when the frame was captured*, which compensates for camera latency.
- Every loop it re-aims from the current Pinpoint pose. This keeps the turret locked while the robot drives or spins, and for `GOAL_MEMORY_SEC` after the tags go out of view.
- The SDK notes say BIOBUZZ tags move, so vision is **not** used for robot localization.

## 5. Turret wiring safety

- Travel is limited to `[MIN_DEG, MAX_DEG]`. A request for any other angle resolves to an equivalent angle inside that range.
- If the goal is in the dead zone, the turret parks at the nearer limit and telemetry shows a warning.
- Power ramps down near each limit, and power that would push further past a limit is cut.
- The turret zero carries over from Auto to TeleOp through `PoseStorage.turretZeroed`. If the Robot Controller app restarts, TeleOp re-zeros at INIT, so **start TeleOp with the turret facing forward**.
- Unit tests: `./gradlew :TeamCode:testDebugUnitTest` (`TurretSafetyTest`).

## 6. Legacy code

- `ManualDrive` (`teleop/MainOpMode`) is kept unchanged as a drivetrain-only fallback.
- The DECODE `AutoRoutine1-4` are `@Disabled` because the hardware they use is gone.
- `BroncoBoTAprilTagService` was patched for the SDK 12 detection API.
