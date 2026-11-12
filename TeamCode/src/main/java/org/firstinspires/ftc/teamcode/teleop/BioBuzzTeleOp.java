package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

/**
 * BIOBUZZ driver-controlled period. Use {@link RedTeleOp} / {@link BlueTeleOp}.
 *
 * Both drivers use Sony PS5 (DualSense) controllers, so buttons use the SDK's PlayStation names
 * (cross/circle/square/triangle/share/options). The PS button is not used.
 *
 * GAMEPAD 1 (driver)
 *   left stick           field-centric translate       right stick X   rotate
 *   OPTIONS (hold 0.75s) reset field-forward heading
 *   (driving is on gamepad 1 ONLY)
 *
 * EITHER gamepad - intake + shoot:
 *   L2 (hold)            intake                        L1 (hold)       outtake (reverse intake)
 *   R1 (hold)            SHOOT - releases balls when flywheel is at speed and turret is on target
 *
 * GAMEPAD 2 (operator)
 *   CROSS                flywheel on/off               TRIANGLE        auto-aim on/off
 *   SQUARE               arm deploy/stow               CIRCLE (hold)   force feed (ignores readiness)
 *   right stick X        auto-aim ON: trim aim  /  auto-aim OFF: rotate turret manually
 *   dpad up/down         flywheel RPM trim +/-50       dpad left       clear aim + RPM trim
 *   (outtake also overrides any feed, to clear jams)
 *   SHARE (hold 1s)      re-zero turret  (ONLY with the turret physically facing forward!)
 *
 * CONTROLLER FEEDBACK (both controllers)
 *   lightbar: alliance colour = flywheel off, amber = spinning up / aiming,
 *             green = READY (a shot will release), magenta = turret parked at its wiring limit
 *   rumble:   one blip when the robot becomes ready to shoot
 */
@Config
public abstract class BioBuzzTeleOp extends OpMode {
    public static double MANUAL_TURRET_DEG_PER_SEC = 120;
    public static double TRIM_DEG_PER_SEC = 10;
    public static double RPM_TRIM_STEP = 50;
    public static double LONG_PRESS_SEC = 0.75;
    public static double ZERO_TURRET_HOLD_SEC = 1.0;

    protected abstract Alliance alliance();

    private Robot robot;
    private Drivetrain drive;
    private PinpointLocalizer localizer;
    private Telemetry tel;

    private double optionsPressedAt = -1;
    private boolean yawResetLatched = false;
    private double backPressedAt = -1;
    private boolean turretZeroLatched = false;
    private double lastTime = 0;
    private Feedback lastFeedback = null;
    private boolean wasReady = false;

    private enum Feedback { IDLE, BUSY, READY, AT_LIMIT }

    @Override
    public void init() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PoseStorage.alliance = alliance();

        // Keep the turret zero from Auto if we have one; otherwise the turret must face forward NOW.
        boolean resetTurret = !PoseStorage.turretZeroed;
        robot = new Robot(hardwareMap, alliance(), resetTurret, true);
        drive = new Drivetrain(hardwareMap);
        localizer = Constants.createLocalizer(hardwareMap);
        // Field-centric "forward" is still defined at init / OPTIONS long-press exactly as in
        // program_drivetrain. The Pinpoint pose only feeds turret aiming.
        if (PoseStorage.pose != null) {
            localizer.setPose(PoseStorage.pose);
        } else {
            localizer.setPose(new Pose(0, 0, 0));
        }

        updateControllerFeedback(); // lightbar shows alliance colour from INIT
        tel.addData("Alliance", alliance());
        tel.addData("Turret", resetTurret ? "ZEROED NOW - must be facing forward" : "using zero from Auto");
        tel.update();
    }

    @Override
    public void init_loop() {
        robot.clearBulkCache();
        localizer.update();
        robot.vision.update();
        tel.addData("Alliance", alliance());
        tel.addData("Camera sees our goal", robot.vision.getLatest() != null);
        tel.update();
    }

    @Override
    public void start() {
        lastTime = getRuntime();
        robot.shooterOn = false;
        robot.autoAim = true;
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = Math.max(1e-3, now - lastTime);
        lastTime = now;

        robot.clearBulkCache();
        localizer.update();
        Pose pose = localizer.pose();

        // ---------------- driver ----------------
        handleYawReset(now);
        drive.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        // ---------------- either gamepad: intake + shoot ----------------
        // Outtake (L1) takes priority over intake and over feeding.
        if (gamepad1.left_bumper || gamepad2.left_bumper) robot.intakeRequest = Intake.State.OUTTAKE;
        else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) robot.intakeRequest = Intake.State.INTAKE;
        else robot.intakeRequest = Intake.State.OFF;
        robot.shootRequested = gamepad1.right_bumper || gamepad2.right_bumper;

        // ---------------- operator ----------------
        if (gamepad2.crossWasPressed()) robot.shooterOn = !robot.shooterOn;
        if (gamepad2.triangleWasPressed()) {
            robot.autoAim = !robot.autoAim;
            robot.manualTurretDeg = robot.turret.getAngleDeg(); // no jump when switching to manual
        }
        if (gamepad2.squareWasPressed()) robot.arm.toggle();
        robot.forceFeed = gamepad2.circle;

        double stick = deadband(gamepad2.right_stick_x);
        if (robot.autoAim) {
            robot.aimTrimDeg = MathUtil.clamp(robot.aimTrimDeg - stick * TRIM_DEG_PER_SEC * dt, -20, 20);
        } else {
            // Manual target is clamped to the wiring-safe range so it can't wind up past a limit.
            robot.manualTurretDeg = MathUtil.clamp(
                    robot.manualTurretDeg - stick * MANUAL_TURRET_DEG_PER_SEC * dt, Turret.MIN_DEG, Turret.MAX_DEG);
        }

        if (gamepad2.dpadUpWasPressed()) Shooter.RPM_OFFSET += RPM_TRIM_STEP;
        if (gamepad2.dpadDownWasPressed()) Shooter.RPM_OFFSET -= RPM_TRIM_STEP;
        if (gamepad2.dpadLeftWasPressed()) {
            Shooter.RPM_OFFSET = 0;
            robot.aimTrimDeg = 0;
        }
        handleTurretZero(now);

        robot.update(pose);
        PoseStorage.pose = pose;
        updateControllerFeedback();

        // ---------------- telemetry ----------------
        tel.addData("SHOOTER", robot.shooterOn ? (robot.shooter.isReady() ? "READY" : "spinning up") : "off");
        tel.addData("AIM", robot.autoAim
                ? (robot.aimer.hasTarget() ? "LOCKED" : "searching (point camera at goal)")
                : "MANUAL");
        if (robot.turret.isClamped()) tel.addLine(">> TURRET AT WIRING LIMIT - rotate the robot toward the goal <<");
        tel.addData("RPM trim / aim trim", "%.0f / %.1f deg", Shooter.RPM_OFFSET, robot.aimTrimDeg);
        tel.addData("pose", "x %.1f  y %.1f  h %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        drive.addTelemetry(tel);
        robot.addTelemetry(tel);
        tel.update();
    }

    @Override
    public void stop() {
        drive.stop();
        robot.stop();
    }

    private void handleYawReset(double now) {
        if (gamepad1.options) {
            if (optionsPressedAt < 0) optionsPressedAt = now;
            if (!yawResetLatched && now - optionsPressedAt > LONG_PRESS_SEC) {
                drive.resetFieldYaw();
                gamepad1.rumble(200);
                yawResetLatched = true;
            }
        } else {
            optionsPressedAt = -1;
            yawResetLatched = false;
        }
    }

    private void handleTurretZero(double now) {
        if (gamepad2.share) {
            if (backPressedAt < 0) backPressedAt = now;
            if (!turretZeroLatched && now - backPressedAt > ZERO_TURRET_HOLD_SEC) {
                robot.turret.zeroHere();
                robot.manualTurretDeg = 0;
                PoseStorage.turretZeroed = true;
                gamepad2.rumble(300);
                turretZeroLatched = true;
            }
        } else {
            backPressedAt = -1;
            turretZeroLatched = false;
        }
    }

    /** DualSense lightbar + rumble. LED commands are only sent when the state changes. */
    private void updateControllerFeedback() {
        boolean ready = robot.isReadyToShoot();
        Feedback f;
        if (robot.turret.isClamped()) f = Feedback.AT_LIMIT;
        else if (!robot.shooterOn) f = Feedback.IDLE;
        else if (ready) f = Feedback.READY;
        else f = Feedback.BUSY;

        if (f != lastFeedback) {
            double r, g, b;
            switch (f) {
                case READY:    r = 0; g = 1;   b = 0; break;
                case BUSY:     r = 1; g = 0.4; b = 0; break;
                case AT_LIMIT: r = 1; g = 0;   b = 1; break;
                default:
                    boolean red = alliance() == Alliance.RED;
                    r = red ? 1 : 0; g = 0; b = red ? 0 : 1;
            }
            gamepad1.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
            lastFeedback = f;
        }
        if (ready && !wasReady) {
            gamepad1.rumbleBlips(1);
            gamepad2.rumbleBlips(1);
        }
        wasReady = ready;
    }

    private static double deadband(double v) {
        return Math.abs(v) < 0.08 ? 0 : v;
    }
}
