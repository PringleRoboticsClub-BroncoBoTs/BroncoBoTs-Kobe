package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageMonitor;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.vision.GoalVision;
import org.firstinspires.ftc.teamcode.vision.TurretAimer;

import java.util.List;

/**
 * All scoring mechanisms + vision aiming. Drive/localization stay in the OpMode
 * (field-centric Drivetrain in TeleOp, Pedro Follower in Auto); the OpMode passes the current
 * Pinpoint pose into {@link #update(Pose)}.
 *
 * Loop order: clearBulkCache() -> update localizer -> set requests -> update(pose).
 */
@Config
public class Robot {
    /** Shots only release when the turret is within this many degrees of the aim angle. */
    public static double AIM_TOLERANCE_DEG = 3.0;

    public final List<LynxModule> hubs;
    public final VoltageMonitor voltage;
    public final Shooter shooter;
    public final Turret turret;
    public final Intake intake;
    public final Arm arm;
    public final Gate gate;
    public final GoalVision vision;
    public final TurretAimer aimer = new TurretAimer();

    // ---------------- requests (set by TeleOp / Auto each loop) ----------------
    /** Lock the turret onto the goal using vision + odometry. */
    public boolean autoAim = true;
    /** Robot-frame turret angle used when autoAim is off (or before the goal was ever seen). */
    public double manualTurretDeg = 0;
    /** Small driver trim added on top of auto-aim (deg). */
    public double aimTrimDeg = 0;
    /** Spin the flywheel. RPM comes from the distance table unless fixedRpm is set (> 0). */
    public boolean shooterOn = false;
    public double fixedRpm = 0;
    /** Release balls as soon as the flywheel is ready and the turret is on target. */
    public boolean shootRequested = false;
    /** Release balls regardless of readiness (driver override). */
    public boolean forceFeed = false;
    /** Intake state when not feeding. */
    public Intake.State intakeRequest = Intake.State.OFF;

    private boolean feeding = false;

    /**
     * @param resetTurret true to zero the turret encoder now (turret must face forward).
     *                    Use {@code !PoseStorage.turretZeroed} in TeleOp.
     */
    public Robot(HardwareMap hw, Alliance alliance, boolean resetTurret, boolean useVision) {
        hubs = hw.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        voltage = new VoltageMonitor(hw);
        shooter = new Shooter(hw, voltage);
        turret = new Turret(hw, resetTurret);
        if (resetTurret) PoseStorage.turretZeroed = true;
        intake = new Intake(hw);
        arm = new Arm(hw);
        gate = new Gate(hw);
        vision = useVision ? new GoalVision(hw, alliance) : null;
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    public void setAlliance(Alliance a) {
        if (vision != null) vision.setAlliance(a);
        aimer.forget();
    }

    public void update(Pose robotPose) {
        // ---- aim ----
        GoalVision.Observation obs = vision != null ? vision.update() : null;
        aimer.update(robotPose, turret.getAngleDeg(), obs);

        if (autoAim && aimer.hasTarget()) {
            turret.setTargetDeg(aimer.getDesiredTurretDeg() + aimTrimDeg);
            turret.setFeedforwardDegPerSec(aimer.getFeedforwardDegPerSec());
        } else if (!autoAim) {
            turret.setTargetDeg(manualTurretDeg);
            turret.setFeedforwardDegPerSec(0);
        } else {
            // Auto-aim on but goal unknown: hold the current target.
            turret.setFeedforwardDegPerSec(0);
        }

        // ---- flywheel ----
        if (shooterOn) {
            if (fixedRpm > 0) shooter.setTargetRpm(fixedRpm);
            else shooter.setTargetFromDistance(aimer.getDistanceIn());
        } else {
            shooter.stop();
        }

        // ---- feed / gate ----
        boolean aimed = !autoAim || (aimer.hasTarget()
                && Math.abs(turret.getTargetDeg() - turret.getAngleDeg()) <= AIM_TOLERANCE_DEG
                && !turret.isClamped());
        feeding = forceFeed || (shootRequested && shooterOn && shooter.isReady() && aimed);
        gate.set(feeding);
        intake.set(feeding ? Intake.State.FEED : intakeRequest);

        shooter.update();
        turret.update();
        intake.update();
        arm.update();
        gate.update();
    }

    public boolean isFeeding() { return feeding; }

    public void stop() {
        shooterOn = false;
        shooter.stop();
        shooter.update();
        intake.set(Intake.State.OFF);
        intake.update();
        turret.setEnabled(false);
        if (vision != null) vision.close();
    }

    public void addTelemetry(Telemetry t) {
        t.addData("robot/feeding", feeding);
        t.addData("robot/voltage", voltage.getVoltage());
        shooter.addTelemetry(t);
        turret.addTelemetry(t);
        aimer.addTelemetry(t);
        if (vision != null) vision.addTelemetry(t);
        intake.addTelemetry(t);
        arm.addTelemetry(t);
        gate.addTelemetry(t);
    }
}
