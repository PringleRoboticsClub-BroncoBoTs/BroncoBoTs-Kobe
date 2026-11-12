package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;

/**
 * Turret setup + PID tuning. START WITH THE TURRET FACING ROBOT-FORWARD (it is zeroed at init).
 *
 * Modes (gamepad1 TRIANGLE cycles):
 *   CALIBRATE  motor output OFF. Rotate the turret by hand: check that CCW (seen from above) reads
 *              POSITIVE and that 90 deg by hand reads ~90. Fix Turret.MOTOR_REVERSED /
 *              MOTOR_REVS_PER_TURRET_REV until it does. Then find the safe wiring limits and set
 *              Turret.MIN_DEG / MAX_DEG.
 *   STEP       square wave between +STEP_DEG and -STEP_DEG. Tune kP, kD, kS on the "Turret" card.
 *              Graph turret/targetDeg vs turret/angleDeg on FTC Dashboard.
 *   STICK      gamepad1 right stick direction = desired turret angle (try pointing it behind the
 *              robot to see the wiring-safe behavior).
 *   SWEEP      desired angle increases forever at SWEEP_DEG_PER_SEC. The turret must stop at the
 *              limit, then unwind the long way round - it must NEVER cross the dead zone.
 * gamepad1 (PS5): CROSS = target 0, CIRCLE = target -90, SQUARE = target +90 (STEP/STICK modes),
 *                 SHARE = re-zero here.
 */
@Config
@TeleOp(name = "Tune: Turret", group = "Tuning")
public class TurretTuner extends OpMode {
    public static double STEP_DEG = 45;
    public static double STEP_PERIOD_SEC = 2.0;
    public static double SWEEP_DEG_PER_SEC = 45;

    private enum Mode { CALIBRATE, STEP, STICK, SWEEP }

    private Mode mode = Mode.CALIBRATE;
    private List<LynxModule> hubs;
    private Turret turret;
    private Telemetry tel;
    private double sweepDeg = 0;
    private double manualTarget = Double.NaN;
    private double lastT = 0;

    @Override
    public void init() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        turret = new Turret(hardwareMap, true);
        PoseStorage.turretZeroed = true;
        tel.addLine("Turret zeroed. It must be facing robot-forward right now.");
        tel.addLine("TRIANGLE: cycle mode (starts in CALIBRATE = output off)");
        tel.update();
    }

    @Override
    public void loop() {
        double t = getRuntime();
        double dt = Math.max(1e-3, t - lastT);
        lastT = t;
        for (LynxModule h : hubs) h.clearBulkCache();

        if (gamepad1.triangleWasPressed()) {
            mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
            sweepDeg = turret.getAngleDeg();
            manualTarget = Double.NaN;
        }
        if (gamepad1.shareWasPressed()) turret.zeroHere();
        if (gamepad1.crossWasPressed()) manualTarget = 0;
        if (gamepad1.circleWasPressed()) manualTarget = -90;
        if (gamepad1.squareWasPressed()) manualTarget = 90;

        turret.setEnabled(mode != Mode.CALIBRATE);
        switch (mode) {
            case STEP:
                if (!Double.isNaN(manualTarget)) {
                    turret.setTargetDeg(manualTarget);
                } else {
                    boolean pos = ((long) (t / STEP_PERIOD_SEC)) % 2 == 0;
                    turret.setTargetDeg(pos ? STEP_DEG : -STEP_DEG);
                }
                break;
            case STICK:
                double sx = gamepad1.right_stick_x, sy = -gamepad1.right_stick_y;
                if (Math.hypot(sx, sy) > 0.6) {
                    // stick up = robot-forward (0 deg), stick left = +90 (CCW)
                    turret.setTargetDeg(Math.toDegrees(Math.atan2(-sx, sy)));
                } else if (!Double.isNaN(manualTarget)) {
                    turret.setTargetDeg(manualTarget);
                }
                break;
            case SWEEP:
                sweepDeg += SWEEP_DEG_PER_SEC * dt;
                turret.setTargetDeg(sweepDeg);
                break;
            default:
                break;
        }
        turret.update();

        tel.addData("mode", mode);
        tel.addData("rawTicks", turret.getRawTicks());
        tel.addData("limits", "[%.0f, %.0f] deg", Turret.MIN_DEG, Turret.MAX_DEG);
        turret.addTelemetry(tel);
        tel.update();
    }

    @Override
    public void stop() {
        turret.setEnabled(false);
    }
}
