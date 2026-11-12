package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;

/**
 * Validates the turret's wiring-safe travel limits, slowly.
 * START WITH THE TURRET FACING ROBOT-FORWARD (it is zeroed at INIT).
 *
 * gamepad1 (PS5), all HOLD-to-move (release = turret holds where it is):
 *   L1     turn slowly toward Turret.MIN_DEG and stop there
 *   R1     turn slowly toward Turret.MAX_DEG and stop there
 *   CROSS  turn slowly back to 0 (forward)
 *
 * At each limit, check the wires still have slack. If not, lower MAX_DEG / raise MIN_DEG on the
 * FTC Dashboard "Turret" card (takes effect immediately), then copy the final values into
 * Turret.java.
 *
 * Safety: if the turret falls more than RUNAWAY_ERROR_DEG behind the slow-moving target (e.g. it is
 * turning the WRONG way because Turret.MOTOR_REVERSED is wrong, or it is jammed), the motor is
 * shut off until the OpMode is restarted. Check the direction first in "Tune: Turret" CALIBRATE.
 */
@Config
@TeleOp(name = "Tune: Turret Limits", group = "Tuning")
public class TurretLimitTest extends OpMode {
    public static double SLOW_DEG_PER_SEC = 15;
    public static double RUNAWAY_ERROR_DEG = 20;

    private List<LynxModule> hubs;
    private Turret turret;
    private Telemetry tel;
    private double target = 0;
    private double lastT = 0;
    private String fault = null;

    @Override
    public void init() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        turret = new Turret(hardwareMap, true);
        PoseStorage.turretZeroed = true;
        tel.addLine("Turret zeroed. It must be facing robot-forward RIGHT NOW.");
        tel.addLine("Hold L1 = slowly to MIN limit, R1 = slowly to MAX limit, CROSS = back to 0.");
        tel.addData("Limits", "MIN %.1f  /  MAX %.1f deg", Turret.MIN_DEG, Turret.MAX_DEG);
        tel.update();
    }

    @Override
    public void start() {
        lastT = getRuntime();
        target = turret.getAngleDeg();
    }

    @Override
    public void loop() {
        double t = getRuntime();
        double dt = Math.max(1e-3, t - lastT);
        lastT = t;
        for (LynxModule h : hubs) h.clearBulkCache();

        double step = SLOW_DEG_PER_SEC * dt;
        String action = "holding";
        if (fault == null) {
            if (gamepad1.left_bumper) {
                target = Math.max(target - step, Turret.MIN_DEG);
                action = "-> MIN";
            } else if (gamepad1.right_bumper) {
                target = Math.min(target + step, Turret.MAX_DEG);
                action = "-> MAX";
            } else if (gamepad1.cross) {
                target = target > 0 ? Math.max(target - step, 0) : Math.min(target + step, 0);
                action = "-> 0";
            }
            // Keep the target inside the limits even if they are edited live on Dashboard.
            target = MathUtil.clamp(target, Turret.MIN_DEG, Turret.MAX_DEG);
        }

        turret.setTargetDeg(target);
        turret.setFeedforwardDegPerSec(0);
        turret.update();

        double error = target - turret.getAngleDeg();
        if (fault == null && Math.abs(error) > RUNAWAY_ERROR_DEG) {
            fault = String.format("turret %.0f deg away from target - wrong direction or jammed?", error);
            turret.setEnabled(false);
        }

        boolean atMin = target <= Turret.MIN_DEG && Math.abs(error) <= Turret.TOLERANCE_DEG * 2;
        boolean atMax = target >= Turret.MAX_DEG && Math.abs(error) <= Turret.TOLERANCE_DEG * 2;

        if (fault != null) {
            tel.addLine("!! MOTOR STOPPED: " + fault);
            tel.addLine("!! Check Turret.MOTOR_REVERSED in 'Tune: Turret' CALIBRATE, then restart.");
        }
        tel.addData("Status", atMin ? "AT MIN LIMIT - check wire slack"
                : atMax ? "AT MAX LIMIT - check wire slack" : action);
        tel.addData("Limits", "MIN %.1f  /  MAX %.1f deg", Turret.MIN_DEG, Turret.MAX_DEG);
        tel.addData("Angle / target", "%.1f / %.1f deg", turret.getAngleDeg(), target);
        tel.addData("Encoder ticks", turret.getRawTicks());
        turret.addTelemetry(tel);
        tel.update();
    }

    @Override
    public void stop() {
        turret.setEnabled(false);
    }
}
