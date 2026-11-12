package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.VoltageMonitor;

import java.util.List;

/**
 * Measures flywheel kV and kS automatically.
 *
 * Runs the flywheel open-loop at several power levels, records the steady-state RPM at each,
 * normalizes power to 12 V, and fits   power12V = kS + kV * rpm   with least squares.
 * Copy the printed kV / kS into Shooter (or onto the Dashboard "Shooter" card).
 *
 * Keep hands clear: the flywheel will run up to full speed.
 */
@Config
@TeleOp(name = "Tune: Flywheel FF Characterize", group = "Tuning")
public class FlywheelFeedforwardTuner extends LinearOpMode {
    public static double MIN_POWER = 0.2;
    public static double MAX_POWER = 1.0;
    public static double POWER_STEP = 0.1;
    public static double SETTLE_SEC = 2.0;   // flywheel has a lot of inertia - let it settle
    public static double SAMPLE_SEC = 0.75;

    @Override
    public void runOpMode() {
        Telemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        VoltageMonitor voltage = new VoltageMonitor(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, voltage);

        tel.addLine("Press START to characterize. Flywheel will spin to full speed.");
        tel.update();
        waitForStart();

        int n = 0;
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        StringBuilder log = new StringBuilder();
        ElapsedTime timer = new ElapsedTime();

        for (double p = MIN_POWER; p <= MAX_POWER + 1e-6 && opModeIsActive(); p += POWER_STEP) {
            timer.reset();
            double rpmSum = 0, voltSum = 0;
            int samples = 0;
            while (opModeIsActive() && timer.seconds() < SETTLE_SEC + SAMPLE_SEC) {
                for (LynxModule h : hubs) h.clearBulkCache();
                shooter.setRawPower(p);
                double rpm = shooter.measureRpm();
                if (timer.seconds() > SETTLE_SEC) {
                    rpmSum += rpm;
                    voltSum += voltage.getVoltage();
                    samples++;
                }
                tel.addData("power", p);
                tel.addData("shooter/rpm", rpm);
                tel.update();
            }
            if (samples == 0) break;
            double rpm = rpmSum / samples;
            double power12 = p * (voltSum / samples) / Shooter.NOMINAL_VOLTAGE;
            log.append(String.format("p=%.2f  p@12V=%.3f  rpm=%.0f%n", p, power12, rpm));
            n++;
            sx += rpm;
            sy += power12;
            sxx += rpm * rpm;
            sxy += rpm * power12;
        }
        shooter.setRawPower(0);

        double denom = n * sxx - sx * sx;
        double kV = denom != 0 ? (n * sxy - sx * sy) / denom : Double.NaN;
        double kS = n > 0 ? (sy - kV * sx) / n : Double.NaN;

        while (opModeIsActive()) {
            tel.addLine(log.toString());
            tel.addData("RESULT kV", "%.8f", kV);
            tel.addData("RESULT kS", "%.4f", kS);
            tel.addLine("Copy these into Shooter.kV / Shooter.kS");
            tel.update();
            sleep(100);
        }
    }
}
