package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.util.InterpLUT;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Turret flywheel: goBILDA 5203 6000 RPM (1:1, 28 CPR) -> 2 mm GT2 belt -> 96 mm wheel + added mass.
 *
 * Control law (all "power" values are normalized to 12 V, then voltage-compensated):
 *   power = kV * targetRPM + kS + kP * error + kI * integral(error)
 *
 * Why custom instead of DcMotorEx.setVelocity(): we get voltage compensation, an integral that only
 * acts near the setpoint (no wind-up during spin-up), and every term is visible on FTC Dashboard.
 *
 * Tune with the "Tune: Flywheel" OpMode + FTC Dashboard (http://192.168.43.1:8080/dash):
 *   1. Run "Tune: Flywheel FF Characterize" to get kV / kS.
 *   2. In "Tune: Flywheel", raise kP until recovery after a shot is fast without oscillation.
 *   3. Add a little kI only if a steady-state error remains.
 * All fields below are live-editable on the Dashboard "Shooter" config card.
 */
@Config
public class Shooter {
    // ---------- Hardware geometry ----------
    public static double MOTOR_TICKS_PER_REV = 28.0;     // 5203 6000 RPM bare motor encoder
    public static double WHEEL_PER_MOTOR = 1.0;          // belt ratio (wheel revs per motor rev); TODO(measure)
    public static boolean REVERSED = false;              // TODO(verify) flip if wheel spins backwards

    // ---------- Controller (power per wheel RPM, at 12 V) ----------
    public static double kV = 1.0 / 6000.0;  // TODO(tune) from FF characterization
    public static double kS = 0.03;          // TODO(tune)
    public static double kP = 0.0006;        // TODO(tune)
    public static double kI = 0.0;
    public static double I_ZONE_RPM = 250;   // integral only accumulates inside this error band
    public static double I_MAX = 0.15;       // max integral contribution (power)
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double VELOCITY_FILTER = 0.6; // 0 = no filtering, 0.9 = heavy filtering

    // ---------- Readiness ----------
    public static double READY_TOLERANCE_RPM = 80;
    public static double READY_HOLD_SEC = 0.08;

    // ---------- Distance -> RPM (range in inches from turret pivot to Cell opening) ----------
    // TODO(tune): shoot from several distances, record the best RPM, update these two arrays.
    public static double[] SHOT_DISTANCE_IN = {24, 36, 48, 60, 72, 96, 120};
    public static double[] SHOT_RPM         = {2600, 2800, 3000, 3250, 3500, 3900, 4300};
    public static double RPM_OFFSET = 0;     // driver trim, also editable from gamepad
    public static double DEFAULT_RPM = 3200; // used when we have no distance estimate

    private final DcMotorEx motor;
    private final VoltageMonitor voltage;

    private double targetRpm = 0;
    private double filteredRpm = 0;
    private double integral = 0;
    private double lastPower = 0;
    private double lastError = 0;
    private long lastNanos = 0;
    private double inToleranceSince = -1;

    public Shooter(HardwareMap hw, VoltageMonitor voltage) {
        this.voltage = voltage;
        motor = hw.get(DcMotorEx.class, RobotConfig.SHOOTER);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we close the loop ourselves
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let the flywheel coast
    }

    /** Target wheel speed in RPM. 0 = coast to stop. */
    public void setTargetRpm(double rpm) {
        if (rpm != targetRpm) inToleranceSince = -1;
        targetRpm = Math.max(0, rpm);
    }

    /** Target from distance to the goal (inches), using the shot table + driver trim. */
    public void setTargetFromDistance(double distanceIn) {
        setTargetRpm(rpmForDistance(distanceIn));
    }

    public static double rpmForDistance(double distanceIn) {
        double base = Double.isNaN(distanceIn)
                ? DEFAULT_RPM
                : new InterpLUT(SHOT_DISTANCE_IN, SHOT_RPM).get(distanceIn);
        return base + RPM_OFFSET;
    }

    public void stop() {
        setTargetRpm(0);
    }

    /** Call once per loop (after bulk cache clear). */
    public void update() {
        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0.02 : (now - lastNanos) / 1e9;
        lastNanos = now;

        double rawRpm = motor.getVelocity() / MOTOR_TICKS_PER_REV * 60.0 * WHEEL_PER_MOTOR;
        filteredRpm = VELOCITY_FILTER * filteredRpm + (1.0 - VELOCITY_FILTER) * rawRpm;

        if (targetRpm <= 0) {
            integral = 0;
            lastError = 0;
            lastPower = 0;
            motor.setPower(0);
            return;
        }

        double error = targetRpm - filteredRpm;
        lastError = error;
        if (Math.abs(error) < I_ZONE_RPM) {
            integral += error * dt;
            if (kI != 0) integral = MathUtil.clamp(integral, -I_MAX / Math.abs(kI), I_MAX / Math.abs(kI));
        } else {
            integral = 0;
        }

        double power12v = kV * targetRpm + kS + kP * error + kI * integral;
        double power = power12v * NOMINAL_VOLTAGE / voltage.getVoltage();
        lastPower = MathUtil.clamp(power, 0.0, 1.0); // never drive the flywheel backwards
        motor.setPower(lastPower);

        double t = now / 1e9;
        if (Math.abs(error) <= READY_TOLERANCE_RPM) {
            if (inToleranceSince < 0) inToleranceSince = t;
        } else {
            inToleranceSince = -1;
        }
    }

    /** True when the wheel has been inside the tolerance band for READY_HOLD_SEC. */
    public boolean isReady() {
        return targetRpm > 0 && inToleranceSince >= 0
                && (System.nanoTime() / 1e9 - inToleranceSince) >= READY_HOLD_SEC;
    }

    public double getTargetRpm() { return targetRpm; }
    public double getRpm() { return filteredRpm; }
    public double getPower() { return lastPower; }

    /** Raw open-loop power, used by the feedforward characterization OpMode. */
    public void setRawPower(double power) {
        targetRpm = 0;
        motor.setPower(power);
        lastPower = power;
    }

    /** Updates the velocity estimate without running the controller (for characterization). */
    public double measureRpm() {
        double rawRpm = motor.getVelocity() / MOTOR_TICKS_PER_REV * 60.0 * WHEEL_PER_MOTOR;
        filteredRpm = VELOCITY_FILTER * filteredRpm + (1.0 - VELOCITY_FILTER) * rawRpm;
        return filteredRpm;
    }

    /** Numeric keys are graphable on FTC Dashboard. */
    public void addTelemetry(Telemetry t) {
        t.addData("shooter/targetRpm", targetRpm);
        t.addData("shooter/rpm", filteredRpm);
        t.addData("shooter/error", lastError);
        t.addData("shooter/power", lastPower);
        t.addData("shooter/ready", isReady());
    }
}
