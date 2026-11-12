package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * goBILDA turret driven by a goBILDA 5203 117 RPM motor (50.9:1) with its encoder for feedback.
 *
 * ANGLE CONVENTION: degrees in the ROBOT frame, 0 = shooter facing robot-forward,
 * positive = counter-clockwise seen from above (same sign as AprilTag "bearing").
 *
 * WIRING SAFETY (the turret can never spin freely - wires run through it):
 *   - Travel is limited to [MIN_DEG, MAX_DEG]. The gap between them (around +/-180 by default) is the
 *     wiring dead zone the turret never crosses.
 *   - Any requested angle is resolved to an equivalent angle (+/- n*360) INSIDE the limits, choosing
 *     the one closest to where the turret is now. If the goal is behind the dead zone, the turret
 *     parks at the nearer limit; once the goal comes out the other side it unwinds the long way
 *     around (never through the wires). WRAP_HYSTERESIS_DEG stops it flip-flopping near the seam.
 *   - Power is ramped down inside SLOW_ZONE_DEG of each limit, and power that would push further
 *     past a limit is cut to zero (power back toward the safe range is still allowed).
 *   - The encoder is zeroed with the turret facing forward. It is NOT re-zeroed between Auto and
 *     TeleOp (see PoseStorage.turretZeroed), so limits stay valid all match.
 *
 * Tune with "Tune: Turret" + FTC Dashboard ("Turret" config card).
 */
@Config
public class Turret {
    // ---------- Wiring-safe travel ----------
    public static double MIN_DEG = -170;            // TODO(verify on robot with wires attached)
    public static double MAX_DEG = 170;             // TODO(verify on robot with wires attached)
    public static double SLOW_ZONE_DEG = 20;
    public static double SLOW_ZONE_MIN_POWER = 0.12;
    public static double WRAP_HYSTERESIS_DEG = 15;

    // ---------- Motor mode: goBILDA 5203 117 RPM (50.9:1) ----------
    public static double MOTOR_TICKS_PER_REV = 1425.1;
    public static double MOTOR_REVS_PER_TURRET_REV = 1.0; // TODO(measure) gearing between motor and turret ring
    public static boolean MOTOR_REVERSED = false;         // TODO(verify) + power must turn turret CCW
    public static double kP = 0.02;     // power per deg error          TODO(tune)
    public static double kD = 0.0008;   // power per deg/s of turret velocity TODO(tune)
    public static double kS = 0.04;     // static friction kick         TODO(tune)
    public static double kV = 0.0015;   // power per deg/s feedforward (counter robot rotation) TODO(tune)
    public static double MAX_POWER = 0.8;
    public static double TOLERANCE_DEG = 1.5;

    private final DcMotorEx motor;

    private double desiredDeg = 0;   // what the caller asked for (any wrap)
    private double targetDeg = 0;    // wiring-safe resolved target inside limits
    private double ffDegPerSec = 0;  // feedforward velocity (robot-frame)
    private double angleDeg = 0;
    private double velocityDegPerSec = 0;
    private double lastPower = 0;
    private boolean clamped = false;
    private boolean enabled = true;

    /**
     * @param resetEncoder true to define the CURRENT physical position as 0 deg (turret must be
     *                     facing forward). Pass false to keep the zero from a previous OpMode.
     */
    public Turret(HardwareMap hw, boolean resetEncoder) {
        motor = hw.get(DcMotorEx.class, RobotConfig.TURRET_MOTOR);
        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (resetEncoder) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleDeg = readMotorAngle();
        targetDeg = MathUtil.clamp(angleDeg, MIN_DEG, MAX_DEG);
        desiredDeg = targetDeg;
    }

    // ------------------------------------------------------------------ commands

    /** Requests a robot-frame angle (any value; it is wrapped safely). */
    public void setTargetDeg(double deg) {
        desiredDeg = deg;
        targetDeg = resolveWithHysteresis(deg);
        clamped = Math.abs(MathUtil.wrapDeg(targetDeg - deg)) > 0.5;
    }

    /** Velocity feedforward in deg/s (e.g. -robotYawRate to stay locked on a target while turning). */
    public void setFeedforwardDegPerSec(double degPerSec) {
        ffDegPerSec = degPerSec;
    }

    /** Disables output (motor floats in tuner calibration mode / coasts). */
    public void setEnabled(boolean enabled) {
        if (enabled == this.enabled) return;
        this.enabled = enabled;
        motor.setZeroPowerBehavior(enabled ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        if (!enabled) motor.setPower(0);
    }

    /** Re-defines the current physical position as 0 deg. Turret MUST be facing forward. */
    public void zeroHere() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleDeg = 0;
        targetDeg = 0;
        desiredDeg = 0;
    }

    public void update() {
        angleDeg = readMotorAngle();
        velocityDegPerSec = motor.getVelocity() / ticksPerDeg();
        double power = enabled ? computeMotorPower() : 0;
        power = applyLimitGuards(power, angleDeg);
        lastPower = power;
        motor.setPower(power);
    }

    // ------------------------------------------------------------------ control

    private double computeMotorPower() {
        double error = targetDeg - angleDeg;
        double power = kP * error - kD * velocityDegPerSec + kV * ffDegPerSec;
        if (Math.abs(error) > TOLERANCE_DEG) power += Math.signum(error) * kS;
        return MathUtil.clamp(power, -MAX_POWER, MAX_POWER);
    }

    /** Never push past a limit; slow down near limits. Package-visible for unit testing. */
    static double applyLimitGuards(double power, double angle) {
        if (power > 0) {
            if (angle >= MAX_DEG) return 0;
            double room = MAX_DEG - angle;
            if (room < SLOW_ZONE_DEG) {
                double cap = Math.max(SLOW_ZONE_MIN_POWER, MAX_POWER * room / SLOW_ZONE_DEG);
                power = Math.min(power, cap);
            }
        } else if (power < 0) {
            if (angle <= MIN_DEG) return 0;
            double room = angle - MIN_DEG;
            if (room < SLOW_ZONE_DEG) {
                double cap = Math.max(SLOW_ZONE_MIN_POWER, MAX_POWER * room / SLOW_ZONE_DEG);
                power = Math.max(power, -cap);
            }
        }
        return power;
    }

    private double resolveWithHysteresis(double desired) {
        double t = resolveSafeTarget(desired, angleDeg, MIN_DEG, MAX_DEG);
        // Avoid a near-full-rotation swing when the goal is hovering around the wiring seam.
        if (Math.abs(t - angleDeg) > 180) {
            double nearLimit = angleDeg >= 0 ? MAX_DEG : MIN_DEG;
            if (Math.abs(MathUtil.wrapDeg(desired - nearLimit)) < WRAP_HYSTERESIS_DEG) return nearLimit;
        }
        return t;
    }

    /**
     * Picks the equivalent of {@code desired} (mod 360) inside [min, max] closest to {@code current}.
     * If no equivalent is inside the limits (goal is in the wiring dead zone), returns the limit
     * angularly closest to the desired direction.
     */
    public static double resolveSafeTarget(double desired, double current, double min, double max) {
        double base = MathUtil.wrapDeg(desired);
        double best = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;
        for (int k = -2; k <= 2; k++) {
            double c = base + 360.0 * k;
            if (c >= min && c <= max && Math.abs(c - current) < bestDist) {
                best = c;
                bestDist = Math.abs(c - current);
            }
        }
        if (!Double.isNaN(best)) return best;
        double toMin = Math.abs(MathUtil.wrapDeg(desired - min));
        double toMax = Math.abs(MathUtil.wrapDeg(desired - max));
        return toMin < toMax ? min : max;
    }

    private double ticksPerDeg() {
        return MOTOR_TICKS_PER_REV * MOTOR_REVS_PER_TURRET_REV / 360.0;
    }

    private double readMotorAngle() {
        return motor.getCurrentPosition() / ticksPerDeg();
    }

    // ------------------------------------------------------------------ getters

    public double getAngleDeg() { return angleDeg; }
    public double getTargetDeg() { return targetDeg; }
    public double getDesiredDeg() { return desiredDeg; }
    public double getVelocityDegPerSec() { return velocityDegPerSec; }
    /** True when the requested angle is in the wiring dead zone and we are parked at a limit. */
    public boolean isClamped() { return clamped; }
    public boolean atTarget() { return Math.abs(targetDeg - angleDeg) <= TOLERANCE_DEG * 2; }
    public int getRawTicks() { return motor.getCurrentPosition(); }

    public void addTelemetry(Telemetry t) {
        t.addData("turret/angleDeg", angleDeg);
        t.addData("turret/targetDeg", targetDeg);
        t.addData("turret/desiredDeg", desiredDeg);
        t.addData("turret/velDegPerSec", velocityDegPerSec);
        t.addData("turret/power", lastPower);
        t.addData("turret/atLimit", clamped);
    }
}
