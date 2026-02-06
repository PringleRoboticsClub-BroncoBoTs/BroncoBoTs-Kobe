package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Shared base class for encoder-based autonomous routines.
 * Drive motors:
 *   "frontRight", "frontLeft", "backLeft", "backRight"
 *
 * Mechanisms:
 *   "intakeMotor", "intakeRampMotor", "shooterMotor", "shooterGate"
 */
public abstract class BroncoBotAutoBase extends LinearOpMode {

    // ********** DRIVE **********
    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor backRight;

    // ********** MECHANISMS **********
    protected DcMotor     intakeMotor;      // "intakeMotor"
    protected DcMotorEx   shooterMotor;     // "shooterMotor" (now DcMotorEx for PIDF)
    protected DcMotor     intakeRampMotor;  // "intakeRampMotor"
    protected Servo       shooterGate;      // "shooterGate"
    protected Servo hoodAdjuster;     // hoodAdjuster

    // ********** DRIVE ENCODER CONSTANTS **********
    // Adjust these if your gearbox / wheels differ
    private static final double DRIVE_TICKS_PER_REV        = 537.7;  // goBilda 312RPM
    private static final double WHEEL_DIAMETER_INCHES      = 4.09;   // 104mm mecanum
    private static final double WHEEL_CIRCUMFERENCE_INCHES =
            Math.PI * WHEEL_DIAMETER_INCHES;

    // Approx ticks per inch of travel
    protected static final double TICKS_PER_INCH =
            DRIVE_TICKS_PER_REV / WHEEL_CIRCUMFERENCE_INCHES;

    // Approx robot track width (distance between left & right wheel centers)
    private static final double TURN_TRACK_WIDTH_INCHES = 15.0;

    // For an in-place turn, each side travels:
    //   arc_per_degree = PI * trackWidth / 360  (inches/deg)
    //   ticks_per_deg  = TICKS_PER_INCH * arc_per_degree
    protected static final double TICKS_PER_DEGREE =
            TICKS_PER_INCH * (Math.PI * TURN_TRACK_WIDTH_INCHES / 360.0);

    // ********** SHOOTER CONSTANTS (copied from MainOpMode) **********
    public static double TARGET_VELOCITY = 1500;  // ticks/sec

    private static final double SHOOTER_TICKS_PER_REV     = 28.0;
    @SuppressWarnings("unused")
    private static final double SHOOTER_MAX_TICKS_PER_SEC =
            (6000 / 60.0) * SHOOTER_TICKS_PER_REV;   // 2800

    public static double kP = 0.04;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 8.0;

    // ********** COMMON INIT **********

    protected void initHardware() {
        HardwareMap hw = hardwareMap;

        // --- Drive motors (same as MainOpMode) ---
        frontRight = hw.get(DcMotor.class, "frontRight");
        frontLeft  = hw.get(DcMotor.class, "frontLeft");
        backLeft   = hw.get(DcMotor.class, "backLeft");
        backRight  = hw.get(DcMotor.class, "backRight");

        // Directions copied from MainOpMode.initDrive()
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveZeroPower();

        // --- Mechanisms (same names & setup as MainOpMode) ---
        shooterMotor    = hw.get(DcMotorEx.class, "shooterMotor");
        intakeMotor     = hw.get(DcMotor.class,   "intakeMotor");
        intakeRampMotor = hw.get(DcMotor.class,   "intakeRampMotor");
        shooterGate     = hw.get(Servo.class,     "shooterGate");
        hoodAdjuster = hw.get(Servo.class,     "hoodAdjuster");
        hoodAdjuster.setDirection(Servo.Direction.REVERSE);
        hoodAdjuster.scaleRange(0, 0.40);
        hoodAdjuster.setPosition(0);

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(
            org.firstinspires.ftc.teamcode.teleop.MainOpMode.kP,
            org.firstinspires.ftc.teamcode.teleop.MainOpMode.kI,
            org.firstinspires.ftc.teamcode.teleop.MainOpMode.kD,
            org.firstinspires.ftc.teamcode.teleop.MainOpMode.kF
        );

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRampMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setDriveZeroPower() {
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ********** DRIVE HELPERS (ENCODER-BASED) **********

    protected void resetDriveEncoders() {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        frontRight.setMode(mode);
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setDrivePower(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    /**
     * Positive inches = forward, negative = backward
     */
    protected void driveStraightInches(double inches, double power) {
        int moveCounts = (int) Math.round(inches * TICKS_PER_INCH);

        resetDriveEncoders();

        frontLeft.setTargetPosition(moveCounts);
        backLeft.setTargetPosition(moveCounts);
        frontRight.setTargetPosition(moveCounts);
        backRight.setTargetPosition(moveCounts);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(Math.abs(power));

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy()
                        || backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("driveStraight", "target=%d", moveCounts);
            telemetry.addData("FL pos", frontLeft.getCurrentPosition());
            telemetry.addData("FR pos", frontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

        setDrivePower(0.0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Positive degrees = turn left (CCW), negative = turn right (CW).
     * Uses in-place tank-style turn via encoders only (no IMU).
     */
    protected void turnDegrees(double degrees, double power) {
        int turnCounts = (int) Math.round(Math.abs(degrees) * TICKS_PER_DEGREE);

        resetDriveEncoders();

        int sign = (degrees >= 0.0) ? 1 : -1;  // + left, - right

        // Left side negative, right side positive for left turn
        int flTarget = -sign * turnCounts;
        int blTarget = -sign * turnCounts;
        int frTarget =  sign * turnCounts;
        int brTarget =  sign * turnCounts;

        frontLeft.setTargetPosition(flTarget);
        backLeft.setTargetPosition(blTarget);
        frontRight.setTargetPosition(frTarget);
        backRight.setTargetPosition(brTarget);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Positive power magnitude only; direction is via target sign
        double p = Math.abs(power);
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy()
                        || backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("turnDegrees", "deg=%.1f target=%d", degrees, turnCounts);
            telemetry.addData("FL pos", frontLeft.getCurrentPosition());
            telemetry.addData("FR pos", frontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

        setDrivePower(0.0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ********** MECHANISM HELPERS **********

    protected void startShooter() {
        startShooterWithVelocity(TARGET_VELOCITY);
    }

    protected void startShooterWithVelocity(double velocity) {
        shooterMotor.setVelocity(velocity);
    }

    protected void stopShooter() {
        shooterMotor.setVelocity(0.0);
    }

    protected void setShooterVelocity(double velocity) {
        shooterMotor.setVelocity(velocity);
    }

    protected void stopShooting() {
        intakeRampMotor.setPower(0.0);
        shooterGate.setPosition(0.0);   // close gate
    }

    protected void shootForSeconds(double seconds) {
        shooterGate.setPosition(0.4);   // open gate to feed
        sleep((long) (0.3 * 1000));
        intakeMotor.setPower(0.6);  // intake into ramp
        intakeRampMotor.setPower(0.6);  // stage into flywheel
        sleep((long) (seconds * 1000));
        stopShooting();
    }

    protected void startIntake(double intakePower, double rampPower) {
        intakeMotor.setPower(intakePower);
        intakeRampMotor.setPower(rampPower);
    }

    protected void stopIntake() {
        intakeMotor.setPower(0.0);
        intakeRampMotor.setPower(0.0);
    }

    /**
     * Drive along a forward arc for a given distance (inches) while changing heading.
     *
     * - inches > 0: forward
     * - turnDegrees > 0: left turn
     * - turnDegrees < 0: right turn
     *
     * Uses differential left/right encoder targets computed from simple arc geometry:
     *   center distance = inches
     *   heading change  = turnDegrees
     *   track width     = TURN_TRACK_WIDTH_INCHES
     *
     * Left distance  = center + (theta * W / 2)
     * Right distance = center - (theta * W / 2)
     */
    protected void driveArcForwardInchesWithTurn(double inches, double turnDegrees, double power) {
        if (inches == 0.0 || turnDegrees == 0.0) {
            // fall back to straight drive if no arc
            driveStraightInches(inches, power);
            return;
        }

        double forwardSign = (inches >= 0.0) ? 1.0 : -1.0;
        double Lc = Math.abs(inches);                        // center path length (in)
        double theta = Math.toRadians(turnDegrees);          // rad
        double W = TURN_TRACK_WIDTH_INCHES;                  // track width (in)

        // Distances for each side (inches)
        double Dl = Lc + (theta * W / 2.0);  // left
        double Dr = Lc - (theta * W / 2.0);  // right

        // Convert to ticks
        int leftCounts  = (int) Math.round(Dl * TICKS_PER_INCH);
        int rightCounts = (int) Math.round(Dr * TICKS_PER_INCH);

        // If we’re going backwards, flip the signs
        leftCounts  *= forwardSign;
        rightCounts *= forwardSign;

        resetDriveEncoders();

        frontLeft.setTargetPosition(leftCounts);
        backLeft.setTargetPosition(leftCounts);
        frontRight.setTargetPosition(rightCounts);
        backRight.setTargetPosition(rightCounts);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = Math.abs(power);
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy()
                        || backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("arc", "in=%.1f  deg=%.1f", inches, turnDegrees);
            telemetry.addData("target L/R", "%d / %d", leftCounts, rightCounts);
            telemetry.addData("FL/FR", "%d / %d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

        setDrivePower(0.0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param inches Forward distance to travel (positive = forward, negative = backward)
     * @param turnDegrees Total heading change to apply over the distance (positive = left/CCW, negative = right/CW)
     * @param power Motor power (0.0 to 1.0)
     */
    protected void driveStraightWithEncoderTurn(double inches, double turnDegrees, double power) {
        // Calculate encoder targets for each side
        double Lc = Math.abs(inches); // center path length (in)
        double theta = Math.toRadians(turnDegrees); // radians
        double W = TURN_TRACK_WIDTH_INCHES; // track width (in)

        // For a straight line with rotation, left and right travel:
        // left = Lc + (theta * W / 2)
        // right = Lc - (theta * W / 2)
        double Dl = Lc + (theta * W / 2.0); // left distance (in)
        double Dr = Lc - (theta * W / 2.0); // right distance (in)

        // Convert to ticks
        int leftCounts = (int) Math.round(Dl * TICKS_PER_INCH);
        int rightCounts = (int) Math.round(Dr * TICKS_PER_INCH);

        // If going backwards, flip the signs
        double forwardSign = (inches >= 0.0) ? -1.0 : 1.0;
        leftCounts *= forwardSign;
        rightCounts *= forwardSign;

        resetDriveEncoders();

        frontLeft.setTargetPosition(leftCounts);
        backLeft.setTargetPosition(leftCounts);
        frontRight.setTargetPosition(rightCounts);
        backRight.setTargetPosition(rightCounts);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = Math.abs(power);
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy()
                        || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("driveStraightWithEncoderTurn", "in=%.1f  deg=%.1f", inches, turnDegrees);
            telemetry.addData("target L/R", "%d / %d", leftCounts, rightCounts);
            telemetry.addData("FL/FR", "%d / %d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

        setDrivePower(0.0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param inches Distance to strafe (positive = right, negative = left)
     * @param power Motor power (0.0 to 1.0)
     */
    protected void strafeInches(double inches, double power) {
        // For mecanum: FL/BR forward, FR/BL backward for right strafe
        int strafeCounts = (int) Math.round(inches * TICKS_PER_INCH);

        resetDriveEncoders();

        frontLeft.setTargetPosition(strafeCounts);
        backRight.setTargetPosition(strafeCounts);
        frontRight.setTargetPosition(-strafeCounts);
        backLeft.setTargetPosition(-strafeCounts);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = Math.abs(power);
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy()
                        || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("strafeInches", "in=%.1f", inches);
            telemetry.addData("target", "%d", strafeCounts);
            telemetry.addData("FL/FR/BL/BR", "%d / %d / %d / %d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

        setDrivePower(0.0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
