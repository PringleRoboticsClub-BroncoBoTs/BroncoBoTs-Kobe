package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.TurboBoTsTeleOp;

/**
 * Autonomous for the TurboBoTs robot ONLY. Self-contained like TurboBoTsTeleOp: it uses that
 * OpMode's motor directions and powers and nothing from the BIOBUZZ robot.
 *   1. spin up the shooter (TurboBoTsTeleOp.SHOOTER_POWER) and wait SPINUP_SEC (3 s)
 *   2. run the intake (TurboBoTsTeleOp.INTAKE_POWER) for FEED_SEC (5 s) to shoot the preloads
 *   3. stop shooter + intake
 *   4. drive FORWARD DRIVE_INCHES (12 in = 1 ft) using the drive-motor encoders only
 *
 * The drive motors' encoder cables must be plugged in.
 */
@Config
@Autonomous(name = "TurboBoTsAuto", group = "TurboBoTs", preselectTeleOp = "TurboBoTsTeleop")
public class TurboBoTsAuto extends LinearOpMode {
    public static double SPINUP_SEC = 3.0;
    public static double FEED_SEC = 5.0;
    public static double DRIVE_INCHES = 12.0;
    public static double DRIVE_POWER = 0.3;
    public static double DRIVE_TIMEOUT_SEC = 4.0;

    // Drive encoder: goBILDA 5203 312 RPM (537.7 ticks/rev) with 104 mm mecanum wheels. TODO(verify)
    public static double DRIVE_TICKS_PER_REV = 537.7;
    public static double WHEEL_DIAMETER_MM = 104.0;

    private DcMotor[] driveMotors;

    @Override
    public void runOpMode() {
        driveMotors = new DcMotor[]{
                motor("frontLeft", TurboBoTsTeleOp.FRONT_LEFT_DIR, DcMotor.ZeroPowerBehavior.BRAKE),
                motor("frontRight", TurboBoTsTeleOp.FRONT_RIGHT_DIR, DcMotor.ZeroPowerBehavior.BRAKE),
                motor("backLeft", TurboBoTsTeleOp.BACK_LEFT_DIR, DcMotor.ZeroPowerBehavior.BRAKE),
                motor("backRight", TurboBoTsTeleOp.BACK_RIGHT_DIR, DcMotor.ZeroPowerBehavior.BRAKE)};
        DcMotor shooter = motor("shooterMotor", TurboBoTsTeleOp.SHOOTER_DIR, DcMotor.ZeroPowerBehavior.FLOAT);
        DcMotor intake = motor("intakeMotor", TurboBoTsTeleOp.INTAKE_DIR, DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("TurboBoTsAuto: shoot preloads, then forward 1 ft.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // 1. spin up
        shooter.setPower(Math.abs(TurboBoTsTeleOp.SHOOTER_POWER));
        runFor(SPINUP_SEC, "Spinning up shooter");

        // 2. feed
        intake.setPower(Math.abs(TurboBoTsTeleOp.INTAKE_POWER));
        runFor(FEED_SEC, "Shooting (intake on)");

        // 3. stop mechanisms
        intake.setPower(0);
        shooter.setPower(0);

        // 4. drive forward on encoders
        if (opModeIsActive()) driveForwardInches(DRIVE_INCHES);

        for (DcMotor m : driveMotors) m.setPower(0);
        telemetry.addLine("Done");
        telemetry.update();
    }

    private void runFor(double seconds, String status) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            telemetry.addData("Step", status);
            telemetry.addData("Time left", "%.1f s", seconds - t.seconds());
            telemetry.update();
            idle();
        }
    }

    private void driveForwardInches(double inches) {
        double ticksPerInch = DRIVE_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_MM / 25.4);
        int ticks = (int) Math.round(inches * ticksPerInch);

        for (DcMotor m : driveMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(ticks);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (DcMotor m : driveMotors) m.setPower(Math.abs(DRIVE_POWER));

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < DRIVE_TIMEOUT_SEC && anyBusy()) {
            telemetry.addData("Step", "Driving forward %.0f in", inches);
            telemetry.addData("Target ticks", ticks);
            telemetry.addData("FL / FR / BL / BR", "%d %d %d %d",
                    driveMotors[0].getCurrentPosition(), driveMotors[1].getCurrentPosition(),
                    driveMotors[2].getCurrentPosition(), driveMotors[3].getCurrentPosition());
            telemetry.update();
            idle();
        }
        for (DcMotor m : driveMotors) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private boolean anyBusy() {
        for (DcMotor m : driveMotors) if (m.isBusy()) return true;
        return false;
    }

    private DcMotor motor(String name, DcMotorSimple.Direction dir, DcMotor.ZeroPowerBehavior zpb) {
        DcMotor m = hardwareMap.get(DcMotor.class, name);
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(zpb);
        return m;
    }
}
