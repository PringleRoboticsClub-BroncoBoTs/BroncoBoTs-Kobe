package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.VoltageMonitor;
import org.firstinspires.ftc.teamcode.teleop.SimpleTeleOp;

/**
 * Simple autonomous for the MAIN (BIOBUZZ) robot. No Pedro, Pinpoint, camera or turret:
 *   1. spin up the shooter (same power as Simple TeleOp) and wait SPINUP_SEC (3 s)
 *   2. open the gate and run the intake for FEED_SEC (5 s) to shoot the preloads
 *   3. stop shooter + intake, close the gate
 *   4. drive FORWARD DRIVE_INCHES (12 in = 1 ft) using the drive-motor encoders only
 *
 * The drive motors' encoder cables must be plugged in. Wheel directions come from Drivetrain.
 */
@Config
@Autonomous(name = "Simple Auto", group = "Competition", preselectTeleOp = "Simple TeleOp")
public class SimpleAuto extends LinearOpMode {
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
        // Drivetrain sets each wheel's direction (+ = forward) and brake mode on these motor objects.
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        drivetrain.stop();
        driveMotors = new DcMotor[]{
                hardwareMap.get(DcMotor.class, RobotConfig.FRONT_LEFT),
                hardwareMap.get(DcMotor.class, RobotConfig.FRONT_RIGHT),
                hardwareMap.get(DcMotor.class, RobotConfig.BACK_LEFT),
                hardwareMap.get(DcMotor.class, RobotConfig.BACK_RIGHT)};

        Shooter shooter = new Shooter(hardwareMap, new VoltageMonitor(hardwareMap));
        Intake intake = new Intake(hardwareMap);
        Gate gate = new Gate(hardwareMap); // homes the gate at INIT
        gate.close();
        gate.update();

        telemetry.addLine("Simple Auto (main robot): shoot preloads, then forward 1 ft.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // 1. spin up
        shooter.setRawPower(SimpleTeleOp.SHOOTER_POWER);
        runFor(SPINUP_SEC, "Spinning up shooter", shooter);

        // 2. feed
        gate.open();
        gate.update();
        intake.set(Intake.State.INTAKE);
        intake.update();
        runFor(FEED_SEC, "Shooting (intake on, gate open)", shooter);

        // 3. stop mechanisms
        intake.set(Intake.State.OFF);
        intake.update();
        gate.close();
        gate.update();
        shooter.setRawPower(0);

        // 4. drive forward on encoders
        if (opModeIsActive()) driveForwardInches(DRIVE_INCHES);

        for (DcMotor m : driveMotors) m.setPower(0);
        telemetry.addLine("Done");
        telemetry.update();
    }

    private void runFor(double seconds, String status, Shooter shooter) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            telemetry.addData("Step", status);
            telemetry.addData("Time left", "%.1f s", seconds - t.seconds());
            telemetry.addData("Shooter rpm", "%.0f", shooter.measureRpm());
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
}
