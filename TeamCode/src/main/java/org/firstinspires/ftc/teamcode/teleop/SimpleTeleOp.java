package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.VoltageMonitor;

/**
 * Simple one-controller TeleOp: no turret, no vision, no auto-aim, fixed shooter power.
 * Handy for testing the drivetrain, intake, flywheel and gate on their own.
 *
 * gamepad1 (PS5):
 *   left stick      drive (robot-centric: "forward" is always the robot's front)
 *   right stick X   rotate
 *   L2 (hold)       intake
 *   L1 (hold)       outtake (wins over intake)
 *   R2 (hold)       shooter motor at SHOOTER_POWER (35%), open-loop
 *   D-pad UP (hold) open the shooter gate (30 deg left of home); release = back to home
 *
 * The gate homes to 0 at INIT. The turret motor is not used.
 */
@Config
@TeleOp(name = "Simple TeleOp", group = "Competition")
public class SimpleTeleOp extends OpMode {
    public static double SHOOTER_POWER = 0.35;

    private Drivetrain drive;
    private Intake intake;
    private Shooter shooter;
    private Gate gate;
    private Telemetry tel;

    @Override
    public void init() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, new VoltageMonitor(hardwareMap));
        gate = new Gate(hardwareMap); // homes the gate servo to 0
        gate.close();
        gate.update();

        tel.addLine("Simple TeleOp ready. Gate homed to 0.");
        tel.addLine("L stick drive | R stick rotate | L2 intake | L1 outtake | R2 shooter | D-pad up gate");
        tel.update();
    }

    @Override
    public void loop() {
        drive.driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.left_bumper) intake.set(Intake.State.OUTTAKE);
        else if (gamepad1.left_trigger > 0.1) intake.set(Intake.State.INTAKE);
        else intake.set(Intake.State.OFF);
        intake.update();

        boolean shooterOn = gamepad1.right_trigger > 0.1;
        shooter.setRawPower(shooterOn ? SHOOTER_POWER : 0);
        double rpm = shooter.measureRpm();

        gate.set(gamepad1.dpad_up);
        gate.update();

        tel.addData("Intake", intake.getState());
        tel.addData("Shooter", shooterOn ? String.format("ON %.0f%%", SHOOTER_POWER * 100) : "off");
        tel.addData("shooter/rpm", rpm);
        tel.addData("Gate", gate.isOpen() ? String.format("OPEN (%.0f deg)", Gate.OPEN_DEG) : "home");
        tel.update();
    }

    @Override
    public void stop() {
        drive.stop();
        intake.set(Intake.State.OFF);
        intake.update();
        shooter.setRawPower(0);
        gate.close();
        gate.update();
    }
}
