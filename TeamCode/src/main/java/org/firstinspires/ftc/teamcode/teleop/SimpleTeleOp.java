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
 * Simple TeleOp (gamepad1 drives; either gamepad runs intake/shooter/gate): no turret, no vision, no auto-aim, fixed shooter power.
 * Handy for testing the drivetrain, intake, flywheel and gate on their own.
 *
 * gamepad1 (PS5, driver) - driving is on gamepad1 ONLY:
 *   left stick      drive (FIELD-centric by default: forward = away from the driver)
 *   right stick X   rotate
 *   SHARE           toggle FIELD-centric / ROBOT-centric (forward = robot's front)
 *   OPTIONS (hold)  reset field "forward" to the way the robot faces now
 *
 * Field-centric "forward" is the way the robot faces when INIT is pressed, so start it facing
 * away from the driver. The IMU mounting is set in Drivetrain (HUB_LOGO / HUB_USB).
 *
 * EITHER gamepad (driver or operator) - intake, shooter, gate:
 *   L2 (hold)       intake
 *   L1 (hold)       outtake (wins over intake)
 *   R2 (hold)       shooter motor at SHOOTER_POWER (45%), open-loop
 *   D-pad UP (hold) open the shooter gate (30 deg left of home); release = back to home
 *
 * The gate homes to 0 at INIT. The turret motor is not used.
 */
@Config
@TeleOp(name = "Simple TeleOp", group = "Competition")
public class SimpleTeleOp extends OpMode {
    public static double SHOOTER_POWER = 0.45;
    public static boolean START_FIELD_CENTRIC = true;
    public static double RESET_HOLD_SEC = 0.75;

    private Drivetrain drive;
    private Intake intake;
    private Shooter shooter;
    private Gate gate;
    private Telemetry tel;
    private boolean fieldCentric;
    private double optionsHeldSince = -1;
    private boolean headingResetDone = false;

    @Override
    public void init() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drivetrain(hardwareMap); // also zeroes the field heading: forward = facing now
        fieldCentric = START_FIELD_CENTRIC;
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, new VoltageMonitor(hardwareMap));
        gate = new Gate(hardwareMap); // homes the gate servo to 0
        gate.close();
        gate.update();

        tel.addLine("Simple TeleOp ready. Gate homed to 0.");
        tel.addLine("GP1: L stick drive | R stick rotate | SHARE field/robot-centric | hold OPTIONS reset forward");
        tel.addLine("Field-centric forward = the way the robot faces NOW.");
        tel.addLine("GP1 or GP2: L2 intake | L1 outtake | R2 shooter | D-pad up gate");
        tel.update();
    }

    @Override
    public void loop() {
        if (gamepad1.shareWasPressed()) fieldCentric = !fieldCentric;
        handleHeadingReset();
        if (fieldCentric) {
            drive.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        } else {
            drive.driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        if (gamepad1.left_bumper || gamepad2.left_bumper) intake.set(Intake.State.OUTTAKE);
        else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) intake.set(Intake.State.INTAKE);
        else intake.set(Intake.State.OFF);
        intake.update();

        boolean shooterOn = gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1;
        shooter.setRawPower(shooterOn ? SHOOTER_POWER : 0);
        double rpm = shooter.measureRpm();

        gate.set(gamepad1.dpad_up || gamepad2.dpad_up);
        gate.update();

        tel.addData("Drive", fieldCentric
                ? String.format("FIELD-centric  heading %.0f deg", Math.toDegrees(drive.getFieldHeadingRad()))
                : "ROBOT-centric");
        tel.addData("Intake", intake.getState());
        tel.addData("Shooter", shooterOn ? String.format("ON %.0f%%", SHOOTER_POWER * 100) : "off");
        tel.addData("shooter/rpm", rpm);
        tel.addData("Gate", gate.isOpen() ? String.format("OPEN (%.0f deg)", Gate.OPEN_DEG) : "home");
        tel.addData("D-pad up GP1 / GP2", "%b / %b", gamepad1.dpad_up, gamepad2.dpad_up);
        gate.addDiagnostics(tel);
        tel.update();
    }

    /** Hold OPTIONS to make the robot's current heading field-forward. */
    private void handleHeadingReset() {
        if (gamepad1.options) {
            if (optionsHeldSince < 0) optionsHeldSince = getRuntime();
            if (!headingResetDone && getRuntime() - optionsHeldSince >= RESET_HOLD_SEC) {
                drive.resetFieldYaw();
                gamepad1.rumble(200);
                headingResetDone = true;
            }
        } else {
            optionsHeldSince = -1;
            headingResetDone = false;
        }
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
