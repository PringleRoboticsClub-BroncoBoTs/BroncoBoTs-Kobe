package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Checks that each drive motor NAME is plugged into the wheel it claims to be, and spins the
 * right way. PUT THE ROBOT ON BLOCKS (wheels off the ground) before running.
 *
 * Hold a button to spin ONE motor "forward" at TEST_POWER (using the same motor directions as
 * TeleOp and Pedro):
 *
 *     L1 = frontLeft          R1 = frontRight
 *     L2 = backLeft           R2 = backRight
 *     CROSS = all four (robot should drive straight forward)
 *
 * The buttons are laid out like the wheels: left side of the controller = left wheels,
 * bumpers (top) = front wheels, triggers (below) = back wheels.
 *
 * For each button, check:
 *   1. The wheel that spins is the one named (e.g. L1 spins the FRONT LEFT wheel).
 *      Wrong wheel -> the cables are swapped: either move the cables on the hub, or swap the
 *      names on those ports in the Driver Station configuration.
 *   2. It spins so the robot would move FORWARD.
 *      Backwards -> change that side's direction in Drivetrain AND pedro/Constants.java.
 */
@Config
@TeleOp(name = "Tune: Drive Motors", group = "Tuning")
public class DriveMotorTest extends OpMode {
    public static double TEST_POWER = 0.3;

    private static final String[] NAMES = {
            RobotConfig.FRONT_LEFT, RobotConfig.FRONT_RIGHT, RobotConfig.BACK_LEFT, RobotConfig.BACK_RIGHT};
    private static final String[] BUTTONS = {"L1", "R1", "L2", "R2"};

    private final DcMotor[] motors = new DcMotor[4];

    @Override
    public void init() {
        // Drivetrain applies the real motor directions + brake mode to these same motor objects.
        new Drivetrain(hardwareMap).stop();
        for (int i = 0; i < 4; i++) motors[i] = hardwareMap.get(DcMotor.class, NAMES[i]);

        telemetry.addLine("ROBOT ON BLOCKS. Hold L1 / R1 / L2 / R2 to spin one wheel, CROSS for all.");
        for (int i = 0; i < 4; i++) {
            telemetry.addData(NAMES[i], "port %d  (%s)", motors[i].getPortNumber(), motors[i].getConnectionInfo());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean all = gamepad1.cross;
        boolean[] held = {
                all || gamepad1.left_bumper,
                all || gamepad1.right_bumper,
                all || gamepad1.left_trigger > 0.5,
                all || gamepad1.right_trigger > 0.5};

        telemetry.addLine("Each button must spin the wheel it names, in the FORWARD direction.");
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(held[i] ? TEST_POWER : 0);
            telemetry.addData(String.format("%-3s %-10s", BUTTONS[i], NAMES[i]),
                    "port %d  %s  encoder %d",
                    motors[i].getPortNumber(), held[i] ? "SPINNING" : "-", motors[i].getCurrentPosition());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotor m : motors) if (m != null) m.setPower(0);
    }
}
