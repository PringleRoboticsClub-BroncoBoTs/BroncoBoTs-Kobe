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
 *      Backwards -> flip that wheel's *_DIR in Drivetrain (Pedro uses the same values).
 *
 * STICK TEST (no buttons held, robot on the floor, at reduced power STICK_SCALE):
 *   The sticks drive ROBOT-centric (IMU ignored). TRIANGLE toggles field-centric to compare.
 *     left stick right  -> robot must STRAFE RIGHT without turning
 *     right stick right -> robot must TURN CLOCKWISE in place
 *   Robot-centric wrong -> mecanum wheels are on the wrong corners (rollers seen from ABOVE
 *                          must form an "X"), since the wiring + directions already passed.
 *   Robot-centric right but field-centric wrong -> IMU problem: turn the robot CCW by hand,
 *                          "IMU yaw" must INCREASE (about +90 after a quarter turn).
 */
@Config
@TeleOp(name = "Tune: Drive Motors", group = "Tuning")
public class DriveMotorTest extends OpMode {
    public static double TEST_POWER = 0.3;
    public static double STICK_SCALE = 0.5;

    private static final String[] NAMES = {
            RobotConfig.FRONT_LEFT, RobotConfig.FRONT_RIGHT, RobotConfig.BACK_LEFT, RobotConfig.BACK_RIGHT};
    private static final String[] BUTTONS = {"L1", "R1", "L2", "R2"};

    private final DcMotor[] motors = new DcMotor[4];
    private Drivetrain drive;
    private boolean fieldCentric = false;

    @Override
    public void init() {
        // Drivetrain applies the real motor directions + brake mode to these same motor objects.
        drive = new Drivetrain(hardwareMap);
        drive.stop();
        for (int i = 0; i < 4; i++) motors[i] = hardwareMap.get(DcMotor.class, NAMES[i]);

        telemetry.addLine("WHEEL TEST on blocks: hold L1 / R1 / L2 / R2 for one wheel, CROSS for all.");
        telemetry.addLine("STICK TEST on the floor: use the sticks (robot-centric, half power).");
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

        boolean anyHeld = held[0] || held[1] || held[2] || held[3];
        if (gamepad1.triangleWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.optionsWasPressed()) drive.resetFieldYaw();

        if (anyHeld) {
            for (int i = 0; i < 4; i++) motors[i].setPower(held[i] ? TEST_POWER : 0);
        } else {
            double x = gamepad1.left_stick_x * STICK_SCALE;
            double y = -gamepad1.left_stick_y * STICK_SCALE;
            double rx = gamepad1.right_stick_x * STICK_SCALE;
            if (fieldCentric) drive.driveFieldCentric(x, y, rx);
            else drive.driveRobotCentric(x, y, rx);
        }

        telemetry.addLine("WHEEL TEST: L1/R1/L2/R2 = one wheel forward, CROSS = all.");
        telemetry.addLine("STICK TEST: left stick right = strafe right, right stick right = turn clockwise.");
        telemetry.addData("Stick mode (TRIANGLE)", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
        telemetry.addData("IMU yaw (OPTIONS = zero)", "%.1f deg  (turn CCW -> goes UP)",
                Math.toDegrees(drive.getFieldHeadingRad()));
        for (int i = 0; i < 4; i++) {
            telemetry.addData(String.format("%-3s %-10s", BUTTONS[i], NAMES[i]),
                    "port %d  power %+.2f  encoder %d",
                    motors[i].getPortNumber(), motors[i].getPower(), motors[i].getCurrentPosition());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotor m : motors) if (m != null) m.setPower(0);
    }
}
