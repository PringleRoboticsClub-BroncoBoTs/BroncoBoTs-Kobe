package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TeleOp for the TurboBoTs robot - a DIFFERENT robot from the BIOBUZZ one, which runs ONLY this
 * OpMode. It is deliberately self-contained: it does not use the BIOBUZZ subsystems, RobotConfig
 * or their tuned values, so tuning one robot can never break the other.
 *
 * Hardware (Driver Station configuration names):
 *   frontLeft, frontRight, backLeft, backRight   mecanum drive motors
 *   shooterMotor                                 shooter
 *   intakeMotor                                  intake
 *   imu                                          Control Hub IMU (only for field-centric drive)
 * Nothing else is used (no odometry, servos or camera). If "imu" is missing, field-centric is
 * simply unavailable and the robot drives robot-centric.
 *
 * gamepad1 (PS5, driver):
 *   left stick      drive
 *   right stick X   rotate
 *   SHARE           toggle ROBOT-centric (forward = robot's front) / FIELD-centric (forward =
 *                   away from the driver, whichever way the robot faces)
 *   OPTIONS (hold)  field-centric: reset "forward" to the way the robot faces now
 *
 * EITHER gamepad (driver or operator) - intake + shooter:
 *   L2 (hold)       intake
 *   L1 (hold)       outtake (wins over intake)
 *   R2 (hold)       shooter at SHOOTER_POWER
 *
 * gamepad2 only (operator):
 *   R1 (hold)       shooter in REVERSE at SHOOTER_REVERSE_POWER (to clear a jam); wins over R2
 *
 * Field-centric: the robot must face AWAY from the driver when INIT is pressed (or hold OPTIONS
 * later to re-define forward). HUB_LOGO / HUB_USB must match how this robot's Control Hub is
 * mounted, otherwise field-centric drive is rotated.
 *
 * All values below are editable live on FTC Dashboard ("TurboBoTsTeleOp" card). Motor directions
 * take effect when the OpMode is re-initialised; copy final values into this file.
 */
@Config
@TeleOp(name = "TurboBoTsTeleop", group = "TurboBoTs")
public class TurboBoTsTeleOp extends OpMode {
    // Motor directions: + power must drive the robot FORWARD / pull balls IN / shoot OUT.
    public static DcMotorSimple.Direction FRONT_LEFT_DIR  = DcMotorSimple.Direction.REVERSE; // from TurboBoTs: Drive Motors test
    public static DcMotorSimple.Direction BACK_LEFT_DIR   = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction FRONT_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction BACK_RIGHT_DIR  = DcMotorSimple.Direction.FORWARD; // from TurboBoTs: Drive Motors test
    public static DcMotorSimple.Direction SHOOTER_DIR     = DcMotorSimple.Direction.REVERSE; // reversed on robot
    public static DcMotorSimple.Direction INTAKE_DIR      = DcMotorSimple.Direction.FORWARD; // TODO(verify)

    public static double DRIVE_SPEED = 1.0;     // scale on all drive power (lower for gentler driving)
    public static double INTAKE_POWER = 0.9;
    public static double OUTTAKE_POWER = 0.6;
    public static double SHOOTER_POWER = 0.70;
    public static double SHOOTER_REVERSE_POWER = 0.70;

    // Field-centric
    public static boolean START_FIELD_CENTRIC = false;
    public static double RESET_HOLD_SEC = 0.75;
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;      // TurboBoTs hub: logo faces LEFT
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;   // TurboBoTs hub: USB faces BACK

    private DcMotor frontLeft, frontRight, backLeft, backRight, shooter, intake;
    private IMU imu; // null if this robot has no "imu" configured
    private boolean fieldCentric;
    private double optionsHeldSince = -1;
    private boolean headingResetDone = false;

    @Override
    public void init() {
        frontLeft  = motor("frontLeft",  FRONT_LEFT_DIR,  DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight = motor("frontRight", FRONT_RIGHT_DIR, DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft   = motor("backLeft",   BACK_LEFT_DIR,   DcMotor.ZeroPowerBehavior.BRAKE);
        backRight  = motor("backRight",  BACK_RIGHT_DIR,  DcMotor.ZeroPowerBehavior.BRAKE);
        shooter    = motor("shooterMotor", SHOOTER_DIR,   DcMotor.ZeroPowerBehavior.FLOAT); // coast down
        intake     = motor("intakeMotor",  INTAKE_DIR,    DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
            imu.resetYaw(); // forward = the way the robot faces at INIT
        } catch (IllegalArgumentException e) {
            imu = null;
        }
        fieldCentric = START_FIELD_CENTRIC && imu != null;

        telemetry.addLine("TurboBoTs TeleOp ready.");
        telemetry.addLine("GP1: L stick drive | R stick rotate | SHARE robot/field-centric | hold OPTIONS reset forward");
        telemetry.addLine("GP1 or GP2: L2 intake | L1 outtake | R2 shooter");
        telemetry.addLine("GP2: R1 shooter reverse");
        if (imu == null) telemetry.addLine("No 'imu' in config: field-centric unavailable.");
        else telemetry.addLine("Field-centric forward = the way the robot faces NOW.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---- drive (robot-centric mecanum) ----
        double y = -gamepad1.left_stick_y;  // forward +
        double x = gamepad1.left_stick_x;   // strafe right +
        double rx = gamepad1.right_stick_x; // rotate clockwise +

        if (gamepad1.shareWasPressed() && imu != null) fieldCentric = !fieldCentric;
        handleHeadingReset();

        double headingDeg = Double.NaN;
        if (fieldCentric) {
            // Rotate the stick vector from the field frame into the robot frame.
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            headingDeg = Math.toDegrees(heading);
            double cos = Math.cos(-heading), sin = Math.sin(-heading);
            double fx = x, fy = y;
            x = fx * cos - fy * sin;
            y = fx * sin + fy * cos;
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double speed = Range.clip(DRIVE_SPEED, 0, 1);
        frontLeft.setPower(speed * (y + x + rx) / denominator);
        backLeft.setPower(speed * (y - x + rx) / denominator);
        frontRight.setPower(speed * (y - x - rx) / denominator);
        backRight.setPower(speed * (y + x - rx) / denominator);

        // ---- intake / outtake ----
        String intakeState;
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            intake.setPower(-Math.abs(OUTTAKE_POWER));
            intakeState = "OUTTAKE";
        } else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
            intake.setPower(Math.abs(INTAKE_POWER));
            intakeState = "INTAKE";
        } else {
            intake.setPower(0);
            intakeState = "off";
        }

        // ---- shooter ----
        boolean shooterReverse = gamepad2.right_bumper;
        boolean shooterOn = !shooterReverse && (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1);
        if (shooterReverse) shooter.setPower(-Range.clip(SHOOTER_REVERSE_POWER, 0, 1));
        else shooter.setPower(shooterOn ? Range.clip(SHOOTER_POWER, 0, 1) : 0);

        telemetry.addData("Drive", imu == null ? "ROBOT-centric (no IMU)"
                : fieldCentric ? String.format("FIELD-centric  heading %.0f deg", headingDeg) : "ROBOT-centric");
        telemetry.addData("Intake", intakeState);
        telemetry.addData("Shooter", shooterReverse ? String.format("REVERSE %.0f%%", SHOOTER_REVERSE_POWER * 100)
                : shooterOn ? String.format("ON %.0f%%", SHOOTER_POWER * 100) : "off");
        telemetry.addData("Drive FL/FR/BL/BR", "%+.2f %+.2f %+.2f %+.2f",
                frontLeft.getPower(), frontRight.getPower(), backLeft.getPower(), backRight.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight, shooter, intake}) {
            if (m != null) m.setPower(0);
        }
    }

    /** Hold OPTIONS to make the robot's current heading field-forward. */
    private void handleHeadingReset() {
        if (imu == null) return;
        if (gamepad1.options) {
            if (optionsHeldSince < 0) optionsHeldSince = getRuntime();
            if (!headingResetDone && getRuntime() - optionsHeldSince >= RESET_HOLD_SEC) {
                imu.resetYaw();
                gamepad1.rumble(200);
                headingResetDone = true;
            }
        } else {
            optionsHeldSince = -1;
            headingResetDone = false;
        }
    }

    private DcMotor motor(String name, DcMotorSimple.Direction dir, DcMotor.ZeroPowerBehavior zpb) {
        DcMotor m = hardwareMap.get(DcMotor.class, name);
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(zpb);
        return m;
    }
}
