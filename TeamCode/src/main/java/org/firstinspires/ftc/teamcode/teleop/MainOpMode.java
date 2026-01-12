package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.broncoBot.BroncoBoTsServices.BroncoBoTAprilTagService;

@TeleOp(name = "ManualDrive", group = "Iterative OpMode")
public class MainOpMode extends OpMode {

    // ********** DRIVE **********
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;

    // ********** MECHANISMS **********
    private DcMotor intakeMotor;        // "IntakeMotor"
    private DcMotor shooterMotor;         // "shooterMotor"
    private DcMotor intakeRampMotor;    // "RampMotor"
    private DcMotor stageMotor;   // Stage Motor
    private Servo shooterGate;      // ShooterGate

    // ********** IMU / FIELD-CENTRIC **********
    private IMU imu;
    private double fieldYawOffsetRad = 0.0;

    // Long-press "select" (options) for yaw reset
    private boolean selectWasPressed = false;
    private boolean yawResetLatched  = false;
    private double  selectPressedTime = 0.0;
    private static final double SELECT_LONG_PRESS_SEC = 0.75;

    private boolean startWasPressed = false;
    private boolean parkingLatched  = false;
    private double startPressedTime = 0.0;

    // ********** SHOOTER CONSTANTS **********
    // Target velocity in ticks per second
    public static double TARGET_VELOCITY = 1800;

    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double SHOOTER_MAX_TICKS_PER_SEC = (6000 / 60.0) * SHOOTER_TICKS_PER_REV;   // 2800

    // Shooter PID + feed-forward gains (tune on robot)
    public static double kP = 0.04;
    public static double kI = 0.01;
    public static double kD = 0.0;
    public static double kF = 90.0; // Tune this value first
    // Shooter state
    private double shooterTargetVelocity = 0.0; // ticks / second

    // ********** APRILTAG SERVICE **********
    private BroncoBoTAprilTagService tagService;

    // Single tag ID of interest (change as needed)
    private static final int TAG_ID_OF_INTEREST = 20;

    // ********** END OF VARIABLES **********

    @Override
    public void init() {
        HardwareMap hw = hardwareMap;

        initDrive(hw);
        initMechanisms(hw);
        initImu(hw);
        initVision(hw);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt  = 0.0;

        handleYawReset(now);
        handleParking(now);

        // Read gamepad
        double y  = -gamepad1.left_stick_y; // forward/back
        double x  =  gamepad1.left_stick_x; // strafe
        double rx =  gamepad1.right_stick_x;// rotate

        double leftTrigger  = gamepad1.left_trigger;   // shoot + auto align
        double rightTrigger = gamepad1.right_trigger;  // stage + intake assist
        boolean leftBumper  = gamepad1.left_bumper;    // intake + ramp
        boolean leftBumper_CNTRL2 = gamepad2.left_bumper;

        boolean finalIntake = leftBumper || leftBumper_CNTRL2;


        // Shooter + tag alignment returns desired auto-rotation contribution
        double autoRotate = updateShooterAndTag(leftTrigger, dt);

        // Intake + stage motors (and ramp)
        updateIntakeStage(finalIntake, rightTrigger);

        // Field-centric drive
        driveFieldCentric(x, y, rx, autoRotate);

        sendTelemetry();

        if (gamepad1.left_trigger > 0.1) {
            shooterTargetVelocity = TARGET_VELOCITY;
            shooterMotor.setPower(0.73);
        } else if (gamepad1.left_trigger < 0.1){
            shooterMotor.setPower(0.0);
        }
    }

    @Override
    public void stop() {
        stopDrive();
        stopMechanisms();

        if (tagService != null) {
            tagService.close();
        }
    }

    // ********** INIT HELPERS **********

    private void initDrive(HardwareMap hw) {
        frontRight = hw.get(DcMotor.class, "frontRight");
        frontLeft  = hw.get(DcMotor.class, "frontLeft");
        backLeft   = hw.get(DcMotor.class, "backLeft");
        backRight  = hw.get(DcMotor.class, "backRight");

        // Adjust to match your wiring
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveZeroPower();
    }

    private void initMechanisms(HardwareMap hw) {
        shooterMotor    = hw.get(DcMotor.class, "shooterMotor");
        intakeMotor     = hw.get(DcMotor.class,   "intakeMotor");
        intakeRampMotor = hw.get(DcMotor.class,   "intakeRampMotor");
        shooterGate       = hw.get(Servo.class,     "shooterGate");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        // shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRampMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // shooterGate.scaleRange(0.0, 0.5);
        shooterGate.setPosition(0.5);
    }

    private void initImu(HardwareMap hw) {
        imu = hw.get(IMU.class, "imu");

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(imuParams);

        // Define current heading as field-forward at init
        fieldYawOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private void initVision(HardwareMap hw) {
        // Webcam name: "webcam1"
        tagService = new BroncoBoTAprilTagService(hw);
    }

    // ********** LOOP HELPERS **********

    private void handleParking(double now) {
        boolean startButton = gamepad2.start;

        if (startButton) {
            if (!startWasPressed) {
                startWasPressed  = true;
                startPressedTime = now;
                parkingLatched   = false;
            } else if (!parkingLatched &&
                    (now - startPressedTime) > SELECT_LONG_PRESS_SEC) {
                setDriveZeroPower();
                parkingLatched = true;
            }
        } else {
            startWasPressed = false;
            parkingLatched  = false;
        }
    }

    private void handleYawReset(double now) {
        // Treat OPTIONS as "select"
        boolean select = gamepad1.options;

        if (select) {
            if (!selectWasPressed) {
                selectWasPressed  = true;
                selectPressedTime = now;
                yawResetLatched   = false;
            } else if (!yawResetLatched &&
                    (now - selectPressedTime) > SELECT_LONG_PRESS_SEC) {
                // Reset yaw: define current heading as field-forward
                fieldYawOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                yawResetLatched = true;
            }
        } else {
            selectWasPressed = false;
            yawResetLatched  = false;
        }
    }

    /**
     * Handles shooter PID + feed-forward, hood angle, and auto-rotation
     * based on AprilTag pose while left trigger is held.
     *
     * @return auto-rotation command to add to driver rx
     */
    private double updateShooterAndTag(double leftTrigger, double dt) {
        double autoRotate = 0.0;

        if (leftTrigger <= 0.05) {
            // Shooter idle
            shooterTargetVelocity = 0.0;
            // shooterMotor.setVelocity(200);
            shooterGate.setPosition(0.5);
            return 0.0;
        }

        // 1) Get tag pose
        BroncoBoTAprilTagService.TagPose pose = tagService.getTagPose(TAG_ID_OF_INTEREST);

        if (pose != null) {
            // Distance is Z only (forward depth)
            double distanceMeters = pose.getDistanceMeters();   // |z|

            // Distance -> shooter velocity (ticks / sec)
            // shooterTargetVelocity = mapDistanceToShooterVelocity(distanceMeters);

            // shooterTargetVelocity = TARGET_VELOCITY; // ticks/sec

            // Distance -> hood servo position
            double hoodPos = mapDistanceToHoodPosition(distanceMeters);
            // hoodServo.setPosition(hoodPos);
            // double currentVelocity = shooterMotor.getVelocity(); // ticks / second

            double headingErrorDeg = pose.yawDeg;
            double kRotate = 0.01; // tune this
            autoRotate = Range.clip(kRotate * headingErrorDeg, -0.4, 0.4);

            telemetry.addData("TagID", pose.id);
            telemetry.addData("Tag Z (m)", distanceMeters);
            telemetry.addData("Tag Heading Error (deg)", headingErrorDeg);
            telemetry.addData("Hood pos", hoodPos);
            // telemetry.addData("shooter current Velocity", currentVelocity);
        } else {
            // No tag
            if (gamepad1.left_trigger > 0.1) {
                shooterTargetVelocity = TARGET_VELOCITY;
            }
        }



        telemetry.addData("shooter Target Velocity", shooterTargetVelocity);

        // shooterMotor.setPower(power);
        // shooterMotor.setVelocity(shooterTargetVelocity);

        return autoRotate;
    }

    /**
     * Intake / stage / ramp logic:
     * - Left bumper:
     *      intakeMotor      = 0.5
     *      intakeRampMotor  = 0.5
     * - Right trigger:
     *      intakeMotor      >= 0.3
     *      intakeRampMotor  >= 0.3
     *   (if left bumper is also pressed, intake & ramp stay at 0.5)
     */
    private void updateIntakeStage(boolean leftBumper, double rightTrigger) {
        double intakePower = 0.0;
        double rampPower   = 0.0;

        // Left bumper: base 0.5 for intake + ramp
        if (leftBumper) {
            intakePower = 0.52;
            rampPower   = 0.90;
        }

        // Right trigger: stage = 0.3, intake/ramp at least 0.3
        if (!leftBumper && rightTrigger > 0.05) {
            intakePower = 0.5;
            rampPower   = 0.7;
            shooterGate.setPosition(0.0);
        } else {
            shooterGate.setPosition(0.5);
        }

        intakeMotor.setPower(intakePower);
        intakeRampMotor.setPower(rampPower);
    }

    private void driveFieldCentric(double x, double y, double rx, double autoRotate) {
        double yawRad       = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double fieldHeading = yawRad - fieldYawOffsetRad;

        // Rotate joystick vector from field frame into robot frame
        double cosA = Math.cos(-fieldHeading);
        double sinA = Math.sin(-fieldHeading);

        double rotatedX = x * cosA - y * sinA;
        double rotatedY = x * sinA + y * cosA;

        double finalRx = Range.clip(rx + autoRotate, -1.0, 1.0);

        double denominator = Math.max(
                Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(finalRx),
                1.0);

        double fl = (rotatedY + rotatedX + finalRx) / denominator;
        double bl = (rotatedY - rotatedX + finalRx) / denominator;
        double fr = (rotatedY - rotatedX - finalRx) / denominator;
        double br = (rotatedY + rotatedX - finalRx) / denominator;

        frontRight.setPower(fr);
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void sendTelemetry() {
        double yawRad       = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double fieldHeading = yawRad - fieldYawOffsetRad;

        telemetry.addData("Field Yaw (deg)", Math.toDegrees(fieldHeading));
        telemetry.addData("Shooter target vel", shooterTargetVelocity);
        telemetry.addData("Shooter power", shooterMotor.getPower());
        telemetry.addData("Intake power", intakeMotor.getPower());
        telemetry.addData("Ramp power", intakeRampMotor.getPower());
        telemetry.update();
    }

    // ********** MAPPING HELPERS **********

    // Distance (Z) -> shooter wheel velocity (ticks / second).
    // *** Tune minDist/maxDist and minFrac/maxFrac on-robot ***
    private double mapDistanceToShooterVelocity(double distanceMeters) {
        // lets make a quadratic equation or a polynomial
        return 0.0;
    }

    // Distance (Z) -> hood servo position (0..1).
    // *** Tune based on your physical hood geometry ***
    private double mapDistanceToHoodPosition(double distanceMeters) {
        double minDist    = 0.5;   // closest shot
        double maxDist    = 3.0;   // farthest shot you care about
        double closeAngle = 0.7;   // hood "up" (more arc)
        double farAngle   = 0.3;   // hood "down" (flatter)

        double d = Range.clip(distanceMeters, minDist, maxDist);
        double t = (d - minDist) / (maxDist - minDist);

        double pos = closeAngle + t * (farAngle - closeAngle);
        return Range.clip(pos, 0.0, 1.0);
    }

    // ********** STOP / UTILS **********

    private void stopDrive() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void stopMechanisms() {
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);
        intakeRampMotor.setPower(0);
    }

    private void setDriveZeroPower() {
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
