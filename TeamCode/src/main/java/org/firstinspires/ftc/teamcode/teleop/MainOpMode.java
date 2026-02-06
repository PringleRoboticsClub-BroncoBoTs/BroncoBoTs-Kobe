package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BroncoBoTsServices.BroncoBoTAprilTagService;

@Config
@TeleOp(name = "ManualDrive-BLUE", group = "Iterative OpMode")
public class MainOpMode extends OpMode {
    // ********** COLOR SENSORS **********
    private com.qualcomm.robotcore.hardware.ColorSensor colorSensorBottom;
    private com.qualcomm.robotcore.hardware.ColorSensor colorSensorTop;
    private long colorDetectedStartTime = 0;
    private boolean colorDetectedLatched = false;

    // Test comment to try push wirelessly from Android Studio
    // ********** DRIVE **********
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;

    // ********** MECHANISMS **********
    private DcMotor intakeMotor; // "IntakeMotor"
    private DcMotorEx shooterMotor; // "shooterMotor" (now DcMotorEx for PIDF)
    private DcMotor intakeRampMotor; // "RampMotor"
    private Servo shooterGate; // shooterGate
    private Servo hoodAdjuster; // hoodAdjuster

    // ********** IMU / FIELD-CENTRIC **********
    private IMU imu;
    private double fieldYawOffsetRad = 0.0;

    // Long-press "select" (options) for yaw reset
    private boolean selectWasPressed = false;
    private boolean yawResetLatched = false;
    private double selectPressedTime = 0.0;
    private static final double SELECT_LONG_PRESS_SEC = 0.75;

    private boolean startWasPressed = false;
    private boolean parkingLatched = false;
    private double startPressedTime = 0.0;
    private double distanceToTag = 0.0;

    // ********** SHOOTER CONSTANTS **********
    // Target velocity in ticks per second
    public static double TARGET_VELOCITY = 1000;

    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double SHOOTER_MAX_TICKS_PER_SEC = (6000 / 60.0) * SHOOTER_TICKS_PER_REV; // 2800

    // Shooter PID + feed-forward gains (tune on robot)
    public static double kP = 105;
    public static double kI = 0.0;
    public static double kD = 2.5;
    public static double kF = 33; // Tune this value first (kF for velocity mode, typically much lower)
    // Shooter state
    public static double shooterTargetVelocity = 0.0; // ticks / second

    // ********** APRILTAG SERVICE **********
    private BroncoBoTAprilTagService tagService;

    // Single tag ID of interest (change as needed)
    private int TAG_ID_OF_INTEREST = 20; // 20 - BLUE, 24 - RED

    // ********** END OF VARIABLES **********

    @Override
    public void init() {
        HardwareMap hw = hardwareMap;
        initDrive(hw);
        initMechanisms(hw);
        initImu(hw);

        initVision(hw);

        // Initialize color sensors
        colorSensorBottom = hw.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensorBottom");
        colorSensorTop = hw.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensorTop");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = 0.0;

        // Update shooter PIDF coefficients live for dashboard tuning
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        handleYawReset(now);
        handleParking(now);

        // Read gamepad
        double y = -gamepad1.left_stick_y; // forward/back
        double x = gamepad1.left_stick_x; // strafe
        double rx = gamepad1.right_stick_x;// rotate

        double leftTrigger = gamepad1.left_trigger; // shoot + auto align
        double rightTrigger = gamepad1.right_trigger; // intake shoot assist
        boolean leftBumper = gamepad1.left_bumper; // intake + ramp
        boolean leftBumper_CNTRL2 = gamepad2.left_bumper; // intake
        boolean rightBumper_CNTRL2 = gamepad2.right_bumper; // Gate Open / Close
        boolean downButton = gamepad1.dpad_down; // Gate open / close
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean finalIntake = leftBumper || leftBumper_CNTRL2;
        boolean gateControl = rightBumper_CNTRL2 || downButton;

        // Shooter + tag alignment returns desired auto-rotation contribution
        double autoRotate = updateShooterAndTag(leftTrigger, dt);

        // Intake (and ramp)
        updateIntakeStage(finalIntake, rightTrigger, gateControl);

        // Field-centric drive
        driveFieldCentric(x, y, rx, autoRotate);

        if (dpadLeft) {
            shooterTargetVelocity += SHOOTER_MAX_TICKS_PER_SEC * 0.05;
            shooterTargetVelocity = Range.clip(shooterTargetVelocity, 0, SHOOTER_MAX_TICKS_PER_SEC);

            double hoodPos = hoodAdjuster.getPosition();
            hoodPos -= 0.02;
            telemetry.addData("hoodPos", hoodPos);
            hoodPos = Range.clip(hoodPos, 0.0, 1.0);
            hoodAdjuster.setPosition(hoodPos);
        }
        // hood angle increment (0.1 per press, up to 1.0)
        if (dpadRight) {
            double hoodPos = hoodAdjuster.getPosition();
            hoodPos += 0.02;
            telemetry.addData("hoodPos", hoodPos);
            hoodPos = Range.clip(hoodPos, 0.0, 1.0);
            hoodAdjuster.setPosition(hoodPos);
        }

        sendTelemetry();

        if (gamepad1.left_trigger > 0.1) {
            // Vibrate gamepad1 when shooter motor is 95% of commanded velocity
            double currentVelocity = shooterMotor.getVelocity();
            if (Math.abs(currentVelocity - velocity) < (velocity * 0.05)) {
                gamepad1.rumble(0.5, 0.5, 200); // 200ms, both motors
            }
        }

        // --- COLOR SENSOR LOGIC ---
        boolean bottomGreenOrPurple = isGreenOrPurple(colorSensorBottom);
        boolean topGreenOrPurple = isGreenOrPurple(colorSensorTop);
        long nowMillis = System.currentTimeMillis();
        if (bottomGreenOrPurple && topGreenOrPurple) {
            if (!colorDetectedLatched) {
                colorDetectedStartTime = nowMillis;
                colorDetectedLatched = true;
            } else if ((nowMillis - colorDetectedStartTime) > 500) {
                gamepad2.rumble(0.3, 0.3, 200); // light rumble for 200ms
            }
        } else {
            colorDetectedLatched = false;
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

    // a simple heuristic to detect green or purple objects based on RGB values
    private boolean isGreenOrPurple(com.qualcomm.robotcore.hardware.ColorSensor sensor) {
        if (sensor == null)
            return false;
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        boolean isGreen = (g > r) && (g > b) && (g > 50);
        boolean isPurple = (r > 50 && b > 50 && Math.abs(r - b) < 30 && g < r && g < b);
        return isGreen || isPurple;
    }

    private void initDrive(HardwareMap hw) {
        frontRight = hw.get(DcMotor.class, "frontRight");
        frontLeft = hw.get(DcMotor.class, "frontLeft");
        backLeft = hw.get(DcMotor.class, "backLeft");
        backRight = hw.get(DcMotor.class, "backRight");

        // Adjust to match your wiring
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveZeroPower();
    }

    private void initMechanisms(HardwareMap hw) {
        shooterMotor = hw.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        intakeMotor = hw.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRampMotor = hw.get(DcMotor.class, "intakeRampMotor");
        intakeRampMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterGate = hw.get(Servo.class, "shooterGate");
        shooterGate.setDirection(Servo.Direction.FORWARD);

        hoodAdjuster = hw.get(Servo.class, "hoodAdjuster");
        hoodAdjuster.setDirection(Servo.Direction.REVERSE);
        hoodAdjuster.scaleRange(0, 0.40);
        hoodAdjuster.setPosition(0);

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

    public void initVision(HardwareMap hw) {
        // Webcam name: "webcam1"
        tagService = new BroncoBoTAprilTagService(hw);
    }

    public void setTagID(int id) {
        TAG_ID_OF_INTEREST = id;
    }

    // ********** LOOP HELPERS **********

    public void handleParking(double now) {
        boolean startButton = gamepad2.start;

        if (startButton) {
            if (!startWasPressed) {
                startWasPressed = true;
                startPressedTime = now;
                parkingLatched = false;
            } else if (!parkingLatched &&
                    (now - startPressedTime) > SELECT_LONG_PRESS_SEC) {
                setDriveZeroPower();
                parkingLatched = true;
            }
        } else {
            startWasPressed = false;
            parkingLatched = false;
        }
    }

    public void handleYawReset(double now) {
        // Treat OPTIONS as "select"
        boolean select = gamepad1.options;

        if (select) {
            if (!selectWasPressed) {
                selectWasPressed = true;
                selectPressedTime = now;
                yawResetLatched = false;
            } else if (!yawResetLatched &&
                    (now - selectPressedTime) > SELECT_LONG_PRESS_SEC) {
                // Reset yaw: define current heading as field-forward
                fieldYawOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                yawResetLatched = true;
            }
        } else {
            selectWasPressed = false;
            yawResetLatched = false;
        }
    }

    /**
     * Handles shooter PID + feed-forward, hood angle, and auto-rotation
     * based on AprilTag pose while left trigger is held.
     *
     * @return auto-rotation command to add to driver rx
     */
    public double updateShooterAndTag(double leftTrigger, double dt) {
        double autoRotate = 0.0;

        // 1) Get tag pose
        BroncoBoTAprilTagService.TagPose pose = tagService.getTagPose(TAG_ID_OF_INTEREST);

        if (pose != null) {
            // Distance is X only (forward depth)
            distanceToTag = pose.getDistanceInches(); // |x|

            // Distance -> shooter velocity (ticks / sec)
            shooterTargetVelocity = mapDistanceToShooterVelocity(distanceToTag);

            // shooterTargetVelocity = TARGET_VELOCITY; // ticks/sec

            // Distance -> hood servo position
            double hoodPos = mapDistanceToHoodPosition(distanceToTag);
            // hoodServo.setPosition(hoodPos);
            double currentVelocity = shooterMotor.getVelocity(); // ticks / second

            double headingErrorDeg = pose.yawDeg;
            headingErrorDeg += 0;
            double kRotate = 0.02;
            autoRotate = Range.clip(kRotate * headingErrorDeg, -0.4, 0.4);
            
            // shoot on the move
            double y = -gamepad1.left_stick_y;
            double kMove = 0.05; 
            autoRotate += kMove * y;
            autoRotate = Range.clip(autoRotate, -0.5, 0.5);

            telemetry.addData("TagID", pose.id);
            telemetry.addData("Tag Z (m)", distanceToTag);
            telemetry.addData("Tag Heading Error (deg)", headingErrorDeg);
            telemetry.addData("Hood pos", hoodPos);
            telemetry.addData("shooter current Velocity", currentVelocity);
        } else {
            // No tag
            if (gamepad1.left_trigger > 0.1) {
                shooterTargetVelocity = TARGET_VELOCITY;
            }

            distanceToTag = 0.0;
        }

        if (leftTrigger <= 0.05) {
            // Shooter idle
            shooterTargetVelocity = 560.0;
            autoRotate = 0.0;
            shooterMotor.setVelocity(shooterTargetVelocity);
        } else {
            shooterMotor.setVelocity(shooterTargetVelocity);
        }

        telemetry.addData("shooter Target Velocity", shooterTargetVelocity);

        return autoRotate;
    }

    public void updateIntakeStage(boolean leftBumper, double rightTrigger, boolean gateControl) {
        double intakePower = 0.0;
        double rampPower = 0.0;

        // Left bumper: base 0.5 for intake + ramp
        if (leftBumper) {
            intakePower = 0.52;
            rampPower = 0.95;
        }

        // Right trigger: intake/ramp at least 0.3
        if (!leftBumper && rightTrigger > 0.05) {
            intakePower = 0.5;
            rampPower = 0.65;
        }

        // Right Bumper / main dPad down - Gate open close
        if (gateControl) {
            shooterGate.setPosition(0.30);
        } else {
            shooterGate.setPosition(0);
        }

        intakeMotor.setPower(intakePower);
        intakeRampMotor.setPower(rampPower);
    }

    private void driveFieldCentric(double x, double y, double rx, double autoRotate) {
        double yawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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

        if (gamepad1.left_trigger >= 0.05) {
            // slow down a bit if we are actively aiming
            fl *= 0.85;
            bl *= 0.85;
            fr *= 0.85;
            br *= 0.85;
        }

        frontRight.setPower(fr);
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void sendTelemetry() {
        double yawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double fieldHeading = yawRad - fieldYawOffsetRad;

        telemetry.addData("Field Yaw Wireless (deg)", Math.toDegrees(fieldHeading));
        telemetry.addData("Shooter target vel", shooterTargetVelocity);
        telemetry.addData("Shooter power", shooterMotor.getPower());
        telemetry.addData("Shooter current Velocity", shooterMotor.getVelocity());
        telemetry.addData("Intake power", intakeMotor.getPower());
        telemetry.addData("Ramp power", intakeRampMotor.getPower());
        telemetry.addData("Shooter Gate Position", shooterGate.getPosition());
        telemetry.addData("Tag Distance", distanceToTag);
        telemetry.addData("Shooter Direction", shooterGate.getDirection());
        telemetry.addData("current hood position", hoodAdjuster.getPosition());
        telemetry.addData("FPS", tagService.visionPortal.getFps());

        telemetry.update();
    }

    // ********** MAPPING HELPERS **********

    // Distance (Z) -> shooter wheel velocity (ticks / second).
    // *** Tune minDist/maxDist and minFrac/maxFrac on-robot ***
    private double mapDistanceToShooterVelocity(double distanceInches) {
        double velocity = 1000;
        velocity = 10 * distanceToTag + 720;
        return velocity;
    }

    // hood servo only accepts 0.0 to 1.0
    private double mapDistanceToHoodPosition(double distanceInches) {
        double minDist = 30.0; // closest shot
        double maxDist = 80.0; // farthest shot you care about
        double closeAngle = 0.7; // hood "up" (more arc)
        double farAngle = 0.3; // hood "down" (flatter)

        return 0.0;
    }

    // ********** STOP / UTILS **********

    public void stopDrive() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void stopMechanisms() {
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);
        intakeRampMotor.setPower(0);
    }

    public void setDriveZeroPower() {
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
