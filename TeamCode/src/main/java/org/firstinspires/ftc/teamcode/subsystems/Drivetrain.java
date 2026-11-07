package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Field-centric mecanum drive, extracted unchanged from the working ManualDrive OpMode on the
 * program_drivetrain branch (same motor names, directions and mixing math).
 * Only used in TeleOp - Pedro Pathing drives the motors in Autonomous.
 *
 * IMU: the Control Hub lies FLAT with its logo/label facing UP and its USB ports pointing to the
 * BACK of the robot. If the hub is ever remounted, change HUB_LOGO / HUB_USB to match, or
 * field-centric drive will be rotated (e.g. pushing forward drives sideways).
 */
public class Drivetrain {
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    private final DcMotor frontRight, frontLeft, backLeft, backRight;
    private final IMU imu;
    private double fieldYawOffsetRad;

    public Drivetrain(HardwareMap hw) {
        frontRight = hw.get(DcMotor.class, RobotConfig.FRONT_RIGHT);
        frontLeft  = hw.get(DcMotor.class, RobotConfig.FRONT_LEFT);
        backLeft   = hw.get(DcMotor.class, RobotConfig.BACK_LEFT);
        backRight  = hw.get(DcMotor.class, RobotConfig.BACK_RIGHT);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontRight, frontLeft, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hw.get(IMU.class, RobotConfig.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        resetFieldYaw();
    }

    /** Defines the robot's current heading as field-forward. */
    public void resetFieldYaw() {
        fieldYawOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Sets field-forward to be {@code headingRad} away from the robot's current heading. */
    public void setFieldHeading(double headingRad) {
        fieldYawOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingRad;
    }

    public double getFieldHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldYawOffsetRad;
    }

    /** x = strafe (right +), y = forward (+), rx = rotate (clockwise +), all -1..1. */
    public void driveFieldCentric(double x, double y, double rx) {
        double fieldHeading = getFieldHeadingRad();
        double cosA = Math.cos(-fieldHeading);
        double sinA = Math.sin(-fieldHeading);

        double rotatedX = x * cosA - y * sinA;
        double rotatedY = x * sinA + y * cosA;
        double finalRx = Range.clip(rx, -1.0, 1.0);

        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(finalRx), 1.0);

        frontLeft.setPower((rotatedY + rotatedX + finalRx) / denominator);
        backLeft.setPower((rotatedY - rotatedX + finalRx) / denominator);
        frontRight.setPower((rotatedY - rotatedX - finalRx) / denominator);
        backRight.setPower((rotatedY + rotatedX - finalRx) / denominator);
    }

    public void stop() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void addTelemetry(Telemetry t) {
        t.addData("drive/fieldYawDeg", Math.toDegrees(getFieldHeadingRad()));
    }
}
