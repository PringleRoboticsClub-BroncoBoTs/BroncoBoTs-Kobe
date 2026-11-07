package org.firstinspires.ftc.teamcode.config;

/**
 * Hardware-map names for the BIOBUZZ robot. These MUST match the names in the
 * Driver Station robot configuration exactly (case sensitive).
 *
 * Motors
 *   frontLeft / frontRight / backLeft / backRight  goBILDA drive motors (program_drivetrain wiring)
 *   shooterMotor   goBILDA 5203 6000 RPM (1:1), drives the 96 mm flywheel via 2 mm GT2 belt
 *   turretMotor    goBILDA 5203 117 RPM (50.9:1) - only when Turret.ACTUATOR == MOTOR
 *   intakeMotor    goBILDA 5203 1620/1650 RPM (3.7:1)
 * Servos
 *   armLeft / armRight   arm in front of the robot (mirrored pair)
 *   gateServo            gate that stops balls from entering the turret
 *   turretServo          only when Turret.ACTUATOR == SERVO
 * I2C
 *   pinpoint   goBILDA Pinpoint odometry computer (configure as "goBILDA Pinpoint Odometry Computer")
 *   imu        Control Hub IMU (used by field-centric drive)
 * USB
 *   Webcam 1   global-shutter USB camera
 */
public final class RobotConfig {
    private RobotConfig() {}

    // Drive (names unchanged from the program_drivetrain branch)
    public static final String FRONT_LEFT  = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT   = "backLeft";
    public static final String BACK_RIGHT  = "backRight";
    public static final String IMU         = "imu";
    public static final String PINPOINT    = "pinpoint";

    // Mechanisms
    public static final String SHOOTER      = "shooterMotor";
    public static final String TURRET_MOTOR = "turretMotor";
    public static final String TURRET_SERVO = "turretServo";
    public static final String INTAKE       = "intakeMotor";
    public static final String ARM_LEFT     = "armLeft";
    public static final String ARM_RIGHT    = "armRight";
    public static final String GATE         = "gateServo";

    // Vision
    public static final String WEBCAM = "Webcam 1";
}
