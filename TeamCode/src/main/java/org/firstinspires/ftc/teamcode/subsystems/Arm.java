package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Front arm driven by two servos on opposite sides. The right servo is reversed so both receive the
 * SAME position value. Find the numbers with "Tune: Servo Positions".
 *
 * Ensure the two servos are mechanically aligned at one position before attaching the horns, or
 * they will fight each other (stall current / heat).
 */
@Config
public class Arm {
    public static double STOWED = 0.10;    // TODO(tune)
    public static double DEPLOYED = 0.60;  // TODO(tune)
    public static boolean LEFT_REVERSED = false;
    public static boolean RIGHT_REVERSED = true;
    /** Small per-servo trim if the pair isn't perfectly aligned. */
    public static double RIGHT_TRIM = 0.0;

    private final Servo left;
    private final Servo right;
    private boolean deployed = false;

    public Arm(HardwareMap hw) {
        left = hw.get(Servo.class, RobotConfig.ARM_LEFT);
        right = hw.get(Servo.class, RobotConfig.ARM_RIGHT);
        left.setDirection(LEFT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        right.setDirection(RIGHT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    public void stow() { deployed = false; }
    public void deploy() { deployed = true; }
    public void toggle() { deployed = !deployed; }
    public boolean isDeployed() { return deployed; }

    /** Directly sets both servos (used by the servo tuner). */
    public void setPosition(double pos) {
        left.setPosition(pos);
        right.setPosition(Math.max(0, Math.min(1, pos + RIGHT_TRIM)));
    }

    public void update() {
        setPosition(deployed ? DEPLOYED : STOWED);
    }

    public void addTelemetry(Telemetry t) {
        t.addData("arm/deployed", deployed);
    }
}
