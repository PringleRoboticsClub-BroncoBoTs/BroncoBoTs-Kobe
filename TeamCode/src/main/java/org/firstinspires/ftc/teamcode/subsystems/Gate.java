package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Servo gate that blocks balls from entering the turret/flywheel until we want to shoot.
 *
 * CLOSED = home = servo position 0 (commanded at INIT). OPEN = OPEN_DEG (30 deg) LEFT of home.
 * Because home is the end of the servo's travel, opening always moves toward higher positions.
 * If the gate turns RIGHT when opening, the servo horn needs re-mounting on the other side of its
 * travel, then set REVERSED = true.
 */
@Config
public class Gate {
    public static double CLOSED = 0.0;           // home position
    public static double OPEN_DEG = 30;          // how far the gate swings open
    public static double SERVO_RANGE_DEG = 300;  // goBILDA 2000-series servo travel for 0 -> 1 TODO(verify)
    public static boolean REVERSED = false;

    private final Servo servo;
    private boolean open = false;

    public Gate(HardwareMap hw) {
        servo = hw.get(Servo.class, RobotConfig.GATE);
        servo.setDirection(REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo.setPosition(CLOSED); // home at INIT
    }

    /** Servo position for the open gate: OPEN_DEG past home. */
    public static double openPosition() {
        return Math.min(1.0, CLOSED + OPEN_DEG / SERVO_RANGE_DEG);
    }

    public void open() { open = true; }
    public void close() { open = false; }
    public void set(boolean open) { this.open = open; }
    public boolean isOpen() { return open; }

    public void setPosition(double pos) { servo.setPosition(pos); }

    public void update() {
        servo.setPosition(open ? openPosition() : CLOSED);
    }

    public void addTelemetry(Telemetry t) {
        t.addData("gate/open", open);
    }
}
