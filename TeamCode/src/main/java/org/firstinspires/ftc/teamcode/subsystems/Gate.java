package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Servo gate that blocks balls from entering the turret/flywheel until we want to shoot. */
@Config
public class Gate {
    public static double CLOSED = 0.5; // TODO(tune) (last season's shooterGate values as a start)
    public static double OPEN = 0.0;   // TODO(tune)

    private final Servo servo;
    private boolean open = false;

    public Gate(HardwareMap hw) {
        servo = hw.get(Servo.class, RobotConfig.GATE);
    }

    public void open() { open = true; }
    public void close() { open = false; }
    public void set(boolean open) { this.open = open; }
    public boolean isOpen() { return open; }

    public void setPosition(double pos) { servo.setPosition(pos); }

    public void update() {
        servo.setPosition(open ? OPEN : CLOSED);
    }

    public void addTelemetry(Telemetry t) {
        t.addData("gate/open", open);
    }
}
