package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Intake roller: goBILDA 5203 1620/1650 RPM (3.7:1). Also feeds balls up to the turret. */
@Config
public class Intake {
    public static boolean REVERSED = true;  // motor was spinning backwards; + power must pull balls in
    public static double INTAKE_POWER = 0.9;
    public static double FEED_POWER = 0.8;  // when pushing balls past the open gate into the flywheel
    public static double OUTTAKE_POWER = -0.6; // reverse: spits balls back out the front (clears jams)

    public enum State { OFF, INTAKE, FEED, OUTTAKE }

    private final DcMotorEx motor;
    private State state = State.OFF;

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, RobotConfig.INTAKE);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void set(State s) {
        state = s;
    }

    public State getState() { return state; }

    public void update() {
        double p;
        switch (state) {
            case INTAKE:  p = INTAKE_POWER;  break;
            case FEED:    p = FEED_POWER;    break;
            case OUTTAKE: p = OUTTAKE_POWER; break;
            default:      p = 0;
        }
        motor.setPower(p);
    }

    public void addTelemetry(Telemetry t) {
        t.addData("intake/state", state);
    }
}
