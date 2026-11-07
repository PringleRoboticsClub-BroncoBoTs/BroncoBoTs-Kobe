package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds servo set-points (Arm.STOWED / DEPLOYED, Gate.CLOSED and the gate's open angle).
 * Servos do NOT move until you press a dpad up/down or TRIANGLE/CROSS, so nothing jumps at start.
 *
 * gamepad1: dpad left/right = select servo
 *           dpad up/down    = +/- 0.01        TRIANGLE / CROSS = +/- 0.05
 *
 * "arm (both)" moves both arm servos together using Arm's direction settings, which is what you
 * want once they are mechanically aligned. Use the individual entries to align them first.
 */
@TeleOp(name = "Tune: Servo Positions", group = "Tuning")
public class ServoTuner extends OpMode {
    private final List<String> names = new ArrayList<>();
    private final List<Servo[]> servos = new ArrayList<>();
    private double[] positions;
    private boolean[] active;
    private int selected = 0;

    @Override
    public void init() {
        Servo left = tryGet(RobotConfig.ARM_LEFT);
        Servo right = tryGet(RobotConfig.ARM_RIGHT);
        if (left != null && right != null) {
            left.setDirection(Arm.LEFT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            right.setDirection(Arm.RIGHT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            add("arm (both)", left, right);
        }
        if (left != null) add(RobotConfig.ARM_LEFT, left);
        if (right != null) add(RobotConfig.ARM_RIGHT, right);
        Servo gate = tryGet(RobotConfig.GATE);
        if (gate != null) add(RobotConfig.GATE, gate);

        positions = new double[names.size()];
        active = new boolean[names.size()];
        for (int i = 0; i < positions.length; i++) positions[i] = 0.5;
    }

    @Override
    public void loop() {
        if (names.isEmpty()) {
            telemetry.addLine("No servos found - check the robot configuration names.");
            telemetry.update();
            return;
        }
        if (gamepad1.dpadRightWasPressed()) selected = (selected + 1) % names.size();
        if (gamepad1.dpadLeftWasPressed()) selected = (selected - 1 + names.size()) % names.size();

        double delta = 0;
        if (gamepad1.dpadUpWasPressed()) delta += 0.01;
        if (gamepad1.dpadDownWasPressed()) delta -= 0.01;
        if (gamepad1.triangleWasPressed()) delta += 0.05;
        if (gamepad1.crossWasPressed()) delta -= 0.05;
        if (delta != 0) {
            positions[selected] = Math.max(0, Math.min(1, positions[selected] + delta));
            active[selected] = true;
        }

        for (int i = 0; i < names.size(); i++) {
            if (!active[i]) continue;
            for (Servo s : servos.get(i)) s.setPosition(positions[i]);
        }

        for (int i = 0; i < names.size(); i++) {
            telemetry.addLine(String.format("%s %-14s %s", i == selected ? ">>" : "  ", names.get(i),
                    active[i] ? String.format("%.3f", positions[i]) : "(not driven)"));
        }
        telemetry.addLine("\ndpad L/R select | dpad U/D +/-0.01 | TRIANGLE/CROSS +/-0.05");
        telemetry.update();
    }

    private void add(String name, Servo... s) {
        names.add(name);
        servos.add(s);
    }

    private Servo tryGet(String name) {
        try {
            return hardwareMap.get(Servo.class, name);
        } catch (IllegalArgumentException e) {
            return null;
        }
    }
}
