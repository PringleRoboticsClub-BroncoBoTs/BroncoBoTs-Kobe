package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.VoltageMonitor;

import java.util.List;

/**
 * Flywheel PID tuning + visualization.
 *
 * 1. Open FTC Dashboard: connect to robot Wi-Fi, browse to http://192.168.43.1:8080/dash
 * 2. Start this OpMode. In the Graph view, plot shooter/targetRpm, shooter/rpm, shooter/power.
 * 3. Edit gains on the "Shooter" config card live; edit test targets on the "FlywheelTuner" card.
 *
 * gamepad1 (PS5):  CROSS = flywheel on/off   SQUARE = step mode on/off (square wave LOW <-> HIGH)
 *            dpad up/down = TARGET_RPM +/-100
 *            R1 (hold) = feed balls (open gate + run intake) to see recovery after shots
 */
@Config
@TeleOp(name = "Tune: Flywheel", group = "Tuning")
public class FlywheelTuner extends OpMode {
    public static double TARGET_RPM = 3000;
    public static double STEP_LOW_RPM = 2500;
    public static double STEP_HIGH_RPM = 3500;
    public static double STEP_PERIOD_SEC = 3.0;

    private List<LynxModule> hubs;
    private Shooter shooter;
    private Gate gate;
    private Intake intake;
    private Telemetry tel;
    private boolean on = false;
    private boolean stepMode = false;

    @Override
    public void init() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        shooter = new Shooter(hardwareMap, new VoltageMonitor(hardwareMap));
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);
        tel.addLine("CROSS: on/off   SQUARE: step mode   dpad: +/-100 rpm   R1: feed");
        tel.update();
    }

    @Override
    public void loop() {
        for (LynxModule h : hubs) h.clearBulkCache();

        if (gamepad1.crossWasPressed()) on = !on;
        if (gamepad1.squareWasPressed()) stepMode = !stepMode;
        if (gamepad1.dpadUpWasPressed()) TARGET_RPM += 100;
        if (gamepad1.dpadDownWasPressed()) TARGET_RPM = Math.max(0, TARGET_RPM - 100);

        double target = TARGET_RPM;
        if (stepMode) {
            boolean high = ((long) (getRuntime() / STEP_PERIOD_SEC)) % 2 == 1;
            target = high ? STEP_HIGH_RPM : STEP_LOW_RPM;
        }
        shooter.setTargetRpm(on ? target : 0);

        boolean feed = gamepad1.right_bumper;
        gate.set(feed);
        intake.set(feed ? Intake.State.FEED : Intake.State.OFF);

        shooter.update();
        gate.update();
        intake.update();

        tel.addData("mode", stepMode ? "STEP" : "CONSTANT");
        tel.addData("on", on);
        shooter.addTelemetry(tel);
        tel.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        shooter.update();
    }
}
