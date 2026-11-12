package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.vision.GoalVision;
import org.firstinspires.ftc.teamcode.vision.TurretAimer;

import java.util.List;

/**
 * Vision + auto-aim check. Turret must face forward at INIT (it is zeroed).
 *
 * Shows the goal cluster detection, the fused field-frame goal estimate, and (on the FTC Dashboard
 * Field view) the robot, turret direction and goal estimate. Push the robot around / spin it by
 * hand to verify the turret stays locked on the goal.
 *
 * gamepad1 (PS5): SQUARE = BLUE, CIRCLE = RED, CROSS = turret auto-aim on/off (starts OFF so you can check vision first)
 */
@TeleOp(name = "Tune: Vision + Auto-Aim", group = "Tuning")
public class AimTuner extends OpMode {
    private List<LynxModule> hubs;
    private GoalVision vision;
    private PinpointLocalizer localizer;
    private Turret turret;
    private final TurretAimer aimer = new TurretAimer();
    private boolean aimOn = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        vision = new GoalVision(hardwareMap, PoseStorage.alliance);
        localizer = Constants.createLocalizer(hardwareMap);
        localizer.setPose(new Pose(0, 0, 0));
        turret = new Turret(hardwareMap, true);
        PoseStorage.turretZeroed = true;
        turret.setEnabled(false);
    }

    @Override
    public void loop() {
        for (LynxModule h : hubs) h.clearBulkCache();
        localizer.update();
        Pose pose = localizer.pose();

        if (gamepad1.squareWasPressed()) { vision.setAlliance(Alliance.BLUE); aimer.forget(); }
        if (gamepad1.circleWasPressed()) { vision.setAlliance(Alliance.RED); aimer.forget(); }
        if (gamepad1.crossWasPressed()) {
            aimOn = !aimOn;
            turret.setEnabled(aimOn);
        }

        GoalVision.Observation obs = vision.update();
        aimer.update(pose, turret.getAngleDeg(), obs);
        if (aimOn && aimer.hasTarget()) {
            turret.setTargetDeg(aimer.getDesiredTurretDeg());
            turret.setFeedforwardDegPerSec(aimer.getFeedforwardDegPerSec());
        } else {
            turret.setFeedforwardDegPerSec(0);
        }
        turret.update();

        telemetry.addData("auto-aim", aimOn ? "ON" : "OFF (A to enable)");
        telemetry.addData("pose", "x %.1f y %.1f h %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        vision.addTelemetry(telemetry);
        aimer.addTelemetry(telemetry);
        turret.addTelemetry(telemetry);
        telemetry.update();

        // Field drawing on FTC Dashboard (inches; relative to where this OpMode started).
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("aim/desiredDeg", aimer.getDesiredTurretDeg());
        packet.put("turret/angleDeg", turret.getAngleDeg());
        packet.put("vision/bearingDeg", obs != null ? obs.bearingDeg : Double.NaN);
        Canvas c = packet.fieldOverlay();
        double h = pose.heading();
        c.setStroke("#3F51B5").strokeCircle(pose.x(), pose.y(), 9)
                .strokeLine(pose.x(), pose.y(), pose.x() + 9 * Math.cos(h), pose.y() + 9 * Math.sin(h));
        double th = h + Math.toRadians(turret.getAngleDeg());
        c.setStroke("#FF9800").strokeLine(pose.x(), pose.y(), pose.x() + 30 * Math.cos(th), pose.y() + 30 * Math.sin(th));
        if (aimer.hasTarget()) {
            c.setFill(vision.getAlliance() == Alliance.RED ? "#F44336" : "#2196F3")
                    .fillCircle(aimer.getGoalX(), aimer.getGoalY(), 3);
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        turret.setEnabled(false);
        vision.close();
    }
}
