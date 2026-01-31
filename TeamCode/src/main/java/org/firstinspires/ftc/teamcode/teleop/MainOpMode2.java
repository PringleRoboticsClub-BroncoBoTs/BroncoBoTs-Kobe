package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@TeleOp(name = "ManualDrive-RED", group = "Iterative OpMode")
public class MainOpMode2 extends OpMode {

    public MainOpMode broncoBaseOpMode;
    // Single tag ID of interest (change as needed)
    private static final int TAG_ID_OF_INTEREST = 24;  // 20 - BLUE, 24 - RED

    // ********** END OF VARIABLES **********

    @Override
    public void init() {
        broncoBaseOpMode = new MainOpMode();
        broncoBaseOpMode.hardwareMap = hardwareMap;
        broncoBaseOpMode.telemetry = telemetry;
        broncoBaseOpMode.gamepad1 = gamepad1;
        broncoBaseOpMode.gamepad2 = gamepad2;
        broncoBaseOpMode.init();
        broncoBaseOpMode.setTagID(TAG_ID_OF_INTEREST);
    }

    @Override
    public void loop() {
        broncoBaseOpMode.loop();
    }

    @Override
    public void stop() {
        broncoBaseOpMode.stop();
    }
}