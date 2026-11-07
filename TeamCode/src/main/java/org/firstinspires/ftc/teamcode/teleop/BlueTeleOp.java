package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "BIOBUZZ TeleOp - BLUE", group = "Competition")
public class BlueTeleOp extends BioBuzzTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
