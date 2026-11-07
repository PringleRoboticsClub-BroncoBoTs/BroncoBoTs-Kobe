package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "BIOBUZZ TeleOp - RED", group = "Competition")
public class RedTeleOp extends BioBuzzTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
