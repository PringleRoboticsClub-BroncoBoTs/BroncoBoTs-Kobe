package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "BIOBUZZ Auto - BLUE", group = "Competition", preselectTeleOp = "BIOBUZZ TeleOp - BLUE")
public class BlueAuto extends BioBuzzAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
