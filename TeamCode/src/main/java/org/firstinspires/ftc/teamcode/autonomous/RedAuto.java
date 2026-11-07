package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "BIOBUZZ Auto - RED", group = "Competition", preselectTeleOp = "BIOBUZZ TeleOp - RED")
public class RedAuto extends BioBuzzAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
