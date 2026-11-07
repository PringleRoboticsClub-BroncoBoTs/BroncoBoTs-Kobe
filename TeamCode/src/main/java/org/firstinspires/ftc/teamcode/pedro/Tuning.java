package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Registers Pedro AutoTune procedures. They appear in the AutoTune web UI
 * (http://192.168.43.1:10158 while connected to the robot), not on the Driver Station list.
 */
public class Tuning {
    @Tuner(name = "1. Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "3. Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants::createLocalizer, h -> new Mecanum(h, Constants.drivetrainConfig));
    }

    @Tuner(name = "4. Tests")
    public static Procedure tests() {
        return new Tests(h -> new Mecanum(h, Constants.drivetrainConfig), Constants::createLocalizer,
                () -> new Foresight(Constants.foresightConfig));
    }
}
