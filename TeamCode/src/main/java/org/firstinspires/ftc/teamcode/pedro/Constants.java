package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Pedro Pathing 3.x configuration for the BIOBUZZ robot.
 *
 * HOW TO TUNE (do these in order, the AutoTune web UI generates the code for you):
 *   1. Connect to the robot Wi-Fi, open http://192.168.43.1:10158 (Pedro AutoTune).
 *   2. Run "Mecanum Tuner"  -> paste the generated MecanumConfig below.
 *   3. Run "Pinpoint Tuner" -> paste the generated PinpointConfig below.
 *   4. Run "Foresight Tuner" -> paste the generated ForesightConfig below.
 *   5. Run "Tests" (Line / Curve / Hold) to confirm.
 *
 * Every value marked TODO(tune) is a placeholder from the Pedro quickstart and MUST be re-tuned.
 */
public class Constants {

    /** Motor names/directions match the working field-centric TeleOp from program_drivetrain. */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConfig.FRONT_LEFT);
        c.backLeftName.set(RobotConfig.BACK_LEFT);
        c.frontRightName.set(RobotConfig.FRONT_RIGHT);
        c.backRightName.set(RobotConfig.BACK_RIGHT);

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    /** goBILDA Pinpoint odometry computer. Offsets are in inches from the robot's center of rotation. */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(RobotConfig.PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); // TODO(tune): SWINGARM if applicable
        c.xPodOffset.set(0.0);  // TODO(tune) forward pod's sideways offset (left = +)
        c.yPodOffset.set(0.0);  // TODO(tune) strafe pod's forward offset (forward = +)
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // TODO(tune)
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // TODO(tune)
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /** Path-following algorithm. ALL numbers below are quickstart placeholders. TODO(tune) via Foresight Tuner. */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryForward   = Controller.proportional(0.3);
        Controller secondaryForward = Controller.proportional(0.1);
        Controller primaryStrafe    = Controller.proportional(0.3);
        Controller secondaryStrafe  = Controller.proportional(0.1);

        c.forwardTranslational.set(Controller.piecewise(secondaryForward).put(2.5, primaryForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryStrafe).put(2.5, primaryStrafe));

        c.coast.set(Controller.proportionalFeedforward(0.011));
        c.brake.set(Controller.proportionalFeedforward(0.0087));

        c.headingFeedback.set(Controller.proportional(5.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.056, 0.0064));

        c.linearBrakeCoefficients.set(Matrix.diag(0.106, 0.087));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00147, 0.00138));

        c.maxAchievableForwardVelocity.set(70.0);
        c.maxAchievableStrafeVelocity.set(52.0);
        c.naturalForwardDeceleration.set(85.0);
        c.naturalStrafeDeceleration.set(100.0);
    });

    public static PinpointLocalizer createLocalizer(HardwareMap h) {
        return new PinpointLocalizer(h, localizerConfig);
    }

    public static Follower create(HardwareMap h) {
        return new Follower(createLocalizer(h), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}
