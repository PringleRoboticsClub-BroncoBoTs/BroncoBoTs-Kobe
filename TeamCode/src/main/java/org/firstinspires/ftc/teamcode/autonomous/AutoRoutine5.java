package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Auto Routine 5 using encoder ticks:
 * Blue Far shots
 */
@Autonomous(name = "Auto Routine 1 - Blue Far", group = "Autonomous")
public class AutoRoutine1 extends BroncoBotAutoBase {

    private static final double SIX_FEET_INCHES  = 72.0;
    private static final double THREE_FEET_INCHES = 36.0;
    private static final double FOUR_FEET_INCHES  = 48.0;
    private static final double TWO_FEET_INCHES = 24.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 5 - Encoders: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        setShooterVelocity(1500);
        sleep((long) (0.5 * 1000));

        turnDegrees(120.0, 0.5);
        shootForSeconds(3.0);

        turnDegrees(-120.0, 0.5);
        driveStraightWithEncoderTurn(24, 0.0, 0.9);

        driveStraightInches(-40, 0.6);
        driveStraightInches(40, 0.6);

        driveStraightWithEncoderTurn(-24, 120.0, 0.8);

        shootForSeconds(3.0);

        turnDegrees(-120.0, 0.5);
        driveStraightWithEncoderTurn(48, 0.0, 0.9);

        driveStraightInches(-40, 0.6);
        driveStraightInches(40, 0.6);

        driveStraightWithEncoderTurn(-48, 120.0, 0.8);

        shootForSeconds(3.0);
  
    }
}
