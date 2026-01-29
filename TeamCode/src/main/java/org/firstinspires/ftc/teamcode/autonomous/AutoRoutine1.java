package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Auto Routine 1 using encoder ticks:
 *
 * 1) Drive straight 6 ft
 * 2) Turn left 80 deg
 * 3) Shoot for 2 sec
 * 4) Turn right 250 deg
 * 5) Move back 3ft+1 with intake active
 * 6) Stop intake
 * 7) Drive forward 3ft+8
 * 8) Turn left 250 deg
 * 9) Shoot for 2 sec
 * 10) Turn right 250 deg
 * 11) Strafe 2ft+3 right
 * 12) Move back 3ft+2 with intake active
 * 13) Stop intake
 * 14) Drive forward 4ft
 * 15) Strafe 2ft+5 left
 * 16) Turn left 250 deg
 * 17) Shoot for 2 sec
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

        telemetry.addLine("Auto Routine 1 - Encoders: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 1) Drive straight 6 ft
        //driveStraightInches(-SIX_FEET_INCHES, 0.7);

        // 2) Turn left 80 deg
        //turnDegrees(-80.0, 0.6);

        startShooter();
        sleep((long) (0.5 * 1000));

        driveStraightWithEncoderTurn(SIX_FEET_INCHES, -80.0, 0.9);

        // 3) Shoot and wait for 2 sec
        shootForSeconds(2.0);

        // 4) Turn right 250 deg
        turnDegrees(250, 0.9);

        // 5) Move back 3 ft while intake active
        startIntake(0.75, 0.5);
        driveStraightInches(THREE_FEET_INCHES+1, 0.8);

        // 6) Stop intake
        stopIntake();

        // 7) Drive forward 3.5 ft
        driveStraightInches(-THREE_FEET_INCHES+8, 0.9);

        //    Then turn left 250 deg (approx equivalent of "while turning")
        turnDegrees(-250, 0.9);

        // 8) Shoot and wait for 2 sec
        shootForSeconds(2.0);

        turnDegrees(250, 0.9);

        strafeInches(-(TWO_FEET_INCHES+3), 0.9);

        startIntake(0.75, 0.5);
        driveStraightInches(THREE_FEET_INCHES+2, 0.9);
        stopIntake();

        driveStraightInches(-FOUR_FEET_INCHES, 0.9);
        strafeInches(TWO_FEET_INCHES+5, 0.9);

        turnDegrees(-250, 0.9);
        shootForSeconds(2.0);

        strafeInches(-TWO_FEET_INCHES, 0.9);

        turnDegrees(250, 0.9);
        strafeInches(-FOUR_FEET_INCHES, 0.9);

        startIntake(0.75, 0.75);
        driveStraightInches(THREE_FEET_INCHES, 0.6);
        stopIntake();

        driveStraightInches(-THREE_FEET_INCHES+4, 0.8);

        strafeInches(FOUR_FEET_INCHES, 0.9);
        turnDegrees(-250, 0.8);
        shootForSeconds(2.0);

        turnDegrees(250, 0.6);
        strafeInches(-TWO_FEET_INCHES, 0.7);

        stopShooter();

        // End: everything should already be stopped by helpers
    }
}
