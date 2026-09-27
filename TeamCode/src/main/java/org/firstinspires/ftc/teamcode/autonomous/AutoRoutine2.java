package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Auto Routine 2 using encoder ticks:
 *
 * 1) Move backwards 4 ft
 * 2) Turn right 20 deg
 * 3) Shoot for 3 sec
 */
@Autonomous(name = "Auto Routine 2 - Blue Near", group = "Autonomous")
public class AutoRoutine2 extends BroncoBotAutoBase {

    private static final double FOUR_FEET_INCHES = 48.0;
    private static final double THREE_FEET_INCHES = 36.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 2: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        startShooter();

        startIntake(0.1, 0.1);

        // 1) Move backwards 4 ft
        driveStraightInches(-FOUR_FEET_INCHES, 0.9);

        // 2) Turn right 20 deg
        //turnDegrees(20.0, 0.6);

        // 3) Shoot for 3 sec
        shootForSeconds(3.0);

        turnDegrees(-80, 0.9);

        // 5) Move back 3 ft while intake active
        startIntake(0.75, 0.75);
        driveStraightInches((THREE_FEET_INCHES+3), 0.8);

        // 6) Stop intake
        stopIntake();

        // 7) Drive forward 3.5 ft
        driveStraightInches(-(THREE_FEET_INCHES+3), 0.9);

        //    Then turn left 250 deg (approx equivalent of "while turning")
        turnDegrees(70, 0.9);

        // Make hood position 40 degrees


        // 8) Shoot and wait for 2 sec
        shootForSeconds(2.0);

        turnDegrees(-60, 0.9);

        strafeInches(-28, 0.9);

        startIntake(0.75, 0.75);
        driveStraightInches((THREE_FEET_INCHES+2), 0.9);
        stopIntake();

        driveStraightInches(-FOUR_FEET_INCHES, 0.9);

       // strafeInches(-12, 0.5);

        stopShooter();
    }
}