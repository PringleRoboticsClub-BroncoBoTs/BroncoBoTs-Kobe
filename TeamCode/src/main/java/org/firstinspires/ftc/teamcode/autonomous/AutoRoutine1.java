package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Auto Routine 1 using encoder ticks:
 *
 * 1) Drive straight 6 ft
 * 2) Turn left 45 deg
 * 3) Shoot for 3 sec
 * 4) Turn right 112.5 deg
 * 5) Move back 4 ft with intake active
 * 6) Stop intake
 * 7) Drive forward 4 ft, then turn left 112.5 deg
 * 8) Shoot for 3 sechttp://192.168.43.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/autonomous/AutoRoutine2.java
 */
@Autonomous(name = "Auto Routine 1 - Blue", group = "Autonomous")
public class AutoRoutine1 extends BroncoBotAutoBase {

    private static final double SIX_FEET_INCHES  = 72.0;
    private static final double FOUR_FEET_INCHES = 48.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 1 - Encoders: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 1) Drive straight 6 ft
        driveStraightInches(-SIX_FEET_INCHES, 0.5);

        // 2) Turn left 70 deg
        turnDegrees(-70.0, 0.4);

        // 3) Shoot for 3 sec
        shootForSeconds(6.0);

        // 4) Turn right 112.5 deg
        // turnDegrees(112.5, 0.4);

        // 5) Move back 4 ft while intake active
        //startIntake(0.5, 0.5);
        //driveStraightInches(FOUR_FEET_INCHES, 0.5);

        // 6) Stop intake
        //stopIntake();

        // 7) Drive forward 4 ft
        //driveStraightInches(FOUR_FEET_INCHES, 0.5);

        //    Then turn left 112.5 deg (approx equivalent of "while turning")
        // turnDegrees(112.5, 0.4);

        // 8) Shoot for 3 sec
        // shootForSeconds(3.0);

        // End: everything should already be stopped by helpers
    }
}
