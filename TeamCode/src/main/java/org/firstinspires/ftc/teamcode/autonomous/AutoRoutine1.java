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
@Autonomous(name = "Auto Routine 1 - Blue Far", group = "Autonomous")
public class AutoRoutine1 extends BroncoBotAutoBase {

    private static final double SIX_FEET_INCHES  = 72.0;
    private static final double THREE_FEET_INCHES = 36.0;
    private static final double TWO_FEET_INCHES = 24.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 1 - Encoders: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 1) Drive straight 6 ft
        driveStraightInches(-SIX_FEET_INCHES, 0.7);

        // 2) Turn left 80 deg
        turnDegrees(-80.0, 0.6);

        // 3) Shoot and wait for 3 sec
        shootForSeconds(3.0);

        // 4) Turn right 250 deg
        turnDegrees(250, 0.6);

        // 5) Move back 2 ft while intake active
        startIntake(0.5, 0.5);
        driveStraightInches(THREE_FEET_INCHES, 0.4);

        // 6) Stop intake
        stopIntake();

        // 7) Drive forward 2 ft
        driveStraightInches(-THREE_FEET_INCHES+4, 0.7);

        //    Then turn left 250 deg (approx equivalent of "while turning")
        turnDegrees(-250, 0.6);

        // 8) Shoot and wait for 3 sec
        shootForSeconds(3.0);

        //    Then turn left 45 deg
        turnDegrees(-45,0.6);

        // Drive forward 2 ft
        driveStraightInches(TWO_FEET_INCHES, 0.7);

        // End: everything should already be stopped by helpers
    }
}
