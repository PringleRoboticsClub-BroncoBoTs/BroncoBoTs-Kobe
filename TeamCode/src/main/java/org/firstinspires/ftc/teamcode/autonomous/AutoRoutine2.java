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

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 2: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 1) Move backwards 4 ft
        driveStraightInches(-FOUR_FEET_INCHES, 0.9);

        // 2) Turn right 20 deg
        turnDegrees(20.0, 0.6);

        // 3) Shoot for 3 sec
        shootForSeconds(3.0);

        stopShooter();
    }
}
