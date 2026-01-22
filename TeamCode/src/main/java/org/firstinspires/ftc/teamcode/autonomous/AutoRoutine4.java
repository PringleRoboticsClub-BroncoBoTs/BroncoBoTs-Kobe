package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Auto Routine 2 using encoder ticks:
 *
 * 1) Move backwards 4 ft
 * 2) Turn right 20 deg
 * 3) Shoot for 3 sec
 */
@Autonomous(name = "Auto Routine 4 - Red Near", group = "Autonomous")
public class AutoRoutine4 extends BroncoBotAutoBase {

    private static final double FOUR_FEET_INCHES = 48.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 2: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        startShooter();

        // 1) Move backwards 2.5 ft
        driveStraightInches(30.0, 0.75);

        // 2) Turn left 20 deg
       // turnDegrees(20.0, 0.4);

        // 3) Shoot for 3 sec
        shootForSeconds(3.0);

        strafeInches(12.0, 0.5);

        stopShooter();
    }
}
