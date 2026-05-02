package org.firstinspires.ftc.teamcode.autonomous.AutoClose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autonomous.BroncoBotAutoBase;
/**
 * Auto Routine 2 using encoder ticks:
 *  1) Move Backward 3 ft
 *  2) Shoot for 2 sec
 *  3) Turn right 45 deg
 *  4) Move straight 3ft with intake active
 *  5) move backward 3ft
 *  6) Turn left 45 deg
 *  7) shoot for 2 sec
 *  8) Move straight 1 ft
 */
@Autonomous(name = "Auto Routine 4 - RedClose-1Ball", group = "Autonomous")
public class RedClose_1Ball extends BroncoBotAutoBase {

    private static final double SIX_FEET_INCHES = 72.0;
    private static final double THREE_FEET_INCHES = 36.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 2: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        driveStraightInches(-(THREE_FEET_INCHES + 6), 0.6);

        shootForSeconds(2.0);
        shooterMotor.setVelocity(1800);
        sleep((long) (2 * 1000));
        stopShooting();

        turnDegrees(45, 0.75);

        startIntake(0.75, 0.5);
        driveStraightInches(THREE_FEET_INCHES + 6, 0.6);

        stopIntake();

        driveStraightInches(-(THREE_FEET_INCHES + 6), 0.6);

        turnDegrees(-45, 0.75);

        shootForSeconds(2.0);
        shooterMotor.setVelocity(1800);
        sleep((long) (2 * 1000));
        stopShooting();

        strafeInches(-24, 0.6);
    }
}