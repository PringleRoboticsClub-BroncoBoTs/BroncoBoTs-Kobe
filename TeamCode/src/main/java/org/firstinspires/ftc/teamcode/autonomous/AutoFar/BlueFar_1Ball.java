package org.firstinspires.ftc.teamcode.autonomous.AutoFar;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autonomous.BroncoBotAutoBase;

/**
 * Auto Routine 2 using encoder ticks:
 *  1) shoot for 2 sec
 *  2) Move straight 3ft
 *  3) Turn left 90 deg
 *  4) Move straight 3ft with intake active
 *  5) move backward 3ft
 *  6) turn left 90 deg
 *  7) Move backward 3 ft
 *  6)shoot for 2 sec
 *  7) Move straight 2 ft
 *
 */
@Autonomous(name = "Auto Routine 2 - Blue Far - 1 ball", group = "Autonomous")
public class BlueFar_1Ball extends BroncoBotAutoBase {
    private static final double FOUR_FEET_INCHES = 48.0;
    private static final double SIX_FEET_INCHES  = 72.0;
    private static final double THREE_FEET_INCHES = 36.0;
    private static final double TWO_FEET_INCHES = 24.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 2: Ready");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        startShooter();

        startShooter();
        sleep((long) (2 * 1000));
        shooterMotor.setVelocity(3000);

        driveStraightWithEncoderTurn(THREE_FEET_INCHES, -90.0, 0.75);

        startIntake(0.75, 0.5);
        driveStraightInches(THREE_FEET_INCHES + 6, 0.6);

        stopIntake();

        driveStraightInches(-(THREE_FEET_INCHES + 6), 0.6);

        turnDegrees(90, 0.75);

        driveStraightInches(-(THREE_FEET_INCHES + 6), 0.75);

        shootForSeconds(2.0);
        sleep((long) (2 * 1000));
        shooterMotor.setVelocity(3000);

        driveStraightWithEncoderTurn(THREE_FEET_INCHES, 90, 0.75);
    }
}