package org.firstinspires.ftc.teamcode.autonomous.AutoFar;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autonomous.BroncoBotAutoBase;

/**
 * Auto Routine 2 using encoder ticks:
 *  1) shoot for 2 sec
 *  2)Move straight 3ft
 *  3) Turn right 90 deg
 *  4) Move straight 3ft with intake active
 *  5) move backward 3ft
 *  6) Turn left 90 deg
 *  7) Move backward 3 ft
 *  7) shoot for 2 sec
 *  8) Move straight
 */
@Autonomous(name = "Auto Routine 4 - RedFar-1Ball", group = "Autonomous")
public class RedFar_1Ball extends BroncoBotAutoBase {

    private static final double SIX_FEET_INCHES = 72.0;
    private static final double THREE_FEET_INCHES = 36.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 2: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        startShooter();
        sleep((long) (0.5 * 1000));
        shooterMotor.setVelocity(3000);

        driveStraightWithEncoderTurn(THREE_FEET_INCHES, 90.0, 0.75);

        startIntake(0.75, 0.5);
        driveStraightInches(THREE_FEET_INCHES + 6, 0.6);

        stopIntake();

        driveStraightInches(-(THREE_FEET_INCHES + 6), 0.6);

        turnDegrees(-90, 0.75);

        driveStraightInches(-(THREE_FEET_INCHES + 6), 0.75);

        shootForSeconds(2.0);
        sleep((long) (0.5 * 1000));
        shooterMotor.setVelocity(3000);

        driveStraightWithEncoderTurn(THREE_FEET_INCHES, 90.0, 0.75);
    }
}