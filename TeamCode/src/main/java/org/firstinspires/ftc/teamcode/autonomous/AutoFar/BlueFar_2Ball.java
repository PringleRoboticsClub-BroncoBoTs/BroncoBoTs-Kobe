package org.firstinspires.ftc.teamcode.autonomous.AutoFar;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autonomous.BroncoBotAutoBase;

/**
 * Auto Routine 1 using encoder ticks:
 * 1) shoot for 2 sec
 * 2)Move straight 3ft
 * 3) Turn left 90 deg
 * 4) Move straight 3ft with intake active
 * 5) move backward 3ft
 * 6) Turn right 90 deg
 * 7) Move backward 3 feet
 * 6)shoot for 2 sec
 * 7) Move straight 6ft
 * 8) Turn left 90 deg
 * 9) Move straight 3ft with intake active
 * 10) Move backward 3ft
 * 11)Turn right 90 deg
 * 12) Move backward 6ft
 * 13) Shoot for 2 sec
 * 14) Move straight 3 ft
 *
 */
@Autonomous(name = "Auto Routine 1 - Blue Far - 2 ball", group = "Autonomous")
public class BlueFar_2Ball extends BroncoBotAutoBase {

    private static final double THREE_FEET_INCHES  = 36.0;
    private static final double SIX_FEET_INCHES  = 72.0;
    private static final double TWO_FEET_INCHES = 24.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Auto Routine 1 - Encoders: Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;


        startShooter();
        sleep((long) (0.5 * 1000));

        driveStraightWithEncoderTurn(THREE_FEET_INCHES, -90.0, 0.75);

        startIntake(0.75, 0.5);
        driveStraightInches(THREE_FEET_INCHES+6, 0.6);

        stopIntake();

        driveStraightInches(-(THREE_FEET_INCHES+6), 0.6);

        turnDegrees(90, 0.75);

        driveStraightInches(-(THREE_FEET_INCHES+6), 0.75);

        shootForSeconds(2.0);
        sleep((long) (2 * 1000));
        shooterMotor.setVelocity(3000);

        driveStraightWithEncoderTurn(SIX_FEET_INCHES, -90.0, 0.75);

        startIntake(0.75, 0.5);
        driveStraightInches(THREE_FEET_INCHES+6, 0.6);

        stopIntake();

        driveStraightInches(-(THREE_FEET_INCHES+6), 0.6);

        turnDegrees(90, 0.75);

        driveStraightInches(-(SIX_FEET_INCHES+6), 0.75);

        shootForSeconds(2.0);
        sleep((long) (2 * 1000));
        shooterMotor.setVelocity(3000);

        driveStraightInches(THREE_FEET_INCHES, 0.75);


        /*strafeInches(TWO_FEET_INCHES+8, 0.9);

        turnDegrees(-260, 0.9);
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

        stopShooter();*/

        // End: everything should already be stopped by helpers
    }
}
