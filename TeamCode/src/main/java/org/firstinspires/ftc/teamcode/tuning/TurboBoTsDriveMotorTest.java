package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.TurboBoTsTeleOp;

/**
 * "Tune: Drive Motors" for the TurboBoTs robot: same buttons and checks, but it uses the wheel
 * directions from {@link TurboBoTsTeleOp} instead of the BIOBUZZ Drivetrain, so the two robots stay
 * independent. If a wheel spins backwards here, flip its *_DIR in TurboBoTsTeleOp (not Drivetrain).
 */
@TeleOp(name = "TurboBoTs: Drive Motors", group = "TurboBoTs")
public class TurboBoTsDriveMotorTest extends DriveMotorTest {
    @Override
    protected void applyDirections(DcMotor[] motors) {
        motors[0].setDirection(TurboBoTsTeleOp.FRONT_LEFT_DIR);
        motors[1].setDirection(TurboBoTsTeleOp.FRONT_RIGHT_DIR);
        motors[2].setDirection(TurboBoTsTeleOp.BACK_LEFT_DIR);
        motors[3].setDirection(TurboBoTsTeleOp.BACK_RIGHT_DIR);
    }
}
