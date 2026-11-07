package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import static com.pedropathing.api.Paths.line;

/**
 * BIOBUZZ autonomous on Pedro Pathing + Pinpoint, with the turret auto-aiming the whole time.
 *
 * Plan: drive to SHOOT -> fire preloads -> drive the pickup line with intake on -> back to SHOOT ->
 * fire -> PARK.
 *
 * Poses are RELATIVE TO THE STARTING POSE (start = 0,0, heading 0 = robot-forward at start),
 * written for RED. BLUE mirrors them (y -> -y, heading -> -heading). Measure them on the field and
 * edit the RED_* values (live-editable on FTC Dashboard "BioBuzzAuto" card).
 *
 * Setup: place the robot with the turret physically facing robot-forward (the turret encoder is
 * zeroed at INIT).
 */
@Config
public abstract class BioBuzzAuto extends LinearOpMode {
    // TODO(measure) all poses - inches / degrees, RED side, relative to the start pose.
    public static double SHOOT_X = 24, SHOOT_Y = 0, SHOOT_H = 0;
    public static double PICKUP_START_X = 36, PICKUP_START_Y = 12, PICKUP_START_H = 90;
    public static double PICKUP_END_X = 36, PICKUP_END_Y = 36, PICKUP_END_H = 90;
    public static double PARK_X = 30, PARK_Y = 24, PARK_H = 0;

    public static double SHOOT_TIME_SEC = 2.5;     // max time to spend firing a volley
    public static double AIM_TIMEOUT_SEC = 1.5;    // fire anyway after this long without a lock
    public static double PATH_TIMEOUT_SEC = 5.0;
    public static double PICKUP_SPEED_TIME_SEC = 0.5; // extra intake time at the end of the pickup line

    protected abstract Alliance alliance();

    private enum Step { TO_SHOOT_1, SHOOT_1, TO_PICKUP, PICKUP, TO_SHOOT_2, SHOOT_2, TO_PARK, DONE }

    private Follower follower;
    private Robot robot;
    private Telemetry tel;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private Step step = Step.TO_SHOOT_1;

    @Override
    public void runOpMode() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PoseStorage.alliance = alliance();
        PoseStorage.pose = null;

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        robot = new Robot(hardwareMap, alliance(), true, true); // zero turret: must face forward
        robot.autoAim = true;
        robot.arm.stow();

        while (opModeInInit()) {
            robot.clearBulkCache();
            follower.update();
            // Track the goal during INIT but do NOT move motors (game rules): no robot.update() here.
            robot.aimer.update(follower.pose(), robot.turret.getAngleDeg(), robot.vision.update());
            tel.addData("Alliance", alliance());
            tel.addData("Goal visible", robot.aimer.hasTarget());
            tel.addLine("Turret was zeroed at INIT - it must be facing forward");
            tel.update();
        }
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        robot.shooterOn = true; // spin up immediately while driving
        startStep(Step.TO_SHOOT_1);
        follower.follow(line(follower.pose(), pose(SHOOT_X, SHOOT_Y, SHOOT_H)));

        while (opModeIsActive() && step != Step.DONE) {
            robot.clearBulkCache();
            follower.update();
            runStateMachine();
            robot.update(follower.pose());
            PoseStorage.pose = follower.pose();

            tel.addData("step", step);
            tel.addData("pose", follower.pose());
            robot.addTelemetry(tel);
            tel.update();
        }

        // Hold position until the period ends so the pose is saved for TeleOp.
        while (opModeIsActive()) {
            robot.clearBulkCache();
            follower.update();
            robot.update(follower.pose());
            PoseStorage.pose = follower.pose();
        }
        follower.stop();
        robot.stop();
    }

    private void runStateMachine() {
        switch (step) {
            case TO_SHOOT_1:
            case TO_SHOOT_2:
                if (pathDone()) startStep(step == Step.TO_SHOOT_1 ? Step.SHOOT_1 : Step.SHOOT_2);
                break;

            case SHOOT_1:
            case SHOOT_2:
                boolean locked = robot.aimer.hasTarget();
                robot.shootRequested = true;
                // If vision never locks, still fire after a timeout rather than wasting the auto.
                robot.forceFeed = !locked && stepTimer.seconds() > AIM_TIMEOUT_SEC && robot.shooter.isReady();
                if (stepTimer.seconds() > SHOOT_TIME_SEC) {
                    robot.shootRequested = false;
                    robot.forceFeed = false;
                    if (step == Step.SHOOT_1) {
                        startStep(Step.TO_PICKUP);
                        follower.follow(line(follower.pose(), pose(PICKUP_START_X, PICKUP_START_Y, PICKUP_START_H)));
                    } else {
                        startStep(Step.TO_PARK);
                        robot.shooterOn = false;
                        follower.follow(line(follower.pose(), pose(PARK_X, PARK_Y, PARK_H)));
                    }
                }
                break;

            case TO_PICKUP:
                robot.intakeRequest = Intake.State.INTAKE;
                if (pathDone()) {
                    startStep(Step.PICKUP);
                    follower.follow(line(follower.pose(), pose(PICKUP_END_X, PICKUP_END_Y, PICKUP_END_H)));
                }
                break;

            case PICKUP:
                robot.intakeRequest = Intake.State.INTAKE;
                if (!follower.isBusy() && stepTimer.seconds() > PICKUP_SPEED_TIME_SEC
                        || stepTimer.seconds() > PATH_TIMEOUT_SEC) {
                    robot.intakeRequest = Intake.State.OFF;
                    startStep(Step.TO_SHOOT_2);
                    follower.follow(line(follower.pose(), pose(SHOOT_X, SHOOT_Y, SHOOT_H)));
                }
                break;

            case TO_PARK:
                if (pathDone()) startStep(Step.DONE);
                break;

            default:
                break;
        }
    }

    private boolean pathDone() {
        return !follower.isBusy() || stepTimer.seconds() > PATH_TIMEOUT_SEC;
    }

    private void startStep(Step s) {
        step = s;
        stepTimer.reset();
    }

    /** RED-side relative pose, mirrored for BLUE. Heading in degrees. */
    private Pose pose(double x, double y, double headingDeg) {
        double sign = alliance() == Alliance.BLUE ? -1 : 1;
        return new Pose(x, sign * y, Math.toRadians(sign * headingDeg));
    }
}
