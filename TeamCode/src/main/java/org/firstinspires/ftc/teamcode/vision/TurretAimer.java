package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Keeps the turret pointed at our goal.
 *
 * How it works:
 *   1. Every vision sighting is converted into a FIELD position of the goal using the robot pose
 *      (Pinpoint) and turret angle AT THE MOMENT THE FRAME WAS CAPTURED (latency compensation from a
 *      short history buffer).
 *   2. Every loop, the turret angle needed to face that field point is recomputed from the CURRENT
 *      Pinpoint pose (fast, ~every loop). So the turret stays locked while the robot drives and spins,
 *      and keeps aiming if the tags are blocked for a moment.
 *   3. The rate of change of that angle is passed to the turret as velocity feedforward.
 *
 * Geometry: robot frame is x = forward, y = left (inches), angles CCW-positive, matching Pedro.
 */
@Config
public class TurretAimer {
    /** True if the camera rotates with the turret; false if it is fixed to the chassis. TODO(verify) */
    public static boolean CAMERA_ON_TURRET = true;
    /** Turret rotation axis, relative to robot center (inches). TODO(measure) */
    public static double TURRET_X_IN = 0.0, TURRET_Y_IN = 0.0;
    /** Camera lens position: relative to the turret axis if on turret, else to robot center. TODO(measure) */
    public static double CAM_FORWARD_IN = 4.0, CAM_LEFT_IN = 0.0;
    /** Camera yaw relative to turret-forward (or robot-forward if chassis-mounted). */
    public static double CAM_YAW_DEG = 0.0;

    /** 0 = trust each new sighting fully, 0.8 = heavy smoothing. */
    public static double GOAL_SMOOTHING = 0.5;
    /** A sighting further than this from the estimate replaces it instead of blending. */
    public static double GOAL_JUMP_RESET_IN = 18.0;
    /** Keep aiming from odometry this long after the last sighting (tags in BIOBUZZ can move). */
    public static double GOAL_MEMORY_SEC = 2.0;
    /** Aim trim (deg, + = CCW), e.g. to compensate for spin/drag curving the shot. */
    public static double AIM_OFFSET_DEG = 0.0;
    public static double FF_FILTER = 0.7;

    private static final int HISTORY = 64;
    private final long[] histNanos = new long[HISTORY];
    private final double[] histX = new double[HISTORY], histY = new double[HISTORY],
            histH = new double[HISTORY], histTurret = new double[HISTORY];
    private int histCount = 0, histHead = 0;

    private double goalX = Double.NaN, goalY = Double.NaN;
    private long goalSeenNanos = 0;
    private double desiredTurretDeg = 0;
    private double prevDesired = Double.NaN;
    private long prevNanos = 0;
    private double ffDegPerSec = 0;
    private double distanceIn = Double.NaN;

    /**
     * @param robotPose current Pinpoint pose (inches, radians)
     * @param turretDeg current measured turret angle (robot frame)
     * @param obs       new vision sighting this loop, or null
     */
    public void update(Pose robotPose, double turretDeg, GoalVision.Observation obs) {
        long now = System.nanoTime();
        record(now, robotPose.x(), robotPose.y(), robotPose.heading(), turretDeg);

        if (obs != null) integrate(obs);

        if (!hasTarget()) {
            prevDesired = Double.NaN;
            ffDegPerSec = 0;
            distanceIn = Double.NaN;
            return;
        }

        double h = robotPose.heading();
        double pivotX = robotPose.x() + TURRET_X_IN * Math.cos(h) - TURRET_Y_IN * Math.sin(h);
        double pivotY = robotPose.y() + TURRET_X_IN * Math.sin(h) + TURRET_Y_IN * Math.cos(h);
        double dx = goalX - pivotX, dy = goalY - pivotY;
        distanceIn = Math.hypot(dx, dy);
        desiredTurretDeg = MathUtil.wrapDeg(Math.toDegrees(Math.atan2(dy, dx) - h) + AIM_OFFSET_DEG);

        if (!Double.isNaN(prevDesired) && now > prevNanos) {
            double dt = (now - prevNanos) / 1e9;
            double rate = MathUtil.wrapDeg(desiredTurretDeg - prevDesired) / dt;
            ffDegPerSec = FF_FILTER * ffDegPerSec + (1 - FF_FILTER) * rate;
        }
        prevDesired = desiredTurretDeg;
        prevNanos = now;
    }

    private void integrate(GoalVision.Observation obs) {
        double[] s = lookup(obs.frameNanos);
        double rx = s[0], ry = s[1], h = s[2], turret = s[3];

        double camX, camY, camHeadingRobot;
        if (CAMERA_ON_TURRET) {
            double t = Math.toRadians(turret);
            camX = TURRET_X_IN + CAM_FORWARD_IN * Math.cos(t) - CAM_LEFT_IN * Math.sin(t);
            camY = TURRET_Y_IN + CAM_FORWARD_IN * Math.sin(t) + CAM_LEFT_IN * Math.cos(t);
            camHeadingRobot = t + Math.toRadians(CAM_YAW_DEG);
        } else {
            camX = CAM_FORWARD_IN;
            camY = CAM_LEFT_IN;
            camHeadingRobot = Math.toRadians(CAM_YAW_DEG);
        }
        double camFieldX = rx + camX * Math.cos(h) - camY * Math.sin(h);
        double camFieldY = ry + camX * Math.sin(h) + camY * Math.cos(h);
        double ch = h + camHeadingRobot;
        double gx = camFieldX + obs.forwardIn * Math.cos(ch) - obs.leftIn * Math.sin(ch);
        double gy = camFieldY + obs.forwardIn * Math.sin(ch) + obs.leftIn * Math.cos(ch);

        boolean stale = (System.nanoTime() - goalSeenNanos) / 1e9 > GOAL_MEMORY_SEC;
        if (Double.isNaN(goalX) || stale || Math.hypot(gx - goalX, gy - goalY) > GOAL_JUMP_RESET_IN) {
            goalX = gx;
            goalY = gy;
        } else {
            goalX = GOAL_SMOOTHING * goalX + (1 - GOAL_SMOOTHING) * gx;
            goalY = GOAL_SMOOTHING * goalY + (1 - GOAL_SMOOTHING) * gy;
        }
        goalSeenNanos = obs.frameNanos;
    }

    // ---------------------------------------------------------------- history buffer

    private void record(long t, double x, double y, double h, double turret) {
        histNanos[histHead] = t;
        histX[histHead] = x;
        histY[histHead] = y;
        histH[histHead] = h;
        histTurret[histHead] = turret;
        histHead = (histHead + 1) % HISTORY;
        histCount = Math.min(histCount + 1, HISTORY);
    }

    /** Robot x, y, heading, turret at time t (linear interpolation; clamps to buffer ends). */
    private double[] lookup(long t) {
        int newest = (histHead - 1 + HISTORY) % HISTORY;
        if (histCount == 0) return new double[]{0, 0, 0, 0};
        if (t >= histNanos[newest] || histCount == 1) return sample(newest);
        for (int i = 1; i < histCount; i++) {
            int a = (newest - i + HISTORY) % HISTORY;      // older
            int b = (newest - i + 1 + HISTORY) % HISTORY;  // newer
            if (histNanos[a] <= t) {
                double f = (double) (t - histNanos[a]) / Math.max(1, histNanos[b] - histNanos[a]);
                return new double[]{
                        histX[a] + f * (histX[b] - histX[a]),
                        histY[a] + f * (histY[b] - histY[a]),
                        histH[a] + f * MathUtil.wrapRad(histH[b] - histH[a]),
                        histTurret[a] + f * (histTurret[b] - histTurret[a])};
            }
        }
        return sample((newest - histCount + 1 + HISTORY) % HISTORY); // older than buffer
    }

    private double[] sample(int i) {
        return new double[]{histX[i], histY[i], histH[i], histTurret[i]};
    }

    // ---------------------------------------------------------------- outputs

    public boolean hasTarget() {
        return !Double.isNaN(goalX) && (System.nanoTime() - goalSeenNanos) / 1e9 <= GOAL_MEMORY_SEC;
    }

    /** Robot-frame turret angle that faces the goal (deg, CCW+). Only valid if hasTarget(). */
    public double getDesiredTurretDeg() { return desiredTurretDeg; }

    public double getFeedforwardDegPerSec() { return ffDegPerSec; }

    /** Horizontal distance from turret axis to the goal (inches), NaN if unknown. */
    public double getDistanceIn() { return distanceIn; }

    public double getGoalX() { return goalX; }
    public double getGoalY() { return goalY; }

    public double getSecondsSinceSeen() {
        return Double.isNaN(goalX) ? Double.POSITIVE_INFINITY : (System.nanoTime() - goalSeenNanos) / 1e9;
    }

    public void forget() {
        goalX = Double.NaN;
        goalY = Double.NaN;
    }

    public void addTelemetry(Telemetry t) {
        t.addData("aim/hasTarget", hasTarget());
        t.addData("aim/desiredDeg", desiredTurretDeg);
        t.addData("aim/ffDegPerSec", ffDegPerSec);
        t.addData("aim/distanceIn", distanceIn);
        t.addData("aim/sinceSeenSec", getSecondsSinceSeen());
    }
}
