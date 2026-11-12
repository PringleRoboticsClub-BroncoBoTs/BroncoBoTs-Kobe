package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Hands state from Autonomous to TeleOp. Static fields survive between OpModes as long as the
 * Robot Controller app is not restarted.
 */
public final class PoseStorage {
    private PoseStorage() {}

    /** Last robot pose from the Pinpoint at the end of auto (Pedro coordinates, inches / radians). */
    public static Pose pose = null;

    /** Alliance that was last run, so TeleOp can default to it. */
    public static Alliance alliance = Alliance.RED;

    /**
     * True once the turret encoder has been zeroed with the turret facing forward. While true,
     * TeleOp does NOT reset the turret encoder so the wiring wrap limits stay correct after auto.
     */
    public static boolean turretZeroed = false;
}
