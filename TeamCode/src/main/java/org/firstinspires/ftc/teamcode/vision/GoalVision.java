package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Tracks OUR alliance's scoring AprilTag cluster (the tags under the hive) with the global-shutter
 * USB camera. SDK 12 groups the BIOBUZZ scoring tags into clusters whose origin is the center of the
 * Cell opening, so the cluster pose is directly the aim point, and it keeps working when some of the
 * tags are blocked. The opponent's cluster and the audience-wall tags are ignored.
 *
 * NOTE (SDK 12 release notes): BIOBUZZ tags move, so they are NOT used for field localization.
 * Field position comes from the Pinpoint; vision only tells us where the goal is relative to us.
 */
@Config
public class GoalVision {
    // Camera / processor settings. Pick a resolution your camera supports and that matches its calibration.
    public static int RESOLUTION_WIDTH = 640;
    public static int RESOLUTION_HEIGHT = 480;
    public static int DECIMATION = 2;           // higher = faster but shorter range
    /** Lens intrinsics (pixels). Leave 0 to use the SDK's built-in calibration if your camera has one. TODO(calibrate) */
    public static double FX = 0, FY = 0, CX = 0, CY = 0;
    /** Manual exposure: short exposure + global shutter = no motion blur while driving/turret moving. */
    public static boolean MANUAL_EXPOSURE = true;
    public static int EXPOSURE_MS = 6;          // TODO(tune) for venue lighting
    public static int GAIN = 200;               // TODO(tune)
    public static boolean LIVE_VIEW = true;     // turn off at competition to save CPU

    /** One sighting of our goal, in the CAMERA frame. */
    public static final class Observation {
        public final double forwardIn;   // along the camera's optical axis
        public final double leftIn;      // to the camera's left
        public final double upIn;
        public final double rangeIn;
        public final double bearingDeg;  // + = goal is to the left (counter-clockwise)
        public final long frameNanos;    // System.nanoTime() at frame capture
        public final int percentTagsFound;

        Observation(double forwardIn, double leftIn, double upIn, double rangeIn, double bearingDeg,
                    long frameNanos, int percentTagsFound) {
            this.forwardIn = forwardIn;
            this.leftIn = leftIn;
            this.upIn = upIn;
            this.rangeIn = rangeIn;
            this.bearingDeg = bearingDeg;
            this.frameNanos = frameNanos;
            this.percentTagsFound = percentTagsFound;
        }
    }

    private final AprilTagProcessor processor;
    private final VisionPortal portal;
    private Alliance alliance;
    private boolean exposureApplied = !MANUAL_EXPOSURE;
    private Observation latest = null;
    private int detectionsLastFrame = 0;

    public GoalVision(HardwareMap hw, Alliance alliance) {
        this.alliance = alliance;
        AprilTagProcessor.Builder pb = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true);
        if (FX > 0 && FY > 0) pb.setLensIntrinsics(FX, FY, CX, CY);
        processor = pb.build();
        processor.setDecimation(DECIMATION);

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, RobotConfig.WEBCAM))
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(LIVE_VIEW)
                .addProcessor(processor)
                .build();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        latest = null;
    }

    public Alliance getAlliance() { return alliance; }

    /**
     * Polls for a NEW frame. Returns an observation of our scoring cluster from that frame, or null
     * if there was no new frame or our goal was not in it.
     */
    public Observation update() {
        if (!exposureApplied) tryApplyExposure();

        List<AprilTagDetection> fresh = processor.getFreshDetections();
        if (fresh == null) return null;
        detectionsLastFrame = fresh.size();

        for (AprilTagDetection d : fresh) {
            if (!(d instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            if (c.metadata == null || !alliance.scoringClusterName.equals(c.metadata.name)) continue;
            if (d.ftcPose == null) continue;
            latest = new Observation(
                    d.ftcPose.y, -d.ftcPose.x, d.ftcPose.z,
                    d.ftcPose.range, d.ftcPose.bearing,
                    d.frameAcquisitionNanoTime, c.percentClusterFound);
            return latest;
        }
        return null;
    }

    /** Most recent sighting (may be old - check frameNanos). */
    public Observation getLatest() { return latest; }

    private void tryApplyExposure() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            GainControl gain = portal.getCameraControl(GainControl.class);
            if (gain != null) gain.setGain(GAIN);
        } catch (RuntimeException e) {
            // Camera does not support manual exposure; keep auto.
        }
        exposureApplied = true;
    }

    public void setStreaming(boolean on) {
        if (on) portal.resumeStreaming(); else portal.stopStreaming();
    }

    public void close() {
        portal.close();
    }

    public void addTelemetry(Telemetry t) {
        t.addData("vision/alliance", alliance);
        t.addData("vision/fps", portal.getFps());
        t.addData("vision/tagsInFrame", detectionsLastFrame);
        if (latest != null) {
            t.addData("vision/ageSec", (System.nanoTime() - latest.frameNanos) / 1e9);
            t.addData("vision/rangeIn", latest.rangeIn);
            t.addData("vision/bearingDeg", latest.bearingDeg);
            t.addData("vision/clusterPct", latest.percentTagsFound);
        }
    }
}
