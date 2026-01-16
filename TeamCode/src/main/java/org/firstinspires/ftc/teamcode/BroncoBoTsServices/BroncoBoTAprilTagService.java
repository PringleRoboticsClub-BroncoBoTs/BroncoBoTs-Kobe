package org.firstinspires.ftc.teamcode.BroncoBoTsServices;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Service class to manage AprilTag detection using VisionPortal and AprilTagProcessor.
 */
public class BroncoBoTAprilTagService {

    public final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    public BroncoBoTAprilTagService(HardwareMap hardwareMap) {

        // 1) Create processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        // 2) Create VisionPortal using webcam named "webcam1"
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280,720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /**
     * Returns the pose of the first detection that matches the requested ID,
     * or null if not currently seen.
     */
    public TagPose getTagPose(int id) {
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.id == id) {
                return new TagPose(
                        detection.id,
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z,
                        detection.ftcPose.bearing);
            }
        }
        return null;
    }

    /**
     * Call this when your OpMode stops.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Simple value object for tag pose (robot-centric).
     * x, y, z are in Inches in the FTC coordinate frame from the camera to the tag.
     * yawDeg is the left/right angle from the camera to the tag (degrees).
     */
    public static class TagPose {
        public final int id;
        public final double xInches;
        public final double yInches;
        public final double zInches;   
        public final double yawDeg;

        public TagPose(int id,
                       double xInches,
                       double yInches,
                       double zInches,
                       double yawDeg) {
            this.id = id;
            this.xInches = xInches;
            this.yInches = yInches;
            this.zInches = zInches;
            this.yawDeg = yawDeg;
        }

        /**
         * "Distance" defined as Z only, since we auto-align to center the tag.
         * Returns |Z| so it is always positive.
         */
        public double getDistanceInches() {
            return Math.abs(this.yInches);
        }
    }
}
