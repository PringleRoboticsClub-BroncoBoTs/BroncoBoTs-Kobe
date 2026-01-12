package org.firstinspires.ftc.broncoBot.BroncoBoTsServices;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * AprilTagService
 * Lightweight AprilTag "service" that can be constructed from any OpMode and
 * queried for the pose of a single tag ID of interest.
 * Usage from MainOpMode:
 *     AprilTagService tagService = new AprilTagService(hardwareMap);
 *     AprilTagService.TagPose pose = tagService.getTagPose(20);
 *     if (pose != null) {
 *         double distanceZ = pose.getDistanceMeters(); // |Z|
 *         double yaw   = pose.yawDeg;
 *     }
 */
public class BroncoBoTAprilTagService {

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    public BroncoBoTAprilTagService(HardwareMap hardwareMap) {

        // 1) Create processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .build();

        // 2) Create VisionPortal using webcam named "webcam1"
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280,720))
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
     * x, y, z are in meters in the FTC coordinate frame from the camera to the tag.
     * yawDeg is the left/right angle from the camera to the tag (degrees).
     */
    public static class TagPose {
        public final int id;
        public final double xMeters;
        public final double yMeters;
        public final double zMeters;   // forward distance
        public final double yawDeg;

        public TagPose(int id,
                       double xMeters,
                       double yMeters,
                       double zMeters,
                       double yawDeg) {
            this.id = id;
            this.xMeters = xMeters;
            this.yMeters = yMeters;
            this.zMeters = zMeters;
            this.yawDeg = yawDeg;
        }

        /**
         * "Distance" defined as Z only, since we auto-align to center the tag.
         * Returns |Z| so it is always positive.
         */
        public double getDistanceMeters() {
            return Math.abs(zMeters);
        }
    }
}
