package org.firstinspires.ftc.teamcode.util;

/**
 * Alliance selection. The scoring cluster names come from
 * AprilTagGameDatabase.getBioBuzzTagLibrary() (SDK 12.0): the BIOBUZZ scoring tags are grouped
 * into clusters whose origin is the center of the Cell opening, which is our aim point.
 */
public enum Alliance {
    RED("RED SCORING"),
    BLUE("BLUE SCORING");

    /** AprilTagClusterMetadata.name of this alliance's scoring (hive) cluster. */
    public final String scoringClusterName;

    Alliance(String scoringClusterName) {
        this.scoringClusterName = scoringClusterName;
    }

    public Alliance other() {
        return this == RED ? BLUE : RED;
    }
}
