package org.firstinspires.ftc.teamcode.util;

public final class MathUtil {
    private MathUtil() {}

    /** Wraps degrees into (-180, 180]. */
    public static double wrapDeg(double deg) {
        double a = deg % 360.0;
        if (a > 180.0) a -= 360.0;
        if (a <= -180.0) a += 360.0;
        return a;
    }

    /** Wraps radians into (-PI, PI]. */
    public static double wrapRad(double rad) {
        return Math.toRadians(wrapDeg(Math.toDegrees(rad)));
    }

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
