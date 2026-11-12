package org.firstinspires.ftc.teamcode.util;

/** Piecewise-linear lookup table. Inputs outside the table are clamped to the end points. */
public final class InterpLUT {
    private final double[] xs;
    private final double[] ys;

    /** @param xs strictly increasing inputs; @param ys outputs (same length) */
    public InterpLUT(double[] xs, double[] ys) {
        if (xs.length != ys.length || xs.length == 0) {
            throw new IllegalArgumentException("InterpLUT needs equal-length, non-empty arrays");
        }
        for (int i = 1; i < xs.length; i++) {
            if (xs[i] <= xs[i - 1]) throw new IllegalArgumentException("InterpLUT xs must be strictly increasing");
        }
        this.xs = xs.clone();
        this.ys = ys.clone();
    }

    public double get(double x) {
        if (x <= xs[0]) return ys[0];
        int n = xs.length;
        if (x >= xs[n - 1]) return ys[n - 1];
        for (int i = 1; i < n; i++) {
            if (x <= xs[i]) {
                double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
                return ys[i - 1] + t * (ys[i] - ys[i - 1]);
            }
        }
        return ys[n - 1];
    }
}
