package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class InterpLUTTest {
    private final InterpLUT lut = new InterpLUT(new double[]{24, 48, 72}, new double[]{2600, 3000, 3500});

    @Test
    public void interpolatesBetweenPoints() {
        assertEquals(2800, lut.get(36), 1e-9);
        assertEquals(3250, lut.get(60), 1e-9);
    }

    @Test
    public void clampsOutsideRange() {
        assertEquals(2600, lut.get(0), 1e-9);
        assertEquals(3500, lut.get(500), 1e-9);
    }

    @Test
    public void wrapDeg() {
        assertEquals(-170, MathUtil.wrapDeg(190), 1e-9);
        assertEquals(180, MathUtil.wrapDeg(-180), 1e-9);
        assertEquals(10, MathUtil.wrapDeg(370), 1e-9);
    }
}
