package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

public class TurretSafetyTest {
    private static final double MIN = -170, MAX = 170, EPS = 1e-9;

    @Before
    public void setUp() {
        Turret.MIN_DEG = MIN;
        Turret.MAX_DEG = MAX;
        Turret.SLOW_ZONE_DEG = 20;
        Turret.SLOW_ZONE_MIN_POWER = 0.12;
        Turret.MAX_POWER = 0.8;
    }

    @Test
    public void reachableTargetIsUsedDirectly() {
        assertEquals(90, Turret.resolveSafeTarget(90, 0, MIN, MAX), EPS);
        assertEquals(-45, Turret.resolveSafeTarget(315, 0, MIN, MAX), EPS);
        assertEquals(10, Turret.resolveSafeTarget(370, 0, MIN, MAX), EPS);
    }

    @Test
    public void deadZoneTargetParksAtNearestLimit() {
        assertEquals(MAX, Turret.resolveSafeTarget(175, 0, MIN, MAX), EPS);
        assertEquals(MIN, Turret.resolveSafeTarget(-175, 0, MIN, MAX), EPS);
    }

    @Test
    public void crossingSeamUnwindsTheLongWayInsideLimits() {
        // Turret parked at +170, goal moves to -160 (just across the seam): must go to -160 via 0,
        // never "+200" through the wires.
        double t = Turret.resolveSafeTarget(-160, 170, MIN, MAX);
        assertEquals(-160, t, EPS);
    }

    @Test
    public void resolvedTargetAlwaysWithinLimits() {
        for (double desired = -1080; desired <= 1080; desired += 0.5) {
            for (double current = MIN; current <= MAX; current += 17) {
                double t = Turret.resolveSafeTarget(desired, current, MIN, MAX);
                assertTrue("desired=" + desired + " current=" + current + " -> " + t, t >= MIN && t <= MAX);
            }
        }
    }

    @Test
    public void widerThan360RangePicksClosestEquivalent() {
        // Limits -200..200 allow two equivalents of 190/-170; pick the one nearest current.
        assertEquals(190, Turret.resolveSafeTarget(-170, 150, -200, 200), EPS);
        assertEquals(-170, Turret.resolveSafeTarget(190, -100, -200, 200), EPS);
    }

    @Test
    public void powerCutWhenPushingPastLimit() {
        assertEquals(0, Turret.applyLimitGuards(0.5, MAX), EPS);
        assertEquals(0, Turret.applyLimitGuards(0.5, MAX + 5), EPS);
        assertEquals(0, Turret.applyLimitGuards(-0.5, MIN - 5), EPS);
        // Driving back toward the safe range is allowed.
        assertEquals(-0.5, Turret.applyLimitGuards(-0.5, MAX + 5), EPS);
        assertEquals(0.5, Turret.applyLimitGuards(0.5, MIN - 5), EPS);
    }

    @Test
    public void powerReducedInSlowZone() {
        double p = Turret.applyLimitGuards(0.8, MAX - 5); // 5 deg of room
        assertEquals(Math.max(0.12, 0.8 * 5 / 20), p, EPS);
        assertEquals(0.8, Turret.applyLimitGuards(0.8, 0), EPS);
    }
}
