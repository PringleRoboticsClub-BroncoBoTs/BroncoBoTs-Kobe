package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/** Battery voltage, sampled at most every SAMPLE_SEC (voltage reads are slow) and low-pass filtered. */
public class VoltageMonitor {
    private static final double SAMPLE_SEC = 0.25;
    private final VoltageSensor sensor;
    private double voltage;
    private long lastSampleNanos = 0;

    public VoltageMonitor(HardwareMap hw) {
        sensor = hw.voltageSensor.iterator().next();
        voltage = Math.max(sensor.getVoltage(), 9.0);
        lastSampleNanos = System.nanoTime();
    }

    public double getVoltage() {
        long now = System.nanoTime();
        if ((now - lastSampleNanos) / 1e9 >= SAMPLE_SEC) {
            lastSampleNanos = now;
            double v = sensor.getVoltage();
            if (v > 6.0) voltage = 0.7 * voltage + 0.3 * v; // ignore glitches
        }
        return voltage;
    }
}
