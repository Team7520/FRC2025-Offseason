package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;

/**
 * Interpolates between (yaw, pitch) pairs to produce (x, y) on a real field.
 * Supports bilinear interpolation for querying continuous (yaw, pitch) values.
 */
public class InterpolatingYawPitch2D {
    // Outer map: pitch -> (inner map)
    // Inner map: yaw -> [x, y] array
    private final TreeMap<Double, TreeMap<Double, double[]>> map = new TreeMap<>();

    /**
     * Adds a calibration point.
     * @param yaw   Yaw angle (degrees or radians, but be consistent)
     * @param pitch Pitch angle
     * @param x     Real-world X (e.g., meters)
     * @param y     Real-world Y (e.g., meters)
     */
    public void put(double yaw, double pitch, double x, double y) {
        map.computeIfAbsent(pitch, k -> new TreeMap<>()).put(yaw, new double[]{x, y});
    }

    /**
     * Gets an interpolated (x, y) value for the provided yaw and pitch.
     * @param yaw   Yaw to query
     * @param pitch Pitch to query
     * @return      Interpolated {x, y}, or null if outside bounds
     */
    public double[] getInterpolated(double yaw, double pitch) {
        // Find surrounding pitches
        Map.Entry<Double, TreeMap<Double, double[]>> lowerPitch = map.floorEntry(pitch);
        Map.Entry<Double, TreeMap<Double, double[]>> upperPitch = map.ceilingEntry(pitch);

        if (lowerPitch == null && upperPitch == null) return null;
        if (lowerPitch == null) return interp1D(upperPitch.getValue(), yaw);
        if (upperPitch == null) return interp1D(lowerPitch.getValue(), yaw);

        // Interpolate at both pitch levels for this yaw
        double[] lowerXY = interp1D(lowerPitch.getValue(), yaw);
        double[] upperXY = interp1D(upperPitch.getValue(), yaw);

        // Now interpolate between those on pitch
        double lowerKey = lowerPitch.getKey();
        double upperKey = upperPitch.getKey();

        if (lowerKey == upperKey) return lowerXY;

        double t = (pitch - lowerKey) / (upperKey - lowerKey);
        return new double[] {
            lowerXY[0] + (upperXY[0] - lowerXY[0]) * t,
            lowerXY[1] + (upperXY[1] - lowerXY[1]) * t
        };
    }

    // Helper for 1D interpolation along yaw at a fixed pitch
    private double[] interp1D(TreeMap<Double, double[]> inner, double yaw) {
        Map.Entry<Double, double[]> lowerYaw = inner.floorEntry(yaw);
        Map.Entry<Double, double[]> upperYaw = inner.ceilingEntry(yaw);

        if (lowerYaw == null && upperYaw == null) return new double[]{0.0, 0.0};
        if (lowerYaw == null) return upperYaw.getValue();
        if (upperYaw == null) return lowerYaw.getValue();

        double lowerKey = lowerYaw.getKey();
        double upperKey = upperYaw.getKey();
        double[] lowerVal = lowerYaw.getValue();
        double[] upperVal = upperYaw.getValue();

        if (lowerKey == upperKey) return lowerVal;

        double t = (yaw - lowerKey) / (upperKey - lowerKey);
        return new double[] {
            lowerVal[0] + (upperVal[0] - lowerVal[0]) * t,
            lowerVal[1] + (upperVal[1] - lowerVal[1]) * t
        };
    }
}