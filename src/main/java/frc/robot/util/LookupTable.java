package frc.robot.util;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * A table of keys and values that can get an interpolated result between data points.
 * Taken from the 2024 robot code
 */
public class LookupTable {
    private NavigableMap<Double, Double> dataPoints = new TreeMap<Double, Double>();

    /**
     * Creates a new LookupTable using the provided map
     * @param map A map of keys and values to act as the data points
     */
    public LookupTable(Map<Double, Double> map) {
        dataPoints.putAll(map);
    }

    /**
     * Gets an interpolated value based on a key
     * @param key Key to the requested value
     */
    public double lookup(double key) {
        if (dataPoints.containsKey(key)) return dataPoints.get(key);

        var higherKey = dataPoints.ceilingKey(key);
        var lowerKey = dataPoints.floorKey(key);

        if (higherKey == null) {
            higherKey = lowerKey;
            lowerKey = dataPoints.lowerKey(higherKey);
        }

        if (lowerKey == null) {
            lowerKey = higherKey;
            higherKey = dataPoints.higherKey(lowerKey);
        }

        double slope = (dataPoints.get(higherKey) - dataPoints.get(lowerKey)) / (higherKey - lowerKey);
        double intercept = dataPoints.get(lowerKey) - slope * lowerKey;
        return (slope * key) + intercept;
    }
}