package frc.robot.subsystems.launcher;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LookupTables {
    public static final InterpolatingDoubleTreeMap flywheelRpmHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.172, 1911.0),
        Map.entry(1.741, 1977.0),
        Map.entry(2.224, 1900.0),
        Map.entry(2.484, 1971.0),
        Map.entry(2.801, 2031.0),
        Map.entry(3.236, 2037.0),
        Map.entry(3.794, 2118.0),
        Map.entry(4.261, 2180.0),
        Map.entry(4.655, 2283.0)
    );

    public static final InterpolatingDoubleTreeMap hoodAngleHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.172, 1.5),
        Map.entry(1.741, 6.2),
        Map.entry(2.224, 12.9),
        Map.entry(2.484, 14.7),
        Map.entry(2.801, 16.8),
        Map.entry(3.236, 22.5),
        Map.entry(3.794, 25.4),
        Map.entry(4.261, 26.0),
        Map.entry(4.655, 26.0)
    );

    public static final InterpolatingDoubleTreeMap timeOfFlightHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.172, 1.17),
        Map.entry(1.741, 1.11),
        Map.entry(2.224, 0.98),
        Map.entry(2.484, 1.02),
        Map.entry(2.801, 1.066),
        Map.entry(3.236, 1.00),
        Map.entry(3.794, 1.06),
        Map.entry(4.261, 1.07),
        Map.entry(4.655, 1.20)
    );

    public static final InterpolatingDoubleTreeMap flywheelRpmPassing = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(4.5000, 1861.0),
        Map.entry(6.9200, 2207.0),
        Map.entry(8.7000, 2450.0),
        Map.entry(12.0000, 2770.0)
    );

    public static final InterpolatingDoubleTreeMap hoodAnglePassing = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 22.0)
    );
    
    public static final InterpolatingDoubleTreeMap timeOfFlightPassing = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 0.0)
    );
}
