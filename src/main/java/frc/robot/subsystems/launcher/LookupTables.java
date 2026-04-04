package frc.robot.subsystems.launcher;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LookupTables {
    public static final InterpolatingDoubleTreeMap flywheelRpmHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.172, 1831.0),
        Map.entry(1.741, 1897.0),
        Map.entry(2.224, 1820.0),
        Map.entry(2.484, 1891.0),
        Map.entry(2.801, 1951.0),
        Map.entry(3.236, 1957.0),
        Map.entry(3.794, 2038.0),
        Map.entry(4.261, 2100.0),
        Map.entry(4.655, 2203.0),
        Map.entry(5.009, 2240.0),
        Map.entry(6.065, 2420.0),
        Map.entry(7.045, 2620.0),
        Map.entry(8.000, 2779.0)
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
        Map.entry(1.172, 1.07),
        Map.entry(1.741, 1.01),
        Map.entry(2.224, 0.88),
        Map.entry(2.484, 0.92),
        Map.entry(2.801, 0.966),
        Map.entry(3.236, 0.90),
        Map.entry(3.794, 0.96),
        Map.entry(4.261, 0.97),
        Map.entry(4.655, 1.08),
        Map.entry(5.009, 1.15),
        Map.entry(6.065, 1.25),
        Map.entry(7.045, 1.35),
        Map.entry(8.000, 1.55)
    );

    public static final InterpolatingDoubleTreeMap flywheelRpmPassing = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(4.5, 1947.0),
        Map.entry(7.5, 2520.0),
        Map.entry(10.0, 2874.0),
        Map.entry(12.5, 3400.0),
        Map.entry(15.3, 3995.0)
    );

    public static final InterpolatingDoubleTreeMap hoodAnglePassing = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 26.0)
    );
    
    public static final InterpolatingDoubleTreeMap timeOfFlightPassing = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(4.5, 1.5),
        Map.entry(7.5, 1.8),
        Map.entry(10.0, 2.1),
        Map.entry(12.5, 2.2),
        Map.entry(15.3, 2.5)
    );
}
