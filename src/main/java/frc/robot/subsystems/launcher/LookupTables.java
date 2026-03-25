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
        Map.entry(4.655, 2283.0),
        Map.entry(5.009, 2320.0),
        Map.entry(6.065, 2500.0),
        Map.entry(7.045, 2700.0),
        Map.entry(8.000, 2859.0)
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
        Map.entry(4.655, 1.20),
        Map.entry(5.009, 1.30),
        Map.entry(6.065, 1.40),
        Map.entry(7.045, 1.50),
        Map.entry(8.000, 1.70)
    );

    public static final InterpolatingDoubleTreeMap flywheelRpmPassing = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(4.5, 2137.0),
        Map.entry(7.5, 2670.0),
        Map.entry(10.0, 3024.0),
        Map.entry(12.5, 3450.0),
        Map.entry(15.3, 4145.0)
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
