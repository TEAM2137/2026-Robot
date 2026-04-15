package frc.robot.subsystems.launcher;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LookupTables {
    public static final InterpolatingDoubleTreeMap flywheelRpmHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.09, 1800.0),
        Map.entry(1.50, 1800.0),
        Map.entry(2.00, 1800.0),
        Map.entry(2.50, 1850.0),
        Map.entry(3.00, 1900.0),
        Map.entry(3.50, 1950.0),
        Map.entry(4.00, 2050.0),
        Map.entry(4.50, 2145.0), // TODO: verify all values past 4.5m
        Map.entry(5.01, 2240.0),
        Map.entry(6.07, 2420.0),
        Map.entry(7.05, 2620.0),
        Map.entry(8.00, 2779.0)
    );

    public static final InterpolatingDoubleTreeMap hoodAngleHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.09, 4.0),
        Map.entry(1.50, 8.8),
        Map.entry(2.00, 12.0),
        Map.entry(2.50, 16.9),
        Map.entry(3.00, 19.3),
        Map.entry(3.50, 24.6),
        Map.entry(4.00, 26.0),
        Map.entry(4.50, 26.0)
    );

    public static final InterpolatingDoubleTreeMap timeOfFlightHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.00, 1.03),
        Map.entry(1.50, 1.00),
        Map.entry(2.00, 1.00),
        Map.entry(2.50, 0.98),
        Map.entry(3.00, 1.03),
        Map.entry(3.50, 1.08),
        Map.entry(4.00, 1.05),
        Map.entry(4.655, 1.10),
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
