package frc.robot.subsystems.launcher;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LookupTables {
    public static final InterpolatingDoubleTreeMap flywheelRpmHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.09, 2005.0),
        Map.entry(1.50, 2005.0),
        Map.entry(2.00, 2005.0),
        Map.entry(2.50, 1950.0),
        Map.entry(3.00, 2020.0),
        Map.entry(3.50, 2080.0),
        Map.entry(4.00, 2190.0),
        Map.entry(4.50, 2325.0),
        Map.entry(5.00, 2460.0),
        Map.entry(5.50, 2555.0),
        Map.entry(6.00, 2680.0),
        Map.entry(7.00, 2975.0),
        Map.entry(8.00, 3245.0)
    );

    public static final InterpolatingDoubleTreeMap hoodAngleHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.09, 4.0),
        Map.entry(1.50, 8.8),
        Map.entry(2.00, 12.0),
        Map.entry(2.50, 16.9),
        Map.entry(3.00, 19.3),
        Map.entry(3.50, 24.6),
        Map.entry(4.00, 26.0)
    );

    public static final InterpolatingDoubleTreeMap timeOfFlightHub = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.00, 1.03),
        Map.entry(1.50, 1.00),
        Map.entry(2.00, 1.00),
        Map.entry(2.50, 0.98),
        Map.entry(3.00, 1.03),
        Map.entry(3.50, 1.08),
        Map.entry(4.00, 1.05),
        Map.entry(4.66, 1.10),
        Map.entry(5.00, 1.23),
        Map.entry(5.50, 1.30),
        Map.entry(6.00, 1.35),
        Map.entry(7.00, 1.55),
        Map.entry(8.00, 1.72)
    );

    public static final InterpolatingDoubleTreeMap flywheelRpmPassing = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(4.5, 2180.0),
        Map.entry(7.5, 3075.0),
        Map.entry(10.0, 3370.0),
        Map.entry(12.5, 3980.0),
        Map.entry(15.3, 4670.0)
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
