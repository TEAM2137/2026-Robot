package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/** Field constants such as important poses or dimensions can be placed here */
public class FieldConstants {
    public static final double fieldLength = 16.541;
    public static final double fieldWidth = 8.0692;

    public static final double hubOffsetX = 3.6449;
    public static final Translation2d blueHub = new Translation2d(fieldLength / 2.0 - hubOffsetX, fieldWidth / 2.0);
    public static final Translation2d redHub = new Translation2d(fieldLength / 2.0 + hubOffsetX, fieldWidth / 2.0);
}
