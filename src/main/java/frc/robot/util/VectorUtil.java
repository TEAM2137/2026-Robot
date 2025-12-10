package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class VectorUtil {
    /**
     * @return A copy of the given translation vector with a magnitude of 1
     */
    public static Translation2d normalize(Translation2d vector) {
        return vector.div(vector.getNorm());
    }

    /**
     * @return The dot product of translations a and b
     */
    public static double dot(Translation2d a, Translation2d b) {
        return (a.getX() * b.getX() + a.getY() * b.getY());
    }
}
