package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Utils {
    /** Logs the active command of the given subsystem */
    public static void logActiveCommand(String subsystemName, SubsystemBase subsystem) {
        Logger.recordOutput(
            subsystemName + "/ActiveCommand",
            subsystem.getCurrentCommand() == null
                ? "None"
                : subsystem.getCurrentCommand().getName()
        );
    }

    /** Gets a constant based on the robot implementation (real, sim) */
    public static <T> T getRealOrSimConstant(T real, T sim) {
        return getConstant(real, real, sim);
    }

    /** Gets a hardware-only constant based on the current robot type (comp, practice) */
    public static <T> T getConstant(T comp, T practice) {
        return getConstant(comp, practice, comp);
    }

    /** Gets a constant based on the current robot type (comp, practice, sim)*/
    public static <T> T getConstant(T comp, T practice, T sim) {
        return switch (Constants.currentRobotType) {
            case COMP -> comp;
            case PRACTICE -> practice;
            case SIM -> sim;
        };
    }
    
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
