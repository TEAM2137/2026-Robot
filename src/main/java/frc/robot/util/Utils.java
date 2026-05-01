package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class Utils {
    /** Logs the active command of the given subsystem */
    public static void logActiveCommand(String subsystemName, SubsystemBase subsystem) {
        Logger.recordOutput(
            subsystemName + "/ActiveCommand",
            subsystem.getCurrentCommand() == null
                ? "Nothing"
                : subsystem.getCurrentCommand().getName()
        );
    }

    /** @return true if the robot is running in sim, false otherwise */
    public static boolean isSim() {
        return Constants.currentRobotType == RobotType.SIM;
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
    
    /** @return A copy of the given translation vector with a magnitude of 1 */
    public static Translation2d normalize(Translation2d vector) {
        return vector.div(vector.getNorm());
    }

    /** @return The dot product of translations a and b */
    public static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public static Translation2d[] createRectOutline(Rectangle2d rect, boolean flip) {
        Translation2d center = rect.getCenter().getTranslation();
        if (flip) center = AllianceFlipUtil.flip(center);

        Translation2d[] outline = new Translation2d[5];
        outline[0] = center.plus(new Translation2d(-rect.getXWidth() / 2.0, -rect.getYWidth() / 2.0));
        outline[1] = center.plus(new Translation2d(-rect.getXWidth() / 2.0, rect.getYWidth() / 2.0));
        outline[2] = center.plus(new Translation2d(rect.getXWidth() / 2.0, rect.getYWidth() / 2.0));
        outline[3] = center.plus(new Translation2d(rect.getXWidth() / 2.0, -rect.getYWidth() / 2.0));
        outline[4] = outline[0];
        
        return outline;
    }

    public static Translation2d[] createAxisLineAt(double value, boolean xAxis) {
        Translation2d[] line = new Translation2d[2];
        if (xAxis) {
            line[0] = new Translation2d(0.0, value);
            line[1] = new Translation2d(FieldConstants.fieldWidth, value);
        }
        return line;
    }
}
