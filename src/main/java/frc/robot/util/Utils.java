package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    /**
     * @return The next match event including a name, timestamp, and if the hub is currently active
     */
    public static MatchEvent getNextMatchEvent() {
        double time = DriverStation.getMatchTime();
        if (DriverStation.isAutonomous()) return new MatchEvent("Teleop", true, time);

        String msg = DriverStation.getGameSpecificMessage();
        boolean wonAuto = false;
        if (msg.equals("R") && AllianceFlipUtil.shouldFlip()) wonAuto = true;
        if (msg.equals("B") && !AllianceFlipUtil.shouldFlip()) wonAuto = true;

        String active = "Hub Active";
        String inactive = "Hub Inactive";

        if (time > 140.0) return new MatchEvent("Teleop (Transition)", true, time - 140.0);
        if (time > 130.0) return new MatchEvent((wonAuto ? inactive : active) + " (P1)", true, time - 130.0);
        if (time > 105.0) return new MatchEvent((wonAuto ? active : inactive) + " (P2)", !wonAuto, time - 105.0);
        if (time > 80.0) return new MatchEvent((wonAuto ? inactive : active) + " (P3)", wonAuto, time - 80.0);
        if (time > 55.0) return new MatchEvent((wonAuto ? active : inactive) + " (P4)", !wonAuto, time - 55.0);
        if (time > 30.0) return new MatchEvent(active + " (Endgame)", wonAuto, time - 30.0);
        return new MatchEvent("Match Ends", true, time);
    }

    public record MatchEvent(String name, boolean isHubActive, double time) {
        @Override
        public final String toString() {
            return name + " in " + ((int)Math.ceil(time)) + "s";
        }
    }
}
