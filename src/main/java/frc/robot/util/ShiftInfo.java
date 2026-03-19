package frc.robot.util;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public enum ShiftInfo {
    AUTO("Auto", 20),
    TRANSITION("Transition", 140),
    SHIFT1("Shift 1", 130),
    SHIFT2("Shift 2", 105),
    SHIFT3("Shift 3", 80),
    SHIFT4("Shift 4", 55),
    ENDGAME("Endgame", 30),
    MATCH_END("Match End", 0),
    NONE("Nothing", -1);

    private final String name;
    private final int startTime;

    ShiftInfo(String name, int startTime) {
        this.name = name;
        this.startTime = startTime;
    }

    public String getName() {
        return this.name;
    }

    public int getStartTime() {
        return this.startTime;
    }

    public int getEndTime() {
        return this.next().getStartTime();
    }

    public boolean isHubActive() {
        boolean wonAuto = didWinAuto().orElse(true);
        return switch (this) {
            case SHIFT1, SHIFT3 -> !wonAuto;
            case SHIFT2, SHIFT4 -> wonAuto;
            default -> true;
        };
    }

    public double timeUntilStart() {
        if (DriverStation.isAutonomous()) return DriverStation.getMatchTime();
        if (this.getStartTime() > DriverStation.getMatchTime()) return 0.0;
        return DriverStation.getMatchTime() - this.getStartTime();
    }

    public double timeSinceStart() {
        if (this.getStartTime() < DriverStation.getMatchTime()) return 0.0;
        return this.getStartTime() - DriverStation.getMatchTime();
    }

    public boolean hasNext() {
        return this.ordinal() + 1 < values().length;
    }

    public boolean hasPrevious() {
        return this.ordinal() - 1 >= 0;
    }

    public ShiftInfo next() {
        if (!this.hasNext()) return ShiftInfo.NONE;
        return values()[this.ordinal() + 1];
    }

    public ShiftInfo previous() {
        if (!this.hasPrevious()) return ShiftInfo.AUTO;
        return values()[this.ordinal() - 1];
    }

    static {
        Trigger autoWinnerMissing = RobotModeTriggers.teleop()
            .and(() -> !hasAutoWinner() && DriverStation.isFMSAttached())
            .debounce(0.5);
        Alerts.add("Auto winner not received from FMS", AlertType.kWarning, autoWinnerMissing);
    }

    public static ShiftInfo getCurrentShift() {
        double matchTime = DriverStation.getMatchTime();
        if (DriverStation.isAutonomous()) return AUTO;
        for (int i = 2; i < values().length; i++) {
            ShiftInfo shift = values()[i];
            if (matchTime > shift.getStartTime()) return values()[i - 1];
        }
        return NONE;
    }

    public static ShiftInfo getNextShift() {
        return getCurrentShift().next();
    }

    public static double getTimeUntilNextShift() {
        return getNextShift().timeUntilStart();
    }

    public static double getTimeUntilActive() {
        ShiftInfo shift = getCurrentShift();
        if (shift.isHubActive()) return 0.0;
        while (shift.hasNext()) {
            shift = shift.next();
            if (shift.isHubActive()) return shift.timeUntilStart();
        }
        return -1.0;
    }

    public static double getTimeUntilInactive() {
        ShiftInfo shift = getCurrentShift();
        if (!shift.isHubActive()) return 0.0;
        while (shift.hasNext()) {
            shift = shift.next();
            if (!shift.isHubActive()) return shift.timeUntilStart();
        }
        return -1.0;
    }

    public static boolean hasAutoWinner() {
        return !DriverStation.getGameSpecificMessage().isBlank();
    }

    public static Optional<Boolean> didWinAuto() {
        if (!hasAutoWinner()) return Optional.empty();
        String msg = DriverStation.getGameSpecificMessage();
        if (msg.equals("R") && AllianceFlipUtil.shouldFlip()) return Optional.of(true);
        if (msg.equals("B") && !AllianceFlipUtil.shouldFlip()) return Optional.of(true);
        return Optional.of(false);
    }

    public static void logShiftInfo() {
        ShiftInfo current = getCurrentShift();
        ShiftInfo next = getNextShift();
        double timeUntilNext = next.timeUntilStart();
        String formatted = next.getName() + " in " + (int) Math.ceil(timeUntilNext) + "s";
        
        Logger.recordOutput("ShiftInfo/NextShiftFormatted", formatted);
        // Logger.recordOutput("ShiftInfo/NextShiftIn", (int) Math.ceil(timeUntilNext));
        Logger.recordOutput("ShiftInfo/NextShiftIn", timeUntilNext);

        Logger.recordOutput("ShiftInfo/CurrentShift", current.getName());
        Logger.recordOutput("ShiftInfo/IsHubActive", current.isHubActive());
        Logger.recordOutput("ShiftInfo/HubActiveIn", Math.max((int) Math.ceil(getTimeUntilActive()), 0));
        Logger.recordOutput("ShiftInfo/HubInactiveIn", Math.max((int) Math.ceil(getTimeUntilInactive()), 0));

        didWinAuto().ifPresent(value -> Logger.recordOutput("ShiftInfo/ActiveFirst", !value));
    }
}
