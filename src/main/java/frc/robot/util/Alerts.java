package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Alerts {
    public static void add(String text, AlertType type, BooleanSupplier condition) {
        add(new Alert(text, type), new Trigger(condition));
    }

    public static void add(String text, AlertType type, Trigger trigger) {
        add(new Alert(text, type), trigger);
    }

    public static void add(Alert alert, Trigger condition) {
        alert.set(condition.getAsBoolean());
        condition.onTrue(Commands.runOnce(() -> alert.set(true)).ignoringDisable(true));
        condition.onFalse(Commands.runOnce(() -> alert.set(false)).ignoringDisable(true));
    }
}
