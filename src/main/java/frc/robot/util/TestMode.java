package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public enum TestMode {
    ALL("All"),
    INTAKE("Intake"),
    LAUNCHER("Launcher");

    private final String name;
    private final Trigger isActive;

    private TestMode(String name) {
        this.name = name;
        this.isActive = RobotModeTriggers.test().and(new Trigger(() -> RobotContainer.getInstance().getTestMode() == this));
    }

    public String getName() {
        return this.name;
    }

    public Trigger isActive() {
        return this.isActive;
    }
}
