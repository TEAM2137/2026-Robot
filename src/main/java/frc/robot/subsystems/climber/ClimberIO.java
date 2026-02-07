package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double position;
        public double targetPosition;

        public boolean upperHookState;
        public boolean lowerHookState;

        public boolean connected;
    }

    default void setPosition(double target) {}

    default void deployUpperHooks() {}
    default void deployLowerHooks() {}
    default void retractUpperHooks() {}
    default void retractLowerHooks() {}

    default void updateInputs(ClimberIOInputs inputs) {}
}
