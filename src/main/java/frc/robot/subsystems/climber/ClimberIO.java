package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double positionInches;
        public double targetPositionInches;

        public boolean upperHookState;
        public boolean lowerHookState;

        public boolean connected;
    }

    default void setPosition(double target) {}
    default void setVoltage(double volts) {}

    default void deployUpperHooks() {}
    default void deployLowerHooks() {}
    default void retractUpperHooks() {}
    default void retractLowerHooks() {}

    default void updateInputs(ClimberIOInputs inputs) {}
}
