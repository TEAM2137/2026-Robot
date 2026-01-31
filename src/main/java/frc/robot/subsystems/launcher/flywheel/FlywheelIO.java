package frc.robot.subsystems.launcher.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double speedRpm;
    }

    default void setRPM(double rpm) {}
    default void setVoltage(double volts) {}

    default void updateInputs(FlywheelIOInputs inputs) {}
}
