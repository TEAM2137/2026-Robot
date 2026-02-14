package frc.robot.subsystems.launcher.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double velocityRpm;
        public double targetVelocityRpm;
        public boolean flywheelConnected;
        public boolean feederConnected;
    }

    default void setRPM(double rpm) {}
    default void setVoltage(double volts) {}

    default void updateInputs(FlywheelIOInputs inputs) {}
}
