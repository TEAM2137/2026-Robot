package frc.robot.subsystems.launcher.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double appliedVolts;
        public double statorCurrentAmps;
        public double supplyCurrentAmps;
        public double velocityRpm;
        public double targetVelocityRpm;
        public boolean flywheelConnected;
    }

    default void setRPM(double rpm) {}
    default void setVoltage(double volts) {}
    default boolean isWithinTarget(double range) { return false; }

    default void updateInputs(FlywheelIOInputs inputs) {}
}
