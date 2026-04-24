package frc.robot.subsystems.launcher.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double targetVelocityRpm;

        public double leaderVelocityRpm;
        public double leaderAppliedVolts;
        public double leaderStatorCurrentAmps;
        public double leaderSupplyCurrentAmps;
        public double leaderMotorTempCelsius;
        public boolean leaderConnected;

        public double followerVelocityRpm;
        public double followerAppliedVolts;
        public double followerStatorCurrentAmps;
        public double followerSupplyCurrentAmps;
        public double followerMotorTempCelsius;
        public boolean followerConnected;
    }

    default void setRPM(double rpm) {}
    default void setVoltage(double volts) {}
    default boolean isWithinTarget(double range) { return false; }

    default void updateInputs(FlywheelIOInputs inputs) {}
}
