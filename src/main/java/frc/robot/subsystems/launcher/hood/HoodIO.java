package frc.robot.subsystems.launcher.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double appliedVolts;
        public double statorCurrentAmps;
        public double supplyCurrentAmps;
        public double motorTempCelsius;
        
        public double angleDegrees;
        public double targetAngleDegrees;
        public boolean connected;
    }

    default void setAngle(double degrees) {}
    default void setVoltage(double volts) {}
    default void resetPosition() {}

    default void updateInputs(HoodIOInputs inputs) {}
}
