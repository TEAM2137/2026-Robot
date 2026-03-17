package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double pivotAppliedVolts;
        public double pivotStatorCurrentAmps;
        public double pivotSupplyCurrentAmps;
        public double pivotTempCelsius;
        public double pivotPositionRotations;
        public double pivotTargetPositionRotations;
        public boolean pivotConnected;
        
        public double rollerAppliedVolts;
        public double rollerStatorCurrentAmps;
        public double rollerSupplyCurrentAmps;
        public double rollerTempCelsius;
        public double rollerVelocityRpm;
        public double rollerTargetVelocityRpm;
        public boolean rollersConnected;
    }

    default void setRollerVoltage(double volts) {}
    default void setRollerRPM(double rpm) {}
    default void setPosition(double position) {}
    default void setPivotVoltage(double volts) {}
    default void resetPosition() {}

    default void updateInputs(IntakeIOInputs inputs) {}
}