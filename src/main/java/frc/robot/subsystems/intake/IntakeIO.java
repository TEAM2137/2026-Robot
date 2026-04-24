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

        public double rollerTargetVelocityRpm;
        
        public double rollerLeaderAppliedVolts;
        public double rollerLeaderStatorCurrentAmps;
        public double rollerLeaderSupplyCurrentAmps;
        public double rollerLeaderTempCelsius;
        public double rollerLeaderVelocityRpm;
        public boolean rollerLeaderConnected;
        
        public double rollerFollowerAppliedVolts;
        public double rollerFollowerStatorCurrentAmps;
        public double rollerFollowerSupplyCurrentAmps;
        public double rollerFollowerTempCelsius;
        public double rollerFollowerVelocityRpm;
        public boolean rollerFollowerConnected;
    }

    default void setRollerVoltage(double volts) {}
    default void setRollerRPM(double rpm) {}
    default void setPosition(double position) {}
    default void setPositionAndHold(double position) {}
    default void setPivotVoltage(double volts) {}
    default void resetPosition() {}

    default void updateInputs(IntakeIOInputs inputs) {}
}