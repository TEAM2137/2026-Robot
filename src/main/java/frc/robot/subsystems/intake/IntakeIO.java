package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double positionRotations;
        public double targetPositionRotations;
        
        public double rollerAppliedVolts;
        public double rollerVelocityRpm;
        public double rollerTargetVelocityRpm;

        public boolean pivotConnected;
        public boolean rollersConnected;
    }

    default void setRollerVoltage(double volts) {}
    default void setRollerRPM(double rpm) {}
    default void setPosition(double position) {}
    default void setPivotVoltage(double volts) {}
    default void resetPosition() {}

    default void updateInputs(IntakeIOInputs inputs) {}
}