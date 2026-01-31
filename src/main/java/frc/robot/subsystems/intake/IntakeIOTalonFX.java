package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeIOTalonFX implements IntakeIO {
    public static class Constants {
        public static final int rollersId = 40;
        public static final double gearing = 1.0;
    }

    protected final TalonFX rollers;

    public IntakeIOTalonFX() {
        this.rollers = new TalonFX(Constants.rollersId);
        this.rollers.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    @Override
    public void runRollers(double volts) {
        this.rollers.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerSpeedVolts = this.rollers.getMotorVoltage().getValueAsDouble();
    }
}