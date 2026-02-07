package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeIOTalonFX implements IntakeIO {
    public static class Constants {
        public static final int rollersId = 40;
        public static final int pivotId = 40;
        public static final double pivotGearing = 3753.0 / 1568.0;
    }

    protected final TalonFX pivot;
    protected final TalonFX rollers;

    public IntakeIOTalonFX() {
        this.rollers = new TalonFX(Constants.rollersId);
        this.rollers.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive));

        this.pivot = new TalonFX(Constants.pivotId);
        this.pivot.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.pivotGearing));
    }

    @Override
    public void runRollers(double volts) {
        this.rollers.setVoltage(volts);
    }

    @Override
    public void setPosition(double position) {
        this.pivot.setPosition(position / 360.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition = 0;
        inputs.rollerSpeedVolts = this.rollers.getMotorVoltage().getValueAsDouble();

        inputs.pivotConnected = false;
        inputs.rollersConnected = this.rollers.isConnected();
    }
}