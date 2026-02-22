package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

public class ClimberIOTalonFX implements ClimberIO {
    public static class Constants {
        public static final int id = 45;
        public static final double gearing = 9.0;
        public static final double spoolDiameter = 0.75; // in

        public static final double kP = 0.0;
        public static final double kD = 0.0;
        public static final double kG = 0.0;
    }

    protected final TalonFXS motor;

    protected double targetPositionInches;

    public ClimberIOTalonFX() {
        this.motor = new TalonFXS(Constants.id);

        this.motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));

        this.motor.getConfigurator().apply(new CommutationConfigs()
            .withMotorArrangement(MotorArrangementValue.NEO_JST));

        this.motor.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.kP).withKD(Constants.kD).withKG(Constants.kG));

        this.motor.setPosition(0.0);
    }

    @Override
    public void setPosition(double target) {
        this.targetPositionInches = target;
        this.motor.setControl(new PositionVoltage(target));
    }

    @Override
    public void setVoltage(double volts) {
        this.targetPositionInches = 0.0;
        this.motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.positionInches = this.motor.getPosition().getValueAsDouble() * Math.PI * Constants.spoolDiameter;
        inputs.targetPositionInches = this.targetPositionInches;
        inputs.connected = this.motor.isConnected();
    }
}
