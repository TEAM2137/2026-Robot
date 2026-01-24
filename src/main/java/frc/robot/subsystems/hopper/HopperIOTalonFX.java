package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;

public class HopperIOTalonFX implements HopperIO {
    public static class Constants {
        public static final int indexerId = 20;
        public static final int feederId = 21;
        public static final double indexerGearing = 1.0;
        public static final double feederGearing = 1.0;
    }

    protected final TalonFX indexerMotor;
    protected final TalonFX feederMotor;

    public HopperIOTalonFX() {
        this.indexerMotor = new TalonFX(Constants.indexerId);
        this.feederMotor = new TalonFX(Constants.feederId);
    }

    @Override
    public void runIndexer(double volts) {
        this.indexerMotor.setVoltage(volts);
    }

    @Override
    public void runFeeder(double volts) {
        this.feederMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.indexerSpeedVolts = this.indexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.feedMotorSpeedVolts = this.feederMotor.getMotorVoltage().getValueAsDouble();
    }
}