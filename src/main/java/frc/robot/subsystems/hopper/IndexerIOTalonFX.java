package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class IndexerIOTalonFX implements IndexerIO {
    public static class Constants {
        public static final int indexerId = 26;
        public static final int feederId = 25;
        public static final double indexerGearing = 1.0;
        public static final double feederGearing = 1.0;
    }

    protected final TalonFX indexer;
    protected final TalonFX feeder;

    public IndexerIOTalonFX() {
        this.indexer = new TalonFX(Constants.indexerId);
        this.indexer.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));
        
        this.feeder = new TalonFX(Constants.feederId);
        this.feeder.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    @Override
    public void runIndexer(double volts) {
        this.indexer.setVoltage(volts);
    }

    @Override
    public void runFeeder(double volts) {
        this.feeder.setVoltage(volts);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerSpeedVolts = this.indexer.getMotorVoltage().getValueAsDouble();
        inputs.feedMotorSpeedVolts = this.feeder.getMotorVoltage().getValueAsDouble();
    }
}