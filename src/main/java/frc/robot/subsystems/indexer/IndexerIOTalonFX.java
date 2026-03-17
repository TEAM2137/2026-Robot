package frc.robot.subsystems.indexer;

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
        this.indexer = new TalonFX(Constants.indexerId, "turret");
        this.indexer.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));
        
        this.feeder = new TalonFX(Constants.feederId, "turret");
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
        inputs.indexerAppliedVolts = this.indexer.getMotorVoltage().getValueAsDouble();
        inputs.indexerStatorCurrentAmps = this.indexer.getStatorCurrent().getValueAsDouble();
        inputs.indexerSupplyCurrentAmps = this.indexer.getSupplyCurrent().getValueAsDouble();
        inputs.indexerTempCelsius = this.indexer.getDeviceTemp().getValueAsDouble();
        inputs.indexerConnected = this.indexer.isConnected();

        inputs.feederAppliedVolts = this.feeder.getMotorVoltage().getValueAsDouble();
        inputs.feederStatorCurrentAmps = this.feeder.getStatorCurrent().getValueAsDouble();
        inputs.feederSupplyCurrentAmps = this.feeder.getSupplyCurrent().getValueAsDouble();
        inputs.feederTempCelsius = this.feeder.getDeviceTemp().getValueAsDouble();
        inputs.feederConnected = this.feeder.isConnected();
    }
}