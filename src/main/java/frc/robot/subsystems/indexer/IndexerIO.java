package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double indexerAppliedVolts;
        public double indexerStatorCurrentAmps;
        public double indexerSupplyCurrentAmps;
        public boolean indexerConnected;

        public double feederAppliedVolts;
        public double feederStatorCurrentAmps;
        public double feederSupplyCurrentAmps;
        public boolean feederConnected;
    }

    default void runIndexer(double volts) {}
    default void runFeeder(double volts) {}

    default void updateInputs(IndexerIOInputs inputs) {}
}
