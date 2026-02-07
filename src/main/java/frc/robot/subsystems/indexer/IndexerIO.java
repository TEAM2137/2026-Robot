package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double indexerSpeedVolts;
        public double feederSpeedVolts;

        public boolean indexerConnected;
        public boolean feederConnected;
    }

    default void runIndexer(double volts) {}
    default void runFeeder(double volts) {}

    default void updateInputs(IndexerIOInputs inputs) {}
}
