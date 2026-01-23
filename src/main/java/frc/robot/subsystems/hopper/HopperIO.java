package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        public double indexerSpeedVolts;
    }

    default void runIndexer(double volts) {}
}
