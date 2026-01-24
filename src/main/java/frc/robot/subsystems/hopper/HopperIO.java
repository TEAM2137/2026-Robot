package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        public double indexerSpeedVolts;
        public double feedMotorSpeedVolts;
    }

    default void runIndexer(double volts) {}
    default void runFeeder(double volts) {}

    default void updateInputs(HopperIOInputs inputs) {}
}
