package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double speedRpm;
    }

    default void setSpeed(double rpm) {}

    default void updateInputs(FlywheelIOInputs inputs) {}
}
