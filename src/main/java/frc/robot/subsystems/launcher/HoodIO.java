package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double angleDegrees;
    }

    default void setAngle(double degrees) {}

    default void updateInputs(HoodIOInputs inputs) {}
}
