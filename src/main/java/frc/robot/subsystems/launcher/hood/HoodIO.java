package frc.robot.subsystems.launcher.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double angleDegrees;
        public boolean connected;
    }

    default void setAngle(double degrees) {}

    default void updateInputs(HoodIOInputs inputs) {}
}
