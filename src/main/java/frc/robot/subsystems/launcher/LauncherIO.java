package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
    @AutoLog
    public static class LauncherIOInputs {
        public double flywheelSpeedRpm;
        public double hoodAngleDegrees;
        public double turretAngleDegrees;
        public boolean didZero;
    }

    default void setFlywheelSpeed(double rpm) {}

    default void setHoodAngle(double degrees) {}

    default void setTurretAngle(double degrees) {}
}
